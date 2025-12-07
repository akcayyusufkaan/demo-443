#include <stdint.h>
#include <stdio.h>

/* SmartHygiene Toilet System*/

/* PI1: GPIO & Timers
    Implements the core MVP:
      - Motion-based light (PA5->input, PA6->LED output) with timed auto-off
      - Handwave-based cover motor trigger (PA11->input, PA12->motor output)
      - Motor runtime timing + cooldown timing using TIM6/TIM7

    Refactoring notes:
      - Split HW init into small functions (GPIO init, TIM init)
      - Introduced small inline helpers to hide register dancing
      - Centralized tunables via #define to keep PI1 timing consistent
*/

/* PI2: IC & OC
     Input Capture (TIM15 CH1 - PA2, IR sensor)
     ------------------------------------------------
     - PA2 is configured as Alternate Function AF14 -> TIM15_CH1.
     - TIM15 Channel 1 is configured as Input Capture on TI1:
          * CC1S = 01 (input, mapped on TI1)
          * CC1P/CC1NP configured for both-edge capture (falling + rising)
     - CC1 interrupt is enabled (CC1IE in DIER) and handled in TIM15_IRQHandler.
     - On each edge, TIM15->CCR1 is read as a hardware timestamp in milliseconds:
          * Falling edge (IR LOW)  = "sit" event   -> sit_start_time_ms = CCR1
          * Rising  edge (IR HIGH) = "stand" event -> sit_end_time_ms   = CCR1
     - The sitting duration is computed from two CCR1 timestamps:
          * elapsed_ms = sit_end_time_ms - sit_start_time_ms
            (with explicit handling of timer overflow using TIM15->ARR).
     - This elapsed_ms value is compared against OCCUPIED_THRESHOLD_MS to
       decide whether the seat was truly occupied or just a short contact.

     Output Compare And PWM (TIM2 CH1 + CH2, Servo motor)
     ------------------------------------------------
     1) TIM2_CH1 -> Servo PWM signal on PA0
        - PA0 is configured as Alternate Function AF1 -> TIM2_CH1.
        - TIM2 Channel 1 is configured as an Output Compare channel:
             * CC1S = 00 (output)
             * OC1M = 110 (PWM mode 1) with preload enabled (OC1PE) and
               ARR preload enabled (ARPE).
        - TIM2->CCR1 holds the pulse width in timer ticks; it is updated
          according to the current servo angle via map_angle_to_pulse().
        - This generates a real hardware PWM signal (1–2 ms pulse width
          within a 20 ms period), which directly drives the servo motor.

     2) TIM2_CH2 -> Output Compare interrupt "time base" (no external pin)
        - TIM2 Channel 2 is configured purely as an Output Compare channel:
             * CC2S = 00 (output compare)
             * OC2M = 000 (frozen mode – we only use the CC2IF event).
        - TIM2->CCR2 is set so that CC2IF occurs once per timer period
          (here CCR2 = 0 while CNT runs from 0 to ARR, every 20 ms).
        - CC2 interrupt (CC2IE in DIER) is enabled and handled in
          TIM2_IRQHandler.
        - On every CC2IF (every 20 ms):
             * If g_servo_delay_ticks > 0 and the servo sweep is not
               started yet (g_servo_enabled == 0), the delay counter is
               decremented. When it reaches zero and at least one sweep is
               pending, a sweep (0° -> 90° -> 0°) is armed.
             * If the servo sweep is active (g_servo_enabled == 1), the
               servo angle g_servo_angle is stepped by SERVO_STEP_DEG on
               each tick, the direction is reversed at the limits
               (0° / 90°), and TIM2->CCR1 is updated with the new pulse
               width via map_angle_to_pulse(). Once a sweep is completed,
               we either insert an extra pause before the next sweep or
               stop if no sweeps remain.
        - Thus TIM2_CH2 Output Compare provides a precise 20 ms time base
          to implement both the auto-flush delay and the servo sweep logic.
*/

/* PI3: ADC, UART & EXTI Refinements
   ---------------------------------
   - Water level sensing (ADC1 on PA1)
       * PA1 configured as analog input (channel 6).
       * ADC1 taken out of deep-power-down, calibrated once, then runs
         continuous conversions.
       * ADC1_2_IRQHandler stores the latest reading in `water_level`
         and immediately starts the next conversion.

   - UART interface (LPUART1 on PG7/PG8)
       * PG7/PG8 set to AF8 for LPUART1 TX/RX at 115200 baud.
       * RX interrupt is used as a simple command interface:
           - 'W' / 'w' → set `uart_print_water_flag`
           - 'I' / 'i' → set `uart_print_presence_flag`
       * `update()` (called in the main loop) checks these flags and prints:
           - Water level as LOW / MEDIUM / HIGH based on `water_level`
           - Restroom occupancy based on `g_light_time`
         so all long prints stay outside ISRs.

   - EXTI changes (motion & hand sensors)
       * PA5 (motion sensor) and PA11 (hand sensor) are now handled fully
         by EXTI interrupts instead of polling in main():
           - EXTI5_IRQHandler: on rising/falling edge of PA5, LED timer is
             (re)started and `g_light_time`/`g_light_sensor` are updated.
           - EXTI11_IRQHandler: on falling edge of PA11, cover motor starts
             if `g_cover_state == 0` and no cooldown is active.
       * Main loop is reduced to `update()` + `wfi`, and all real-time
         behavior (timers, EXTI, ADC, UART) is interrupt-driven.

   - TIM16 / TIM17 IRQ handlers (GPO timing)
       * TIM16 and TIM17 are used as simple general-purpose timers for
         GPIO-related output timing (GPO).
*/


/* ===================== System Parameters from PI1 ===================== */
#define LED_INPUT_PIN         5                   // PA5
#define LED_OUTPUT_PIN        (LED_INPUT_PIN+1)   // PA6
#define COVER_INPUT_PIN       11                  // PA11
#define COVER_OUTPUT_PIN      (COVER_INPUT_PIN+1) // PA12

#define LED_ON_SEC                  7  // seconds light remains ON after turn off switch
#define COVER_COOLDOWN_SEC          7  // MIN time between cover rotations
#define MOTOR_RUNNING_SEC           3  // cover motor running time
#define OCCUPIED_THRESHOLD_SEC      5 // seconds seated to confirm usage
#define AUTO_FLUSH_DELAY_SEC        3  // delay after standing up (auto flush)
#define WATER_LEVEL_HIGH_MM       150  // mm from bottom

/* ===================== System Parameters from PI2 ===================== */
// IR / LED pins
#define IR_INPUT_PIN         2        // PA2
#define RED_LED_PIN          9        // PA9
#define BLUE_LED_PIN         7        // PB7
// Servo pin
#define SERVO_SIGNAL_PIN     0     // PA0
// TIM2 parameters for servo PWM
#define TIM2_PSC_VAL      3
#define TIM2_ARR_VAL      19999 // For a 20 ms period we need 20000 counts
#define TIM2_PERIOD_MS    20u   // 20 ms period with these PSC/ARR settings
// TIM15 parameters for input capture
#define TIM15_PSC_VAL     3999   // 1 MHz timer clock (1 µs resolution)
#define TIM15_ARR_VAL     0xFFFF // 16-bit timer
// Servo pulse widths in timer ticks (1 tick = 1 µs at 1 MHz)
#define SERVO_MIN_PULSE   1000  // 1000 µs = 1 ms  (0 degrees)
#define SERVO_MAX_PULSE   2000  // 2000 µs = 2 ms  (90 degrees)
#define SERVO_MIN_ANGLE   0     // Servo angle limits
#define SERVO_MAX_ANGLE   65
#define SERVO_STEP_DEG    4     // Angular step per OC event
// How many TIM2 periods to wait before starting servo sweep
#define AUTO_FLUSH_DELAY_TICKS     ((AUTO_FLUSH_DELAY_SEC * 1000u) / TIM2_PERIOD_MS)
// Delay between two sweeps (2 second)
#define INTER_SWEEP_DELAY_SEC      2
#define INTER_SWEEP_DELAY_TICKS    ((INTER_SWEEP_DELAY_SEC * 1000u) / TIM2_PERIOD_MS)

#define LPUART_TX_BUF_SIZE 1024

volatile char     lpuart_tx_buf[LPUART_TX_BUF_SIZE]; //buffer to store the strings send by mc
volatile uint16_t lpuart_tx_head = 0;   // next position to write
volatile uint16_t lpuart_tx_tail = 0;   // next position to read



/* ===================== Global System State from PI1 ===================== */
int g_light_sensor = 0;       // 0 dark, 1 light
int g_light_time = 0;         // accumulated light seconds
int g_cover_cooldown = 0;     // cover rotation rate-limit
int g_cover_state=0;          // 0:closed,1:open

/* ===================== Global IC State from PI2 ===================== */
static uint8_t  is_sitting                  = 0; // 0: nobody sitting, 1: someone sitting
static uint32_t sit_start_time_ms           = 0;
static const uint32_t OCCUPIED_THRESHOLD_MS = OCCUPIED_THRESHOLD_SEC * 1000u;

/* ===================== Global Servo State from PI2 ===================== */
static volatile int      g_servo_angle            = 0;   // Current angle (0..90)
static volatile int      g_servo_dir              = 1;   // 1: increasing, -1: decreasing
static volatile uint8_t  g_servo_enabled          = 0;   // 1 while sweep is running
static volatile uint8_t  g_servo_reached_max      = 0;   // 1 after reaching 90° in current sweep
static volatile uint32_t g_servo_delay_ticks      = 0;   // countdown (TIM2 periods) before next sweep
static volatile uint8_t  g_servo_sweeps_remaining = 0;   // how many sweeps to perform

volatile uint16_t water_level = 0; // digital value read by ADC
volatile uint8_t uart_print_water_flag = 0; // flag  triggered by admin panel to sent water level
volatile uint8_t uart_print_presence_flag = 0; // flag  triggered by admin panel to sent presence info
volatile uint8_t adc_ready= 0;



/* ===================== Peripheral Register from PI1 ===================== */
typedef struct{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    uint32_t reserved;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    uint32_t reserved1[3];
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
} TIMxBasicType;
typedef struct{
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFRL;
    volatile uint32_t AFRH;
    volatile uint32_t BRR;
    uint32_t reserved;
    volatile uint32_t SECCFGR;
} GPIO;

typedef struct{
    volatile uint32_t RTSR1;
    volatile uint32_t FTSR1;
    volatile uint32_t SWIER1;
    volatile uint32_t RPR1;
    volatile uint32_t FPR1;
    volatile uint32_t SECCFGR1;
    volatile uint32_t PRIVCFG1;
    uint32_t reserved1;
    volatile uint32_t RTSR2;
    volatile uint32_t FTSR2;
    volatile uint32_t SWIER2;
    volatile uint32_t RPR2;
    volatile uint32_t FPR2;
    volatile uint32_t SECCFG2;
    volatile uint32_t PRIVCFG2;
    uint32_t reserved2[9];
    volatile uint32_t EXTICR1;
    volatile uint32_t EXTICR2;
    volatile uint32_t EXTICR3;
    volatile uint32_t EXTICR4;
    volatile uint32_t LOCKR;
    uint32_t reserved3[3];
    volatile uint32_t IMR1;
    volatile uint32_t EMR1;
    uint32_t reserved4[2];
    volatile uint32_t IMR2;
    volatile uint32_t EMR2;
} EXTI_struct;

/* ===================== Peripheral Register from PI2 ===================== */
// TIM15 Structure
typedef struct {
    volatile uint32_t CR1;        // 0x00
    volatile uint32_t CR2;        // 0x04
    volatile uint32_t SMCR;       // 0x08
    volatile uint32_t DIER;       // 0x0C
    volatile uint32_t SR;         // 0x10
    volatile uint32_t EGR;        // 0x14
    volatile uint32_t CCMR1;      // 0x18
    uint32_t reserved1;           // 0x1C
    volatile uint32_t CCER;       // 0x20
    volatile uint32_t CNT;        // 0x24
    volatile uint32_t PSC;        // 0x28
    volatile uint32_t ARR;        // 0x2C
    volatile uint32_t RCR;        // 0x30
    volatile uint32_t CCR1;       // 0x34
    volatile uint32_t CCR2;       // 0x38
    uint32_t reserved2[2];        // 0x3C, 0x40
    volatile uint32_t BDTR;       // 0x44
    volatile uint32_t DCR;        // 0x48
    volatile uint32_t DMAR;       // 0x4C
    volatile uint32_t OR1;        // 0x50
    uint32_t reserved3[3];        // 0x54, 0x58, 0x5C
    volatile uint32_t OR2;        // 0x60
} TIM15_General_Purpose_Type;
// TIM2/3/4/5 Structure (32-bit General-Purpose Timer)
typedef struct {
    volatile uint32_t CR1;        // 0x00
    volatile uint32_t CR2;        // 0x04
    volatile uint32_t SMCR;       // 0x08
    volatile uint32_t DIER;       // 0x0C
    volatile uint32_t SR;         // 0x10
    volatile uint32_t EGR;        // 0x14
    volatile uint32_t CCMR1;      // 0x18
    volatile uint32_t CCMR2;      // 0x1C
    volatile uint32_t CCER;       // 0x20
    volatile uint32_t CNT;        // 0x24 (32-bit)
    volatile uint32_t PSC;        // 0x28
    volatile uint32_t ARR;        // 0x2C (32-bit)
    uint32_t reserved1;           // 0x30
    volatile uint32_t CCR1;       // 0x34 (32-bit)
    volatile uint32_t CCR2;       // 0x38 (32-bit)
    volatile uint32_t CCR3;       // 0x3C (32-bit)
    volatile uint32_t CCR4;       // 0x40 (32-bit)
    uint32_t reserved2;           // 0x44
    volatile uint32_t DCR;        // 0x48
    volatile uint32_t DMAR;       // 0x4C
} TIM_General_Purpose_32bit_Type;

/* ===================== Peripheral Register from PI3 ===================== */
// ADC Structure

typedef struct{
	volatile uint32_t ISR;     //0
	volatile uint32_t IER;     //4
	volatile uint32_t CR;      //8
	volatile uint32_t CFGR;    //C
	volatile uint32_t CFG2;    //10
	volatile uint32_t SMPR1;   //14
	volatile uint32_t SMPR2;   //18
	uint32_t reserved2;        //1C
	volatile uint32_t TR1;     //20
	volatile uint32_t TR2;     //24
	volatile uint32_t TR3;     //28
	uint32_t reserved3;        //2C
	volatile uint32_t SQR1;    //30
	volatile uint32_t SQR2;    //34
	volatile uint32_t SQR3;    //38
	volatile uint32_t SQR4;    //3C
	volatile uint32_t DR;      //40
	uint32_t reserved4[2];     //44 48
	volatile uint32_t JSQR;    //4C
	uint32_t reserved5[4];     //50 54 58 5C
	volatile uint32_t OFR1;    //60
	volatile uint32_t OFR2;    //64
	volatile uint32_t OFR3;    //68
	volatile uint32_t OFR4;    //6C
	uint32_t reserved6[4];     //70 74 78 7C
	volatile uint32_t JDR1;    //80
	volatile uint32_t JDR2;    //84
	volatile uint32_t JDR3;    //88
	volatile uint32_t JDR4;    //8C
	uint32_t reserved7[4];     //90 94 98 9C
	volatile uint32_t AWD2CR;  //A0
	volatile uint32_t AWD3CR;  //A4
	uint32_t reserved8[2];     //A8 AC
	volatile uint32_t DIFSEL;  //B0
	volatile uint32_t CALFACT; //B4
} ADCType;


// ADCCommon Structure
typedef struct{
	volatile uint32_t CSR; //0
	uint32_t reserved1;    //4
	volatile uint32_t CCR; //8
	volatile uint32_t CDR; //C
} ADCCommon;

// LPUART Structure
typedef struct{
	volatile uint32_t CR1; //0
	volatile uint32_t CR2; //4
	volatile uint32_t CR3; //8
	volatile uint32_t BRR; //C
	uint32_t reserved1[2]; //10 14
	volatile uint32_t RQR; //18
	volatile uint32_t ISR; //1C
	volatile uint32_t ICR; //20
	volatile uint32_t RDR; //24
	volatile uint32_t TDR; //28
	volatile uint32_t PRESC; //2C
} LPUARTType;

/* ===================== Peripheral Base Addresses from PI1 ===================== */
#define RCC_AHB2ENR *((volatile uint32_t *) 0x4002104C)
#define RCC_APB1ENR1 *((volatile uint32_t *) 0x40021058)
#define RCC_APB1ENR2 *((volatile uint32_t *) 0x4002105C)
#define GPIOA ((GPIO *) 0x42020000)
#define TIM6 ((TIMxBasicType *) 0x40001000)
#define TIM7 ((TIMxBasicType *) 0x40001400)

/* ===================== Peripheral Base Addresses from PI2 ===================== */
#define RCC_APB2ENR   (*(volatile uint32_t *) 0x40021060) // for TIM15... clocks enable

#define GPIOB ((GPIO *) 0x42020400)
#define TIM15 ((TIM15_General_Purpose_Type *) 0x40014000)
#define TIM2  ((TIM_General_Purpose_32bit_Type *) 0x40000000)
#define EXTI  ((EXTI_struct *) 0x4002F400)
// NVIC ISER registers
#define ISER0 		(*((volatile uint32_t *) 0xE000E100))	// NVIC ISER0 -> IRQ 0..31
#define ISER1         (*(volatile uint32_t *) 0xE000E104)  // NVIC ISER1 -> IRQ 32..63
#define ISER2         (*(volatile uint32_t *) 0xE000E108)  // NVIC ISER2 -> IRQ 64..95
#define TIM15_IRQN                   69        // TIM15 global interrupt = IRQ 69
#define TIM15_IRQ_BIT  (TIM15_IRQN - 64)       // 5
#define TIM2_IRQN                    45        // TIM2 global interrupt = IRQ 45
#define TIM2_IRQ_BIT    (TIM2_IRQN - 32)       // 13


/* ===================== Peripheral Base Addresses from PI2 ===================== */

#define LPUART1 ((LPUARTType *)  0x40008000)   // Base address of LPUART1 (UART used for PC communication)
#define GPIOG   ((GPIO *)        0x42021800)   // GPIOG base – provides PG7/PG8 used for LPUART1 TX/RX pins

#define ADC1    ((ADCType *)     0x42028000)   // ADC1 peripheral base – performs water-level ADC conversions
#define ADC     ((ADCCommon *)   0x42028300)   // ADC common registers – shared clock and analog configuration

#define RCC_CCIPR1 *((volatile uint32_t *) 0x40021088)
#define PWR_CR1 *((volatile uint32_t *) 0x40007000)
#define PWR_CR2 *((volatile uint32_t *) 0x40007004)

int pulse = 0;


// Map servo angle to pulse width
static uint32_t map_angle_to_pulse(int angle)
{
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    // Linear mapping: (angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180 + SERVO_MIN_PULSE
    pulse = (uint32_t)(((int32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180
                      + SERVO_MIN_PULSE);
    return pulse;
}

/* ===================== Motion Sensor GPIO Setup from PI1 ===================== */
void motion_sensor_initialize(void) {
    // GPIOA setup
    RCC_AHB2ENR |= 1;          // Enable GPIOA clock
    GPIOA->MODER &= ~(0xF << (LED_INPUT_PIN * 2)); // PA5-6 is cleared.
    GPIOA->MODER |= (0x4 << (LED_INPUT_PIN * 2)); // PA5-6 is input-output.
}

/* ===================== Hand Sensor GPIO Setup from PI1 ===================== */
void hand_sensor_initialize(void) {
    // GPIOA setup
    RCC_AHB2ENR |= 1;          // Enable GPIOA clock
    GPIOA->MODER &= ~(0xF << (COVER_INPUT_PIN * 2)); // PA11-12 is cleared.
    GPIOA->MODER |= (0x4 << (COVER_INPUT_PIN * 2)); // PA11-12 is input-output.
}

/* ===================== Light Timer Setup from PI1 ===================== */
void light_timer_initialize(void) {
    // TIM6 setup for LED timeout
    RCC_APB1ENR1 |= (1 << 4);       // Enable TIM6 clock

    TIM6->PSC = 3999;                           // 4 MHz / (3999+1) = 1 kHz
    TIM6->ARR = (1000 * LED_ON_SEC - 1);        // LED_ON_SEC seconds

    TIM6->CR1 &= ~(1 << 1);                     // URS = 0: overflow -> update event
    TIM6->DIER |= 1u;                           // UIE = 1: enable update interrupt
    ISER1 |= (1 << 17);							// Enable global interrupts for TIM6
    TIM6->SR = 0;                               // Clear any pending UIF

    TIM6->CR1 |= 1;                             // CEN = 1: start counter



}




/* ===================== Cover Motor Timer Setup from PI1 ===================== */
void cover_motor_timer_initialize(void) {
    // TIM7 setup for motor runtime + cooldown
    RCC_APB1ENR1 |= (1 << 5);       // Enable TIM7 clock

    TIM7->PSC = 3999;                               // 1 kHz
    TIM7->ARR = (1000 * MOTOR_RUNNING_SEC - 1);     // motor run time

    TIM7->CR1 &= ~(1 << 1);                         // URS = 0
    TIM7->DIER |= 1u;                               // UIE = 1: enable update interrupt
    ISER1 |= (1u << 18);							// Enable global interrupts for TIM7
    TIM7->SR = 0;                                   // Clear UIF

    TIM7->CR1 |= 1;                                 // CEN = 1: start counter


}



/* ===================== GPIO Setup (LED + IR + Servo) from PI2 ===================== */
// We used PS7 and PS8 code files.
static void gpio_initialize(void)
{
    // Enable GPIOA, GPIOB clock
    RCC_AHB2ENR |= (0b11u << 0);

    // ---- LEDs ----
    // PA9 -> RED LED, output
    GPIOA->MODER &= ~(0b11u << (RED_LED_PIN * 2));
    GPIOA->MODER |=  (0b01u << (RED_LED_PIN * 2));

    // PB7 -> BLUE LED, output
    GPIOB->MODER &= ~(0b11u << (BLUE_LED_PIN * 2));
    GPIOB->MODER |=  (0b01u << (BLUE_LED_PIN * 2));

    // ---- IR Input Capture Pin ----
    // 1. Set PA2 to Alternate Function mode
    GPIOA->MODER &= ~(0b11u << (IR_INPUT_PIN * 2));
    GPIOA->MODER |=  (0b10u << (IR_INPUT_PIN * 2)); // AF mode
    // 2. Select AF14 (TIM15_CH1) for PA2
    GPIOA->AFRL &= ~(0xFu << (IR_INPUT_PIN * 4));
    GPIOA->AFRL |=  (0xEu << (IR_INPUT_PIN * 4));   // AF14

    // ---- Servo PWM Pin ----
    // PA0 -> TIM2_CH1 (AF1)
    GPIOA->MODER &= ~(0b11u << (SERVO_SIGNAL_PIN * 2));
    GPIOA->MODER |=  (0b10u << (SERVO_SIGNAL_PIN * 2)); // AF mode

    GPIOA->OSPEEDR |= (0b11u << (SERVO_SIGNAL_PIN * 2)); // High speed

    GPIOA->AFRL &= ~(0xFu << (SERVO_SIGNAL_PIN * 4));
    GPIOA->AFRL |=  (0x1u << (SERVO_SIGNAL_PIN * 4));    // AF1 = TIM2_CH1
}

/* ===================== TIM15 Input Capture Setup from PI2 ===================== */
// We used PS7 and PS8 code files.
static void tim15_initialize(void)
{
    RCC_APB2ENR |= (1u << 16); // Enable TIM15 clock

    TIM15->PSC = TIM15_PSC_VAL;  // 4 MHz / (3999 + 1) = 1 kHz -> 1 ms per tick
    TIM15->ARR = TIM15_ARR_VAL;  // ~65 s max measurement window

    // CHANNEL 1 as Input Capture on TI1
    TIM15->CCMR1 &= ~(3u << 0); // Clear CC1S bits
    TIM15->CCMR1 |=  (1u << 0); // CC1S = 01: input, mapped on TI1

    // Both edges capture
    TIM15->CCER &= ~(0xFu);     // Clear CC1P, CC1E etc.
    TIM15->CCER |=  0b1011u;    // CC1P=10, CC1E=11 -> both edges

    // Enable interrupt for CH1
    TIM15->DIER |= (1u << 1);   // CC1IE

    TIM15->SR = 0;              // Clear all flags

    // Enable TIM15 interrupt in NVIC
    ISER2 |= (1u << TIM15_IRQ_BIT);

    // Start counter
    TIM15->CR1 |= 1u;           // CEN = 1
}

/* ===================== TIM2 Servo PWM + OC Setup from PI2 ===================== */
// We used PS7 and PS8 code files.
static void tim2_initialize(void)
{
    RCC_APB1ENR1 |= (1u << 0);   // Enable TIM2 clock

    TIM2->CR1 &= ~(1u << 0);     // Stop timer while configuring -> CEN = 0

    TIM2->PSC = TIM2_PSC_VAL;    // 1 MHz timer clock (assuming 4 MHz base)
    TIM2->ARR = TIM2_ARR_VAL;    // 20 ms period

    // Initial global servo state (idle at 0 degrees)
    g_servo_angle            = SERVO_MIN_ANGLE;
    g_servo_dir              = 1;
    g_servo_enabled          = 0;
    g_servo_reached_max      = 0;
    g_servo_delay_ticks      = 0;
    g_servo_sweeps_remaining = 0;

    // ------ CHANNEL 1: PWM output to servo ------
    // 1. CH1 as output compare
    TIM2->CCMR1 &= ~(3u << 0);   // CC1S = 00 -> output

    // 2. PWM mode 1
    TIM2->CCMR1 &= ~(0b111u << 4);
    TIM2->CCMR1 |=  (0b110u << 4); // OC1M = 110: PWM mode 1

    // 3. Enable preload for CCR1
    TIM2->CCMR1 |= (1u << 3);   // OC1PE

    // 4. Enable ARR preload
    TIM2->CR1 |= (1u << 7);     // ARPE

    // 5. Enable CH1 output
    TIM2->CCER |= (1u << 0);    // CC1E

    // 6. Initial pulse width for CH1 (0 deg)
    TIM2->CCR1 = map_angle_to_pulse(g_servo_angle);

    // ------ CHANNEL 2: OC just for timing / interrupts ------
    // 1. CH2 as output compare
    TIM2->CCMR1 &= ~(3u << 8);  // CC2S = 00 -> output compare

    // 2. Frozen mode (we only want CC2IF event)
    TIM2->CCMR1 &= ~(0b111u << 12); // OC2M = 000

    // 3. CCR2: CC2IF set each time CNT reaches this value
    TIM2->CCR2 = 0;             // Interrupt every period (CNT wraps at ARR)

    // 4. Enable CH2 output (even if not mapped to pin)
    TIM2->CCER |= (1u << 4);    // CC2E

    // 5. Enable interrupt for CH2
    TIM2->DIER |= (1u << 2);    // CC2IE

    TIM2->SR = 0;               // Clear flags

    // Generate update event to load PSC/ARR/CCRx
    TIM2->EGR |= (1u << 0);     // UG

    // Enable TIM2 interrupt in NVIC
    ISER1 |= (1u << TIM2_IRQ_BIT);

    // Start timer
    TIM2->CR1 |= (1u << 0);     // CEN = 1
}

/* ===================== EXTI Interrupts Setup from PI3 ===================== */
// We used PS7 code files.

void exti_sensors_initialize(void)
{
    // ---- Route EXTI5 to PA5 ----
    // EXTI_EXTICR2: lines 4..7 (m = 4)
    // EXTIm+1[7:0] (bits 15:8) controls line 5
    EXTI->EXTICR2 &= ~(0xFFu << 8);   // 0x00 → PA5

    // ---- Route EXTI11 to PA11 ----
    // EXTI_EXTICR3: lines 8..11 (m = 8)
    // EXTIm+3[7:0] (bits 31:24) controls line 11
    EXTI->EXTICR3 &= ~(0xFFu << 24);  // 0x00 → PA11

    // ---- Configure edge triggers ----

    // Motion sensor (PIR) on PA5, active HIGH → rising edge
    EXTI->RTSR1 |=  (1u << LED_INPUT_PIN);  // rising
    EXTI->FTSR1 |=  (1u << LED_INPUT_PIN);  // falling

    // Hand sensor (KY-032) on PA11, active LOW (we used ==0 before) → falling edge
    EXTI->FTSR1 |=  (1u << COVER_INPUT_PIN);
    EXTI->RTSR1 &= ~(1u << COVER_INPUT_PIN);

    // ---- Unmask lines in IMR1 ----
    EXTI->IMR1 |= (1u << LED_INPUT_PIN);
    EXTI->IMR1 |= (1u << COVER_INPUT_PIN);

    // ---- Enable NVIC interrupts ----
    ISER0 |= 1u << 16;
    ISER0 |= 1u << 22;
}

/* =====================  ADC Setup from PI3 ===================== */
// We used PS9 and PS10 code files.

void ADC_initialize(void)
{
	RCC_AHB2ENR |= 1 << 0; // GPIOA Clock
	GPIOA->MODER |= 0b11 << 2; // PA1 Analog Mode

	RCC_AHB2ENR |= 1 << 13; // ADC Clock Enable
	ADC1->CR &= ~(1 << 29); // close deep-power down
	ADC1->CR |= (1 << 28); // open voltage regulator

	RCC_CCIPR1 |= 3 << 28; // ADC Clock = SYSCLK
	ADC->CCR |= 3 << 16; // HCLK/4


	ADC1->SMPR1 &= ~(0b111 << 18); // Sampling time

	ADC1->SQR1 &=  ~(0b1111 << 0);
	ADC1->SQR1 |= 6 << 6; // Channel 6

	// initializing TIM7 for ADC calibration done interrupt
	RCC_APB1ENR1 |= (1 << 5);
	TIM7->PSC = 3999;
	TIM7->ARR = 50;
	TIM7->SR = 0;
	TIM7->CR1 &= ~(1<<1);
	TIM7->DIER |= 1;
	ISER1 |= 1 << 18;
	TIM7->CR1 &= ~1;

	adc_ready = 1;

	ADC1->CR |= (1 << 31); // write 1 to calibrate ADC
	TIM7->CR1 |= 1; // activate TIM7
	__asm volatile ("wfi"); // wait for TIM7 interrupt for calibration
	__asm volatile ("wfi"); // wait for ADC ready interrupt

}

/* =====================  LPUART Setup from PI3 ===================== */
// We used PS9 and PS10 code files.

void LPUART1_initialize(void)
{
	// LPUART Clock and GPIO Settings
	RCC_APB1ENR1 |= 1 << 28; // Power Clock
	PWR_CR1 |= 1 << 14;
	PWR_CR2 |= 1 << 9; // VDDIO2

	RCC_AHB2ENR |= 1 << 6; // GPIOG Clock
	GPIOG->MODER &= ~(0b0101 << (7 * 2));
	GPIOG->MODER |= 0b1010 << (7 * 2); // G7 pin
	GPIOG->AFRL &= ~(0b0111 << (7 * 4));
	GPIOG->AFRL |= 0b1000 << (7 * 4); // AF8
	GPIOG->AFRH &= ~0b0111;
	GPIOG->AFRH |= 0b1000; // AF8

	RCC_APB1ENR2 |= 1; // LPUART Clock Enable
	LPUART1 -> BRR = 8888; // 115200 Baud (4MHz sysclk)
	LPUART1->CR1 |= 1 << 29; // FIFO Enable
	LPUART1->CR1 |= 0b11 << 2; // TE, RE Enable
	LPUART1->CR1 |= 1 << 5; // RXNE Interrupt Enable
	ISER2 |= 1 << 2; // NVIC Enable for LPUART
	LPUART1->CR1 |= 1; // UART Enable
}



/* ===================== Helper functions from PI1 ===================== */
static inline void start_or_retrigger_led(void){

    // Ensure TIM6 period matches LED ON time before starting
    TIM6->CNT = 0;
    TIM6->SR  = 0;
    GPIOA->ODR |= (1 << LED_OUTPUT_PIN);
}

static inline void start_cover(void){
    // Ensure TIM7 period matches motor runtime before starting
    TIM7->ARR = (1000*MOTOR_RUNNING_SEC-1);
    TIM7->CNT = 0;
    TIM7->SR  = 0;
    GPIOA->ODR |= (1 << COVER_OUTPUT_PIN);
}

static inline void check_led_timeout_and_turn_off(void){
	if ((GPIOA->IDR & (1u<< LED_INPUT_PIN))!=0) {
		start_or_retrigger_led();
		g_light_time=1;
		g_light_sensor=1;
	} else {
	    TIM6->SR = 0;
	    GPIOA->ODR &= ~(1 << LED_OUTPUT_PIN);
	    g_light_time=0;
	}
  // Check if LED timer has expired

}

static inline void check_cover_timeout_and_turn_off(void){
    // Check if cover motor timer has expired
    TIM7->SR = 0;
    // Motor period elapsed -> stop motor and start cooldown
    GPIOA->ODR &= ~(1 << COVER_OUTPUT_PIN);
    g_cover_cooldown = COVER_COOLDOWN_SEC;

    // Reconfigure TIM7 to 1s tick for cooldown countdown
    TIM7->ARR = (1000*g_cover_cooldown-1);
    TIM7->CNT = 0;
    g_cover_state=0;
}

static inline void check_cover_cooldown(void){
    // Check if cooldown timer has expired
        TIM7->SR = 0;
        g_cover_cooldown = 0;

}


// --- Helper FUnctions From PI3 ---

static inline uint16_t tx_next_index(uint16_t idx) {
    return (uint16_t)((idx + 1U) % LPUART_TX_BUF_SIZE);
}


// Put one char into TX buffer and trigger TXE interrupt
void LPUART1_SendChar(char c) {
    uint16_t next = tx_next_index(lpuart_tx_head);

    // If buffer is full, you can either block or drop.
    // Simpler: block until there is space (short in practice).
    while (next == lpuart_tx_tail) {
        // buffer full, wait for ISR to send some bytes
    }

    lpuart_tx_buf[lpuart_tx_head] = c;
    lpuart_tx_head = next;

    // Enable TXE interrupt so ISR starts sending
    LPUART1->CR1 |= (1 << 7);  // TXEIE = 1
}


// Sends a string by sending each char
void LPUART1_SendString(char *str) {
    while (*str) {
        LPUART1_SendChar(*str++);
    }
}

// This function now prints a status string based on the water level value.
void LPUART1_SendInteger(int num) {
    if (num <= 1600) {
        LPUART1_SendString("LOW"); // 0 - 1600
    } else if (num <= 1970) {
        LPUART1_SendString("MEDIUM"); // 1601 - 1970
    } else {
        LPUART1_SendString("HIGH"); // 1971 and above
    }
}

/* ===================== ADC Interrupt Handler  from PI3 ===================== */
// We used PS9 and PS10 code files.
int debug =0;
void ADC1_2_IRQHandler(void)
{
	if((ADC1->ISR & 1<<2) != 0)
	{
		water_level = ADC1->DR;
		ADC1->CR |= 1<<2; // start new conversion
	}

    // ADC ready (ADRDY) – happens once after enabling ADC
    if ((ADC1->ISR & 1) != 0) {
    	debug =2;
        ADC1->ISR |= 1;           // clear ADRDY
        ADC1->IER &= ~1;          // disable ADRDY interrupt
        ADC1->IER |= 1 << 2;      // enable EOC interrupt
        ADC1->CR  |= 1 << 2;      // start first conversion (ADSTART)
    }
}

/* ===================== LPUART Interrupt Handler  from PI3 ===================== */
// We used PS9 and PS10 code files.

void LPUART1_IRQHandler(void) {

    uint32_t isr = LPUART1->ISR;
    uint32_t cr1 = LPUART1->CR1;

    // ---------- RX: command interface ----------
    if (isr & (1 << 5)) {  // RXNE flag
        char received = (char)LPUART1->RDR;  // reading clears RXNE

        if (received == 'W' || received == 'w') {
            uart_print_water_flag = 1;
        } else if (received == 'I' || received == 'i') {
            uart_print_presence_flag = 1;
        }
    }

    // ---------- TX: send next byte from buffer ----------
    // TXE flag & TXE interrupt enabled?
    if ((isr & (1 << 7)) && (cr1 & (1 << 7))) {  // TXE && TXEIE
        if (lpuart_tx_head != lpuart_tx_tail) {
            // There is data to send
            LPUART1->TDR = (uint8_t)lpuart_tx_buf[lpuart_tx_tail];
            lpuart_tx_tail = tx_next_index(lpuart_tx_tail);
        } else {
            // Buffer empty -> disable TXE interrupt
            LPUART1->CR1 &= ~(1 << 7);  // TXEIE = 0
        }
    }
}



/* ===================== EXTI Interrupt Handler  from PI3 ===================== */
// We used PS7 code files.

void EXTI5_IRQHandler(void)
{
    // Rising edge pending on line 5?
    if (EXTI->RPR1 & (1u << LED_INPUT_PIN)) {
        // Clear rising flag by writing 1
        EXTI->RPR1 |= (1u << LED_INPUT_PIN);

        // Old main-loop logic
        start_or_retrigger_led();
        g_light_time   = 1;
        g_light_sensor = 1;
    }

    if (EXTI->FPR1 & (1u << LED_INPUT_PIN)) {
		EXTI->FPR1 |= (1u << LED_INPUT_PIN);  // clear falling flag


        start_or_retrigger_led();
        g_light_time   = 1;
        g_light_sensor = 1;
    }
}

/* ===================== EXTI Interrupt Handler from PI3 ===================== */
// We used PS7 code files.
void EXTI11_IRQHandler(void)
{
    // Falling edge pending on line 11?
    if (EXTI->FPR1 & (1u << COVER_INPUT_PIN)) {
        // Clear falling flag by writing 1
        EXTI->FPR1 |= (1u << COVER_INPUT_PIN);

        if (g_cover_state == 0 && g_cover_cooldown == 0) {
            start_cover();
            g_cover_state = 1;
        }
    }
}

/* ===================== TIM16 Interrupt Handler  from PI3 ===================== */
// We used PS7 code files.
void TIM6_IRQHandler(void)
{
    // This fires when TIM6 update (UIF) happens
    check_led_timeout_and_turn_off();
}

/* ===================== TIM17 Interrupt Handler  from PI3 ===================== */
void TIM7_IRQHandler(void)
{
    TIM7->SR = 0;


	if (adc_ready !=0){


		//while((ADC1->CR & (1 << 31)) != 0) {}

		if ((ADC1->CR & (1 << 31)) == 0) { // calibration has been finished
			adc_ready =0;
			TIM7->CR1 &= ~1;
			ISER1 |= 1 << 5; // enable ADC1_2 global interrupt
			ADC1->IER |= 1; // enable interrupt for ADC ready
			ADC1->CR |= 1; // enable ADC
		}
	}

    // Two modes: motor running OR cooldown
    if (g_cover_state == 1) {
        // Motor is running, first timeout -> stop motor & start cooldown
        check_cover_timeout_and_turn_off();
    } else if (g_cover_state == 0 && g_cover_cooldown > 0) {
        // In cooldown phase, second timeout -> end cooldown
        check_cover_cooldown();
    }






}

/* ===================== Helper functions from PI2 ===================== */
// LED control helpers
// We used PS1.
static void turn_on_RED(void)  { GPIOA->ODR |=  (1u << RED_LED_PIN); }
static void turn_off_RED(void) { GPIOA->ODR &= ~(1u << RED_LED_PIN); }
static void turn_on_BLUE(void) { GPIOB->ODR |=  (1u << BLUE_LED_PIN); }
static void turn_off_BLUE(void){ GPIOB->ODR &= ~(1u << BLUE_LED_PIN); }



/* ===================== TIM15 Interrupt Handler (Input Capture) from PI2 ===================== */
// We used PS7 and PS8 code files.

void TIM15_IRQHandler(void)
{
    // Input Capture Channel 1 interrupt
    if (TIM15->SR & (1u << 1)) {         // CC1IF
        TIM15->SR &= ~(1u << 1);         // Clear CC1IF

        uint32_t captured_time_ms = TIM15->CCR1; // captured time (ms)
        uint8_t level_now = (GPIOA->IDR & (1u << IR_INPUT_PIN)) ? 1u : 0u;

        // level_now == 0 -> pin LOW  -> someone sitting
        // level_now == 1 -> pin HIGH -> someone standing up

        if (level_now == 0u) {          // Sitting now (FALLING edge)
            if (!is_sitting) {          // Previously empty -> new sit
                is_sitting        = 1u;
                sit_start_time_ms = captured_time_ms;

                turn_off_RED();
                turn_off_BLUE();

                // Stop any ongoing or pending flush when someone sits
                g_servo_enabled          = 0u;
                g_servo_reached_max      = 0u;
                g_servo_delay_ticks      = 0u;
                g_servo_sweeps_remaining = 0u;
            }
            // else -> bouncing while already sitting: ignore
        } else {                        // Standing up now (RISING edge)
            if (is_sitting) {           // Previously sitting -> real stand up
                is_sitting = 0u;
                uint32_t sit_end_time_ms = captured_time_ms;
                uint32_t elapsed_ms;

                if (sit_end_time_ms >= sit_start_time_ms) {
                    elapsed_ms = sit_end_time_ms - sit_start_time_ms;
                } else {
                    // Timer overflowed
                    elapsed_ms = (TIM15->ARR + 1u) - sit_start_time_ms + sit_end_time_ms;
                }

                turn_on_RED();

                if (elapsed_ms >= OCCUPIED_THRESHOLD_MS) {
                    // Valid sit -> BLUE LED ON immediately
                    // but servo should start AFTER AUTO_FLUSH_DELAY_SEC
                    turn_on_BLUE();

                    // Prepare servo at 0° but DO NOT start yet
                    g_servo_angle            = SERVO_MIN_ANGLE;
                    g_servo_dir              = 1;
                    g_servo_reached_max      = 0;
                    g_servo_enabled          = 0;
                    g_servo_delay_ticks      = AUTO_FLUSH_DELAY_TICKS;
                    g_servo_sweeps_remaining = 2u;    // run two sweeps with a pause

                    TIM2->CCR1 = map_angle_to_pulse(g_servo_angle);
                } else {
                    // Temporary contact, not a valid sit
                    turn_off_BLUE();
                    g_servo_enabled          = 0u;
                    g_servo_reached_max      = 0u;
                    g_servo_delay_ticks      = 0u;
                    g_servo_sweeps_remaining = 0u;
                }
            }
            // else -> bouncing while already empty: ignore
        }
    }

    // Overcapture Event
    if (TIM15->SR & (1u << 9)) {        // CC1OF
        TIM15->SR &= ~(1u << 9);        // Clear CC1OF
        // Reset state for safety
        is_sitting = 0u;
        turn_off_BLUE();
        turn_on_RED();
        g_servo_enabled          = 0u;
        g_servo_reached_max      = 0u;
        g_servo_delay_ticks      = 0u;
        g_servo_sweeps_remaining = 0u;
    }
}

/* ===================== TIM2 Interrupt Handler (Servo motion + delay) from PI2 ===================== */
// We used PS7 and PS8 code files.
void TIM2_IRQHandler(void)
{
    // Output Compare Channel 2 interrupt (our 20 ms "tick")
    if (TIM2->SR & (1u << 2)) {      // CC2IF
        TIM2->SR &= ~(1u << 2);      // Clear CC2IF

        // 1) Handle delayed start of servo sweep (AUTO_FLUSH + inter-sweep pause)
        if (!g_servo_enabled && g_servo_delay_ticks > 0u) {
            g_servo_delay_ticks--;
            if ((g_servo_delay_ticks == 0u) && (g_servo_sweeps_remaining > 0u)) {
                // Start a sweep (0° -> 90° -> 0°) after the pending delay
                g_servo_angle        = SERVO_MIN_ANGLE;
                g_servo_dir          = 1;
                g_servo_reached_max  = 0;
                g_servo_enabled      = 1;
                g_servo_sweeps_remaining--;  // one sweep is now in progress
                TIM2->CCR1 = map_angle_to_pulse(g_servo_angle);
            }
        }

        // 2) Handle active sweep motion
        if (g_servo_enabled) {
            // Move by one step
            g_servo_angle += (g_servo_dir * SERVO_STEP_DEG);

            if (g_servo_dir > 0 && g_servo_angle >= SERVO_MAX_ANGLE) {
                // Hit max -> clamp and start moving back
                g_servo_angle       = SERVO_MAX_ANGLE;
                g_servo_dir         = -1;
                g_servo_reached_max = 1;
            } else if (g_servo_dir < 0 && g_servo_angle <= SERVO_MIN_ANGLE) {
                // Returned to min
                g_servo_angle = SERVO_MIN_ANGLE;

                if (g_servo_reached_max) {
                    // Completed one full sweep (0° -> 90° -> 0°)
                    if (g_servo_sweeps_remaining > 0u) {
                        // NEW: stop now and wait INTER_SWEEP_DELAY before next sweep
                        g_servo_enabled     = 0;
                        g_servo_dir         = 1;
                        g_servo_reached_max = 0;
                        g_servo_delay_ticks = INTER_SWEEP_DELAY_TICKS;
                    } else {
                        // No more sweeps requested -> stop
                        g_servo_enabled = 0;
                    }
                } else {
                    // Safety path, normally not used
                    g_servo_dir = 1;
                }
            }

            // Update PWM duty for new angle (also runs on the last step)
            TIM2->CCR1 = map_angle_to_pulse(g_servo_angle);
        }
    }

    // Update Interrupt Flag
    if (TIM2->SR & (1u << 0)) { // UIF
        TIM2->SR &= ~(1u << 0);
    }
}

//update function
void update(void){
    // Check if PC requested water-level report
    if (uart_print_water_flag) {
        uart_print_water_flag = 0;      // Clear request flag

        LPUART1_SendString("\r\nWater Level: ");
        LPUART1_SendInteger(water_level); // Print LOW / MEDIUM / HIGH
        LPUART1_SendString("\r\n");

    // Check if PC requested occupancy status
    } else if (uart_print_presence_flag) {
        uart_print_presence_flag = 0;   // Clear request flag

        if (g_light_time) {
            LPUART1_SendString("\r\n Restroom Occupied.\r\n");
            LPUART1_SendString("\r\n");

        } else {
            LPUART1_SendString("\r\n Restroom Not Occupied.\r\n");
            LPUART1_SendString("\r\n");

        }
    }
}


/* ===================== Main ===================== */
int main(void) {
    printf("Conceptual Design - SmartHygiene Toilet System\n");

    // ADC + UART (PI3 initializations)
    ADC_initialize();
    LPUART1_initialize();

    // PI1 Initializations
    motion_sensor_initialize();
    hand_sensor_initialize();
    light_timer_initialize();
    cover_motor_timer_initialize();
    exti_sensors_initialize();

    // PI2 Initializations
    gpio_initialize();   // LEDs, IR (TIM15_CH1), Servo (TIM2_CH1)
    tim15_initialize();  // Input Capture for IR
    tim2_initialize();   // Servo PWM + OC

    // Global interrupt enable from PI2
    __asm volatile(
        "mov r0, #0      \n\t"
        "msr primask, r0 \n\t"
    );

    while(1){


    	update(); // Handle updated Uart fumctions

        // Wait for interrupt
        __asm volatile("wfi");

    }
}
