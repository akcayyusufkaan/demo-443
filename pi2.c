/*
 * Input Capture (TIM15 CH1 - PA2, IR sensor)
 * ------------------------------------------------
 * - PA2 is configured as Alternate Function AF14 -> TIM15_CH1.
 * - TIM15 Channel 1 is configured as Input Capture on TI1:
 *      * CC1S = 01 (input, mapped on TI1)
 *      * CC1P/CC1NP configured for both-edge capture (falling + rising)
 * - CC1 interrupt is enabled (CC1IE in DIER) and handled in TIM15_IRQHandler.
 * - On each edge, TIM15->CCR1 is read as a hardware timestamp in milliseconds:
 *      * Falling edge (IR LOW)  = "sit" event   -> sit_start_time_ms = CCR1
 *      * Rising  edge (IR HIGH) = "stand" event -> sit_end_time_ms   = CCR1
 * - The sitting duration is computed from two CCR1 timestamps:
 *      * elapsed_ms = sit_end_time_ms - sit_start_time_ms
 *        (with explicit handling of timer overflow using TIM15->ARR).
 * - This elapsed_ms value is compared against OCCUPIED_THRESHOLD_MS to
 *   decide whether the seat was truly occupied or just a short contact.
 *
 * Output Compare And PWM (TIM2 CH1 + CH2, Servo motor)
 * ------------------------------------------------
 * 1) TIM2_CH1 -> Servo PWM signal on PA0
 *    - PA0 is configured as Alternate Function AF1 -> TIM2_CH1.
 *    - TIM2 Channel 1 is configured as an Output Compare channel:
 *         * CC1S = 00 (output)
 *         * OC1M = 110 (PWM mode 1) with preload enabled (OC1PE) and
 *           ARR preload enabled (ARPE).
 *    - TIM2->CCR1 holds the pulse width in timer ticks; it is updated
 *      according to the current servo angle via map_angle_to_pulse().
 *    - This generates a real hardware PWM signal (1–2 ms pulse width
 *      within a 20 ms period), which directly drives the servo motor.
 *
 * 2) TIM2_CH2 -> Output Compare interrupt "time base" (no external pin)
 *    - TIM2 Channel 2 is configured purely as an Output Compare channel:
 *         * CC2S = 00 (output compare)
 *         * OC2M = 000 (frozen mode – we only use the CC2IF event).
 *    - TIM2->CCR2 is set so that CC2IF occurs once per timer period
 *      (here CCR2 = 0 while CNT runs from 0 to ARR, every 20 ms).
 *    - CC2 interrupt (CC2IE in DIER) is enabled and handled in
 *      TIM2_IRQHandler.
 *    - On every CC2IF (every 20 ms):
 *         * If g_servo_delay_ticks > 0 and the servo sweep is not
 *           started yet (g_servo_enabled == 0), the delay counter is
 *           decremented. When it reaches zero, a single sweep
 *           (0° -> 180° -> 0°) is armed and g_servo_enabled is set.
 *         * If the servo sweep is active (g_servo_enabled == 1), the
 *           servo angle g_servo_angle is stepped by SERVO_STEP_DEG on
 *           each tick, the direction is reversed at the limits
 *           (0° / 180°), and TIM2->CCR1 is updated with the new pulse
 *           width via map_angle_to_pulse(). Once a full up-and-down
 *           sweep is completed, g_servo_enabled is cleared.
 *    - Thus TIM2_CH2 Output Compare provides a precise 20 ms time base
 *      to implement both the auto-flush delay and the servo sweep logic.
 *
 * Summary
 * ------------------------------------------------
 * - REAL Input Capture:
 *      * TIM15_CH1 on PA2 measures the time between "sit" and "stand"
 *        events using hardware capture timestamps from CCR1
 *        (both-edge capture + overcapture handling).
 *
 * - REAL Output Compare:
 *      * TIM2_CH1 generates the servo PWM signal entirely through
 *        hardware compare (CCR1 vs CNT in PWM mode 1).
 *      * TIM2_CH2 generates periodic compare events (CC2IF) that serve
 *        as a 20 ms tick driving the delay counter and servo motion
 *        state machine.
 */

#include <stdint.h>

// ===================== System Parameters =====================
#define OCCUPIED_THRESHOLD_SEC      2  // seconds seated to confirm usage
#define AUTO_FLUSH_DELAY_SEC        4  // delay AFTER valid sit & stand-up before servo starts

// IR / LED pins
#define IR_INPUT_PIN   2  // PA2
#define RED_LED_PIN    9  // PA9
#define BLUE_LED_PIN   7  // PB7

// Servo pin
#define SERVO_SIGNAL_PIN  0     // PA0

// TIM2 parameters for servo PWM
#define TIM2_PSC_VAL      3
#define TIM2_ARR_VAL      19999 // For a 20 ms period we need 20000 counts
#define TIM2_PERIOD_MS    20u   // 20 ms period with these PSC/ARR settings

// TIM15 parameters for input capture
#define TIM15_PSC_VAL     3999   // 1 MHz timer clock (1 µs resolution)
#define TIM15_ARR_VAL     0xFFFF // 16-bit timer

// Servo pulse widths in timer ticks (1 tick = 1 µs at 1 MHz)
#define SERVO_MIN_PULSE   1000  // 1000 µs = 1 ms  (0 degrees)
#define SERVO_MAX_PULSE   2000  // 2000 µs = 2 ms  (180 degrees)

#define SERVO_MIN_ANGLE   0     // Servo angle limits
#define SERVO_MAX_ANGLE   180

#define SERVO_STEP_DEG    1     // Angular step per OC event -> 1 degree / 20 ms -> ~50°/s

// How many TIM2 periods to wait before starting servo sweep
#define AUTO_FLUSH_DELAY_TICKS  ((AUTO_FLUSH_DELAY_SEC * 1000u) / TIM2_PERIOD_MS)

// ===================== Peripheral Structures =====================
typedef struct {
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

// TIM15 Structure (from reference)
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

// ===================== Base addresses =====================
#define RCC_AHB2ENR   (*(volatile uint32_t *) 0x4002104C) // for GPIOA/B/C... clocks enable
#define RCC_APB2ENR   (*(volatile uint32_t *) 0x40021060) // for TIM15... clocks enable
#define RCC_APB1ENR1  (*(volatile uint32_t *) 0x40021058) // for TIM2/3/4/5... clocks enable

#define GPIOA ((GPIO *) 0x42020000)
#define GPIOB ((GPIO *) 0x42020400)

#define TIM15 ((TIM15_General_Purpose_Type *) 0x40014000)
#define TIM2  ((TIM_General_Purpose_32bit_Type *) 0x40000000)

// NVIC ISER registers
#define ISER1         (*(volatile uint32_t *) 0xE000E104)  // NVIC ISER1 -> IRQ 32..63
#define ISER2         (*(volatile uint32_t *) 0xE000E108)  // NVIC ISER2 -> IRQ 64..95

#define TIM15_IRQN                   69        // TIM15 global interrupt = IRQ 69
#define TIM15_IRQ_BIT  (TIM15_IRQN - 64)       // 5

#define TIM2_IRQN                   45         // TIM2 global interrupt = IRQ 45
#define TIM2_IRQ_BIT   (TIM2_IRQN - 32)        // 13

// ===================== Global IC state =====================
static uint8_t  is_sitting         = 0; // 0: nobody sitting, 1: someone sitting
static uint32_t sit_start_time_ms  = 0;
static const uint32_t OCCUPIED_THRESHOLD_MS = OCCUPIED_THRESHOLD_SEC * 1000u;

// ===================== Global Servo State =====================
static volatile int      g_servo_angle        = 0;   // Current angle (0..180)
static volatile int      g_servo_dir          = 1;   // 1: increasing, -1: decreasing
static volatile uint8_t  g_servo_enabled      = 0;   // 1 while sweep is running
static volatile uint8_t  g_servo_reached_max  = 0;   // 1 after reaching 180° in current sweep
static volatile uint32_t g_servo_delay_ticks  = 0;   // countdown (TIM2 periods) before starting sweep

// ===================== LED Control Functions =====================
static void turn_on_RED(void)  { GPIOA->ODR |=  (1u << RED_LED_PIN); }
static void turn_off_RED(void) { GPIOA->ODR &= ~(1u << RED_LED_PIN); }
static void turn_on_BLUE(void) { GPIOB->ODR |=  (1u << BLUE_LED_PIN); }
static void turn_off_BLUE(void){ GPIOB->ODR &= ~(1u << BLUE_LED_PIN); }

// ===================== Helper: Map servo angle to pulse width =====================
static uint32_t map_angle_to_pulse(int angle)
{
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    // Linear mapping: (angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180 + SERVO_MIN_PULSE
    return (uint32_t)(((int32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180
                      + SERVO_MIN_PULSE);
}

// ===================== GPIO Setup (LED + IR + Servo) =====================
static void init_GPIO(void)
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

// ===================== TIM15 Input Capture Setup =====================
static void init_TIM15(void)
{
    RCC_APB2ENR |= (1u << 16); // Enable TIM15 clock

    TIM15->PSC = TIM15_PSC_VAL;     // 4 MHz / (3999 + 1) = 1 kHz -> 1 ms per tick
    TIM15->ARR = TIM15_ARR_VAL;    // ~65 s max measurement window

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

// ===================== TIM2 Servo PWM + OC Setup =====================
static void init_TIM2(void)
{
    RCC_APB1ENR1 |= (1u << 0);   // Enable TIM2 clock

    TIM2->CR1 &= ~(1u << 0);     // Stop timer while configuring -> CEN = 0

    TIM2->PSC = TIM2_PSC_VAL;    // 1 MHz timer clock (assuming 4 MHz base)
    TIM2->ARR = TIM2_ARR_VAL;    // 20 ms period

    // Initial global servo state (idle at 0 degrees)
    g_servo_angle        = SERVO_MIN_ANGLE;
    g_servo_dir          = 1;
    g_servo_enabled      = 0;
    g_servo_reached_max  = 0;
    g_servo_delay_ticks  = 0;

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

// ===================== TIM15 Interrupt Handler (Input Capture) =====================
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
                g_servo_enabled      = 0u;
                g_servo_reached_max  = 0u;
                g_servo_delay_ticks  = 0u;
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
                    g_servo_angle        = SERVO_MIN_ANGLE;
                    g_servo_dir          = 1;
                    g_servo_reached_max  = 0;
                    g_servo_enabled      = 0;
                    g_servo_delay_ticks  = AUTO_FLUSH_DELAY_TICKS;

                    TIM2->CCR1 = map_angle_to_pulse(g_servo_angle);
                } else {
                    // Temporary contact, not a valid sit
                    turn_off_BLUE();
                    g_servo_enabled      = 0u;
                    g_servo_reached_max  = 0u;
                    g_servo_delay_ticks  = 0u;
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
        g_servo_enabled      = 0u;
        g_servo_reached_max  = 0u;
        g_servo_delay_ticks  = 0u;
    }
}

// ===================== TIM2 Interrupt Handler (Servo motion + delay) =====================
void TIM2_IRQHandler(void)
{
    // Output Compare Channel 2 interrupt (our 20 ms "tick")
    if (TIM2->SR & (1u << 2)) {      // CC2IF
        TIM2->SR &= ~(1u << 2);      // Clear CC2IF

        // 1) Handle delayed start of servo sweep
        if (!g_servo_enabled && g_servo_delay_ticks > 0u) {
            g_servo_delay_ticks--;
            if (g_servo_delay_ticks == 0u) {
                // Start ONE sweep 0° -> 180° -> 0°
                g_servo_angle        = SERVO_MIN_ANGLE;
                g_servo_dir          = 1;
                g_servo_reached_max  = 0;
                g_servo_enabled      = 1;
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
                    // We already went up to max and came back -> ONE full sweep is done
                    g_servo_enabled = 0;  // stop updating angle
                } else {
                    // Safety path, normally not used
                    g_servo_dir = 1;
                }
            }

            // Update PWM duty for new angle
            TIM2->CCR1 = map_angle_to_pulse(g_servo_angle);
        }
    }

    // Update Interrupt Flag
    if (TIM2->SR & (1u << 0)) { // UIF
        TIM2->SR &= ~(1u << 0);
    }
}

// ===================== MAIN =====================
int main(void)
{
    init_GPIO();   // LEDs, IR (TIM15_CH1), Servo (TIM2_CH1)
    init_TIM15();  // Input Capture for IR
    init_TIM2();   // Servo PWM + OC

    // Global interrupt enable
    __asm volatile(
        "mov r0, #0      \n\t"
        "msr primask, r0 \n\t"
    );

    while (1) {
        __asm volatile("wfi");
    }

    return 0;
}
