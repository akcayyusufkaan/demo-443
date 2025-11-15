#include <stdint.h>

// System Parameters
#define SERVO_SIGNAL_PIN  0     // PA0

#define TIM2_PSC_VAL      3
#define TIM2_ARR_VAL      19999 // For a 20 ms period we need 20000 counts

// Servo pulse widths in timer ticks (1 tick = 1 µs at 1 MHz)
#define SERVO_MIN_PULSE   1000  // 1000 µs = 1 ms  (0 degrees)
#define SERVO_MAX_PULSE   2000  // 2000 µs = 2 ms  (180 degrees)

#define SERVO_MIN_ANGLE   0     // Servo angle limits
#define SERVO_MAX_ANGLE   180

#define SERVO_STEP_DEG    1     // // Angular step per OC event -> 1 degree / 20 ms -> ~50°/s

// GPIO Structure
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

// Base Addresses
#define RCC_AHB2ENR   (*(volatile uint32_t *) 0x4002104C) // for GPIOA/B/C... clocks enable
#define RCC_APB1ENR1  (*(volatile uint32_t *) 0x40021058) // for TIM2/3/4/5... clocks enable

#define GPIOA ((GPIO *) 0x42020000)
#define GPIOB ((GPIO *) 0x42020400)

#define TIM2  ((TIM_General_Purpose_32bit_Type *) 0x40000000)

#define ISER1    (*(volatile uint32_t *)0xE000E104) // NVIC ISER1 -> IRQ 32..63
#define TIM2_IRQN                  45        // TIM2 global interrupt = IRQ 45
#define TIM2_IRQ_BIT  (TIM2_IRQN - 32)       // 13

// Global Servo State
static volatile int g_servo_angle    = 0;   // Current angle (0..180)
static volatile int g_servo_dir      = 1;   // 1: increasing, -1: decreasing

uint32_t map_angle_to_pulse(int angle)
{
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    // Linear mapping: (angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180 + SERVO_MIN_PULSE
    return (uint32_t)(((int32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180
                      + SERVO_MIN_PULSE);
}

// Servo PWM and Output Compare Setup
void init_GPIO(void)
{
    RCC_AHB2ENR  |= (1 << 0);   // Enable GPIOA clock

    // 1. Set PA0 to Alternate Function mode
    GPIOA->MODER &= ~(0b11 << (SERVO_SIGNAL_PIN * 2));  // Clear PA0 mode
    GPIOA->MODER |=  (0b10 << (SERVO_SIGNAL_PIN * 2));  // Set PA0 as AF
    // 2. Set high speed for PA0
    GPIOA->OSPEEDR |= (0b11 << (SERVO_SIGNAL_PIN * 2)); // High speed output
    // 3. Select the correct AF (AF1 for TIM2)
    GPIOA->AFRL   &= ~(0b1111 << (SERVO_SIGNAL_PIN * 4)); // Clear AF for PA0
    GPIOA->AFRL   |=  (0b0001 << (SERVO_SIGNAL_PIN * 4)); // AF1 = TIM2_CH1
}

void init_TIM2(void)
{
    RCC_APB1ENR1 |= (1 << 0);   // Enable TIM2 clock

    TIM2->CR1 &= ~(1 << 0); // // Stop timer while configuring -> CEN = 0

    TIM2->PSC = TIM2_PSC_VAL; // Set PSC -> 1 MHz timer clock

    TIM2->ARR = TIM2_ARR_VAL; // Set ARR -> 20 ms period

    // Set global servo state
    g_servo_angle = 0; // Start at 0 degrees
    g_servo_dir   = 1; // Initial direction: increasing

    /* ------ CHANNEL 1 CONFIGURATION ------ */
    // 1. Set CH1 as output compare
    TIM2->CCMR1 &= ~(3u << 0); // CC1S = 00: Channel 1 is output compare

    // 2. Set PWM Mode 1 for CH1
    TIM2->CCMR1 &= ~(0b111 << 4); // Clear OC1M bits
    TIM2->CCMR1 |=  (0b110 << 4); // Set OC1M = 110: PWM Mode 1

    // 3. Enable preload for CCR1
    TIM2->CCMR1 |= (1 << 3); // OC1PE = 1

    // 4. Enable ARR preload
    TIM2->CR1 |= (1 << 7); // ARPE = 1

    // 5. Enable CH1 output
    TIM2->CCER |= (1 << 0); // CC1E = 1

    // 6. Set initial pulse width for CH1
    TIM2->CCR1 = map_angle_to_pulse(g_servo_angle); // Set CCR1 (1000..2000)

    /* ------ CHANNEL 2 CONFIGURATION ------ */
    // 1. Set CH2 as output compare (used only for timing / interrupts)
    TIM2->CCMR1 &= ~(3u << 8); // CC2S = 00: Channel 2 output compare

    // 2. Set CH2 PWM mode to Frozen (we don't drive a pin, we only want the event)
    TIM2->CCMR1 &= ~(0b111 << 12); // OC2M = 000: Frozen

    // 3. CCR2: CC2IF is set each time CNT reaches the value
    TIM2->CCR2 = 0; // Set CCR2 to 0 means that interrupt occurs at every overflow

    // 4. Enable CH2 output (even if not connected to a pin)
    TIM2->CCER |= (1 << 4); // CC2E = 1

    // 5. Enable Interrupts for CH2
    TIM2->DIER |= (1 << 2); // Enable CC2IE: Output Compare Channel 2 interrupt

    TIM2->SR = 0; // Clear all SR flags

    // Generate an update event (UG) to load PSC/ARR/CCR shadow registers
    TIM2->EGR |= (1 << 0);

    // Enable TIM2 interrupt in NVIC
    ISER1 |= (1u << TIM2_IRQ_BIT);   // Enable IRQ 45

    // Start the counter
    TIM2->CR1 |= (1 << 0); // CEN = 1
}

// TIM2 Interrupt Handler
void TIM2_IRQHandler(void)
{
    // Check for Output Compare Channel 2 interrupt
    if (TIM2->SR & (1 << 2)) // SR -> CC2IF
    {
        TIM2->SR &= ~(1 << 2); // Clear the flag SR -> CC2IF

        // Change servo angle by one step
        g_servo_angle += (g_servo_dir * SERVO_STEP_DEG);

        if (g_servo_angle >= SERVO_MAX_ANGLE)
        {
            g_servo_angle = SERVO_MAX_ANGLE;
            g_servo_dir   = -1;  // Reverse direction at max
        }
        else if (g_servo_angle <= SERVO_MIN_ANGLE)
        {
            g_servo_angle = SERVO_MIN_ANGLE;
            g_servo_dir   = 1;   // Reverse direction at min
        }

        // Write new pulse width for updated angle into CCR1
        TIM2->CCR1 = map_angle_to_pulse(g_servo_angle);
    }

    // Check for Update Interrupt Flag
    if (TIM2->SR & (1 << 0))   // SR -> UIF
    {
        TIM2->SR &= ~(1 << 0); // Clear the flag SR -> UIF
    }
}

// MAIN
int main(void)
{
    init_GPIO(); // PA0 pin
    init_TIM2(); // OC + PWM

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
