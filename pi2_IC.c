#include <stdint.h>

/* ---------------- System Parameters ---------------- */
#define OCCUPIED_THRESHOLD_SEC      2  // seconds seated to confirm usage
#define SECOND_LED_DELAY_SEC        4  // delay after standing up (not used yet)

#define IR_INPUT_PIN   2  // PA2
#define RED_LED_PIN    9  // PA9
#define BLUE_LED_PIN   7  // PB7

/* ---------------- GPIO Structure ---------------- */
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

/* ---------------- TIM15 Structure (from reference) ---------------- */
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

// Base addresses
#define RCC_AHB2ENR   (*(volatile uint32_t *) 0x4002104C)
#define RCC_APB2ENR   (*(volatile uint32_t *) 0x40021060)

#define GPIOA ((GPIO *) 0x42020000)
#define GPIOB ((GPIO *) 0x42020400)

#define TIM15 ((TIM15_General_Purpose_Type *) 0x40014000)
#define ISER2         (*(volatile uint32_t *) 0xE000E108)  // NVIC ISER2

// Global IC state
static uint8_t  is_sitting         = 0; // 0: nobody sitting, 1: someone sitting
static uint32_t sit_start_time_ms  = 0;
static const uint32_t OCCUPIED_THRESHOLD_MS = OCCUPIED_THRESHOLD_SEC * 1000u;

// GPIO Setup
void GPIO_setup(void)
{
    RCC_AHB2ENR |= (0b11 << 0);  // Enable GPIOA, GPIOB clock

    GPIOA->MODER &= ~(0b11 << (RED_LED_PIN * 2)); // Clear PA9 mode
    GPIOA->MODER |=  (0b01 << (RED_LED_PIN * 2)); // Set PA9 as output

    GPIOB->MODER &= ~(0b11 << (BLUE_LED_PIN * 2)); // Clear PB7 mode
    GPIOB->MODER |=  (0b01 << (BLUE_LED_PIN * 2)); // Set PB7 as output

    // 1. Set PA2 to Alternate Function mode
    GPIOA->MODER &= ~(0b11 << (IR_INPUT_PIN * 2)); // Clear PA2 mode
    GPIOA->MODER |=  (0b10 << (IR_INPUT_PIN * 2)); // Set PA2 as AF
    // 2. Select the correct AF (AF14 for TIM15)
    GPIOA->AFRL &= ~(0xF << (IR_INPUT_PIN * 4)); // Clear AF for PA2
    GPIOA->AFRL |=  (0xE << (IR_INPUT_PIN * 4)); // AF14 = TIM15_CH1
}

// TIM15 Input Capture Setup
void TIM15_setup(void)
{
    RCC_APB2ENR |= (1u << 16); // Enable TIM15 clock

    TIM15->PSC = 3999; // 4 MHz / (3999 + 1) = 1 kHz -> 1 ms per tick.
    TIM15->ARR = 0xFFFF; // ~65 s max ölçüm aralığı

    // 1. Set CH1 as input, mapped on TI1
    TIM15->CCMR1 &= ~(3u << 0); // Clear CC1S bits
    TIM15->CCMR1 |=  (1u << 0); // CC1S = 01: Channel 1 is input, IC1 is mapped on TI1.

    // 2. Configure edge detection and enable capture
    TIM15->CCER &= ~(0xFu); // Clear CC1P, CC1E
    TIM15->CCER |=  0b1011; // CC1P=10, CC1E=11 -> both edges

    // 3. Enable Interrupts
    TIM15->DIER |= (1u << 1); // CC1IE : Enable Capture/Compare 1 interrupt

    TIM15->SR = 0; // Clear all SR flags

    // 4. Enable TIM15 interrupt in NVIC
    ISER2 |= (1u << 5); // Enable IRQ 69

    // 5. Start the counter
    TIM15->CR1 |= 1u;  // CEN = 1
}

/* ---------------- LED Control Functions ---------------- */
void turn_on_RED(void) { GPIOA->ODR |= (1 << RED_LED_PIN); }
void turn_off_RED(void) { GPIOA->ODR &= ~(1 << RED_LED_PIN); }
void turn_on_BLUE(void) { GPIOB->ODR |= (1 << BLUE_LED_PIN); }
void turn_off_BLUE(void) { GPIOB->ODR &= ~(1 << BLUE_LED_PIN); }

/* ---------------- TIM15 IRQ ---------------- */
void TIM15_IRQHandler(void)
{
    // Check for Input Capture 1 interrupt
    if (TIM15->SR & (1u << 1)) { // SR -> CC1IF

        TIM15->SR &= ~(1u << 1); // Clear the flag SR -> CC1IF

        uint32_t captured_time_ms   = TIM15->CCR1; // The time when edge is captured (ms)
        uint8_t  level_now  = (GPIOA->IDR & (1 << IR_INPUT_PIN)) ? 1 : 0;
        // level_now == 0 -> pin LOW  -> someone sitting
        // level_now == 1 -> pin HIGH -> someone standing up

        if (level_now == 0) { // If someone sitting now (FALLING edge is detected)
            if (!is_sitting) { // Previously empty -> someone really sit                
                is_sitting   = 1; // Update state
                sit_start_time_ms = captured_time_ms;

                //turn_off_RED();
                //turn_off_BLUE();
            }
            // else -> No action because already sitting, this edge is bounce
        }
        else { // If someone standing up now (RISING edge is detected)
            if (is_sitting) { // Previously sitting -> someone really stand up
                is_sitting = 0; // Update state
                uint32_t sit_end_time_ms = captured_time_ms;
                uint32_t elapsed_ms;

                if (sit_end_time_ms >= sit_start_time_ms) {
                    elapsed_ms = sit_end_time_ms - sit_start_time_ms;
                } else {
                    // If the timer overflowed (ARR reached max and wrapped around)
                    elapsed_ms = (TIM15->ARR + 1u) - sit_start_time_ms + sit_end_time_ms;
                }

                turn_on_RED();

                // If the time is more than OCCUPIED_THRESHOLD_MS, count as a valid sit
                if (elapsed_ms >= OCCUPIED_THRESHOLD_MS) {
                    turn_on_BLUE();
                } else {
                    // Temporary contact, not a valid sit
                    // turn_off_BLUE();
                }
            }
            // else -> No action because already empty, this edge is bounce
        }
    }

    // Check for Overcapture Event
    if (TIM15->SR & (1u << 9)) { // SR -> CC1OF
        TIM15->SR &= ~(1u << 9); // Clear the flag SR -> CC1OF
        // Reset state for safety
        is_sitting = 0;
        turn_off_BLUE();
        turn_on_RED();
    }
}

// MAIN
int main(void)
{
    GPIO_setup();   // PA9 / PB7 LED + PA2 IC pins
    TIM15_setup();  // IC + timer

    // Global interrupt enable
    __asm volatile (
        "mov r0, #0      \n\t"
        "msr primask, r0 \n\t"
    );

    while (1) {
        __asm volatile ("wfi");
    }
}
