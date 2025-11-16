#include <stdint.h>
#include <stdio.h>

#define LED_ON_SEC                  1
#define OCCUPIED_THRESHOLD_SEC      1
#define AUTO_FLUSH_DELAY_SEC        1
#define COVER_COOLDOWN_SEC          7
#define WATER_LEVEL_HIGH_MM       150
#define MOTOR_RUNNING_SEC           3

//
#define IR_INPUT_PIN   2
#define RED_LED_PIN    9
#define BLUE_LED_PIN   7

#define SERVO_SIGNAL_PIN  0

#define TIM2_PSC_VAL      3
#define TIM2_ARR_VAL      19999
#define TIM2_PERIOD_MS    20u

#define TIM15_PSC_VAL     3999
#define TIM15_ARR_VAL     0xFFFF

#define SERVO_MIN_PULSE   1000
#define SERVO_MAX_PULSE   2000

#define SERVO_MIN_ANGLE   0
#define SERVO_MAX_ANGLE   65

#define SERVO_STEP_DEG    4

#define AUTO_FLUSH_DELAY_TICKS     ((AUTO_FLUSH_DELAY_SEC * 1000u) / TIM2_PERIOD_MS)

#define INTER_SWEEP_DELAY_SEC      2
#define INTER_SWEEP_DELAY_TICKS    ((INTER_SWEEP_DELAY_SEC * 1000u) / TIM2_PERIOD_MS)
// 

int g_light_sensor = 0;
int g_seat = 0;
int g_hand_motion = 0;

int g_light_time = 0;
int g_seat_time = 0;
int g_usage_confirmed = 0;
int g_flush_delay = 0;
int g_cover_cooldown = 0;
int g_prev_hand_motion = 0;
int g_cover_state=0;

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

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    uint32_t reserved1;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    uint32_t reserved2[2];
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
    volatile uint32_t OR1;
    uint32_t reserved3[3];
    volatile uint32_t OR2;
} TIM15_General_Purpose_Type;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    uint32_t reserved1;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    uint32_t reserved2;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
} TIM_General_Purpose_32bit_Type;

#define RCC_AHB2ENR *((volatile uint32_t *) 0x4002104C)
#define RCC_APB1ENR1 *((volatile uint32_t *) 0x40021058)
#define GPIOA ((GPIO *) 0x42020000)
#define TIM6 ((TIMxBasicType *) 0x40001000)
#define TIM7 ((TIMxBasicType *) 0x40001400)
#define LED_INPUT_PIN         5
#define LED_OUTPUT_PIN        (LED_INPUT_PIN+1)
#define COVER_INPUT_PIN       11
#define COVER_OUTPUT_PIN      (COVER_INPUT_PIN+1)

//
#define RCC_APB2ENR   (*(volatile uint32_t *) 0x40021060)

#define GPIOB ((GPIO *) 0x42020400)

#define TIM15 ((TIM15_General_Purpose_Type *) 0x40014000)
#define TIM2  ((TIM_General_Purpose_32bit_Type *) 0x40000000)

#define ISER1         (*(volatile uint32_t *) 0xE000E104)
#define ISER2         (*(volatile uint32_t *) 0xE000E108)

#define TIM15_IRQN                   69
#define TIM15_IRQ_BIT  (TIM15_IRQN - 64)

#define TIM2_IRQN                   45
#define TIM2_IRQ_BIT   (TIM2_IRQN - 32)

static uint8_t  is_sitting         = 0;
static uint32_t sit_start_time_ms  = 0;
static const uint32_t OCCUPIED_THRESHOLD_MS = OCCUPIED_THRESHOLD_SEC * 1000u;

static volatile int      g_servo_angle            = 0;
static volatile int      g_servo_dir              = 1;
static volatile uint8_t  g_servo_enabled          = 0;
static volatile uint8_t  g_servo_reached_max      = 0;
static volatile uint32_t g_servo_delay_ticks      = 0;
static volatile uint8_t  g_servo_sweeps_remaining = 0;

static void turn_on_RED(void)  { GPIOA->ODR |=  (1u << RED_LED_PIN); }
static void turn_off_RED(void) { GPIOA->ODR &= ~(1u << RED_LED_PIN); }
static void turn_on_BLUE(void) { GPIOB->ODR |=  (1u << BLUE_LED_PIN); }
static void turn_off_BLUE(void){ GPIOB->ODR &= ~(1u << BLUE_LED_PIN); }

static uint32_t map_angle_to_pulse(int angle)
{
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    return (uint32_t)(((int32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180
                      + SERVO_MIN_PULSE);
}

//

void motion_sensor_initialize(void) {
    RCC_AHB2ENR |= 1;
    GPIOA->MODER &= ~(0xF << (LED_INPUT_PIN * 2));
    GPIOA->MODER |= (0x4 << (LED_INPUT_PIN * 2));
}

void hand_sensor_initialize(void) {
    RCC_AHB2ENR |= 1;
    GPIOA->MODER &= ~(0xF << (COVER_INPUT_PIN * 2));
    GPIOA->MODER |= (0x4 << (COVER_INPUT_PIN * 2));
}

void light_timer_initialize(void) {
    RCC_APB1ENR1 |= 1 << 4;
    TIM6->PSC = 3999;
    TIM6->ARR = (1000*LED_ON_SEC-1);
    TIM6->CR1 &= ~(1<<1);
    TIM6->CR1 |= 1;
}

void cover_motor_timer_initialize(void) {
    RCC_APB1ENR1 |= 1 << 5;
    TIM7->PSC = 3999;
    TIM7->ARR = (1000*MOTOR_RUNNING_SEC-1);
    TIM7->CR1 &= ~(1<<1);
    TIM7->CR1 |= 1;
}

//
static void gpio_initialize(void)
{
    RCC_AHB2ENR |= (0b11u << 0);

    GPIOA->MODER &= ~(0b11u << (RED_LED_PIN * 2));
    GPIOA->MODER |=  (0b01u << (RED_LED_PIN * 2));

    GPIOB->MODER &= ~(0b11u << (BLUE_LED_PIN * 2));
    GPIOB->MODER |=  (0b01u << (BLUE_LED_PIN * 2));

    GPIOA->MODER &= ~(0b11u << (IR_INPUT_PIN * 2));
    GPIOA->MODER |=  (0b10u << (IR_INPUT_PIN * 2));

    GPIOA->AFRL &= ~(0xFu << (IR_INPUT_PIN * 4));
    GPIOA->AFRL |=  (0xEu << (IR_INPUT_PIN * 4));

    GPIOA->MODER &= ~(0b11u << (SERVO_SIGNAL_PIN * 2));
    GPIOA->MODER |=  (0b10u << (SERVO_SIGNAL_PIN * 2));

    GPIOA->OSPEEDR |= (0b11u << (SERVO_SIGNAL_PIN * 2));

    GPIOA->AFRL &= ~(0xFu << (SERVO_SIGNAL_PIN * 4));
    GPIOA->AFRL |=  (0x1u << (SERVO_SIGNAL_PIN * 4));
}

static void tim15_initialize(void)
{
    RCC_APB2ENR |= (1u << 16);

    TIM15->PSC = TIM15_PSC_VAL;
    TIM15->ARR = TIM15_ARR_VAL;

    TIM15->CCMR1 &= ~(3u << 0);
    TIM15->CCMR1 |=  (1u << 0);

    TIM15->CCER &= ~(0xFu);
    TIM15->CCER |=  0b1011u;

    TIM15->DIER |= (1u << 1);

    TIM15->SR = 0;

    ISER2 |= (1u << TIM15_IRQ_BIT);

    TIM15->CR1 |= 1u;
}

static void tim2_initialize(void)
{
    RCC_APB1ENR1 |= (1u << 0);

    TIM2->CR1 &= ~(1u << 0);

    TIM2->PSC = TIM2_PSC_VAL;
    TIM2->ARR = TIM2_ARR_VAL;

    g_servo_angle            = SERVO_MIN_ANGLE;
    g_servo_dir              = 1;
    g_servo_enabled          = 0;
    g_servo_reached_max      = 0;
    g_servo_delay_ticks      = 0;
    g_servo_sweeps_remaining = 0;

    TIM2->CCMR1 &= ~(3u << 0);

    TIM2->CCMR1 &= ~(0b111u << 4);
    TIM2->CCMR1 |=  (0b110u << 4);

    TIM2->CCMR1 |= (1u << 3);

    TIM2->CR1 |= (1u << 7);

    TIM2->CCER |= (1u << 0);

    TIM2->CCR1 = map_angle_to_pulse(g_servo_angle);

    TIM2->CCMR1 &= ~(3u << 8);

    TIM2->CCMR1 &= ~(0b111u << 12);

    TIM2->CCR2 = 0;

    TIM2->CCER |= (1u << 4);

    TIM2->DIER |= (1u << 2);

    TIM2->SR = 0;

    TIM2->EGR |= (1u << 0);

    ISER1 |= (1u << TIM2_IRQ_BIT);

    TIM2->CR1 |= (1u << 0);
}
//

static inline void start_or_retrigger_led(void){
    TIM6->CNT = 0;
    TIM6->SR  = 0;
    GPIOA->ODR |= (1 << LED_OUTPUT_PIN);
}

static inline void start_cover(void){
    TIM7->ARR = (1000*MOTOR_RUNNING_SEC-1);
    TIM7->CNT = 0;
    TIM7->SR  = 0;
    GPIOA->ODR |= (1 << COVER_OUTPUT_PIN);
}

static inline void check_led_timeout_and_turn_off(void){
  if (TIM6->SR & 1u){
    TIM6->SR = 0;
    GPIOA->ODR &= ~(1 << LED_OUTPUT_PIN);
    g_light_time=0;
  }
}

static inline void check_cover_timeout_and_turn_off(void){
  if (TIM7->SR & 1u){
    TIM7->SR = 0;
    GPIOA->ODR &= ~(1 << COVER_OUTPUT_PIN);
    g_cover_cooldown = COVER_COOLDOWN_SEC;
    
    TIM7->ARR = (1000*g_cover_cooldown-1);
    TIM7->CNT = 0;
    g_cover_state=0;
  }
}

static inline void check_cover_cooldown(void){
    if (TIM7->SR & 1u){
        TIM7->SR = 0;
        g_cover_cooldown = 0;

        }
}

//
void TIM15_IRQHandler(void)
{
    if (TIM15->SR & (1u << 1)) {
        TIM15->SR &= ~(1u << 1);

        uint32_t captured_time_ms = TIM15->CCR1;
        uint8_t level_now = (GPIOA->IDR & (1u << IR_INPUT_PIN)) ? 1u : 0u;

        if (level_now == 0u) {
            if (!is_sitting) {
                is_sitting        = 1u;
                sit_start_time_ms = captured_time_ms;

                turn_off_RED();
                turn_off_BLUE();

                g_servo_enabled          = 0u;
                g_servo_reached_max      = 0u;
                g_servo_delay_ticks      = 0u;
                g_servo_sweeps_remaining = 0u;
            }
        } else {
            if (is_sitting) {
                is_sitting = 0u;
                uint32_t sit_end_time_ms = captured_time_ms;
                uint32_t elapsed_ms;

                if (sit_end_time_ms >= sit_start_time_ms) {
                    elapsed_ms = sit_end_time_ms - sit_start_time_ms;
                } else {
                    elapsed_ms = (TIM15->ARR + 1u) - sit_start_time_ms + sit_end_time_ms;
                }

                turn_on_RED();

                if (elapsed_ms >= OCCUPIED_THRESHOLD_MS) {
                    turn_on_BLUE();

                    g_servo_angle            = SERVO_MIN_ANGLE;
                    g_servo_dir              = 1;
                    g_servo_reached_max      = 0;
                    g_servo_enabled          = 0;
                    g_servo_delay_ticks      = AUTO_FLUSH_DELAY_TICKS;
                    g_servo_sweeps_remaining = 2u;

                    TIM2->CCR1 = map_angle_to_pulse(g_servo_angle);
                } else {
                    turn_off_BLUE();
                    g_servo_enabled          = 0u;
                    g_servo_reached_max      = 0u;
                    g_servo_delay_ticks      = 0u;
                    g_servo_sweeps_remaining = 0u;
                }
            }
        }
    }

    if (TIM15->SR & (1u << 9)) {
        TIM15->SR &= ~(1u << 9);
        is_sitting = 0u;
        turn_off_BLUE();
        turn_on_RED();
        g_servo_enabled          = 0u;
        g_servo_reached_max      = 0u;
        g_servo_delay_ticks      = 0u;
        g_servo_sweeps_remaining = 0u;
    }
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & (1u << 2)) {
        TIM2->SR &= ~(1u << 2);

        if (!g_servo_enabled && g_servo_delay_ticks > 0u) {
            g_servo_delay_ticks--;
            if ((g_servo_delay_ticks == 0u) && (g_servo_sweeps_remaining > 0u)) {
                g_servo_angle        = SERVO_MIN_ANGLE;
                g_servo_dir          = 1;
                g_servo_reached_max  = 0;
                g_servo_enabled      = 1;
                g_servo_sweeps_remaining--;
                TIM2->CCR1 = map_angle_to_pulse(g_servo_angle);
            }
        }

        if (g_servo_enabled) {
            g_servo_angle += (g_servo_dir * SERVO_STEP_DEG);

            if (g_servo_dir > 0 && g_servo_angle >= SERVO_MAX_ANGLE) {
                g_servo_angle       = SERVO_MAX_ANGLE;
                g_servo_dir         = -1;
                g_servo_reached_max = 1;
            } else if (g_servo_dir < 0 && g_servo_angle <= SERVO_MIN_ANGLE) {
                g_servo_angle = SERVO_MIN_ANGLE;

                if (g_servo_reached_max) {
                    if (g_servo_sweeps_remaining > 0u) {
                        g_servo_enabled     = 0;
                        g_servo_dir         = 1;
                        g_servo_reached_max = 0;
                        g_servo_delay_ticks = INTER_SWEEP_DELAY_TICKS;
                    } else {
                        g_servo_enabled = 0;
                    }
                } else {
                    g_servo_dir = 1;
                }
            }

            TIM2->CCR1 = map_angle_to_pulse(g_servo_angle);
        }
    }

    if (TIM2->SR & (1u << 0)) {
        TIM2->SR &= ~(1u << 0);
    }
}
//

int main(void) {
    printf("Conceptual Design - Smart Toilet (no manual button)\n");
    motion_sensor_initialize();
    hand_sensor_initialize();
    light_timer_initialize();
    cover_motor_timer_initialize();

    g_cover_cooldown=0;

    gpio_initialize();
    tim15_initialize();
    tim2_initialize();

    __asm volatile(
        "mov r0, #0      \n\t"
        "msr primask, r0 \n\t"
    );

    while(1){
        if ((GPIOA->IDR & (1u<< LED_INPUT_PIN))!=0) {
            start_or_retrigger_led();
            g_light_time=1;
            g_light_sensor=1;
        }
        if ((GPIOA->IDR & (1u<<COVER_INPUT_PIN))==0 && g_cover_state==0 && g_cover_cooldown==0){
            start_cover();
            g_cover_state=1;
        }
        if (g_light_time==1)
        {
            g_light_sensor=0;
            check_led_timeout_and_turn_off();
        }
        if  (g_cover_state==1)
        {
            check_cover_timeout_and_turn_off();
        }
        if(g_cover_state==0 && g_cover_cooldown>0)
        {
            check_cover_cooldown();
        }
        __asm volatile("wfi");

    }
}
