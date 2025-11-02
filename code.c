#include <stdint.h>
#include <stdio.h>

/* ---------------- System Parameters (tunable, mock) ---------------- */
#define LED_ON_SEC                 15 // 
#define OCCUPIED_THRESHOLD_SEC     10  // seconds seated to confirm usage
#define AUTO_FLUSH_DELAY_SEC        5  // delay after standing up (auto flush)
#define COVER_COOLDOWN_SEC          8  // MIN time between cover rotations
#define WATER_LEVEL_HIGH_MM       150 // mm from bottom
#define MOTOR_RUNNING_SEC           3  // cover motor running time

int ir_sensor ;
int pir_sensor;
/* ---------------- Global Inputs (mocked each step) ------------------ */
int g_light_sensor = 0;      // 0 dark, 1 light
int g_seat = 0;               // 0 empty, 1 occupied
int g_hand_motion = 0;        // 0 idle, 1 motion near cover

/* ---------------- System State ----------------------- */
int g_light_time = 0;         // accumulated light seconds
int g_seat_time = 0;          // accumulated seated seconds
int g_usage_confirmed = 0;    // became 1 when seat_time >= threshold
int g_flush_delay = 0;        // countdown for auto flush
int g_cover_cooldown = 0;     // cover rotation rate-limit
int g_prev_hand_motion = 0;   // edge detection for motion
int g_cover_state=0;        //0:closed,1:open

///* ---------------- Peripheral Register ------------- */
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
// Peripheral base addresses
#define RCC_AHB2ENR *((volatile uint32_t *) 0x4002104C)
#define RCC_APB1ENR1 *((volatile uint32_t *) 0x40021058)
#define GPIOA ((GPIO *) 0x42020000)
#define TIM6 ((TIMxBasicType *) 0x40001000)
#define TIM7 ((TIMxBasicType *) 0x40001400)
#define LED_INPUT_PIN         5  // PA5
#define LED_OUTPUT_PIN        (LED_INPUT_PIN+1) // PA6
#define COVER_INPUT_PIN       11  // PA11
#define COVER_OUTPUT_PIN      (COVER_INPUT_PIN+1) // PA12

/* ---------------- Mock Inputs (advance through sequences) ----------- */
void mock_read_seat_sensor(void) {
    static int seq[] = {0,1,1,1,1,1,1,1,1,1,0}; // ~10s sit, then stand
    static int i = 0;
    if (i >= (int)(sizeof(seq)/sizeof(seq[0]))) i = (int)(sizeof(seq)/sizeof(seq[0])) - 1;
    g_seat = seq[i++];
    printf("Seat sensor -> %d\n", g_seat);
}
/*void mock_read_hand_motion(void) {
    static int seq[] = {0,0,1,0,0,0,1,0,0,1,0}; // motion at steps 2,6,9
    static int i = 0;
    if (i >= (int)(sizeof(seq)/sizeof(seq[0]))) i = (int)(sizeof(seq)/sizeof(seq[0])) - 1;
    g_hand_motion = seq[i++];
    printf("Hand motion -> %d\n", g_hand_motion);
}*/

/* ---------------- Pure Calculations -------------------------------- */
int calculate_light_time(int light_sensor) {
    return light_sensor ? 1 : 0;
}
int calculate_usage_confirmation(int seat_now, int seat_time_sec) {
    return (seat_now == 1 && seat_time_sec >= OCCUPIED_THRESHOLD_SEC) ? 1 : 0;
}
int calculate_flush_delay_seconds(int usage_confirmed) {
    return usage_confirmed ? AUTO_FLUSH_DELAY_SEC : 0;
}

/* ---------------- Timer “updates” (prints only in mock) ------------- */
void update_timer_flush_delay(int s)   { if (s>0) printf("Timer(FlushDelay) %ds\n", s); }
void update_timer_cover_cooldown(int s){ if (s>0) printf("Timer(CoverCD) %ds\n", s); }

/* ---------------- Actuators (prints only in mock) ------------------- */
void actuate_flush(void)   { printf("** FLUSH **\n"); }
void actuate_cover(void)   { printf("** ROTATE COVER **\n"); }


void motion_sensor_initialize(void) {
    // GPIOA setup
    RCC_AHB2ENR |= 1;          // Enable GPIOA clock
    GPIOA->MODER &= ~(0xF << (LED_INPUT_PIN * 2)); // PA5-6 is cleared.
    GPIOA->MODER |= (0x4 << (LED_INPUT_PIN * 2)); // PA5-6 is input-output.
}
void hand_sensor_initialize(void) {
    // GPIOA setup
    RCC_AHB2ENR |= 1;          // Enable GPIOA clock
    GPIOA->MODER &= ~(0xF << (COVER_INPUT_PIN * 2)); // PA11-12 is cleared.
    GPIOA->MODER |= (0x4 << (COVER_INPUT_PIN * 2)); // PA11-12 is input-output.
}
void light_timer_initialize(void) {
    // TIM6 setup for 100ms tick
    RCC_APB1ENR1 |= 1 << 4;   // Enable TIM6 clock
    TIM6->PSC = 3999; // Set Prescaler to divide the system clock to 1 KHz.
    TIM6->ARR = (1000*LED_ON_SEC-1); // Set Delay. The counter will reach 99 and return to 0.
    TIM6->CR1 &= ~(1<<1); // OVF will generate an event.
    //TIM6->CR1 |= (1<<7); // Enable auto-reload shadow register.
    TIM6->CR1 |= 1; // Enable counter register (clocked).
}
void cover_motor_timer_initialize(void) {
    // TIM7 setup for 1s tick
    RCC_APB1ENR1 |= 1 << 5;   // Enable TIM7 clock
    TIM7->PSC = 3999; // Set Prescaler to divide the system clock to 1 KHz.
    TIM7->ARR = (1000*MOTOR_RUNNING_SEC-1); // Set Delay. The counter will reach 999 and return to 0.
    TIM7->CR1 &= ~(1<<1); // OVF will generate an event
    //TIM7->CR1 |= (1<<7); // Enable auto-reload shadow register.
    TIM7->CR1 |= 1; // Enable counter register (clocked).
}


static inline void start_or_retrigger_led(void){
    TIM6->CNT = 0;
    TIM6->SR  = 0;            
    GPIOA->ODR |= (1 << LED_OUTPUT_PIN);
}

static inline void start_or_retrigger_cover(void){
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
    g_cover_state=0;
  }
}
int main(void) {
    printf("Conceptual Design – Smart Toilet (no manual button)\n");
    motion_sensor_initialize();
    hand_sensor_initialize();
    light_timer_initialize();
    cover_motor_timer_initialize();


    

    while(1){
        if ((GPIOA->IDR & (1u<< LED_INPUT_PIN))!=0&& g_light_time==0) {
            start_or_retrigger_led();
            g_light_time=1;
        }
        if ((GPIOA->IDR & (1u<<COVER_INPUT_PIN))==0&& g_cover_state==0){
            start_or_retrigger_cover();
            g_cover_state=1;
        }
        if (g_light_time==1)
        {
            check_led_timeout_and_turn_off();
        }
        if(g_cover_state==1)
        {
            check_cover_timeout_and_turn_off();
        }

    }
    //for (int t = 0; t < 11; ++t) {
    //    printf("\n--- Step %d ---\n", t);
//
    //    /* 1) Read mocks */
    //    mock_read_seat_sensor();
    //    mock_read_hand_motion();
//
    //    /* 2) Seat timing & auto-flush scheduling */
    //    if (g_seat == 1) {
    //        g_seat_time++;
    //        if (!g_usage_confirmed && calculate_usage_confirmation(g_seat, g_seat_time)) {
    //            g_usage_confirmed = 1;
    //            printf("Usage confirmed at %d sec seated\n", g_seat_time);
    //        }
    //    } else {
    //        if (g_seat_time > 0) {
    //            printf("User stood up after %d sec\n", g_seat_time);
    //            int delay = calculate_flush_delay_seconds(g_usage_confirmed);
    //            if (delay > 0) { g_flush_delay = delay; update_timer_flush_delay(g_flush_delay); }
    //            else           { printf("Short seating, no auto-flush\n"); }
    //            g_seat_time = 0;
    //        }
    //    }
//
    //    /* 3) Hand motion -> cover rotation (with two guards):
    //          - Guard A: DO NOT rotate while seated.
    //          - Guard B: Rate-limit with COVER_COOLDOWN_SEC (edge-triggered). */
    //    if (g_cover_cooldown > 0) g_cover_cooldown--;
    //    int motion_rising_edge = (g_hand_motion == 1 && g_prev_hand_motion == 0);
    //    if (motion_rising_edge) {
    //        if (g_seat == 1) {
    //            printf("Motion detected but seat is occupied -> cover rotation blocked\n");
    //        } else if (g_cover_cooldown > 0) {
    //            printf("Motion detected but in cooldown (%ds left) -> blocked\n", g_cover_cooldown);
    //        } else {
    //            actuate_cover();
    //            g_cover_cooldown = COVER_COOLDOWN_SEC;
    //            update_timer_cover_cooldown(g_cover_cooldown);
    //        }
    //    }
    //    g_prev_hand_motion = g_hand_motion;
//
    //    /* 4) Auto-flush countdown -> flush */
    //    if (g_flush_delay > 0) {
    //        g_flush_delay--;
    //        if (g_flush_delay == 0) {
    //            actuate_flush();
    //            g_usage_confirmed = 0;  // reset for next cycle
    //        }
    //    }
    //}
    //return 0;
}