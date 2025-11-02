#include <stdint.h>
#include <stdio.h>

/* ---------------- System Parameters (tunable, mock) ---------------- */
#define OCCUPIED_THRESHOLD_SEC     10  // seconds seated to confirm usage
#define AUTO_FLUSH_DELAY_SEC        5  // delay after standing up (auto flush)
#define COVER_COOLDOWN_SEC          8  // MIN time between cover rotations

/* ---------------- Global Inputs (mocked each step) ------------------ */
int g_seat = 0;               // 0 empty, 1 occupied
int g_hand_motion = 0;        // 0 idle, 1 motion near cover

/* ---------------- System State ------------------------------------- */
int g_seat_time = 0;          // accumulated seated seconds
int g_usage_confirmed = 0;    // became 1 when seat_time >= threshold
int g_flush_delay = 0;        // countdown for auto flush
int g_cover_cooldown = 0;     // cover rotation rate-limit
int g_prev_hand_motion = 0;   // edge detection for motion

/* ---------------- Mock Inputs (advance through sequences) ----------- */
void mock_read_seat_sensor(void) {
    static int seq[] = {0,1,1,1,1,1,1,1,1,1,0}; // ~10s sit, then stand
    static int i = 0;
    if (i >= (int)(sizeof(seq)/sizeof(seq[0]))) i = (int)(sizeof(seq)/sizeof(seq[0])) - 1;
    g_seat = seq[i++];
    printf("Seat sensor -> %d\n", g_seat);
}
void mock_read_hand_motion(void) {
    static int seq[] = {0,0,1,0,0,0,1,0,0,1,0}; // motion at steps 2,6,9
    static int i = 0;
    if (i >= (int)(sizeof(seq)/sizeof(seq[0]))) i = (int)(sizeof(seq)/sizeof(seq[0])) - 1;
    g_hand_motion = seq[i++];
    printf("Hand motion -> %d\n", g_hand_motion);
}

/* ---------------- Pure Calculations -------------------------------- */
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
void actuate_perfume(void) { printf("** PERFUME (auto-flush only) **\n"); }
void actuate_cover(void)   { printf("** COVER ROTATE **\n"); }

int main(void) {
    printf("Conceptual Design – Smart Toilet (no manual button)\n");

    for (int t = 0; t < 11; ++t) {
        printf("\n--- Step %d ---\n", t);

        /* 1) Read mocks */
        mock_read_seat_sensor();
        mock_read_hand_motion();

        /* 2) Seat timing & auto-flush scheduling */
        if (g_seat == 1) {
            g_seat_time++;
            if (!g_usage_confirmed && calculate_usage_confirmation(g_seat, g_seat_time)) {
                g_usage_confirmed = 1;
                printf("Usage confirmed at %d sec seated\n", g_seat_time);
            }
        } else {
            if (g_seat_time > 0) {
                printf("User stood up after %d sec\n", g_seat_time);
                int delay = calculate_flush_delay_seconds(g_usage_confirmed);
                if (delay > 0) { g_flush_delay = delay; update_timer_flush_delay(g_flush_delay); }
                else           { printf("Short seating, no auto-flush\n"); }
                g_seat_time = 0;
            }
        }

        /* 3) Hand motion -> cover rotation (with two guards):
              - Guard A: DO NOT rotate while seated.
              - Guard B: Rate-limit with COVER_COOLDOWN_SEC (edge-triggered). */
        if (g_cover_cooldown > 0) g_cover_cooldown--;
        int motion_rising_edge = (g_hand_motion == 1 && g_prev_hand_motion == 0);
        if (motion_rising_edge) {
            if (g_seat == 1) {
                printf("Motion detected but seat is occupied -> cover rotation blocked\n");
            } else if (g_cover_cooldown > 0) {
                printf("Motion detected but in cooldown (%ds left) -> blocked\n", g_cover_cooldown);
            } else {
                actuate_cover();
                g_cover_cooldown = COVER_COOLDOWN_SEC;
                update_timer_cover_cooldown(g_cover_cooldown);
            }
        }
        g_prev_hand_motion = g_hand_motion;

        /* 4) Auto-flush countdown -> flush + perfume */
        if (g_flush_delay > 0) {
            g_flush_delay--;
            if (g_flush_delay == 0) {
                actuate_flush();
                actuate_perfume();      // only after auto-flush
                g_usage_confirmed = 0;  // reset for next cycle
            }
        }
    }
    return 0;
}