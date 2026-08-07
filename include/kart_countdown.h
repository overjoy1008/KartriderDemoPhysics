#ifndef KART_COUNTDOWN_H
#define KART_COUNTDOWN_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Race start countdown recovered from the original demo.

   FUN_00451EB0 and FUN_00456CD0 are the same state machine at two different
   field offsets. Entering the ready state sets a deadline:

     deadline = now + 7000;                 // +0xE0 / +0x114
     schedule("start", deadline - 3000);

   and the per-frame body then counts down against it:

     stage 0, deadline <= now + 3000  ->  count_3, stage 1
     stage 1, deadline <= now + 2000  ->  count_2, stage 2
     stage 2, deadline <= now + 1000  ->  count_1, stage 3
     now >= deadline                  ->  count_go, race state 2,
                                          every kart released

   The start boost lives in the accelerate action of the input handler
   FUN_004529D0, measured against the same deadline:

     if (deadline != 0 &&
         eventTime <= deadline + 100 && deadline - 100 <= eventTime)
         GoKart_StartTimedBoost(kart, 1000);        // 0x00431AB0

   0x00431AB0 writes the duration to kart+0x158 and raises the flag at
   kart+0xE5, which is the same flag vftable slot 26 reports, so the start
   boost drives the booster sound and the camera's wide field of view exactly
   like an item boost. */

#define KART_COUNTDOWN_TOTAL_MS 7000u
#define KART_COUNTDOWN_START_CUE_MS 3000u
/* Half-width of the window around the deadline that grants the start boost. */
#define KART_START_BOOST_WINDOW_MS 100u
#define KART_START_BOOST_DURATION_MS 1000u

typedef enum KartCountdownStage {
    KART_COUNTDOWN_IDLE = 0,
    KART_COUNTDOWN_THREE = 1,
    KART_COUNTDOWN_TWO = 2,
    KART_COUNTDOWN_ONE = 3,
    KART_COUNTDOWN_RUNNING = 4
} KartCountdownStage;

typedef struct KartCountdown {
    /* Absolute time of the GO moment. */
    unsigned int deadline_ms;
    KartCountdownStage stage;
    bool armed;
    bool released;
} KartCountdown;

/* Cues raised by a single update. Each is true only on the update that crosses
   its threshold. */
typedef struct KartCountdownCues {
    bool play_three;
    bool play_two;
    bool play_one;
    bool play_go;
    /* False until GO; the kart is held at the line while it is false. */
    bool released;
    /* Milliseconds until GO, zero once running. */
    unsigned int remaining_ms;
} KartCountdownCues;

/* Arms the countdown: the deadline becomes now + 7000 ms. */
void kart_countdown_start(KartCountdown *countdown, unsigned int now_ms);

KartCountdownCues kart_countdown_update(
    KartCountdown *countdown, unsigned int now_ms);

/* True when a forward press at press_ms falls inside the +/-100 ms window
   around the deadline. Grants KART_START_BOOST_DURATION_MS. */
bool kart_countdown_start_boost_granted(
    const KartCountdown *countdown, unsigned int press_ms);

#ifdef __cplusplus
}
#endif

#endif
