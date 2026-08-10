#include "kart_countdown.h"

#include <stddef.h>

void kart_countdown_start(KartCountdown *countdown, unsigned int now_ms)
{
    if (countdown == NULL) return;
    countdown->deadline_ms = now_ms + KART_COUNTDOWN_TOTAL_MS;
    countdown->stage = KART_COUNTDOWN_IDLE;
    countdown->armed = true;
    countdown->released = false;
}

KartCountdownCues kart_countdown_update(
    KartCountdown *countdown, unsigned int now_ms)
{
    KartCountdownCues cues = {0};
    if (countdown == NULL || !countdown->armed) {
        cues.released = true;
        return cues;
    }

    if (now_ms < countdown->deadline_ms) {
        cues.remaining_ms = countdown->deadline_ms - now_ms;
        /* Each threshold is written as the original tests it: the deadline
           compared against now plus the offset, so only one stage advances per
           update even if several are already due. */
        if (countdown->stage == KART_COUNTDOWN_IDLE &&
            countdown->deadline_ms <= now_ms + 3000u) {
            countdown->stage = KART_COUNTDOWN_THREE;
            cues.play_three = true;
        } else if (countdown->stage == KART_COUNTDOWN_THREE &&
                   countdown->deadline_ms <= now_ms + 2000u) {
            countdown->stage = KART_COUNTDOWN_TWO;
            cues.play_two = true;
        } else if (countdown->stage == KART_COUNTDOWN_TWO &&
                   countdown->deadline_ms <= now_ms + 1000u) {
            countdown->stage = KART_COUNTDOWN_ONE;
            cues.play_one = true;
        }
    } else if (!countdown->released) {
        countdown->stage = KART_COUNTDOWN_RUNNING;
        countdown->released = true;
        cues.play_go = true;
    }

    cues.released = countdown->released;
    return cues;
}

bool kart_countdown_start_boost_granted(
    const KartCountdown *countdown, unsigned int press_ms)
{
    if (countdown == NULL || !countdown->armed || countdown->deadline_ms == 0) {
        return false;
    }
    /* The original compares both bounds against the same deadline, so the
       window is symmetric and inclusive on each side. */
    return press_ms <= countdown->deadline_ms + KART_START_BOOST_WINDOW_MS &&
           countdown->deadline_ms - KART_START_BOOST_WINDOW_MS <= press_ms;
}
