/* Race start countdown recovered from KartRider.exe FUN_00451EB0 / FUN_00456CD0
   and the accelerate action of FUN_004529D0. */

#include "kart_countdown.h"

#include <assert.h>
#include <stdio.h>

static void test_deadline_is_seven_seconds_out(void)
{
    KartCountdown countdown;
    kart_countdown_start(&countdown, 1000u);
    assert(countdown.deadline_ms == 1000u + KART_COUNTDOWN_TOTAL_MS);
    assert(countdown.stage == KART_COUNTDOWN_IDLE);
    assert(!countdown.released);
}

static void test_cues_fire_at_the_recovered_thresholds(void)
{
    KartCountdown countdown;
    KartCountdownCues cues;
    const unsigned int start = 10000u;
    const unsigned int deadline = start + KART_COUNTDOWN_TOTAL_MS;
    kart_countdown_start(&countdown, start);

    /* Nothing for the first four seconds, and the kart stays held. */
    cues = kart_countdown_update(&countdown, start);
    assert(!cues.play_three && !cues.play_two && !cues.play_one && !cues.play_go);
    assert(!cues.released);
    cues = kart_countdown_update(&countdown, deadline - 3001u);
    assert(!cues.play_three);

    cues = kart_countdown_update(&countdown, deadline - 3000u);
    assert(cues.play_three && !cues.released);
    /* Only once. */
    cues = kart_countdown_update(&countdown, deadline - 2999u);
    assert(!cues.play_three && !cues.play_two);

    cues = kart_countdown_update(&countdown, deadline - 2000u);
    assert(cues.play_two);
    cues = kart_countdown_update(&countdown, deadline - 1500u);
    assert(!cues.play_two && !cues.play_one);
    cues = kart_countdown_update(&countdown, deadline - 1000u);
    assert(cues.play_one);

    /* Still held right up to the deadline. */
    cues = kart_countdown_update(&countdown, deadline - 1u);
    assert(!cues.play_go && !cues.released);

    cues = kart_countdown_update(&countdown, deadline);
    assert(cues.play_go && cues.released);
    assert(cues.remaining_ms == 0);
    /* GO fires once; the kart stays released. */
    cues = kart_countdown_update(&countdown, deadline + 500u);
    assert(!cues.play_go && cues.released);
}

static void test_remaining_counts_down(void)
{
    KartCountdown countdown;
    const unsigned int start = 0u;
    kart_countdown_start(&countdown, start);
    assert(kart_countdown_update(&countdown, start).remaining_ms ==
           KART_COUNTDOWN_TOTAL_MS);
    assert(kart_countdown_update(&countdown, start + 4000u).remaining_ms == 3000u);
    assert(kart_countdown_update(&countdown, start + 6500u).remaining_ms == 500u);
}

static void test_start_boost_window(void)
{
    KartCountdown countdown;
    const unsigned int start = 5000u;
    const unsigned int deadline = start + KART_COUNTDOWN_TOTAL_MS;
    kart_countdown_start(&countdown, start);

    /* Inclusive on both sides, 100 ms either way. */
    assert(kart_countdown_start_boost_granted(&countdown, deadline));
    assert(kart_countdown_start_boost_granted(
        &countdown, deadline - KART_START_BOOST_WINDOW_MS));
    assert(kart_countdown_start_boost_granted(
        &countdown, deadline + KART_START_BOOST_WINDOW_MS));
    assert(!kart_countdown_start_boost_granted(
        &countdown, deadline - KART_START_BOOST_WINDOW_MS - 1u));
    assert(!kart_countdown_start_boost_granted(
        &countdown, deadline + KART_START_BOOST_WINDOW_MS + 1u));
    /* Pressing early, during the "3", earns nothing. */
    assert(!kart_countdown_start_boost_granted(&countdown, deadline - 3000u));

    assert(KART_START_BOOST_DURATION_MS == 1000u);
}

static void test_unarmed_countdown_leaves_the_kart_free(void)
{
    KartCountdown countdown = {0};
    const KartCountdownCues cues = kart_countdown_update(&countdown, 1234u);
    assert(cues.released);
    assert(!cues.play_three && !cues.play_go);
    assert(!kart_countdown_start_boost_granted(&countdown, 1234u));
}

int main(void)
{
    test_deadline_is_seven_seconds_out();
    test_cues_fire_at_the_recovered_thresholds();
    test_remaining_counts_down();
    test_start_boost_window();
    test_unarmed_countdown_leaves_the_kart_free();
    printf("countdown tests passed\n");
    return 0;
}
