/* The mixer's voice cursor. A 32-bit 16.16 cursor wraps at 65536 source frames
   (2.97 s at 22050 Hz), which silently turned booster.wav, at 68379 frames,
   into an endless one-shot. */

#include "kart_audio_win32.h"

#include <assert.h>
#include <stdio.h>

#define KART_AUDIO_TEST_RATE 22050

/* Runs a voice to completion and returns how many frames it produced, or -1 if
   it did not finish within the limit. */
static long long play_to_end(
    unsigned int frame_count, unsigned int step, long long limit)
{
    unsigned long long position = 0;
    long long produced = 0;
    while (produced <= limit) {
        if (kart_audio_next_frame(&position, step, frame_count, false) < 0) {
            return produced;
        }
        ++produced;
    }
    return -1;
}

static void test_one_shot_terminates_past_the_32bit_wrap(void)
{
    /* Every original sample, including the one that used to hang. */
    static const unsigned int frame_counts[] = {
        17248,  /* motor    0.78 s */
        32064,  /* drift    1.45 s */
        9398,   /* crash    0.43 s */
        6368,   /* shock    0.29 s */
        68379,  /* booster  3.10 s, past the 65536 frame wrap */
    };
    size_t i;
    for (i = 0; i < sizeof(frame_counts) / sizeof(frame_counts[0]); ++i) {
        const unsigned int frames = frame_counts[i];
        const long long produced = play_to_end(frames, 1u << 16, frames * 4LL);
        assert(produced == (long long)frames);
    }

    /* Well past the wrap too, so the fix is not merely sized for booster.wav. */
    assert(play_to_end(500000u, 1u << 16, 2000000LL) == 500000LL);
}

static void test_pitch_changes_the_consumption_rate(void)
{
    /* Half rate reads each source frame twice; double rate skips every other. */
    assert(play_to_end(1000u, 1u << 15, 10000LL) == 2000LL);
    assert(play_to_end(1000u, 2u << 16, 10000LL) == 500LL);
}

static void test_looping_never_ends_and_stays_in_range(void)
{
    const unsigned int frames = 68379;
    unsigned long long position = 0;
    long long step;
    for (step = 0; step < 400000; ++step) {
        const long index = kart_audio_next_frame(&position, 1u << 16, frames, true);
        assert(index >= 0);
        assert(index < (long)frames);
    }
}

static void test_degenerate_inputs(void)
{
    unsigned long long position = 0;
    assert(kart_audio_next_frame(&position, 1u << 16, 0u, false) < 0);
    assert(kart_audio_next_frame(&position, 1u << 16, 0u, true) < 0);
    assert(kart_audio_next_frame(NULL, 1u << 16, 100u, false) < 0);
}

int main(void)
{
    test_one_shot_terminates_past_the_32bit_wrap();
    test_pitch_changes_the_consumption_rate();
    test_looping_never_ends_and_stays_in_range();
    test_degenerate_inputs();
    printf("audio cursor tests passed\n");
    return 0;
}
