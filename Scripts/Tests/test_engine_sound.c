/* Kart sound driving recovered from KartRider.exe FUN_00452E60 / FUN_00458000.
   These pin the constants and the trigger shapes so a later edit cannot turn
   them into invented behaviour. */

#include "kart_engine_sound.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static int near(float actual, float expected, float epsilon)
{
    return fabsf(actual - expected) <= epsilon;
}

/* Advances past the 64 ms throttle so the engine values refresh. */
static KartSoundState refresh(
    KartSoundDriver *driver, float speed, unsigned int *clock)
{
    *clock += KART_SOUND_MOTOR_INTERVAL_MS + 1u;
    return kart_sound_driver_update(
        driver, speed, false, false, false, 0.0f, 0.0f, *clock);
}

static void test_engine_ramp_matches_recovered_constants(void)
{
    KartSoundDriver driver;
    unsigned int clock = 0;
    KartSoundState state;

    kart_sound_driver_reset(&driver);
    kart_sound_driver_update(&driver, 0.0f, false, false, false, 0.0f, 0.0f, clock);

    /* speed * 0.01171875 + 0.25, shared by pitch and volume. */
    state = refresh(&driver, 0.0f, &clock);
    assert(near(state.motor_pitch, KART_SOUND_MOTOR_BASE, 1e-6f));
    assert(near(state.motor_volume, KART_SOUND_MOTOR_BASE, 1e-6f));

    state = refresh(&driver, 32.0f, &clock);
    assert(near(state.motor_pitch, 32.0f * KART_SOUND_MOTOR_SLOPE +
                                       KART_SOUND_MOTOR_BASE, 1e-6f));
    assert(near(state.motor_volume, state.motor_pitch, 1e-6f));

    /* Volume reaches its cap exactly at speed 64, so the curve is continuous. */
    state = refresh(&driver, 63.9f, &clock);
    assert(state.motor_volume < KART_SOUND_MOTOR_VOLUME_CAP);
    assert(near(state.motor_volume, KART_SOUND_MOTOR_VOLUME_CAP, 0.002f));
    state = refresh(&driver, 64.0f, &clock);
    assert(near(state.motor_volume, KART_SOUND_MOTOR_VOLUME_CAP, 1e-6f));
    state = refresh(&driver, 500.0f, &clock);
    assert(near(state.motor_volume, KART_SOUND_MOTOR_VOLUME_CAP, 1e-6f));

    /* Pitch is pinned at 1.5 from speed 128, where the ramp would have reached
       1.75. The original really does step down there. */
    state = refresh(&driver, 127.9f, &clock);
    assert(state.motor_pitch > KART_SOUND_MOTOR_PITCH_CAP);
    assert(near(state.motor_pitch, 1.7488f, 0.001f));
    state = refresh(&driver, 128.0f, &clock);
    assert(near(state.motor_pitch, KART_SOUND_MOTOR_PITCH_CAP, 1e-6f));
}

static void test_engine_only_refreshes_every_64ms(void)
{
    KartSoundDriver driver;
    unsigned int clock = 1000;
    KartSoundState state;

    kart_sound_driver_reset(&driver);
    kart_sound_driver_update(&driver, 0.0f, false, false, false, 0.0f, 0.0f, clock);
    state = refresh(&driver, 10.0f, &clock);
    {
        const float held = state.motor_pitch;
        /* Well inside the window: the speed change must not be picked up. */
        clock += 10;
        state = kart_sound_driver_update(
            &driver, 100.0f, false, false, false, 0.0f, 0.0f, clock);
        assert(near(state.motor_pitch, held, 1e-6f));
        /* Exactly at the boundary is still inside; the test is `>`. */
        clock += KART_SOUND_MOTOR_INTERVAL_MS - 10u;
        state = kart_sound_driver_update(
            &driver, 100.0f, false, false, false, 0.0f, 0.0f, clock);
        assert(near(state.motor_pitch, held, 1e-6f));
        /* One millisecond past it refreshes. */
        clock += 1;
        state = kart_sound_driver_update(
            &driver, 100.0f, false, false, false, 0.0f, 0.0f, clock);
        assert(!near(state.motor_pitch, held, 1e-6f));
    }
}

static void test_drift_loop_follows_the_flag(void)
{
    KartSoundDriver driver;
    unsigned int clock = 0;
    KartSoundState state;
    kart_sound_driver_reset(&driver);

    state = kart_sound_driver_update(&driver, 10.0f, false, false, false, 0, 0, ++clock);
    assert(!state.drift_looping);
    state = kart_sound_driver_update(&driver, 10.0f, true, false, false, 0, 0, ++clock);
    assert(state.drift_looping);
    /* Holding the flag keeps it looping rather than restarting it. */
    state = kart_sound_driver_update(&driver, 10.0f, true, false, false, 0, 0, ++clock);
    assert(state.drift_looping);
    state = kart_sound_driver_update(&driver, 10.0f, false, false, false, 0, 0, ++clock);
    assert(!state.drift_looping);
}

static void test_booster_starts_once_per_activation(void)
{
    /* The original re-opens the booster channel every frame the flag is set and
       relies on the mode 0xC single-instance guard; the edge here is the same
       audible result. */
    KartSoundDriver driver;
    unsigned int clock = 0;
    KartSoundState state;
    kart_sound_driver_reset(&driver);

    state = kart_sound_driver_update(&driver, 10.0f, false, false, false, 0, 0, ++clock);
    assert(!state.start_booster);
    state = kart_sound_driver_update(&driver, 10.0f, false, true, false, 0, 0, ++clock);
    assert(state.start_booster);
    state = kart_sound_driver_update(&driver, 10.0f, false, true, false, 0, 0, ++clock);
    assert(!state.start_booster);
    state = kart_sound_driver_update(&driver, 10.0f, false, false, false, 0, 0, ++clock);
    assert(!state.start_booster);
    state = kart_sound_driver_update(&driver, 10.0f, false, true, false, 0, 0, ++clock);
    assert(state.start_booster);
}

static void test_instant_boost_is_tracked_apart_from_the_item_boost(void)
{
    /* The original drives one booster sound from a single flag. Splitting them
       is a simulator-side choice so each can have its own sample, so neither
       edge may leak into the other. */
    KartSoundDriver driver;
    unsigned int clock = 0;
    KartSoundState state;
    kart_sound_driver_reset(&driver);

    state = kart_sound_driver_update(&driver, 10.0f, false, false, true, 0, 0, ++clock);
    assert(state.start_instant_boost && !state.start_booster);
    state = kart_sound_driver_update(&driver, 10.0f, false, false, true, 0, 0, ++clock);
    assert(!state.start_instant_boost);

    /* An item boost while the instant boost is still held fires only its own. */
    state = kart_sound_driver_update(&driver, 10.0f, false, true, true, 0, 0, ++clock);
    assert(state.start_booster && !state.start_instant_boost);

    /* Both drop, then both rise together: two independent edges. */
    state = kart_sound_driver_update(&driver, 10.0f, false, false, false, 0, 0, ++clock);
    assert(!state.start_booster && !state.start_instant_boost);
    state = kart_sound_driver_update(&driver, 10.0f, false, true, true, 0, 0, ++clock);
    assert(state.start_booster && state.start_instant_boost);
}

static void test_impact_volumes(void)
{
    /* clamp(magnitude * scale, 0.1, 1.0) for both, with different scales. */
    assert(near(kart_sound_impact_volume(0.0f, KART_SOUND_CRASH_SCALE),
                KART_SOUND_IMPACT_MIN_VOLUME, 1e-6f));
    assert(near(kart_sound_impact_volume(5.0f, KART_SOUND_CRASH_SCALE), 0.5f, 1e-6f));
    assert(near(kart_sound_impact_volume(100.0f, KART_SOUND_CRASH_SCALE),
                KART_SOUND_IMPACT_MAX_VOLUME, 1e-6f));
    /* Shock is quieter for the same magnitude: 0.04 against 0.1. */
    assert(near(kart_sound_impact_volume(10.0f, KART_SOUND_SHOCK_SCALE), 0.4f, 1e-6f));
    assert(kart_sound_impact_volume(10.0f, KART_SOUND_SHOCK_SCALE) <
           kart_sound_impact_volume(10.0f, KART_SOUND_CRASH_SCALE));

    {
        KartSoundDriver driver;
        unsigned int clock = 0;
        KartSoundState state;
        kart_sound_driver_reset(&driver);
        state = kart_sound_driver_update(&driver, 0.0f, false, false, false,
                                         0.0f, 0.0f, ++clock);
        assert(!state.start_crash && !state.start_shock);
        state = kart_sound_driver_update(&driver, 0.0f, false, false, false,
                                         20.0f, 3.0f, ++clock);
        assert(state.start_crash && near(state.crash_volume, 1.0f, 1e-6f));
        assert(state.start_shock && near(state.shock_volume, 0.12f, 1e-6f));
    }
}

int main(void)
{
    test_engine_ramp_matches_recovered_constants();
    test_engine_only_refreshes_every_64ms();
    test_drift_loop_follows_the_flag();
    test_booster_starts_once_per_activation();
    test_instant_boost_is_tracked_apart_from_the_item_boost();
    test_impact_volumes();
    printf("engine sound tests passed\n");
    return 0;
}
