#include "kart_engine_sound.h"

#include <stddef.h>

float kart_sound_impact_volume(float magnitude, float scale)
{
    /* 0x0042AFA0 then 0x00431EA0: max(magnitude * scale, 0.1) then min(.., 1.0). */
    float volume = magnitude * scale;
    if (volume < KART_SOUND_IMPACT_MIN_VOLUME) {
        volume = KART_SOUND_IMPACT_MIN_VOLUME;
    }
    if (volume > KART_SOUND_IMPACT_MAX_VOLUME) {
        volume = KART_SOUND_IMPACT_MAX_VOLUME;
    }
    return volume;
}

void kart_sound_driver_reset(KartSoundDriver *driver)
{
    if (driver == NULL) return;
    driver->previous_motor_ms = 0;
    /* The engine loop is opened at volume 0 and only rises once the first
       64 ms refresh runs. */
    driver->motor_pitch = KART_SOUND_MOTOR_BASE;
    driver->motor_volume = 0.0f;
    driver->drift_active = false;
    driver->booster_active = false;
    driver->instant_boost_active = false;
    driver->initialized = false;
}

KartSoundState kart_sound_driver_update(
    KartSoundDriver *driver,
    float speed,
    bool drift_active,
    bool boost_active,
    bool instant_boost_active,
    float crash_magnitude,
    float shock_magnitude,
    unsigned int now_ms)
{
    KartSoundState state = {0};
    if (driver == NULL) return state;

    if (!driver->initialized) {
        driver->previous_motor_ms = now_ms;
        driver->initialized = true;
    }

    /* 0x004537A5: if (0x40 < now - previous) refresh, otherwise hold. */
    if (now_ms - driver->previous_motor_ms > KART_SOUND_MOTOR_INTERVAL_MS) {
        const float ramp = speed * KART_SOUND_MOTOR_SLOPE + KART_SOUND_MOTOR_BASE;
        driver->previous_motor_ms = now_ms;
        driver->motor_pitch = speed >= KART_SOUND_MOTOR_PITCH_SPEED
            ? KART_SOUND_MOTOR_PITCH_CAP
            : ramp;
        driver->motor_volume = speed >= KART_SOUND_MOTOR_VOLUME_SPEED
            ? KART_SOUND_MOTOR_VOLUME_CAP
            : ramp;
    }
    state.motor_pitch = driver->motor_pitch;
    state.motor_volume = driver->motor_volume;

    /* The drift loop is edge driven: opened when the flag rises, stopped when
       it falls. Holding the flag does not reopen it. */
    state.drift_looping = drift_active;
    driver->drift_active = drift_active;

    /* The booster is level triggered every frame in the original; its mode 0xC
       single-instance guard is what stops it restarting. Reproducing that as a
       rising edge gives the same audible result without needing the guard.

       Each activation starts a fresh voice even while an earlier one is still
       playing, so rapid repeats overlap instead of being swallowed. That is a
       simulator-side choice: the original's guard would have dropped them. */
    state.start_booster = boost_active && !driver->booster_active;
    driver->booster_active = boost_active;
    state.start_instant_boost =
        instant_boost_active && !driver->instant_boost_active;
    driver->instant_boost_active = instant_boost_active;

    if (crash_magnitude > 0.0f) {
        state.start_crash = true;
        state.crash_volume =
            kart_sound_impact_volume(crash_magnitude, KART_SOUND_CRASH_SCALE);
    }
    if (shock_magnitude > 0.0f) {
        state.start_shock = true;
        state.shock_volume =
            kart_sound_impact_volume(shock_magnitude, KART_SOUND_SHOCK_SCALE);
    }
    return state;
}
