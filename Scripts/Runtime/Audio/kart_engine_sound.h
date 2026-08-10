#ifndef KART_ENGINE_SOUND_H
#define KART_ENGINE_SOUND_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Kart sound driving recovered from the original demo's FUN_00452E60 and
   FUN_00458000. This header holds the laws only; playback is the platform's
   problem.

   The original registers each sound once and opens channels through
   0x004DA090(system, sound, mode). The mode is a bit field: bit 3 refuses a new
   channel while one is already running, which is what stops a level-triggered
   sound from restarting every frame.

     motor, drift   mode 3     opened once, stopped explicitly
     booster        mode 0xC   one-shot, single-instance guard
     crash, shock   mode 0xE   one-shot, single-instance guard */

/* 0x00452E60 only recomputes the engine sound when this much time has passed. */
#define KART_SOUND_MOTOR_INTERVAL_MS 64u

/* pitch and volume share one ramp: speed * slope + base.
     0x005730D0 = 0.01171875f
     0x005712C0 = 0.25f */
#define KART_SOUND_MOTOR_SLOPE 0.01171875f
#define KART_SOUND_MOTOR_BASE 0.25f
/* Above these speeds each output is pinned instead of following the ramp.
     0x005730D4 = 128.0f -> 1.5f
     0x005730CC =  64.0f -> 1.0f */
#define KART_SOUND_MOTOR_PITCH_SPEED 128.0f
#define KART_SOUND_MOTOR_PITCH_CAP 1.5f
#define KART_SOUND_MOTOR_VOLUME_SPEED 64.0f
#define KART_SOUND_MOTOR_VOLUME_CAP 1.0f

/* Impact volumes: clamp(magnitude * scale, 0.1, 1.0).
     crash 0x00571D44 = 0.1f
     shock 0x005730C8 = 0.04f */
#define KART_SOUND_CRASH_SCALE 0.1f
#define KART_SOUND_SHOCK_SCALE 0.04f
#define KART_SOUND_IMPACT_MIN_VOLUME 0.1f
#define KART_SOUND_IMPACT_MAX_VOLUME 1.0f

typedef struct KartSoundDriver {
    unsigned int previous_motor_ms;
    float motor_pitch;
    float motor_volume;
    bool drift_active;
    bool booster_active;
    bool instant_boost_active;
    bool initialized;
} KartSoundDriver;

/* What the platform layer should make true after an update. */
typedef struct KartSoundState {
    /* Held between updates; the original only refreshes them every 64 ms. */
    float motor_pitch;
    float motor_volume;
    /* The drift loop runs exactly while the kart's drift flag is set. */
    bool drift_looping;
    /* Edges, true for the single update that should start the one-shot. */
    bool start_booster;
    bool start_instant_boost;
    bool start_crash;
    bool start_shock;
    float crash_volume;
    float shock_volume;
} KartSoundState;

void kart_sound_driver_reset(KartSoundDriver *driver);

/* speed is the magnitude of the kart's linear velocity, as the original reads
   it from kart+0x5C.

   The original drives one booster sound from the kart's vftable slot 26 flag,
   which covers the item boost and the instant boost alike. This split is a
   simulator-side choice so the two can have different samples.

   crash_magnitude and shock_magnitude are zero when nothing was hit. */
KartSoundState kart_sound_driver_update(
    KartSoundDriver *driver,
    float speed,
    bool drift_active,
    bool boost_active,
    bool instant_boost_active,
    float crash_magnitude,
    float shock_magnitude,
    unsigned int now_ms);

/* clamp(magnitude * scale, 0.1, 1.0). Exposed for tests. */
float kart_sound_impact_volume(float magnitude, float scale);

#ifdef __cplusplus
}
#endif

#endif
