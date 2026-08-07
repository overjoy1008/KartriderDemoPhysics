#ifndef KART_SOUND_WIN32_H
#define KART_SOUND_WIN32_H

/* Joins the recovered sound driver to the waveOut mixer. Shared by both demos
   so they stay audibly identical. */

#include "kart_audio_win32.h"
#include "kart_engine_sound.h"
#include "kart_simulation.h"
#include "kart_sound_resources.h"

#include <math.h>

typedef struct KartDemoSound {
    KartAudio *audio;
    KartSoundDriver driver;
    int motor_sound;
    int drift_sound;
    int booster_sound;
    int crash_sound;
    int shock_sound;
    int motor_voice;
    int drift_voice;
} KartDemoSound;

static int kart_demo_load_sound_resource(
    HINSTANCE instance, KartAudio *audio, int resource_id)
{
#if defined(KART_EMBED_SOUNDS)
    HRSRC resource = FindResourceA(
        instance, MAKEINTRESOURCEA(resource_id), RT_RCDATA);
    HGLOBAL loaded;
    const void *data;
    DWORD size;
    if (resource == NULL) return -1;
    loaded = LoadResource(instance, resource);
    size = SizeofResource(instance, resource);
    data = loaded != NULL ? LockResource(loaded) : NULL;
    if (data == NULL || size == 0) return -1;
    return kart_audio_load_wav(audio, data, (size_t)size);
#else
    (void)instance;
    (void)audio;
    (void)resource_id;
    return -1;
#endif
}

static void kart_demo_sound_start(HINSTANCE instance, KartDemoSound *sound)
{
    memset(sound, 0, sizeof(*sound));
    sound->motor_sound = -1;
    sound->drift_sound = -1;
    sound->booster_sound = -1;
    sound->crash_sound = -1;
    sound->shock_sound = -1;
    sound->motor_voice = -1;
    sound->drift_voice = -1;
    kart_sound_driver_reset(&sound->driver);
    /* A machine with no output device simply runs silent. */
    if (!kart_audio_start(&sound->audio)) return;
    sound->motor_sound =
        kart_demo_load_sound_resource(instance, sound->audio, IDR_SOUND_MOTOR);
    sound->drift_sound =
        kart_demo_load_sound_resource(instance, sound->audio, IDR_SOUND_DRIFT);
    sound->booster_sound =
        kart_demo_load_sound_resource(instance, sound->audio, IDR_SOUND_BOOSTER);
    sound->crash_sound =
        kart_demo_load_sound_resource(instance, sound->audio, IDR_SOUND_CRASH);
    sound->shock_sound =
        kart_demo_load_sound_resource(instance, sound->audio, IDR_SOUND_SHOCK);
    /* The original opens the engine loop once, at volume 0, and only ever
       changes its volume and rate afterwards. */
    if (sound->motor_sound >= 0) {
        sound->motor_voice = kart_audio_play_loop(
            sound->audio, sound->motor_sound, 0.0f, KART_SOUND_MOTOR_BASE);
    }
}

static void kart_demo_sound_stop(KartDemoSound *sound)
{
    if (sound->audio != NULL) kart_audio_stop(sound->audio);
    sound->audio = NULL;
}

/* Called once per frame with the same elapsed clock the simulation uses. */
static void kart_demo_sound_update(
    KartDemoSound *sound,
    const KartSimulationState *kart,
    bool boost_active,
    float crash_magnitude,
    float shock_magnitude,
    unsigned int now_ms)
{
    const KartVec3 v = kart->linear_velocity;
    const float speed = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    const bool drift_active = kart->drift.input_active ||
                              kart->drift.trigger_active ||
                              kart->drift.slip_detected;
    KartSoundState state;

    if (sound->audio == NULL) return;
    state = kart_sound_driver_update(
        &sound->driver, speed, drift_active, boost_active,
        crash_magnitude, shock_magnitude, now_ms);

    if (sound->motor_voice >= 0) {
        kart_audio_set_voice(
            sound->audio, sound->motor_voice,
            state.motor_volume, state.motor_pitch);
    }
    if (state.drift_looping) {
        if (sound->drift_voice < 0 && sound->drift_sound >= 0) {
            sound->drift_voice = kart_audio_play_loop(
                sound->audio, sound->drift_sound, 1.0f, 1.0f);
        }
    } else if (sound->drift_voice >= 0) {
        kart_audio_stop_voice(sound->audio, sound->drift_voice);
        sound->drift_voice = -1;
    }
    if (state.start_booster && sound->booster_sound >= 0) {
        kart_audio_play_once(sound->audio, sound->booster_sound, 1.0f);
    }
    if (state.start_crash && sound->crash_sound >= 0) {
        kart_audio_play_once(sound->audio, sound->crash_sound, state.crash_volume);
    }
    if (state.start_shock && sound->shock_sound >= 0) {
        kart_audio_play_once(sound->audio, sound->shock_sound, state.shock_volume);
    }
}

#endif
