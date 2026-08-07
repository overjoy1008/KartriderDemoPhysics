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
    int instant_boost_sound;
    /* Revved while the player holds drift with the throttle before GO. */
    int booster_idle_sound;
    int crash_sound;
    int shock_sound;
    int count_sound[4];   /* 3, 2, 1, go */
    int motor_voice;
    int drift_voice;
    int booster_idle_voice;
    /* Kept so a boost that is cut short takes its sample with it. */
    int booster_voice;
    int instant_boost_voice;
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
    sound->instant_boost_sound = -1;
    sound->booster_idle_sound = -1;
    sound->crash_sound = -1;
    sound->shock_sound = -1;
    sound->count_sound[0] = -1;
    sound->count_sound[1] = -1;
    sound->count_sound[2] = -1;
    sound->count_sound[3] = -1;
    sound->motor_voice = -1;
    sound->drift_voice = -1;
    sound->booster_idle_voice = -1;
    sound->booster_voice = -1;
    sound->instant_boost_voice = -1;
    kart_sound_driver_reset(&sound->driver);
    /* A machine with no output device simply runs silent. */
    if (!kart_audio_start(&sound->audio)) return;
    sound->motor_sound =
        kart_demo_load_sound_resource(instance, sound->audio, IDR_SOUND_MOTOR);
    sound->drift_sound =
        kart_demo_load_sound_resource(instance, sound->audio, IDR_SOUND_DRIFT);
    sound->booster_sound =
        kart_demo_load_sound_resource(instance, sound->audio, IDR_SOUND_BOOSTER);
    sound->instant_boost_sound = kart_demo_load_sound_resource(
        instance, sound->audio, IDR_SOUND_INSTANT_BOOST);
    sound->booster_idle_sound = kart_demo_load_sound_resource(
        instance, sound->audio, IDR_SOUND_BOOSTER_IDLE);
    sound->count_sound[0] =
        kart_demo_load_sound_resource(instance, sound->audio, IDR_SOUND_COUNT_3);
    sound->count_sound[1] =
        kart_demo_load_sound_resource(instance, sound->audio, IDR_SOUND_COUNT_2);
    sound->count_sound[2] =
        kart_demo_load_sound_resource(instance, sound->audio, IDR_SOUND_COUNT_1);
    sound->count_sound[3] = kart_demo_load_sound_resource(
        instance, sound->audio, IDR_SOUND_COUNT_GO);
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
    /* The item boost and the instant boost are tracked apart so each can have
       its own sample. */
    state = kart_sound_driver_update(
        &sound->driver, speed, drift_active,
        kart->timed_boost.active, kart->instant_boost.active,
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
    /* Both boosters layer: firing again before the previous one finishes adds
       a voice rather than being dropped. */
    if (state.start_booster && sound->booster_sound >= 0) {
        sound->booster_voice = kart_audio_play_overlapping(
            sound->audio, sound->booster_sound, 1.0f);
    } else if (!kart->timed_boost.active && sound->booster_voice >= 0) {
        /* Same rule as the drift loop: the sample belongs to the state, so a
           boost that ends early takes it with it. */
        kart_audio_stop_voice(sound->audio, sound->booster_voice);
        sound->booster_voice = -1;
    }
    if (state.start_instant_boost && sound->instant_boost_sound >= 0) {
        sound->instant_boost_voice = kart_audio_play_overlapping(
            sound->audio, sound->instant_boost_sound, 1.0f);
    } else if (!kart->instant_boost.active &&
               sound->instant_boost_voice >= 0) {
        kart_audio_stop_voice(sound->audio, sound->instant_boost_voice);
        sound->instant_boost_voice = -1;
    }
    if (state.start_crash && sound->crash_sound >= 0) {
        kart_audio_play_once(sound->audio, sound->crash_sound, state.crash_volume);
    }
    if (state.start_shock && sound->shock_sound >= 0) {
        kart_audio_play_once(sound->audio, sound->shock_sound, state.shock_volume);
    }
}

/* The booster idle loop, held for as long as the player revs on the line. */
static void kart_demo_sound_set_booster_idle(KartDemoSound *sound, bool on)
{
    if (sound->audio == NULL) return;
    if (on) {
        if (sound->booster_idle_voice < 0 && sound->booster_idle_sound >= 0) {
            sound->booster_idle_voice = kart_audio_play_loop(
                sound->audio, sound->booster_idle_sound, 1.0f, 1.0f);
        }
    } else if (sound->booster_idle_voice >= 0) {
        kart_audio_stop_voice(sound->audio, sound->booster_idle_voice);
        sound->booster_idle_voice = -1;
    }
}

/* index 0..3 selects count_3, count_2, count_1, count_go. */
static void kart_demo_sound_play_count(KartDemoSound *sound, int index)
{
    if (sound->audio == NULL || index < 0 || index > 3) return;
    if (sound->count_sound[index] < 0) return;
    kart_audio_play_overlapping(sound->audio, sound->count_sound[index], 1.0f);
}

#endif
