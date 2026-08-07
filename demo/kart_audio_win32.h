#ifndef KART_AUDIO_WIN32_H
#define KART_AUDIO_WIN32_H

/* A small streaming mixer over waveOut, sized for this demo's needs: a handful
   of 16-bit mono PCM sources, one looping engine voice whose playback rate and
   volume change continuously, one looping drift voice, and a few one-shots.

   waveOut is used rather than XAudio2 or a third-party library so the demos
   keep depending only on what the OS already provides. */

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <stdbool.h>

#define KART_AUDIO_MAX_SOUNDS 8
#define KART_AUDIO_MAX_VOICES 16

typedef struct KartAudio KartAudio;

/* Starts the output device. Returns false if no device is available, in which
   case every call below is a safe no-op and the demo simply runs silent. */
bool kart_audio_start(KartAudio **audio);
void kart_audio_stop(KartAudio *audio);

/* Parses a RIFF PCM buffer and keeps a copy. Returns a sound index, or -1.
   Only 16-bit mono is accepted, which is what the original assets are. */
int kart_audio_load_wav(KartAudio *audio, const void *data, size_t size);

/* Starts a looping voice, or returns the existing one if already running.
   Returns a voice index, or -1. */
int kart_audio_play_loop(KartAudio *audio, int sound, float volume, float pitch);
/* Fire and forget. Ignored if this sound already has a one-shot running, which
   is the original's mode-0xC/0xE single-instance guard. */
void kart_audio_play_once(KartAudio *audio, int sound, float volume);
void kart_audio_stop_voice(KartAudio *audio, int voice);
void kart_audio_set_voice(KartAudio *audio, int voice, float volume, float pitch);

/* Advances a voice's fixed-point read cursor by one output frame and returns
   the source frame to read, or -1 once a non-looping voice has run out.

   The cursor is 48.16 rather than 16.16 on purpose: a 32-bit cursor wraps at
   65536 source frames, which is 2.97 s at 22050 Hz. booster.wav is 3.10 s, so
   a 32-bit cursor wrapped before it could ever reach its end and the one-shot
   played forever. Exposed so that stays covered by a test. */
long kart_audio_next_frame(
    unsigned long long *position,
    unsigned int step,
    unsigned int frame_count,
    bool looping);

#endif
