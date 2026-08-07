#include "kart_audio_win32.h"

#include <mmsystem.h>
#include <stdlib.h>
#include <string.h>

/* The original assets are all 16-bit mono 22050 Hz, so the mixer runs at that
   rate and never has to convert formats. */
#define KART_AUDIO_RATE 22050
#define KART_AUDIO_BLOCK_FRAMES 512
#define KART_AUDIO_BLOCKS 4

typedef struct KartAudioSound {
    short *samples;
    unsigned int frame_count;
} KartAudioSound;

typedef struct KartAudioVoice {
    int sound;
    bool active;
    bool looping;
    /* 48.16 fixed-point read cursor, so pitch is a resampling step. See
       kart_audio_next_frame for why this is not 32 bits. */
    unsigned long long position;
    unsigned int step;
    float volume;
} KartAudioVoice;

long kart_audio_next_frame(
    unsigned long long *position,
    unsigned int step,
    unsigned int frame_count,
    bool looping)
{
    unsigned long long index;
    if (position == NULL || frame_count == 0) return -1;
    index = *position >> 16;
    if (index >= (unsigned long long)frame_count) {
        if (!looping) return -1;
        *position %= (unsigned long long)frame_count << 16;
        index = *position >> 16;
    }
    *position += step;
    return (long)index;
}

struct KartAudio {
    HWAVEOUT device;
    WAVEHDR headers[KART_AUDIO_BLOCKS];
    short *blocks[KART_AUDIO_BLOCKS];
    unsigned int next_block;
    CRITICAL_SECTION lock;
    HANDLE thread;
    volatile LONG running;
    HANDLE block_ready;
    KartAudioSound sounds[KART_AUDIO_MAX_SOUNDS];
    int sound_count;
    KartAudioVoice voices[KART_AUDIO_MAX_VOICES];
};

static unsigned int pitch_to_step(float pitch)
{
    /* 1.0 plays at the source rate. Clamped so a bad value cannot run the
       cursor away. */
    if (pitch < 0.05f) pitch = 0.05f;
    if (pitch > 4.0f) pitch = 4.0f;
    return (unsigned int)(pitch * 65536.0f + 0.5f);
}

/* Mixes one block. Called only from the feeder thread. */
static void mix_block(KartAudio *audio, short *output)
{
    int frame;
    int i;
    static int accumulator[KART_AUDIO_BLOCK_FRAMES];

    memset(accumulator, 0, sizeof(accumulator));
    EnterCriticalSection(&audio->lock);
    for (i = 0; i < KART_AUDIO_MAX_VOICES; ++i) {
        KartAudioVoice *voice = &audio->voices[i];
        const KartAudioSound *sound;
        unsigned long long position;
        unsigned int step;
        int gain;
        if (!voice->active) continue;
        sound = &audio->sounds[voice->sound];
        if (sound->frame_count == 0) {
            voice->active = false;
            continue;
        }
        position = voice->position;
        step = voice->step;
        gain = (int)(voice->volume * 4096.0f);
        for (frame = 0; frame < KART_AUDIO_BLOCK_FRAMES; ++frame) {
            const long index = kart_audio_next_frame(
                &position, step, sound->frame_count, voice->looping);
            if (index < 0) {
                voice->active = false;
                break;
            }
            accumulator[frame] += (sound->samples[index] * gain) >> 12;
        }
        voice->position = position;
    }
    LeaveCriticalSection(&audio->lock);

    for (frame = 0; frame < KART_AUDIO_BLOCK_FRAMES; ++frame) {
        int value = accumulator[frame];
        if (value > 32767) value = 32767;
        if (value < -32768) value = -32768;
        output[frame] = (short)value;
    }
}

static DWORD WINAPI feeder(LPVOID parameter)
{
    KartAudio *audio = (KartAudio *)parameter;
    while (InterlockedCompareExchange(&audio->running, 1, 1) == 1) {
        bool queued = false;
        int i;
        for (i = 0; i < KART_AUDIO_BLOCKS; ++i) {
            WAVEHDR *header = &audio->headers[i];
            if ((header->dwFlags & WHDR_INQUEUE) != 0) continue;
            if ((header->dwFlags & WHDR_DONE) != 0) {
                waveOutUnprepareHeader(audio->device, header, sizeof(*header));
                header->dwFlags &= ~(DWORD)WHDR_DONE;
            }
            mix_block(audio, audio->blocks[i]);
            header->lpData = (LPSTR)audio->blocks[i];
            header->dwBufferLength = KART_AUDIO_BLOCK_FRAMES * sizeof(short);
            header->dwFlags = 0;
            if (waveOutPrepareHeader(audio->device, header, sizeof(*header)) == MMSYSERR_NOERROR) {
                waveOutWrite(audio->device, header, sizeof(*header));
                queued = true;
            }
        }
        /* Roughly one block of audio; the loop refills whatever drained. */
        if (!queued) {
            Sleep(KART_AUDIO_BLOCK_FRAMES * 1000 / KART_AUDIO_RATE / 2 + 1);
        }
    }
    return 0;
}

bool kart_audio_start(KartAudio **out)
{
    KartAudio *audio;
    WAVEFORMATEX format;
    int i;

    if (out == NULL) return false;
    *out = NULL;
    if (waveOutGetNumDevs() == 0) return false;

    audio = (KartAudio *)calloc(1, sizeof(*audio));
    if (audio == NULL) return false;

    memset(&format, 0, sizeof(format));
    format.wFormatTag = WAVE_FORMAT_PCM;
    format.nChannels = 1;
    format.nSamplesPerSec = KART_AUDIO_RATE;
    format.wBitsPerSample = 16;
    format.nBlockAlign = 2;
    format.nAvgBytesPerSec = KART_AUDIO_RATE * 2;

    if (waveOutOpen(&audio->device, WAVE_MAPPER, &format, 0, 0, CALLBACK_NULL)
            != MMSYSERR_NOERROR) {
        free(audio);
        return false;
    }
    for (i = 0; i < KART_AUDIO_BLOCKS; ++i) {
        audio->blocks[i] = (short *)calloc(KART_AUDIO_BLOCK_FRAMES, sizeof(short));
        if (audio->blocks[i] == NULL) {
            kart_audio_stop(audio);
            return false;
        }
    }
    InitializeCriticalSection(&audio->lock);
    audio->running = 1;
    audio->thread = CreateThread(NULL, 0, feeder, audio, 0, NULL);
    if (audio->thread == NULL) {
        audio->running = 0;
        DeleteCriticalSection(&audio->lock);
        kart_audio_stop(audio);
        return false;
    }
    *out = audio;
    return true;
}

void kart_audio_stop(KartAudio *audio)
{
    int i;
    if (audio == NULL) return;
    if (audio->thread != NULL) {
        InterlockedExchange(&audio->running, 0);
        WaitForSingleObject(audio->thread, 2000);
        CloseHandle(audio->thread);
        DeleteCriticalSection(&audio->lock);
    }
    if (audio->device != NULL) {
        waveOutReset(audio->device);
        for (i = 0; i < KART_AUDIO_BLOCKS; ++i) {
            if ((audio->headers[i].dwFlags & WHDR_PREPARED) != 0) {
                waveOutUnprepareHeader(audio->device, &audio->headers[i],
                                       sizeof(audio->headers[i]));
            }
        }
        waveOutClose(audio->device);
    }
    for (i = 0; i < KART_AUDIO_BLOCKS; ++i) free(audio->blocks[i]);
    for (i = 0; i < audio->sound_count; ++i) free(audio->sounds[i].samples);
    free(audio);
}

int kart_audio_load_wav(KartAudio *audio, const void *data, size_t size)
{
    const unsigned char *bytes = (const unsigned char *)data;
    size_t offset = 12;
    const unsigned char *pcm = NULL;
    unsigned int pcm_size = 0;
    unsigned short channels = 0;
    unsigned short bits = 0;
    KartAudioSound *sound;

    if (audio == NULL || data == NULL || size < 44) return -1;
    if (audio->sound_count >= KART_AUDIO_MAX_SOUNDS) return -1;
    if (memcmp(bytes, "RIFF", 4) != 0 || memcmp(bytes + 8, "WAVE", 4) != 0) return -1;

    while (offset + 8 <= size) {
        const unsigned char *id = bytes + offset;
        unsigned int chunk;
        memcpy(&chunk, bytes + offset + 4, 4);
        if (offset + 8 + chunk > size) break;
        if (memcmp(id, "fmt ", 4) == 0 && chunk >= 16) {
            memcpy(&channels, bytes + offset + 10, 2);
            memcpy(&bits, bytes + offset + 22, 2);
        } else if (memcmp(id, "data", 4) == 0) {
            pcm = bytes + offset + 8;
            pcm_size = chunk;
        }
        offset += 8 + chunk + (chunk & 1u);
    }
    if (pcm == NULL || pcm_size < 2 || channels != 1 || bits != 16) return -1;

    sound = &audio->sounds[audio->sound_count];
    sound->frame_count = pcm_size / 2u;
    sound->samples = (short *)malloc(sound->frame_count * sizeof(short));
    if (sound->samples == NULL) return -1;
    memcpy(sound->samples, pcm, sound->frame_count * sizeof(short));
    return audio->sound_count++;
}

static int allocate_voice(KartAudio *audio)
{
    int i;
    for (i = 0; i < KART_AUDIO_MAX_VOICES; ++i) {
        if (!audio->voices[i].active) return i;
    }
    return -1;
}

int kart_audio_play_loop(KartAudio *audio, int sound, float volume, float pitch)
{
    int index;
    if (audio == NULL || sound < 0 || sound >= audio->sound_count) return -1;
    EnterCriticalSection(&audio->lock);
    for (index = 0; index < KART_AUDIO_MAX_VOICES; ++index) {
        KartAudioVoice *voice = &audio->voices[index];
        if (voice->active && voice->looping && voice->sound == sound) {
            LeaveCriticalSection(&audio->lock);
            return index;
        }
    }
    index = allocate_voice(audio);
    if (index >= 0) {
        KartAudioVoice *voice = &audio->voices[index];
        voice->sound = sound;
        voice->looping = true;
        voice->position = 0;
        voice->step = pitch_to_step(pitch);
        voice->volume = volume;
        voice->active = true;
    }
    LeaveCriticalSection(&audio->lock);
    return index;
}

static void start_one_shot(KartAudio *audio, int sound, float volume)
{
    const int index = allocate_voice(audio);
    if (index >= 0) {
        KartAudioVoice *voice = &audio->voices[index];
        voice->sound = sound;
        voice->looping = false;
        voice->position = 0;
        voice->step = pitch_to_step(1.0f);
        voice->volume = volume;
        voice->active = true;
    }
}

void kart_audio_play_once(KartAudio *audio, int sound, float volume)
{
    int index;
    if (audio == NULL || sound < 0 || sound >= audio->sound_count) return;
    EnterCriticalSection(&audio->lock);
    /* The original's mode bit 3 refuses a second channel for a sound that is
       still playing, so a level-triggered effect cannot stutter. */
    for (index = 0; index < KART_AUDIO_MAX_VOICES; ++index) {
        const KartAudioVoice *voice = &audio->voices[index];
        if (voice->active && !voice->looping && voice->sound == sound) {
            LeaveCriticalSection(&audio->lock);
            return;
        }
    }
    start_one_shot(audio, sound, volume);
    LeaveCriticalSection(&audio->lock);
}

void kart_audio_play_overlapping(KartAudio *audio, int sound, float volume)
{
    if (audio == NULL || sound < 0 || sound >= audio->sound_count) return;
    EnterCriticalSection(&audio->lock);
    start_one_shot(audio, sound, volume);
    LeaveCriticalSection(&audio->lock);
}

void kart_audio_stop_voice(KartAudio *audio, int voice)
{
    if (audio == NULL || voice < 0 || voice >= KART_AUDIO_MAX_VOICES) return;
    EnterCriticalSection(&audio->lock);
    audio->voices[voice].active = false;
    LeaveCriticalSection(&audio->lock);
}

void kart_audio_set_voice(KartAudio *audio, int voice, float volume, float pitch)
{
    if (audio == NULL || voice < 0 || voice >= KART_AUDIO_MAX_VOICES) return;
    EnterCriticalSection(&audio->lock);
    if (audio->voices[voice].active) {
        audio->voices[voice].volume = volume;
        audio->voices[voice].step = pitch_to_step(pitch);
    }
    LeaveCriticalSection(&audio->lock);
}
