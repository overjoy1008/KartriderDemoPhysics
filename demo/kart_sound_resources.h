#ifndef KART_SOUND_RESOURCES_H
#define KART_SOUND_RESOURCES_H

/* The original demo's own effect samples, all 16-bit mono 22050 Hz PCM:
     kart.rho        motor.wav drift.wav crash.wav shock.wav
     sound_fx_item   booster/booster.wav */
#define IDR_SOUND_MOTOR 401
#define IDR_SOUND_DRIFT 402
#define IDR_SOUND_BOOSTER 403
#define IDR_SOUND_CRASH 404
#define IDR_SOUND_SHOCK 405
/* Supplied separately, converted to the same PCM format. The original has no
   dedicated instant-boost sample; this replaces it. */
#define IDR_SOUND_INSTANT_BOOST 406
/* Idle / coasting. Loaded and reserved, not yet driven by anything. */
#define IDR_SOUND_BOOSTER_IDLE 407
/* Race start countdown, from sound_fx_etc.rho. */
#define IDR_SOUND_COUNT_3 408
#define IDR_SOUND_COUNT_2 409
#define IDR_SOUND_COUNT_1 410
#define IDR_SOUND_COUNT_GO 411

#endif
