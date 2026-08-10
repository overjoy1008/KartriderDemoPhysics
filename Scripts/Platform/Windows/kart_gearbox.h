#ifndef KART_GEARBOX_H
#define KART_GEARBOX_H

/* Gearbox — a simulator-side experiment, not recovered code.

   The original has no gear or crankshaft of any kind: the engine note is one
   straight ramp in speed, and that is what SINGLE keeps, exactly as
   FUN_00452E60 computes it. Nothing about the car's motion is affected by
   either mode; this only drives the engine note and its dial.

   MULTI is invented. It splits the same speed range into gear bands and runs the
   note from idle to redline inside each one, so the note is a sawtooth in speed
   rather than a single line: it drops on an upshift and jumps on a downshift.
   The band edges, idle and redline below are chosen to make that shape easy to
   watch, and are not claimed to be anything the original did. */

#include <stdbool.h>

#define KART_GEAR_COUNT 4

typedef struct KartGearBand {
    float upper_speed; /* where this gear hands over */
    float low_pitch;   /* the note just after engaging it */
    float high_pitch;  /* the note just before leaving it */
} KartGearBand;

/* Each gear has its own stretch of the note, and consecutive gears overlap, so
   an upshift drops the note only as far as the next gear's bottom rather than
   all the way back to idle. Lower gears cover the widest pitch range, which is
   what makes the first shift the loudest step and the later ones progressively
   milder. Past the top gear the note holds at its high pitch. */
static const KartGearBand KART_GEAR_BANDS[KART_GEAR_COUNT] = {
    {40.0f, 0.25f, 1.10f},
    {55.0f, 0.95f, 1.20f},
    {70.0f, 1.10f, 1.30f},
    /* Top gear: there is nothing to shift into, so this speed only sets where
       its note stops climbing. */
    {128.0f, 1.20f, 1.50f},
};

/* A shift is fast but not instant: the note slews to the new gear's at this
   many pitch units per second, so 0.15 takes about 30 ms. Steep enough to hear
   as a step, finite enough that the trace has a slope instead of a wall. */
#define KART_GEAR_PITCH_SLEW 5.0f
/* Downshifts wait until the speed is this far below the band, so a kart sitting
   on an edge does not chatter between two gears. */
#define KART_GEAR_DOWNSHIFT_MARGIN 4.0f

typedef enum KartGearMode {
    KART_GEAR_SINGLE = 0,
    KART_GEAR_MULTI = 1
} KartGearMode;

typedef struct KartGearbox {
    KartGearMode mode;
    int gear;    /* 1..KART_GEAR_COUNT, valid in MULTI */
    float pitch; /* the note this gear is producing */
} KartGearbox;

static float kart_gear_lower_speed(int gear)
{
    return gear <= 1 ? 0.0f : KART_GEAR_BANDS[gear - 2].upper_speed;
}

static void kart_gearbox_update(KartGearbox *gearbox, float speed, float dt)
{
    const KartGearBand *band;
    float low;
    float high;
    float t;
    float target;
    float step;
    if (gearbox->gear < 1) gearbox->gear = 1;
    if (gearbox->gear > KART_GEAR_COUNT) gearbox->gear = KART_GEAR_COUNT;
    if (gearbox->mode != KART_GEAR_MULTI) {
        gearbox->pitch = 0.0f;
        return;
    }
    while (gearbox->gear < KART_GEAR_COUNT &&
           speed > KART_GEAR_BANDS[gearbox->gear - 1].upper_speed) {
        gearbox->gear += 1;
    }
    while (gearbox->gear > 1 &&
           speed < kart_gear_lower_speed(gearbox->gear) -
                       KART_GEAR_DOWNSHIFT_MARGIN) {
        gearbox->gear -= 1;
    }
    band = &KART_GEAR_BANDS[gearbox->gear - 1];
    low = kart_gear_lower_speed(gearbox->gear);
    high = band->upper_speed;
    t = high > low ? (speed - low) / (high - low) : 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    target = band->low_pitch + (band->high_pitch - band->low_pitch) * t;

    /* Within a gear the note moves slowly enough that the limit never binds;
       it only shapes the step at a shift. */
    if (gearbox->pitch <= 0.0f || dt <= 0.0f) {
        gearbox->pitch = target;
        return;
    }
    step = KART_GEAR_PITCH_SLEW * dt;
    if (target > gearbox->pitch + step) {
        gearbox->pitch += step;
    } else if (target < gearbox->pitch - step) {
        gearbox->pitch -= step;
    } else {
        gearbox->pitch = target;
    }
}

#endif
