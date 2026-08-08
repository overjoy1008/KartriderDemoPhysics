#ifndef KART_MODEL_RESOURCES_H
#define KART_MODEL_RESOURCES_H

/* The 26 kart models from the demo's own kart.rho, exported to KTRK and packed
   into the same KTKZ container the track scenes use. Ids start at 501 to stay
   clear of the track scenes at 201, the minimaps at 301 and the sounds at 401.
   The minimaps are RCDATA too, so an overlap is a link failure, not a silent
   mix-up.

   mine1 is deliberately absent. It is the 27th folder in the archive but it
   carries only a model.1s - no parameter.xml, no textures - so it is not one of
   the 26 karts and has no entry in KARTS[]. */

#define IDR_KART_MODEL_PRACTICE1 501
#define IDR_KART_MODEL_BURST1 502
#define IDR_KART_MODEL_BURST2 503
#define IDR_KART_MODEL_BURST3 504
#define IDR_KART_MODEL_BURST4 505
#define IDR_KART_MODEL_BURST5 506
#define IDR_KART_MODEL_COTTEN1 507
#define IDR_KART_MODEL_COTTEN2 508
#define IDR_KART_MODEL_COTTEN3 509
#define IDR_KART_MODEL_COTTEN4 510
#define IDR_KART_MODEL_COTTEN5 511
#define IDR_KART_MODEL_MARATHON1 512
#define IDR_KART_MODEL_MARATHON2 513
#define IDR_KART_MODEL_MARATHON3 514
#define IDR_KART_MODEL_MARATHON4 515
#define IDR_KART_MODEL_MARATHON5 516
#define IDR_KART_MODEL_SABER1 517
#define IDR_KART_MODEL_SABER2 518
#define IDR_KART_MODEL_SABER3 519
#define IDR_KART_MODEL_SABER4 520
#define IDR_KART_MODEL_SABER5 521
#define IDR_KART_MODEL_SOLID1 522
#define IDR_KART_MODEL_SOLID2 523
#define IDR_KART_MODEL_SOLID3 524
#define IDR_KART_MODEL_SOLID4 525
#define IDR_KART_MODEL_SOLID5 526

#ifndef RC_INVOKED
#include "kart_demo_data.h"

/* Parallel to KARTS[] in src/kart_demo_data.c, so entry i belongs to
   kart_demo_kart_at(i). */
static const int KART_MODEL_RESOURCE_IDS[] = {
    IDR_KART_MODEL_PRACTICE1,
    IDR_KART_MODEL_BURST1,
    IDR_KART_MODEL_BURST2,
    IDR_KART_MODEL_BURST3,
    IDR_KART_MODEL_BURST4,
    IDR_KART_MODEL_BURST5,
    IDR_KART_MODEL_COTTEN1,
    IDR_KART_MODEL_COTTEN2,
    IDR_KART_MODEL_COTTEN3,
    IDR_KART_MODEL_COTTEN4,
    IDR_KART_MODEL_COTTEN5,
    IDR_KART_MODEL_MARATHON1,
    IDR_KART_MODEL_MARATHON2,
    IDR_KART_MODEL_MARATHON3,
    IDR_KART_MODEL_MARATHON4,
    IDR_KART_MODEL_MARATHON5,
    IDR_KART_MODEL_SABER1,
    IDR_KART_MODEL_SABER2,
    IDR_KART_MODEL_SABER3,
    IDR_KART_MODEL_SABER4,
    IDR_KART_MODEL_SABER5,
    IDR_KART_MODEL_SOLID1,
    IDR_KART_MODEL_SOLID2,
    IDR_KART_MODEL_SOLID3,
    IDR_KART_MODEL_SOLID4,
    IDR_KART_MODEL_SOLID5,
};

#define KART_MODEL_CAPACITY \
    (sizeof(KART_MODEL_RESOURCE_IDS) / sizeof(KART_MODEL_RESOURCE_IDS[0]))

_Static_assert(
    KART_MODEL_CAPACITY == KART_DEMO_KART_COUNT,
    "one model resource id per kart, in KARTS[] order");
#endif

#endif
