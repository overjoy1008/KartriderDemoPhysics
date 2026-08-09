#ifndef KART_SKYDOME_RESOURCES_H
#define KART_SKYDOME_RESOURCES_H

/* The dome that ships beside each track.1s as skydome.1s, exported as its own
   KTRK. Only seven of the thirteen are here: the desert and forest domes carry
   a class stamp the asset reader has no type for, so those tracks have no
   entry and simply draw no sky. */

#define IDR_SKYDOME_ICE_I01 281
#define IDR_SKYDOME_ICE_I02 282
#define IDR_SKYDOME_ICE_R01 283
#define IDR_SKYDOME_VILLAGE_I01 284
#define IDR_SKYDOME_VILLAGE_I02 285
#define IDR_SKYDOME_VILLAGE_R01 286
#define IDR_SKYDOME_VILLAGE_R03 287

#ifndef RC_INVOKED
#include "kart_demo_data.h"

/* Parallel to TRACKS[] in src/kart_demo_data.c, so entry i belongs to
   kart_demo_track_at(i). 0 means the track has no exported dome. */
static const int KART_SKYDOME_RESOURCE_IDS[] = {
    0, /* flat_test: synthetic, no assets at all */
    0, /* desert_I01: unsupported class 1f4b04fc */
    0, /* desert_I02 */
    0, /* desert_R01 */
    0, /* forest_I01: unsupported class 1f4b04fc */
    0, /* forest_I02 */
    0, /* forest_R02 */
    IDR_SKYDOME_ICE_I01,
    IDR_SKYDOME_ICE_I02,
    IDR_SKYDOME_ICE_R01,
    IDR_SKYDOME_VILLAGE_I01,
    IDR_SKYDOME_VILLAGE_I02,
    IDR_SKYDOME_VILLAGE_R01,
    IDR_SKYDOME_VILLAGE_R03,
};
#endif

#endif
