#ifndef KART_DEMO_DATA_H
#define KART_DEMO_DATA_H

#include "kart_simulation.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct KartDemoKartSpec {
    const char *asset_name;
    KartDynamicsConfig dynamics;
    KartSimulationGeometry geometry;
    float model_height;
} KartDemoKartSpec;

typedef struct KartDemoTrackSpec {
    const char *asset_name;
    const char *display_name;
    KartVec3 minimum;
    KartVec3 maximum;
} KartDemoTrackSpec;

unsigned int kart_demo_kart_count(void);
const KartDemoKartSpec *kart_demo_kart_at(unsigned int index);
const KartDemoKartSpec *kart_demo_find_kart(const char *asset_name);
const KartDemoKartSpec *kart_demo_default_kart(void);

unsigned int kart_demo_track_count(void);
const KartDemoTrackSpec *kart_demo_track_at(unsigned int index);
const KartDemoTrackSpec *kart_demo_find_track(const char *asset_name);
const KartDemoTrackSpec *kart_demo_default_track(void);

float kart_demo_track_width(const KartDemoTrackSpec *track);
float kart_demo_track_length(const KartDemoTrackSpec *track);

#ifdef __cplusplus
}
#endif

#endif
