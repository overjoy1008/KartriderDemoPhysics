#ifndef KART_TRACK_COLLISION_H
#define KART_TRACK_COLLISION_H

#include "kart_demo_data.h"
#include "kart_track_scene.h"

KartVec3 kart_track_scene_world_vertex(
    const KartTrackSceneVertex *vertex,
    const KartDemoTrackSpec *track);

bool kart_track_scene_query_ground(
    const KartTrackScene *scene,
    const KartDemoTrackSpec *track,
    KartVec3 ray_start,
    KartVec3 ray_delta,
    KartGroundHit *hit);

unsigned int kart_track_scene_query_body_collisions(
    const KartTrackScene *scene,
    const KartDemoTrackSpec *track,
    const KartSimulationState *state,
    KartBodyContact *contacts,
    unsigned int capacity);

#endif
