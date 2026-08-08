#ifndef KART_TRACK_COLLISION_H
#define KART_TRACK_COLLISION_H

#include "kart_demo_data.h"
#include "kart_track_scene.h"

/* Does the segment `start` .. `start + delta` pierce the triangle?

   The same Moller-Trumbore routine the ground ray uses, which is also what
   0x00434b40 is; 0x00425fe0 calls it for a checkpoint gate and throws the hit
   fraction and normal away, so this reports only whether there was a hit. */
bool kart_track_segment_triangle_hit(
    KartVec3 start,
    KartVec3 delta,
    KartVec3 a,
    KartVec3 b,
    KartVec3 c);

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
