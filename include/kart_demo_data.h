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

/* Which horizontal axis the start line runs along. The stripe crosses the road,
   so the racing direction is perpendicular to its long edge. */
typedef enum KartTrackStartAxis {
    KART_TRACK_AXIS_NONE = 0,
    KART_TRACK_AXIS_X,
    KART_TRACK_AXIS_Y
} KartTrackStartAxis;

/* How much of the start pose is actually supported by evidence.

   The mesh data fixes the start line's position and axis but never its sign:
   nothing in track.1s says which way round the lap is driven. Everything below
   CONFIRMED therefore assumes the +axis direction and needs checking against
   the original game. */
typedef enum KartTrackStartKind {
    /* No road-flagged start quad in the scene; spawn at the bounds centre. */
    KART_TRACK_START_NONE = 0,
    /* Quad found but nearly square, so even the axis is a guess. */
    KART_TRACK_START_AXIS_WEAK,
    /* Quad clearly elongated; axis read from geometry, sign assumed. */
    KART_TRACK_START_AXIS_CLEAR,
    /* Direction compared against the original game. */
    KART_TRACK_START_CONFIRMED
} KartTrackStartKind;

typedef struct KartDemoTrackSpec {
    const char *asset_name;
    const char *display_name;
    const char *race_mode;
    unsigned int difficulty;
    KartVec3 minimum;
    KartVec3 maximum;
    /* False for the synthetic flat track, which has no decoded track.1s mesh
       and deliberately keeps the flat ground plus AABB walls. */
    bool has_scene;
    KartTrackStartKind start_kind;
    KartTrackStartAxis start_axis;
    /* Start-quad centroid in asset space; z doubles as the scene ground plane. */
    KartVec3 start_line;
} KartDemoTrackSpec;

unsigned int kart_demo_kart_count(void);
const KartDemoKartSpec *kart_demo_kart_at(unsigned int index);
const KartDemoKartSpec *kart_demo_find_kart(const char *asset_name);
const KartDemoKartSpec *kart_demo_default_kart(void);

/* Compile-time mirror of kart_demo_track_count(), so the demos' index-parallel
   resource tables can be checked against it with _Static_assert. */
#define KART_DEMO_TRACK_COUNT 14

unsigned int kart_demo_track_count(void);
const KartDemoTrackSpec *kart_demo_track_at(unsigned int index);
const KartDemoTrackSpec *kart_demo_find_track(const char *asset_name);
const KartDemoTrackSpec *kart_demo_default_track(void);

float kart_demo_track_width(const KartDemoTrackSpec *track);
float kart_demo_track_length(const KartDemoTrackSpec *track);
bool kart_demo_track_start_position(
    const KartDemoTrackSpec *track,
    KartVec3 *position);
bool kart_demo_track_start_orientation(
    const KartDemoTrackSpec *track,
    KartQuat *orientation);
bool kart_demo_track_mirror_x(const KartDemoTrackSpec *track);
float kart_demo_track_scene_ground_z(const KartDemoTrackSpec *track);
/* World Z below the whole scene. A kart under this has tunnelled through the
   road triangles and can never land on anything, so the demos respawn it. */
float kart_demo_track_fall_limit(const KartDemoTrackSpec *track);
/* Short label for the start pose's evidence level, for HUD and docs. */
const char *kart_demo_track_start_kind_label(const KartDemoTrackSpec *track);

#ifdef __cplusplus
}
#endif

#endif
