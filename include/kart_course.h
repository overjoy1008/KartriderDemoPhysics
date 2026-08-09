#ifndef KART_COURSE_H
#define KART_COURSE_H

#include <stdbool.h>

#include "kart_simulation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The original's course: a graph of rectangular gates across the road, the
   crossing test that decides progress, and the lap counter built on it.

   docs/ORIGINAL_COURSE.md holds the decompilation. A checkpoint here is not a
   radius around a point: it is the two triangles of a quad standing across the
   road, and a kart passes it when the segment between two consecutive
   positions pierces one of them (0x00425fe0). There is no tolerance anywhere;
   the only threshold in the whole mechanism is zero.

   Everything below is in the simulator's world space. The generated tables in
   src/kart_course_data.c already carry the asset-to-world transform. */

/* --- generated asset tables (src/kart_course_data.c) --------------------- */

typedef struct KartCourseRecord {
    /* One point on the road's centreline, and the direction of travel there.
       The gate's normal is record 0's direction (0x00424e00), and the point
       list a node carries is these records in order. */
    float position[3];
    float direction[3];
} KartCourseRecord;

typedef struct KartCourseElement {
    /* NULL unless the asset names it; `start`/`end`/`final` match against it. */
    const char *name;
    /* The gate quad's two triangles, corners in the asset's index order. The
       graph builder permutes them by `reverse`. */
    float face[2][3][3];
    /* ToRoad element +0x1c. Only ice_R01 carries "warpnext". */
    const char *extra;
    const KartCourseRecord *records;
    unsigned int record_count;
} KartCourseElement;

typedef struct KartCourseSection {
    /* A `road` tag: walk `elements` from `start` to `end`, wrapping. NULL for
       a `branch`, whose alternatives are sub-courses in their own right. */
    const KartCourseElement *elements;
    unsigned int element_count;
    const char *start;
    const char *end;
    const char *final;
    unsigned int reverse;
    const struct KartCourseSection *const *alternatives;
    const unsigned int *alternative_counts;
    unsigned int alternative_count;
} KartCourseSection;

typedef struct KartCourseAsset {
    const char *track;
    const KartCourseSection *sections;
    unsigned int section_count;
} KartCourseAsset;

unsigned int kart_course_asset_count(void);
const KartCourseAsset *kart_course_asset_at(unsigned int index);
const KartCourseAsset *kart_course_find_asset(const char *asset_name);

/* --- the graph 0x00424e00 builds ---------------------------------------- */

#define KART_COURSE_NO_INDEX 0xffffffffu

typedef struct KartCourseGate {
    /* The 0x5c record: two triangles at +0x04 and +0x28, the signed normal at
       +0x4c, and the `final` flag at +0x58. */
    KartVec3 face[2][3];
    KartVec3 normal;
    bool is_final;
} KartCourseGate;

typedef struct KartCourseLink {
    /* One entry of a node's forward (+0x14) or backward (+0x08) list: the gate
       to test and the node reached by crossing it. */
    unsigned int gate;
    unsigned int node;
    unsigned int next;
} KartCourseLink;

typedef struct KartCoursePoint {
    KartVec3 position;
    KartVec3 direction;
} KartCoursePoint;

typedef struct KartCourseNode {
    unsigned int id;
    /* Heads of the two link lists. The names are the original's field names,
       and they read backwards: crossing a `backward` link's gate the positive
       way moves the kart *forward* along the course. */
    unsigned int forward;
    unsigned int backward;
    unsigned int point_first;
    unsigned int point_count;
    /* +0x2c: the summed length of the point polyline. */
    float length;
    /* Asset-authored discontinuity in a "warpnext" centreline. */
    bool warp_next;
    KartVec3 warp_source;
    KartVec3 warp_destination;
    KartVec3 warp_source_direction;
    KartVec3 warp_destination_direction;
    float warp_radius_squared;
} KartCourseNode;

typedef struct KartCourse {
    KartCourseNode *nodes;
    unsigned int node_count;
    KartCourseGate *gates;
    unsigned int gate_count;
    KartCourseLink *links;
    unsigned int link_count;
    KartCoursePoint *points;
    unsigned int point_count;
    /* +0x58 and +0x5c: the first and last node. Only crossing into the first
       one moves the lap-progress counter. */
    unsigned int first_node;
    unsigned int last_node;
    /* +0x60: how many laps the race is, which the `final` gate is checked
       against. Set by kart_course_set_lap_count; 0 until then. */
    unsigned int lap_count;
    /* +0x28 and +0x34: the start pose 0x004240f0 derives from the first node's
       first point. `start_basis` is column-major, the original's layout, so
       column 1 is the negated direction of travel. */
    KartVec3 start_position;
    float start_basis[3][3];
} KartCourse;

/* Builds the graph. Returns false and leaves `course` zeroed if the asset is
   empty or a section names an element list it cannot walk. */
bool kart_course_build(KartCourse *course, const KartCourseAsset *asset);
void kart_course_free(KartCourse *course);
void kart_course_set_lap_count(KartCourse *course, unsigned int laps);

/* --- per-kart progress (0x00424b30) ------------------------------------- */

typedef struct KartCourseProgress {
    unsigned int node;
    /* The original's record[1]: +1 for each crossing of the start node, -1 for
       each crossing back over it. Every other gate only moves `node`, which is
       why cutting the course stops laps from counting. */
    int advance;
    unsigned int node_id;
    /* record[3] and record[4]: how far along the current node's point polyline
       the kart is, and which point it is past (0x00426670). */
    float node_distance;
    unsigned int point;
    unsigned int lap_start_ms;
    unsigned int lap;
    bool lap_completed;
    /* record[9]: 0 until a lap has been timed. */
    unsigned int best_lap_ms;
    /* record[10]. True only when the kart both points and moves against the
       course direction; either one agreeing with it clears the flag. */
    bool wrong_way;
} KartCourseProgress;

/* Puts the kart at the last node with no progress, which is where
   0x00424530 starts every kart: behind the first node's gate. */
void kart_course_progress_init(
    const KartCourse *course,
    KartCourseProgress *progress,
    KartVec3 position);

/* One trail segment, the unit 0x00426470 works in. `time_ms` is the race clock
   0x00424b30 stamps lap times with. Returns the advance the segment produced,
   which is -1, 0 or +1. */
int kart_course_progress_step(
    const KartCourse *course,
    KartCourseProgress *progress,
    KartVec3 previous_position,
    KartVec3 position,
    KartQuat orientation,
    KartVec3 velocity,
    unsigned int time_ms);

/* The gate test at 0x00425fe0: +1 crossing with the normal, -1 against it, 0
   for no crossing. Exposed because it is the whole checkpoint mechanism. */
int kart_course_gate_crossing(
    const KartCourseGate *gate,
    KartVec3 segment_start,
    KartVec3 segment_end);

/* Returns the asset-authored warp destination and horizontal turn when the
   active node's "warpnext" plane is crossed in the forward direction. */
bool kart_course_warp_next(
    const KartCourse *course,
    KartVec3 segment_start,
    KartVec3 segment_end,
    KartVec3 *destination,
    float *yaw_radians);

/* --- placement ---------------------------------------------------------- */

/* 0x00424530: the start grid. `grid_index` is the kart's slot; slot 0 sits on
   the course's start pose and the rest spread sideways along it. The caller
   supplies the ground query the original runs (10 above, 100 down) by passing
   the scene; pass NULL for scene/track to skip the snap. */
void kart_course_start_pose(
    const KartCourse *course,
    unsigned int grid_index,
    KartVec3 *position,
    KartQuat *orientation);

/* 0x00424640: what the original does 500 ms after a reset or a fall. The kart
   returns to the first point of the node it is currently in, half a unit back
   from that point and facing along the course. */
bool kart_course_respawn_pose(
    const KartCourse *course,
    const KartCourseProgress *progress,
    KartVec3 *position,
    KartQuat *orientation);

#ifdef __cplusplus
}
#endif

#endif
