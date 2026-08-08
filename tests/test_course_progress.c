/* The course graph, gate crossing and lap counter over all 13 real tracks.

   The strong check here is the lap drive: a kart is walked along the course's
   own centreline, which is the path the gates were built across, and the lap
   counter has to agree with the number of loops. That exercises the graph
   build, the link direction, the segment-triangle crossing and the advance
   rule together, and it fails on any off-by-one in the element walk. */

#include "kart_course.h"
#include "kart_demo_data.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

static int failures = 0;

#define CHECK(condition, ...)                                                 \
    do {                                                                      \
        if (!(condition)) {                                                   \
            printf("FAIL %s:%d ", __FILE__, __LINE__);                         \
            printf(__VA_ARGS__);                                              \
            printf("\n");                                                     \
            ++failures;                                                       \
        }                                                                     \
    } while (0)

static KartVec3 subtract(KartVec3 a, KartVec3 b)
{
    return (KartVec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

static KartVec3 add(KartVec3 a, KartVec3 b)
{
    return (KartVec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

static KartVec3 scale(KartVec3 v, float s)
{
    return (KartVec3){v.x * s, v.y * s, v.z * s};
}

static float dot(KartVec3 a, KartVec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static float length_of(KartVec3 v)
{
    return sqrtf(dot(v, v));
}

/* The node sequence a kart following the course visits, taking the first link
   at every step. On a branch track that picks one alternative, which is what
   a kart driving down one side of the split does. */
static unsigned int *node_order(
    const KartCourse *course,
    unsigned int *count,
    bool *closed)
{
    unsigned int *order = malloc((size_t)course->node_count * sizeof *order);
    unsigned int node = course->first_node;
    unsigned int visited = 0;
    *closed = false;
    while (visited < course->node_count) {
        unsigned int link = course->nodes[node].backward;
        order[visited++] = node;
        while (link != KART_COURSE_NO_INDEX &&
               course->links[link].node == KART_COURSE_NO_INDEX) {
            link = course->links[link].next;
        }
        if (link == KART_COURSE_NO_INDEX) break;
        node = course->links[link].node;
        if (node == course->first_node) {
            *closed = true;
            break;
        }
    }
    *count = visited;
    return order;
}

/* Every centreline point of those nodes, in order: the lap the kart drives. */
static KartVec3 *lap_path(
    const KartCourse *course,
    const unsigned int *order,
    unsigned int order_count,
    unsigned int *count)
{
    unsigned int total = 0;
    unsigned int index;
    unsigned int written = 0;
    KartVec3 *path;
    for (index = 0; index < order_count; ++index) {
        total += course->nodes[order[index]].point_count;
    }
    path = malloc((size_t)(total != 0 ? total : 1) * sizeof *path);
    for (index = 0; index < order_count; ++index) {
        const KartCourseNode *node = &course->nodes[order[index]];
        unsigned int point;
        for (point = 0; point < node->point_count; ++point) {
            path[written++] = course->points[node->point_first + point].position;
        }
    }
    *count = written;
    return path;
}

/* Walks the path in steps no longer than `step`, feeding one trail segment per
   step, exactly as the simulation would. */
typedef struct Driver {
    const KartCourse *course;
    KartCourseProgress progress;
    KartVec3 position;
    unsigned int time_ms;
    bool facing_backwards;
} Driver;

/* The same pose turned 180 degrees about its own vertical axis, so the kart
   faces back down the course. */
static KartQuat turned_around(KartQuat q)
{
    return (KartQuat){.w = -q.z, .x = q.y, .y = -q.x, .z = q.w};
}

/* Returns false when the kart did not actually move: several tracks repeat a
   centreline record, and a zero-length step has no velocity to judge. */
static bool drive_to(Driver *driver, KartVec3 target, float step)
{
    const float distance = length_of(subtract(target, driver->position));
    const unsigned int steps = (unsigned int)(distance / step) + 1u;
    unsigned int i;
    if (distance < 1.0e-4f) return false;
    for (i = 1; i <= steps; ++i) {
        const KartVec3 previous = driver->position;
        const KartVec3 next = add(previous,
            scale(subtract(target, previous), (float)1.0f / (float)(steps - i + 1u)));
        KartQuat orientation;
        KartVec3 velocity = subtract(next, previous);
        /* Facing along the course, which is what a kart driving it does. The
           respawn pose is the original's own way of saying that. */
        if (!kart_course_respawn_pose(driver->course, &driver->progress, NULL,
                                      &orientation)) {
            orientation = (KartQuat){1.0f, 0.0f, 0.0f, 0.0f};
        }
        if (driver->facing_backwards) orientation = turned_around(orientation);
        driver->time_ms += 16;
        kart_course_progress_step(driver->course, &driver->progress, previous,
                                  next, orientation, velocity, driver->time_ms);
        driver->position = next;
    }
    return true;
}

static void test_track(const KartDemoTrackSpec *track)
{
    const KartCourseAsset *asset = kart_course_find_asset(track->asset_name);
    KartCourse course;
    unsigned int order_count = 0;
    unsigned int path_count = 0;
    unsigned int *order;
    KartVec3 *path;
    Driver driver;
    KartVec3 start_position;
    KartQuat start_orientation;
    unsigned int index;
    unsigned int lap;
    bool closed = false;
    const unsigned int laps = 3;

    CHECK(asset != NULL, "%s has no course asset", track->asset_name);
    if (asset == NULL) return;
    CHECK(kart_course_build(&course, asset), "%s course build failed",
          track->asset_name);
    if (course.node_count == 0) return;
    /* Out of reach of the advance the drive below reaches, so the `final`
       gate's end-of-race credit stays out of the lap arithmetic; it gets its
       own check further down. */
    kart_course_set_lap_count(&course, 1000u);

    /* Every node has a gate on each side once the ring is closed, and every
       link resolves. A dangling link would mean the closure never ran. */
    for (index = 0; index < course.node_count; ++index) {
        unsigned int link;
        unsigned int forward = 0;
        unsigned int backward = 0;
        for (link = course.nodes[index].forward; link != KART_COURSE_NO_INDEX;
             link = course.links[link].next) {
            CHECK(course.links[link].node != KART_COURSE_NO_INDEX,
                  "%s node %u has a dangling forward link", track->asset_name, index);
            CHECK(course.links[link].gate < course.gate_count,
                  "%s node %u forward link gate out of range", track->asset_name, index);
            ++forward;
        }
        for (link = course.nodes[index].backward; link != KART_COURSE_NO_INDEX;
             link = course.links[link].next) {
            CHECK(course.links[link].node != KART_COURSE_NO_INDEX,
                  "%s node %u has a dangling backward link", track->asset_name, index);
            ++backward;
        }
        CHECK(forward > 0 && backward > 0,
              "%s node %u has %u forward and %u backward links",
              track->asset_name, index, forward, backward);
        CHECK(course.nodes[index].point_count > 0,
              "%s node %u carries no centreline points", track->asset_name, index);
        CHECK(course.nodes[index].length > 0.0f,
              "%s node %u has zero length", track->asset_name, index);
    }

    /* A branch track has nodes on the alternative the drive does not take, so
       only a track without one visits every node. Either way the walk has to
       come back to where it started, which is what closes the ring. */
    order = node_order(&course, &order_count, &closed);
    CHECK(order_count > 2, "%s: following the course visits %u nodes",
          track->asset_name, order_count);
    CHECK(closed, "%s: following the course never returns to the first node",
          track->asset_name);

    /* The start pose faces down the course: the kart's forward axis is column
       1 of the basis negated, and column 1 is the negated travel direction. */
    kart_course_start_pose(&course, 0, &start_position, &start_orientation);
    {
        const KartVec3 travel =
            course.points[course.nodes[course.first_node].point_first].direction;
        const KartVec3 column1 = {course.start_basis[0][1], course.start_basis[1][1],
                                  course.start_basis[2][1]};
        CHECK(dot(travel, column1) < -0.99f,
              "%s start basis column 1 is not the negated travel direction (%.3f)",
              track->asset_name, dot(travel, column1));
        CHECK(fabsf(length_of(subtract(
                  course.points[course.nodes[course.first_node].point_first].position,
                  start_position)) - 0.5f) < 1.0e-3f,
              "%s start position is not half a unit behind the first gate",
              track->asset_name);
    }

    /* The lap drive. Starting behind the first gate on the last node, each
       loop of the centreline has to raise the lap counter by exactly one. */
    path = lap_path(&course, order, order_count, &path_count);
    CHECK(path_count > 2, "%s centreline has %u points", track->asset_name, path_count);
    memset(&driver, 0, sizeof driver);
    driver.course = &course;
    driver.position = start_position;
    kart_course_progress_init(&course, &driver.progress, start_position);
    CHECK(driver.progress.node == course.last_node,
          "%s does not start on the last node", track->asset_name);

    /* The first pass only gets the kart onto the course: the drive begins on
       the first gate's own plane, where whether that opening segment counts as
       a crossing is down to the last bit of the barycentric test. What is
       asserted is what a lap counter has to do - every loop after that raises
       the counter by exactly one. */
    for (index = 0; index < path_count; ++index) {
        drive_to(&driver, path[index], 2.0f);
    }
    CHECK(driver.progress.lap >= 1,
          "%s: driving a full loop did not start a lap", track->asset_name);
    for (lap = 0; lap < laps; ++lap) {
        const unsigned int expected_lap = driver.progress.lap + 1u;
        const int expected_advance = driver.progress.advance + 1;
        for (index = 0; index < path_count; ++index) {
            drive_to(&driver, path[index], 2.0f);
        }
        CHECK(driver.progress.lap == expected_lap,
              "%s after loop %u the lap counter is %u, expected %u",
              track->asset_name, lap + 1u, driver.progress.lap, expected_lap);
        CHECK(driver.progress.advance == expected_advance,
              "%s after loop %u the advance is %d, expected %d",
              track->asset_name, lap + 1u, driver.progress.advance,
              expected_advance);
        CHECK(!driver.progress.wrong_way,
              "%s reports wrong way while driving the centreline forwards",
              track->asset_name);
    }
    CHECK(driver.progress.best_lap_ms > 0,
          "%s never timed a lap", track->asset_name);

    /* Driving the same path backwards from where the forward drive ended takes
       the advance back down again, one per crossing of the first node, and
       raises the wrong-way flag once the kart is both pointed and moving
       against the course. */
    {
        const int before = driver.progress.advance;
        unsigned int wrong = 0;
        unsigned int samples = 0;
        driver.facing_backwards = true;
        index = path_count;
        while (index-- > 0) {
            if (!drive_to(&driver, path[index], 2.0f)) continue;
            ++samples;
            if (driver.progress.wrong_way) ++wrong;
        }
        driver.facing_backwards = false;
        CHECK(driver.progress.advance < before,
              "%s driving backwards did not lower the advance (%d -> %d)",
              track->asset_name, before, driver.progress.advance);
        /* Not every sample: the -0.5 threshold is a 120-degree cone against
           the straight line between the neighbouring gates, so a tight corner
           clears the flag even while the kart is going backwards. Driving
           forwards never raises it at all, which is checked above. */
        CHECK(samples > 0 && wrong * 4u >= samples * 3u,
              "%s reported the wrong way for only %u of %u samples while driving "
              "the course backwards", track->asset_name, wrong, samples);
    }

    /* 0x00426470's second reason to award progress: a `final` gate, crossed on
       the lap the race ends on. Only ice_R01 marks one, and setting the lap
       count to where the kart already is makes that gate pay out. */
    {
        unsigned int finals = 0;
        for (index = 0; index < course.gate_count; ++index) {
            if (course.gates[index].is_final) ++finals;
        }
        if (strcmp(track->asset_name, "ice_R01") == 0) {
            const int before = driver.progress.advance;
            CHECK(finals == 1, "ice_R01 has %u final gates, expected 1", finals);
            /* The loop crosses the start gate first, so the advance is one
               higher by the time the final gate comes round. */
            kart_course_set_lap_count(&course, (unsigned int)before + 1u);
            for (index = 0; index < path_count; ++index) {
                drive_to(&driver, path[index], 2.0f);
            }
            CHECK(driver.progress.advance == before + 2,
                  "ice_R01 loop on the last lap advanced %d, expected 2 (the "
                  "start gate and the final gate)",
                  driver.progress.advance - before);
        } else {
            CHECK(finals == 0, "%s has %u final gates, expected none",
                  track->asset_name, finals);
        }
    }

    free(path);
    free(order);
    kart_course_free(&course);
}

static void test_gate_crossing(void)
{
    /* A unit square gate in the x = 0 plane, normal +x, as two triangles. */
    KartCourseGate gate;
    gate.face[0][0] = (KartVec3){0.0f, -1.0f, -1.0f};
    gate.face[0][1] = (KartVec3){0.0f, 1.0f, -1.0f};
    gate.face[0][2] = (KartVec3){0.0f, 1.0f, 1.0f};
    gate.face[1][0] = (KartVec3){0.0f, -1.0f, -1.0f};
    gate.face[1][1] = (KartVec3){0.0f, 1.0f, 1.0f};
    gate.face[1][2] = (KartVec3){0.0f, -1.0f, 1.0f};
    gate.normal = (KartVec3){1.0f, 0.0f, 0.0f};
    gate.is_final = false;

    CHECK(kart_course_gate_crossing(&gate, (KartVec3){-1.0f, 0.0f, 0.0f},
                                    (KartVec3){1.0f, 0.0f, 0.0f}) == 1,
          "crossing with the normal is not +1");
    CHECK(kart_course_gate_crossing(&gate, (KartVec3){1.0f, 0.0f, 0.0f},
                                    (KartVec3){-1.0f, 0.0f, 0.0f}) == -1,
          "crossing against the normal is not -1");
    /* Stopping short is not a crossing: the test is over the segment, not the
       infinite ray, which is why a kart nosing up to a gate does not pass it. */
    CHECK(kart_course_gate_crossing(&gate, (KartVec3){-1.0f, 0.0f, 0.0f},
                                    (KartVec3){-0.5f, 0.0f, 0.0f}) == 0,
          "a segment that stops short of the gate counted as a crossing");
    /* Past the edge of the quad. There is no radius anywhere in the mechanism,
       so a kart that drives around the gate misses it entirely. */
    CHECK(kart_course_gate_crossing(&gate, (KartVec3){-1.0f, 2.0f, 0.0f},
                                    (KartVec3){1.0f, 2.0f, 0.0f}) == 0,
          "a segment beside the gate counted as a crossing");
}

int main(void)
{
    unsigned int index;
    test_gate_crossing();
    CHECK(kart_course_asset_count() == 13,
          "expected 13 course assets, found %u", kart_course_asset_count());
    for (index = 0; index < kart_demo_track_count(); ++index) {
        const KartDemoTrackSpec *track = kart_demo_track_at(index);
        if (!track->has_scene) continue;
        test_track(track);
    }
    if (failures != 0) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("course progress tests passed\n");
    return 0;
}
