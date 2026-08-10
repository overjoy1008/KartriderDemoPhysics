/* End-to-end check over the real packed track scenes.

   Walks every track in TRACKS[], loads its KTKZ resource from disk, and casts a
   wheel-suspension ray straight down at the recorded spawn. This exercises the
   whole chain the demos rely on: the container, the inflate, the KTRK loader,
   the asset-to-world transform, and the road-triangle query.

   Registered only when the exports are present, since they are not tracked. */

#include "kart_course.h"
#include "kart_demo_data.h"
#include "kart_track_collision.h"
#include "kart_track_scene.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Binds one loaded scene to the simulation's world callbacks so a track can be
   driven headlessly. */
typedef struct DriveWorld {
    const KartTrackScene *scene;
    const KartDemoTrackSpec *track;
} DriveWorld;

static bool drive_query_ground(
    void *user_data,
    KartVec3 start,
    KartVec3 delta,
    KartGroundHit *hit)
{
    const DriveWorld *world = (const DriveWorld *)user_data;
    return kart_track_scene_query_ground(
        world->scene, world->track, start, delta, hit);
}

static unsigned int drive_query_body(
    void *user_data,
    const KartSimulationState *state,
    KartBodyContact *contacts,
    unsigned int capacity)
{
    const DriveWorld *world = (const DriveWorld *)user_data;
    return kart_track_scene_query_body_collisions(
        world->scene, world->track, state, contacts, capacity);
}

static unsigned char *read_file(const char *path, size_t *size)
{
    FILE *file = fopen(path, "rb");
    unsigned char *data;
    long length;
    if (file == NULL) return NULL;
    if (fseek(file, 0, SEEK_END) != 0) { fclose(file); return NULL; }
    length = ftell(file);
    if (length <= 0 || fseek(file, 0, SEEK_SET) != 0) { fclose(file); return NULL; }
    data = (unsigned char *)malloc((size_t)length);
    if (data == NULL || fread(data, 1, (size_t)length, file) != (size_t)length) {
        free(data);
        fclose(file);
        return NULL;
    }
    fclose(file);
    *size = (size_t)length;
    return data;
}

int main(int argc, char **argv)
{
    const char *packed_dir = argc > 1 ? argv[1] : "Assets/Tracks/packed";
    const unsigned int count = kart_demo_track_count();
    unsigned int i;
    unsigned int grounded = 0;
    unsigned int course_grounded = 0;

    for (i = 0; i < count; ++i) {
        const KartDemoTrackSpec *track = kart_demo_track_at(i);
        char path[1024];
        unsigned char *data;
        size_t size = 0;
        KartTrackScene scene;
        KartVec3 start;
        KartGroundHit hit;
        unsigned int mesh;
        unsigned int road_meshes = 0;

        if (!track->has_scene) {
            printf("skip %-12s synthetic, no decoded mesh\n", track->asset_name);
            continue;
        }
        snprintf(path, sizeof(path), "%s/track_%s.ktkz", packed_dir,
                 track->asset_name);
        data = read_file(path, &size);
        if (data == NULL) {
            printf("missing %s\n", path);
            return 1;
        }
        memset(&scene, 0, sizeof(scene));
        if (!kart_track_scene_load_compressed(&scene, data, size)) {
            printf("FAIL %s: container rejected\n", track->asset_name);
            free(data);
            return 1;
        }
        free(data);

        assert(scene.mesh_count > 0);
        assert(scene.total_triangle_count > 0);
        for (mesh = 0; mesh < scene.mesh_count; ++mesh) {
            const KartTrackSceneMesh *entry = &scene.meshes[mesh];
            if (entry->flags & 1u) ++road_meshes;
            /* Bounds are derived at load; every non-empty mesh needs usable
               ones or the collision queries would skip it. */
            if (entry->vertex_count != 0) {
                assert(entry->minimum[0] <= entry->maximum[0]);
                assert(entry->minimum[1] <= entry->maximum[1]);
                assert(entry->minimum[2] <= entry->maximum[2]);
                assert(entry->minimum[0] >= scene.minimum[0] - 0.01f);
                assert(entry->maximum[0] <= scene.maximum[0] + 0.01f);
            }
        }
        assert(road_meshes > 0);

        /* The KTRK header bounds must be the AABB the track table carries, or
           the scene would sit off-centre from its own walls and minimap. */
        assert(fabsf(scene.minimum[0] - track->minimum.x) < 0.01f);
        assert(fabsf(scene.minimum[1] - track->minimum.y) < 0.01f);
        assert(fabsf(scene.minimum[2] - track->minimum.z) < 0.01f);
        assert(fabsf(scene.maximum[0] - track->maximum.x) < 0.01f);
        assert(fabsf(scene.maximum[1] - track->maximum.y) < 0.01f);
        assert(fabsf(scene.maximum[2] - track->maximum.z) < 0.01f);

        if (kart_demo_track_start_position(track, &start)) {
            /* The demos cast a 1.0-long ray down from each wheel; start a
               little above the plane the spawn sits on. */
            const KartVec3 origin = {start.x, start.y, 0.4f};
            const KartVec3 delta = {0.0f, 0.0f, -1.0f};
            if (kart_track_scene_query_ground(&scene, track, origin, delta, &hit)) {
                ++grounded;
                /* Positive, not merely non-zero. The query reports the face
                   normal as the original does, without turning it upward, so a
                   negative z here would mean the export's winding is inverted
                   and the suspension would be pulling the kart into the road.
                   At or past 0.65 because that is the filter the ray applies. */
                assert(hit.normal.z >= 0.649f);
                assert(hit.point.z <= 0.4f && hit.point.z >= -0.6f);
            } else {
                printf("note %-12s: no road hit at the recorded start line\n",
                       track->asset_name);
            }
        }

        /* Drive it. Grounding once at the spawn says the ray works; holding the
           throttle for two seconds says the suspension, the body box and the
           collision response all agree well enough to keep the kart on the
           road. A sign error in any of them ends with the kart under the
           track, which the fall limit catches.

           The pose comes from the course now, not from the start-line mesh.
           That is what settles which way round the lap is driven: the mesh
           never said, and a kart pointed the wrong way would drive away from
           the first checkpoint, which the gate count below catches. */
        {
            DriveWorld bound = {&scene, track};
            const KartSimulationWorld world = {
                drive_query_ground, drive_query_body, &bound};
            KartSimulationControls controls = {0};
            KartSimulationState kart;
            const float fall_limit = kart_demo_track_fall_limit(track);
            const KartCourseAsset *asset =
                kart_course_find_asset(track->asset_name);
            KartCourse course;
            KartCourseProgress progress;
            unsigned int grounded_steps = 0;
            unsigned int wrong_way_steps = 0;
            unsigned int gates = 0;
            unsigned int step;
            KartVec3 previous;

            if (asset == NULL || !kart_course_build(&course, asset)) {
                printf("FAIL %s: no usable course\n", track->asset_name);
                return 1;
            }
            kart_course_set_lap_count(&course, 3u);
            kart_simulation_init(&kart, NULL, NULL);
            kart_course_start_pose(&course, 0u, &kart.position, &kart.orientation);
            /* 0x004260e0 finishes the grid placement with a ray from 10 above
               the slot, 100 down. */
            {
                const KartVec3 above = {
                    kart.position.x, kart.position.y, kart.position.z + 10.0f};
                const KartVec3 down = {0.0f, 0.0f, -100.0f};
                if (kart_track_scene_query_ground(&scene, track, above, down, &hit)) {
                    ++course_grounded;
                    assert(hit.normal.z >= 0.649f);
                    kart.position = hit.point;
                } else {
                    printf("note %-12s: the start grid snap found no road\n",
                           track->asset_name);
                }
            }
            kart_course_progress_init(&course, &progress, kart.position);
            previous = kart.position;
            kart.position.z += 0.5f;
            controls.forward_input = 1.0f;
            for (step = 0; step < 100; ++step) {
                kart_simulate_milliseconds(&kart, &controls, &world, 20);
                if (kart.grounded) ++grounded_steps;
                if (kart.position.z < fall_limit) {
                    printf("FAIL %s: fell through at step %u (z %.2f < %.2f)\n",
                           track->asset_name, step, kart.position.z, fall_limit);
                    return 1;
                }
                if (kart_course_progress_step(
                        &course, &progress, previous, kart.position,
                        kart.orientation, kart.linear_velocity,
                        (step + 1u) * 20u) != 0) {
                    ++gates;
                }
                if (progress.wrong_way) ++wrong_way_steps;
                previous = kart.position;
            }
            if (grounded_steps * 2u < 100u) {
                printf("FAIL %s: grounded for only %u of 100 steps\n",
                       track->asset_name, grounded_steps);
                return 1;
            }
            /* Two seconds of throttle from the line always clears the start
               gate, which sits half a unit ahead of the spawn. Pointed the
               other way the kart would never reach it. */
            if (gates == 0) {
                printf("FAIL %s: two seconds from the line crossed no gate\n",
                       track->asset_name);
                return 1;
            }
            if (wrong_way_steps != 0) {
                printf("FAIL %s: reported the wrong way for %u of 100 steps "
                       "while driving away from the line\n",
                       track->asset_name, wrong_way_steps);
                return 1;
            }
            printf("ok   %-12s meshes=%4u road=%3u tris=%6u grounded %3u/100 "
                   "nodes=%3u lap=%u node=%u\n",
                   track->asset_name, scene.mesh_count, road_meshes,
                   scene.total_triangle_count, grounded_steps,
                   course.node_count, progress.lap, progress.node_id);
            kart_course_free(&course);
        }
        kart_track_scene_free(&scene);
    }

    /* All 13 real tracks have a start quad, and every mesh is collidable, so
       the ray must land for each of them. */
    printf("%u of %u tracks grounded at their start line, %u at their start "
           "grid\n", grounded, count, course_grounded);
    assert(grounded == 13);
    assert(course_grounded == 13);
    return 0;
}
