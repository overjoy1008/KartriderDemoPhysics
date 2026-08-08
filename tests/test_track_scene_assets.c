/* End-to-end check over the real packed track scenes.

   Walks every track in TRACKS[], loads its KTKZ resource from disk, and casts a
   wheel-suspension ray straight down at the recorded spawn. This exercises the
   whole chain the demos rely on: the container, the inflate, the KTRK loader,
   the asset-to-world transform, and the road-triangle query.

   Registered only when the exports are present, since they are not tracked. */

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
    const char *packed_dir = argc > 1 ? argv[1] : "analysis/track-assets/packed";
    const unsigned int count = kart_demo_track_count();
    unsigned int i;
    unsigned int grounded = 0;

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
           track, which the fall limit catches. */
        {
            DriveWorld bound = {&scene, track};
            const KartSimulationWorld world = {
                drive_query_ground, drive_query_body, &bound};
            KartSimulationControls controls = {0};
            KartSimulationState kart;
            const float fall_limit = kart_demo_track_fall_limit(track);
            unsigned int grounded_steps = 0;
            unsigned int step;

            kart_simulation_init(&kart, NULL, NULL);
            if (kart_demo_track_start_position(track, &kart.position)) {
                kart_demo_track_start_orientation(track, &kart.orientation);
            }
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
            }
            if (grounded_steps * 2u < 100u) {
                printf("FAIL %s: grounded for only %u of 100 steps\n",
                       track->asset_name, grounded_steps);
                return 1;
            }
            printf("ok   %-12s meshes=%4u road=%3u tris=%6u grounded %3u/100 %s\n",
                   track->asset_name, scene.mesh_count, road_meshes,
                   scene.total_triangle_count, grounded_steps,
                   kart_demo_track_start_kind_label(track));
        }
        kart_track_scene_free(&scene);
    }

    /* All 13 real tracks have a start quad, and every mesh is collidable, so
       the ray must land for each of them. */
    printf("%u of %u tracks grounded at their start line\n", grounded, count);
    assert(grounded == 13);
    return 0;
}
