#include "kart_track_collision.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static int near(float actual, float expected, float epsilon)
{
    return fabsf(actual - expected) <= epsilon;
}

int main(void)
{
    static KartTrackSceneVertex vertices[] = {
        {-5.0f, -5.0f, 0.0f, 0, 0},
        { 5.0f, -5.0f, 0.0f, 0, 0},
        { 5.0f,  5.0f, 0.0f, 0, 0},
        {-5.0f,  5.0f, 0.0f, 0, 0},
        { 0.0f, -5.0f,-1.0f, 0, 0},
        { 0.0f,  5.0f,-1.0f, 0, 0},
        { 0.0f,  5.0f, 2.0f, 0, 0},
        { 0.0f, -5.0f, 2.0f, 0, 0},
    };
    static uint32_t indices[] = {
        0, 1, 2, 0, 2, 3,
        4, 5, 6, 4, 6, 7,
    };
    /* Deliberately flagged as scenery: the name-based road/wall classifier is
       only a hint, and every mesh in a scene is solid. */
    KartTrackSceneMesh mesh = {
        .flags = 0,
        .vertex_count = 8,
        .index_count = 12,
        .vertices = vertices,
        .indices = indices,
    };
    KartTrackScene scene = {
        .mesh_count = 1,
        .meshes = &mesh,
    };
    const KartDemoTrackSpec track = {
        .asset_name = "synthetic",
        .display_name = "synthetic",
        .race_mode = "test",
        .difficulty = 0,
        .minimum = {-5.0f, -5.0f, 0.0f},
        .maximum = { 5.0f,  5.0f, 2.0f},
    };
    KartGroundHit ground;
    KartSimulationState kart;
    KartBodyContact contacts[4];
    unsigned int count;

    /* A hand-built scene has no bounds until this is called, and the queries
       reject meshes by their bounds. */
    kart_track_scene_compute_bounds(&scene);
    assert(near(mesh.minimum[0], -5.0f, 0.0001f));
    assert(near(mesh.maximum[2], 2.0f, 0.0001f));

    assert(kart_track_scene_query_ground(
        &scene, &track,
        (KartVec3){2.0f, 0.0f, 0.5f},
        (KartVec3){0.0f, 0.0f, -1.0f},
        &ground));
    assert(near(ground.point.z, 0.0f, 0.0001f));
    assert(ground.normal.z > 0.99f);
    assert(!kart_track_scene_query_ground(
        &scene, &track,
        (KartVec3){8.0f, 0.0f, 0.5f},
        (KartVec3){0.0f, 0.0f, -1.0f},
        &ground));

    kart_simulation_init(&kart, NULL, NULL);
    kart.position = (KartVec3){0.5f, 0.0f, 0.0f};
    kart.linear_velocity = (KartVec3){-10.0f, 0.0f, 0.0f};
    count = kart_track_scene_query_body_collisions(
        &scene, &track, &kart, contacts, 4);
    assert(count == 1);
    assert(contacts[0].normal.x > 0.99f);
    assert(contacts[0].surface_id == 4);

    kart.position.x = 3.0f;
    count = kart_track_scene_query_body_collisions(
        &scene, &track, &kart, contacts, 4);
    assert(count == 0);

    puts("kart_track_collision_tests: ok");
    return 0;
}
