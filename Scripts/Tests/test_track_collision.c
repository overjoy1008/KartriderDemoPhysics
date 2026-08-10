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
    /* Wound so the floor faces up and the wall faces +X, the way a real asset
       faces its drivable side. Both queries now report the face normal as it
       is, so a face wound the other way would be reported pointing away and
       the resolver would drop it as a separating contact. */
    static uint32_t indices[] = {
        0, 1, 2, 0, 2, 3,
        4, 6, 5, 4, 7, 6,
    };
    /* Flagged the way the exporter flags a mesh whose asset carries a
       `property/road` block. Without the flag nothing here would collide. */
    KartTrackSceneMesh mesh = {
        .flags = KART_TRACK_SCENE_MESH_COLLIDABLE,
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

    /* The ground ray only sees faces the original would let the wheels touch.
       Two slopes through the origin, one either side of the 0.65 limit. */
    {
        /* normal z 0.8: shallow enough, so the wheels find it. */
        static KartTrackSceneVertex shallow_vertices[] = {
            {0.0f, -5.0f,  0.0f, 0, 0},
            {0.0f,  5.0f,  0.0f, 0, 0},
            {1.0f, -5.0f, -0.75f, 0, 0},
        };
        /* normal z 0.5: steeper than the limit, so it is a wall and the ray
           must miss it even though the segment crosses the triangle. */
        static KartTrackSceneVertex steep_vertices[] = {
            {0.0f, -5.0f,  0.0f, 0, 0},
            {0.0f,  5.0f,  0.0f, 0, 0},
            {1.0f, -5.0f, -1.7320508f, 0, 0},
        };
        static uint32_t slope_indices[] = {0, 1, 2};
        KartTrackSceneMesh slope = {
            .flags = KART_TRACK_SCENE_MESH_COLLIDABLE,
            .vertex_count = 3,
            .index_count = 3,
            .vertices = shallow_vertices,
            .indices = slope_indices,
        };
        KartTrackScene slope_scene = {.mesh_count = 1, .meshes = &slope};
        /* Negative X because the scene transform mirrors that axis, so a slope
           authored on +X lands on -X in world space. */
        const KartVec3 start = {-0.5f, -2.0f, 1.0f};
        const KartVec3 delta = {0.0f, 0.0f, -3.0f};

        kart_track_scene_compute_bounds(&slope_scene);
        assert(kart_track_scene_query_ground(
            &slope_scene, &track, start, delta, &ground));
        assert(near(fabsf(ground.normal.z), 0.8f, 0.001f));

        slope.vertices = steep_vertices;
        kart_track_scene_compute_bounds(&slope_scene);
        assert(!kart_track_scene_query_ground(
            &slope_scene, &track, start, delta, &ground));
    }

    /* The body box is oriented, so turning the kart changes what it reaches.
       A wall 1.3 out is past the 1.0 half-width head-on and inside the 1.414
       the same box spans across its diagonal. */
    {
        static KartTrackSceneVertex wall_vertices[] = {
            {1.3f, -5.0f, -1.0f, 0, 0},
            {1.3f,  5.0f, -1.0f, 0, 0},
            {1.3f,  5.0f,  3.0f, 0, 0},
        };
        static uint32_t wall_indices[] = {0, 1, 2};
        KartTrackSceneMesh wall = {
            .flags = KART_TRACK_SCENE_MESH_COLLIDABLE,
            .vertex_count = 3,
            .index_count = 3,
            .vertices = wall_vertices,
            .indices = wall_indices,
        };
        KartTrackScene wall_scene = {.mesh_count = 1, .meshes = &wall};
        const float half_turn = 0.3826834f; /* sin(22.5 deg), a 45 deg yaw */

        kart_track_scene_compute_bounds(&wall_scene);
        kart_simulation_init(&kart, NULL, NULL);
        kart.position = (KartVec3){0.0f, 0.0f, 0.0f};
        assert(kart_track_scene_query_body_collisions(
            &wall_scene, &track, &kart, contacts, 4) == 0);

        kart.orientation = (KartQuat){0.9238795f, 0.0f, 0.0f, half_turn};
        assert(kart_track_scene_query_body_collisions(
            &wall_scene, &track, &kart, contacts, 4) == 1);
    }

    /* A shallow face reaches the body query too. The original applies no
       steepness filter there, and the normal arrives unflattened, which is what
       lets the resolver take its landing branch instead of its wall branch. */
    {
        static KartTrackSceneVertex floor_vertices[] = {
            {-5.0f, -5.0f, 1.0f, 0, 0},
            { 5.0f, -5.0f, 1.0f, 0, 0},
            { 5.0f,  5.0f, 1.0f, 0, 0},
        };
        static uint32_t floor_indices[] = {0, 1, 2};
        KartTrackSceneMesh floor = {
            .flags = KART_TRACK_SCENE_MESH_COLLIDABLE,
            .vertex_count = 3,
            .index_count = 3,
            .vertices = floor_vertices,
            .indices = floor_indices,
        };
        KartTrackScene floor_scene = {.mesh_count = 1, .meshes = &floor};

        kart_track_scene_compute_bounds(&floor_scene);
        kart_simulation_init(&kart, NULL, NULL);
        kart.position = (KartVec3){0.0f, 0.0f, 0.0f};
        count = kart_track_scene_query_body_collisions(
            &floor_scene, &track, &kart, contacts, 4);
        assert(count == 1);
        assert(contacts[0].normal.z > 0.99f);
        assert(near(contacts[0].point.z, 1.0f, 0.0001f));

        /* Drop the same face below the box and it stops being a contact: the
           box sits one unit up the chassis axis with a 0.7 half-height, so it
           does not scrape the road it is driving on. */
        floor_vertices[0].z = 0.0f;
        floor_vertices[1].z = 0.0f;
        floor_vertices[2].z = 0.0f;
        kart_track_scene_compute_bounds(&floor_scene);
        assert(kart_track_scene_query_body_collisions(
            &floor_scene, &track, &kart, contacts, 4) == 0);
    }

    /* Scenery is invisible to both queries. The original never puts a mesh
       without a `road` tag into its collision grid, so a tree or a signpost is
       driven through rather than hit. */
    {
        mesh.flags = 0;
        assert(!kart_track_scene_query_ground(
            &scene, &track,
            (KartVec3){2.0f, 0.0f, 0.5f},
            (KartVec3){0.0f, 0.0f, -1.0f},
            &ground));
        kart_simulation_init(&kart, NULL, NULL);
        kart.position = (KartVec3){0.5f, 0.0f, 0.0f};
        kart.linear_velocity = (KartVec3){-10.0f, 0.0f, 0.0f};
        assert(kart_track_scene_query_body_collisions(
            &scene, &track, &kart, contacts, 4) == 0);
        mesh.flags = KART_TRACK_SCENE_MESH_COLLIDABLE;
    }

    puts("kart_track_collision_tests: ok");
    return 0;
}
