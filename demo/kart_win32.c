#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "kart_axis_gizmo_win32.h"
#include "kart_camera.h"
#include "kart_countdown.h"
#include "kart_demo_data.h"
#include "kart_demo_win32_ui.h"
#include "kart_gauge.h"
#include "kart_gearbox.h"
#include "kart_input.h"
#include "kart_minimap_win32.h"
#include "kart_model_resources.h"
#include "kart_model_win32.h"
#include "kart_params_win32.h"
#include "kart_screenshot_win32.h"
#include "kart_simulation.h"
#include "kart_sound_win32.h"
#include "kart_track_collision.h"
#include "kart_track_scene.h"
#include "kart_track_scene_resources.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_SKID_SEGMENTS 2048
#define SKID_LIFETIME_MS 15000

/* Half-extents of the top-down view's window onto the world, in metres. */
static const float VIEW_HALF_WIDTH = 55.0f;
static const float VIEW_HALF_HEIGHT = 38.0f;

/* The two demos were the same simulation behind two renderers, so they are one
   executable now and V switches between them. */
typedef enum DemoViewMode {
    DEMO_VIEW_CHASE = 0,
    DEMO_VIEW_TOPDOWN = 1
} DemoViewMode;

typedef struct SkidSegment3D {
    float left_x0;
    float left_y0;
    float left_x1;
    float left_y1;
    float right_x0;
    float right_y0;
    float right_x1;
    float right_y1;
    unsigned int created_ms;
} SkidSegment3D;

typedef struct Demo3DState {
    KartSimulationState kart;
    const KartDemoKartSpec *kart_spec;
    const KartDemoTrackSpec *track_spec;
    DWORD previous_tick;
    unsigned int simulation_time_ms;
    unsigned int skid_head;
    unsigned int skid_count;
    SkidSegment3D skid_segments[MAX_SKID_SEGMENTS];
    float previous_skid_left_x;
    float previous_skid_left_y;
    float previous_skid_right_x;
    float previous_skid_right_y;
    DemoViewMode view_mode;
    bool view_key_was_down;
    bool vector_key_was_down;
    bool param_key_was_down;
    bool gauge_key_was_down;
    bool boost_press_allowed;
    bool show_vectors;
    KartGaugeConfig gauge_config;
    KartGaugeState gauge;
    KartGearbox gearbox;
    bool gear_key_was_down;
    /* Last step's telemetry, kept so the panel can show what the simulation
       actually did rather than only what its state ended up as. */
    KartSimulationStepResult last_step;
    KartVec3 previous_velocity;
    KartVec3 acceleration;
    bool kart_key_was_down;
    bool track_key_was_down;
    bool drag_key_was_down;
    bool shot_key_was_down;
    bool drag_trigger_active;
    bool previous_skid_active;
    bool boost_active;
    KartSteeringInputState steering;
    /* Counts down after an out-of-world respawn so the HUD can say why the
       kart moved. */
    DWORD respawn_notice_ms;
    /* Counts down after a screenshot so the HUD can show where it went. */
    DWORD shot_notice_ms;
    char shot_name[64];
    /* Recovered chase camera; advanced with the same elapsed time as the
       simulation. */
    KartChaseCameraFollow camera_follow;
    KartChaseCameraPose camera_pose;
    KartDemoSound sound;
    KartCountdown countdown;
    unsigned int countdown_remaining_ms;
    /* Counts down after GO so the START flash outlives the single cue frame. */
    DWORD start_notice_ms;
    /* Drift plus throttle held on the line: booster idle loop and boost look. */
    bool countdown_rev_active;
    bool forward_key_was_down;
    KartTrackScene scenes[KART_TRACK_SCENE_CAPACITY];
    /* The recovered kart models, in KARTS[] order. Loaded once up front: all 26
       together are about 340 KB of geometry, so there is nothing to gain from
       loading them on demand when the driver presses K. */
    KartTrackScene models[KART_MODEL_CAPACITY];
    KartModelParts model_parts[KART_MODEL_CAPACITY];
    bool show_model;
    bool show_model_bounds;
    bool model_key_was_down;
    bool bounds_key_was_down;
    KartDemoMinimapSet minimaps;
} Demo3DState;

typedef struct Camera3D {
    KartVec3 position;
    KartVec3 right;
    KartVec3 up;
    KartVec3 forward;
    float focal_length;
} Camera3D;

typedef struct ProjectedPoint {
    POINT point;
    float depth;
    bool visible;
} ProjectedPoint;

static bool load_scene_resource(
    HINSTANCE instance,
    int resource_id,
    KartTrackScene *scene)
{
    HRSRC resource = FindResourceA(
        instance, MAKEINTRESOURCEA(resource_id), RT_RCDATA);
    HGLOBAL loaded;
    const void *data;
    DWORD size;
    if (resource == NULL) {
        return false;
    }
    loaded = LoadResource(instance, resource);
    size = SizeofResource(instance, resource);
    data = loaded != NULL ? LockResource(loaded) : NULL;
    return data != NULL && size != 0 &&
           kart_track_scene_load_compressed(scene, data, (size_t)size);
}

static void load_track_scenes(HINSTANCE instance, Demo3DState *demo)
{
#if defined(KART_EMBED_TRACK_SCENES)
    unsigned int i;
    const unsigned int count = kart_demo_track_count();
    for (i = 0; i < count && i < KART_TRACK_SCENE_CAPACITY; ++i) {
        /* Id 0 marks a track with no mesh, such as the flat test track. */
        if (KART_TRACK_SCENE_RESOURCE_IDS[i] == 0) continue;
        load_scene_resource(
            instance, KART_TRACK_SCENE_RESOURCE_IDS[i], &demo->scenes[i]);
    }
#else
    (void)instance;
    (void)demo;
#endif
}

/* The kart models ride in the same KTKZ containers as the track scenes, so the
   same loader reads them; only what the parts mean differs. */
static void load_kart_models(HINSTANCE instance, Demo3DState *demo)
{
#if defined(KART_EMBED_KART_MODELS)
    unsigned int i;
    const unsigned int count = kart_demo_kart_count();
    for (i = 0; i < count && i < KART_MODEL_CAPACITY; ++i) {
        if (!load_scene_resource(
                instance, KART_MODEL_RESOURCE_IDS[i], &demo->models[i])) {
            continue;
        }
        kart_model_parts_build(&demo->models[i], &demo->model_parts[i]);
    }
#else
    (void)instance;
    (void)demo;
#endif
}

static void free_track_scenes(Demo3DState *demo)
{
    unsigned int i;
    for (i = 0; i < KART_TRACK_SCENE_CAPACITY; ++i) {
        kart_track_scene_free(&demo->scenes[i]);
    }
    for (i = 0; i < KART_MODEL_CAPACITY; ++i) {
        kart_track_scene_free(&demo->models[i]);
    }
}

/* The model for the kart currently under the driver, or NULL when the models
   were not embedded. */
static const KartTrackScene *active_kart_model(
    const Demo3DState *demo,
    const KartModelParts **parts)
{
    unsigned int i;
    const unsigned int count = kart_demo_kart_count();
    for (i = 0; i < count && i < KART_MODEL_CAPACITY; ++i) {
        if (kart_demo_kart_at(i) != demo->kart_spec) continue;
        if (demo->models[i].mesh_count == 0) break;
        if (parts != NULL) *parts = &demo->model_parts[i];
        return &demo->models[i];
    }
    if (parts != NULL) *parts = NULL;
    return NULL;
}

static const KartTrackScene *active_track_scene(const Demo3DState *demo)
{
    unsigned int i;
    const unsigned int count = kart_demo_track_count();
    for (i = 0; i < count && i < KART_TRACK_SCENE_CAPACITY; ++i) {
        if (kart_demo_track_at(i) == demo->track_spec) {
            return demo->scenes[i].mesh_count != 0 ? &demo->scenes[i] : NULL;
        }
    }
    return NULL;
}

static KartVec3 vec_add(KartVec3 a, KartVec3 b)
{
    return (KartVec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

static KartVec3 vec_sub(KartVec3 a, KartVec3 b)
{
    return (KartVec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

static KartVec3 vec_scale(KartVec3 value, float amount)
{
    return (KartVec3){value.x * amount, value.y * amount, value.z * amount};
}

static float vec_dot(KartVec3 a, KartVec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static KartVec3 vec_cross(KartVec3 a, KartVec3 b)
{
    return (KartVec3){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

static KartVec3 vec_normalize(KartVec3 value)
{
    const float length = sqrtf(vec_dot(value, value));
    return length > 0.0f ? vec_scale(value, 1.0f / length) : (KartVec3){0};
}

static void orientation_axes(
    KartQuat q,
    KartVec3 *right,
    KartVec3 *forward,
    KartVec3 *up)
{
    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;
    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;
    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;

    *right = (KartVec3){
        1.0f - 2.0f * (yy + zz),
        2.0f * (xy + wz),
        2.0f * (xz - wy),
    };
    *forward = (KartVec3){
        -2.0f * (xy - wz),
        -(1.0f - 2.0f * (xx + zz)),
        -2.0f * (yz + wx),
    };
    *up = (KartVec3){
        2.0f * (xz + wy),
        2.0f * (yz - wx),
        1.0f - 2.0f * (xx + yy),
    };
}

static bool query_flat_ground(
    void *user_data,
    KartVec3 start,
    KartVec3 delta,
    KartGroundHit *hit)
{
    float fraction;
    (void)user_data;
    if (delta.z >= 0.0f || start.z < 0.0f || start.z + delta.z > 0.0f) {
        return false;
    }
    fraction = -start.z / delta.z;
    hit->point = (KartVec3){
        start.x + delta.x * fraction,
        start.y + delta.y * fraction,
        0.0f,
    };
    hit->normal = (KartVec3){0.0f, 0.0f, 1.0f};
    hit->surface_id = 1;
    return true;
}

static bool query_track_ground(
    void *user_data,
    KartVec3 start,
    KartVec3 delta,
    KartGroundHit *hit)
{
    const Demo3DState *demo = (const Demo3DState *)user_data;
    const KartTrackScene *scene = active_track_scene(demo);
    if (scene != NULL) {
        return kart_track_scene_query_ground(
            scene, demo->track_spec, start, delta, hit);
    }
    return query_flat_ground(user_data, start, delta, hit);
}

static unsigned int query_track_walls(
    void *user_data,
    const KartSimulationState *state,
    KartBodyContact *contacts,
    unsigned int capacity)
{
    const Demo3DState *demo = (const Demo3DState *)user_data;
    const float track_half_width = kart_demo_track_width(demo->track_spec) * 0.5f;
    const float track_half_height = kart_demo_track_length(demo->track_spec) * 0.5f;
    const KartTrackScene *scene = active_track_scene(demo);
    if (scene != NULL) {
        /* A decoded track collides against its own geometry and nothing else.
           The original has no invisible box round the level; driving off the
           edge is caught by the fall limit, not by a wall. */
        return kart_track_scene_query_body_collisions(
            scene, demo->track_spec, state, contacts, capacity);
    }
    /* Only the synthetic flat track reaches here. It has no mesh at all, so its
       AABB is the only thing that can contain the kart. */
    unsigned int count = 0;
#define ADD_CONTACT(nx, ny) do { \
    if (count < capacity) { \
        contacts[count].normal = (KartVec3){(nx), (ny), 0.0f}; \
        contacts[count].point = state->position; \
        contacts[count].sweep_fraction = 0.5f; \
        contacts[count].surface_id = 2; \
        count += 1; \
    } \
} while (0)
    if (state->position.x >= track_half_width && state->linear_velocity.x > 0.0f) {
        ADD_CONTACT(-1.0f, 0.0f);
    }
    if (state->position.x <= -track_half_width && state->linear_velocity.x < 0.0f) {
        ADD_CONTACT(1.0f, 0.0f);
    }
    if (state->position.y >= track_half_height && state->linear_velocity.y > 0.0f) {
        ADD_CONTACT(0.0f, -1.0f);
    }
    if (state->position.y <= -track_half_height && state->linear_velocity.y < 0.0f) {
        ADD_CONTACT(0.0f, 1.0f);
    }
#undef ADD_CONTACT
    return count;
}

/* GetAsyncKeyState is global, so without this gate the kart would keep driving
   while the user is typing in another application. */
static bool demo_has_focus(HWND window)
{
    const HWND foreground = GetForegroundWindow();
    return foreground == window || IsChild(window, foreground);
}

static HWND g_input_window = NULL;

static int key_down(int virtual_key)
{
    if (g_input_window != NULL && !demo_has_focus(g_input_window)) return 0;
    return (GetAsyncKeyState(virtual_key) & 0x8000) != 0;
}

static void reset_kart(Demo3DState *demo)
{
    KartVec3 start_position;
    KartQuat start_orientation;
    if (demo->kart_spec == NULL) {
        demo->kart_spec = kart_demo_default_kart();
    }
    if (demo->track_spec == NULL) {
        demo->track_spec = kart_demo_default_track();
    }
    kart_simulation_init(
        &demo->kart, &demo->kart_spec->dynamics, &demo->kart_spec->geometry);
    if (kart_demo_track_start_position(demo->track_spec, &start_position)) {
        demo->kart.position = start_position;
    }
    if (kart_demo_track_start_orientation(demo->track_spec, &start_orientation)) {
        demo->kart.orientation = start_orientation;
    }
    demo->previous_tick = GetTickCount();
    demo->simulation_time_ms = 0;
    demo->skid_head = 0;
    demo->skid_count = 0;
    demo->previous_skid_active = false;
    demo->boost_active = false;
    demo->previous_velocity = demo->kart.linear_velocity;
    demo->acceleration = (KartVec3){0.0f, 0.0f, 0.0f};
    demo->drag_trigger_active = false;
    /* The original spends the first 4 s of the 7 s countdown on the intro
       camera sweep. The demo has no intro, so the arming time is backdated to
       where that sweep ends and the digits appear about a second after a reset;
       the deadline the boost window is measured against is unchanged. Only the
       first reset arms it; R and respawns put the kart back on the line without
       running the lights again. */
    if (!demo->countdown.armed) {
        kart_countdown_start(
            &demo->countdown,
            demo->previous_tick - KART_DEMO_COUNTDOWN_PREROLL_MS);
        demo->start_notice_ms = 0;
    }
    /* The original snaps the camera after a reset rather than swinging to the
       new pose over 400 ms. */
    kart_chase_camera_follow_reset(&demo->camera_follow);
    demo->camera_pose = kart_chase_camera_update(
        &demo->camera_follow, demo->kart.position, demo->kart.orientation,
        0.0f, false, 0, KART_CHASE_FOLLOW_OVERHEAD_MS);
}

static bool drift_visual_active(const KartSimulationState *kart)
{
    return kart->drift.input_active || kart->drift.trigger_active ||
           kart->drift.slip_detected || kart->drift.linger_timer > 0.0f;
}

static void update_skid_marks(Demo3DState *demo)
{
    KartVec3 right;
    KartVec3 forward;
    KartVec3 up;
    float speed;
    float rear_offset;
    float side_offset;
    float rear_x;
    float rear_y;
    float left_x;
    float left_y;
    float right_x;
    float right_y;
    bool active;

    orientation_axes(demo->kart.orientation, &right, &forward, &up);
    (void)up;
    speed = sqrtf(
        demo->kart.linear_velocity.x * demo->kart.linear_velocity.x +
        demo->kart.linear_velocity.y * demo->kart.linear_velocity.y);
    rear_offset = demo->kart.geometry.half_length * 0.8f;
    side_offset = demo->kart.geometry.half_width * 0.8f;
    rear_x = demo->kart.position.x - forward.x * rear_offset;
    rear_y = demo->kart.position.y - forward.y * rear_offset;
    left_x = rear_x - right.x * side_offset;
    left_y = rear_y - right.y * side_offset;
    right_x = rear_x + right.x * side_offset;
    right_y = rear_y + right.y * side_offset;
    active = demo->kart.grounded && speed > 5.0f &&
             drift_visual_active(&demo->kart);

    if (active && demo->previous_skid_active) {
        SkidSegment3D *segment = &demo->skid_segments[demo->skid_head];
        *segment = (SkidSegment3D){
            .left_x0 = demo->previous_skid_left_x,
            .left_y0 = demo->previous_skid_left_y,
            .left_x1 = left_x,
            .left_y1 = left_y,
            .right_x0 = demo->previous_skid_right_x,
            .right_y0 = demo->previous_skid_right_y,
            .right_x1 = right_x,
            .right_y1 = right_y,
            .created_ms = demo->simulation_time_ms,
        };
        demo->skid_head = (demo->skid_head + 1) % MAX_SKID_SEGMENTS;
        if (demo->skid_count < MAX_SKID_SEGMENTS) {
            demo->skid_count += 1;
        }
    }

    demo->previous_skid_left_x = left_x;
    demo->previous_skid_left_y = left_y;
    demo->previous_skid_right_x = right_x;
    demo->previous_skid_right_y = right_y;
    demo->previous_skid_active = active;
}

/* The chase pose comes from the recovered ChaseCameraman update; this only
   turns its field of view into the projection's focal length. */
static Camera3D make_chase_camera(const KartChaseCameraPose *pose, RECT client)
{
    Camera3D camera;
    const float height = (float)(client.bottom - client.top);
    camera.position = pose->position;
    camera.right = pose->right;
    camera.up = pose->up;
    camera.forward = pose->forward;
    camera.focal_length =
        kart_chase_camera_focal_length(pose->field_of_view_degrees, height);
    return camera;
}

static ProjectedPoint project_point(RECT client, Camera3D camera, KartVec3 world)
{
    const KartVec3 relative = vec_sub(world, camera.position);
    ProjectedPoint result;
    const float x = vec_dot(relative, camera.right);
    const float y = vec_dot(relative, camera.up);
    result.depth = vec_dot(relative, camera.forward);
    result.visible = result.depth > 0.1f;
    if (result.visible) {
        result.point.x = (LONG)((client.right + client.left) * 0.5f +
                                camera.focal_length * x / result.depth);
        result.point.y = (LONG)((client.bottom + client.top) * 0.52f -
                                camera.focal_length * y / result.depth);
    } else {
        result.point = (POINT){0, 0};
    }
    return result;
}

static void draw_line_3d(
    HDC dc,
    RECT client,
    Camera3D camera,
    KartVec3 a,
    KartVec3 b)
{
    const float near_depth = 0.15f;
    float depth_a = vec_dot(vec_sub(a, camera.position), camera.forward);
    float depth_b = vec_dot(vec_sub(b, camera.position), camera.forward);
    ProjectedPoint pa;
    ProjectedPoint pb;

    if (depth_a <= near_depth && depth_b <= near_depth) {
        return;
    }
    if (depth_a <= near_depth) {
        const float t = (near_depth - depth_a) / (depth_b - depth_a);
        a = vec_add(a, vec_scale(vec_sub(b, a), t));
        depth_a = near_depth;
    } else if (depth_b <= near_depth) {
        const float t = (near_depth - depth_b) / (depth_a - depth_b);
        b = vec_add(b, vec_scale(vec_sub(a, b), t));
        depth_b = near_depth;
    }
    (void)depth_a;
    (void)depth_b;
    pa = project_point(client, camera, a);
    pb = project_point(client, camera, b);
    if (pa.visible && pb.visible) {
        MoveToEx(dc, pa.point.x, pa.point.y, NULL);
        LineTo(dc, pb.point.x, pb.point.y);
    }
}

static void draw_track_scene_pass(
    HDC dc,
    RECT client,
    Camera3D camera,
    const KartDemoTrackSpec *track,
    const KartTrackScene *scene,
    bool collision_candidates)
{
    const float draw_radius = 210.0f;
    const float draw_radius_squared = draw_radius * draw_radius;
    uint32_t mesh_index;

    for (mesh_index = 0; mesh_index < scene->mesh_count; ++mesh_index) {
        const KartTrackSceneMesh *mesh = &scene->meshes[mesh_index];
        const bool candidate =
            (mesh->flags & KART_TRACK_SCENE_MESH_COLLIDABLE) != 0;
        const uint32_t triangle_count = mesh->index_count / 3u;
        const uint32_t stride = candidate ? 1u : 5u;
        uint32_t triangle;
        if (candidate != collision_candidates) {
            continue;
        }
        for (triangle = 0; triangle < triangle_count; triangle += stride) {
            const uint32_t base = triangle * 3u;
            const KartVec3 a = kart_track_scene_world_vertex(
                &mesh->vertices[mesh->indices[base]], track);
            const KartVec3 b = kart_track_scene_world_vertex(
                &mesh->vertices[mesh->indices[base + 1u]], track);
            const KartVec3 c = kart_track_scene_world_vertex(
                &mesh->vertices[mesh->indices[base + 2u]], track);
            const float center_x = (a.x + b.x + c.x) / 3.0f;
            const float center_y = (a.y + b.y + c.y) / 3.0f;
            const float delta_x = center_x - camera.position.x;
            const float delta_y = center_y - camera.position.y;
            if (delta_x * delta_x + delta_y * delta_y > draw_radius_squared) {
                continue;
            }
            draw_line_3d(dc, client, camera, a, b);
            draw_line_3d(dc, client, camera, b, c);
            draw_line_3d(dc, client, camera, c, a);
        }
    }
}

/* Sutherland-Hodgman clip of a convex polygon against the camera near plane.
   Road triangles span tens of units, so one close to the camera routinely has a
   vertex behind it; dropping those would leave the ground under the kart
   unshaded. Clipping a triangle by one plane yields at most four points. */
static int clip_to_near_plane(
    const KartVec3 *input,
    int count,
    Camera3D camera,
    float near_depth,
    KartVec3 *output)
{
    int out_count = 0;
    int i;
    for (i = 0; i < count; ++i) {
        const KartVec3 current = input[i];
        const KartVec3 next = input[(i + 1) % count];
        const float depth_current =
            vec_dot(vec_sub(current, camera.position), camera.forward) - near_depth;
        const float depth_next =
            vec_dot(vec_sub(next, camera.position), camera.forward) - near_depth;
        if (depth_current >= 0.0f) {
            output[out_count++] = current;
        }
        if ((depth_current >= 0.0f) != (depth_next >= 0.0f)) {
            const float t = depth_current / (depth_current - depth_next);
            output[out_count++] =
                vec_add(current, vec_scale(vec_sub(next, current), t));
        }
    }
    return out_count;
}

static LONG clamp_device_coordinate(LONG value)
{
    /* Vertices just past the near plane project a long way off screen. GDI is
       happy with large coordinates but not unbounded ones. */
    const LONG limit = 30000;
    if (value < -limit) return -limit;
    if (value > limit) return limit;
    return value;
}

/* Fills the faces the physics treats as solid. Drawn opaque here and blended in
   by the caller, so a surface reads as a tint over whatever is behind it.

   Every mesh is filled, scenery included, because every mesh now collides. The
   colour comes from the face normal using the same thresholds as
   kart_track_collision.c, so what is shaded as floor is exactly what the wheel
   rays can hit and what is shaded as wall is what the body can hit. */
static void fill_track_scene_faces(
    HDC dc,
    RECT client,
    Camera3D camera,
    const KartDemoTrackSpec *track,
    const KartTrackScene *scene)
{
    const float draw_radius_squared = 210.0f * 210.0f;
    /* Further out than the wireframe's near plane, which keeps the projected
       coordinates of a freshly clipped edge within a sane range. */
    const float near_depth = 0.5f;
    HBRUSH floor_brush = CreateSolidBrush(RGB(196, 176, 120));
    HBRUSH wall_brush = CreateSolidBrush(RGB(96, 150, 210));
    HGDIOBJ old_brush = SelectObject(dc, floor_brush);
    HGDIOBJ old_pen = SelectObject(dc, GetStockObject(NULL_PEN));
    uint32_t mesh_index;

    for (mesh_index = 0; mesh_index < scene->mesh_count; ++mesh_index) {
        const KartTrackSceneMesh *mesh = &scene->meshes[mesh_index];
        const uint32_t triangle_count = mesh->index_count / 3u;
        uint32_t triangle;
        for (triangle = 0; triangle < triangle_count; ++triangle) {
            const uint32_t base = triangle * 3u;
            const KartVec3 vertices[3] = {
                kart_track_scene_world_vertex(
                    &mesh->vertices[mesh->indices[base]], track),
                kart_track_scene_world_vertex(
                    &mesh->vertices[mesh->indices[base + 1u]], track),
                kart_track_scene_world_vertex(
                    &mesh->vertices[mesh->indices[base + 2u]], track),
            };
            const float center_x =
                (vertices[0].x + vertices[1].x + vertices[2].x) / 3.0f;
            const float center_y =
                (vertices[0].y + vertices[1].y + vertices[2].y) / 3.0f;
            const float delta_x = center_x - camera.position.x;
            const float delta_y = center_y - camera.position.y;
            KartVec3 normal;
            float normal_z;
            KartVec3 clipped[8];
            POINT points[8];
            int clipped_count;
            int i;
            if (delta_x * delta_x + delta_y * delta_y > draw_radius_squared) {
                continue;
            }
            clipped_count = clip_to_near_plane(
                vertices, 3, camera, near_depth, clipped);
            if (clipped_count < 3) {
                continue;
            }
            for (i = 0; i < clipped_count; ++i) {
                const ProjectedPoint projected =
                    project_point(client, camera, clipped[i]);
                points[i].x = clamp_device_coordinate(projected.point.x);
                points[i].y = clamp_device_coordinate(projected.point.y);
            }
            /* The normal comes from the unclipped triangle so the floor/wall
               split matches what the collision query sees. */
            normal = vec_normalize(vec_cross(
                vec_sub(vertices[1], vertices[0]),
                vec_sub(vertices[2], vertices[0])));
            normal_z = normal.z < 0.0f ? -normal.z : normal.z;
            if (normal_z >= 0.20f) {
                SelectObject(dc, floor_brush);
            } else {
                SelectObject(dc, wall_brush);
            }
            Polygon(dc, points, clipped_count);
        }
    }
    SelectObject(dc, old_pen);
    SelectObject(dc, old_brush);
    DeleteObject(floor_brush);
    DeleteObject(wall_brush);
}

/* Composites the filled faces at partial alpha, then draws the wireframe over
   them at full strength. Copying the frame into the layer first means untouched
   pixels blend with themselves and stay exactly as they were. */
static void draw_track_scene_faces(
    HDC dc,
    RECT client,
    Camera3D camera,
    const KartDemoTrackSpec *track,
    const KartTrackScene *scene)
{
    const int width = client.right - client.left;
    const int height = client.bottom - client.top;
    /* Roughly one third opacity: enough to read the surface, light enough that
       the wireframe and the grid behind it stay legible. */
    BLENDFUNCTION blend = {AC_SRC_OVER, 0, 85, 0};
    HDC layer;
    HBITMAP bitmap;
    HGDIOBJ old_bitmap;
    if (width <= 0 || height <= 0) {
        return;
    }
    layer = CreateCompatibleDC(dc);
    if (layer == NULL) {
        return;
    }
    bitmap = CreateCompatibleBitmap(dc, width, height);
    if (bitmap == NULL) {
        DeleteDC(layer);
        return;
    }
    old_bitmap = SelectObject(layer, bitmap);
    BitBlt(layer, 0, 0, width, height, dc, 0, 0, SRCCOPY);
    fill_track_scene_faces(layer, client, camera, track, scene);
    AlphaBlend(dc, 0, 0, width, height, layer, 0, 0, width, height, blend);
    SelectObject(layer, old_bitmap);
    DeleteObject(bitmap);
    DeleteDC(layer);
}

static void draw_track_scene(
    HDC dc,
    RECT client,
    Camera3D camera,
    const KartDemoTrackSpec *track,
    const KartTrackScene *scene)
{
    HPEN scenery_pen;
    HPEN road_pen;
    HGDIOBJ old_pen;
    if (scene == NULL) {
        return;
    }
    draw_track_scene_faces(dc, client, camera, track, scene);
    scenery_pen = CreatePen(PS_SOLID, 1, RGB(93, 125, 95));
    road_pen = CreatePen(PS_SOLID, 2, RGB(205, 194, 159));
    old_pen = SelectObject(dc, scenery_pen);
    draw_track_scene_pass(dc, client, camera, track, scene, false);
    SelectObject(dc, road_pen);
    draw_track_scene_pass(dc, client, camera, track, scene, true);
    SelectObject(dc, old_pen);
    DeleteObject(scenery_pen);
    DeleteObject(road_pen);
}

static void draw_track(
    HDC dc,
    RECT client,
    Camera3D camera,
    const KartDemoTrackSpec *track)
{
    const float track_half_width = kart_demo_track_width(track) * 0.5f;
    const float track_half_height = kart_demo_track_length(track) * 0.5f;
    const float grid_radius = 140.0f;
    const float grid_step = 10.0f;
    const float local_min_x = fmaxf(
        -track_half_width,
        floorf((camera.position.x - grid_radius) / grid_step) * grid_step);
    const float local_max_x = fminf(
        track_half_width,
        ceilf((camera.position.x + grid_radius) / grid_step) * grid_step);
    const float local_min_y = fmaxf(
        -track_half_height,
        floorf((camera.position.y - grid_radius) / grid_step) * grid_step);
    const float local_max_y = fminf(
        track_half_height,
        ceilf((camera.position.y + grid_radius) / grid_step) * grid_step);
    HPEN grid_pen = CreatePen(PS_SOLID, 1, RGB(86, 98, 108));
    HPEN wall_pen = CreatePen(PS_SOLID, 4, RGB(75, 220, 255));
    HGDIOBJ old_pen = SelectObject(dc, grid_pen);
    float value;

    /* Draw a camera-local patch instead of full-AABB lines whose endpoints
       can both fall outside the view on the demo's very large tracks. */
    for (value = local_min_x; value <= local_max_x; value += grid_step) {
        draw_line_3d(
            dc, client, camera,
            (KartVec3){value, local_min_y, 0.0f},
            (KartVec3){value, local_max_y, 0.0f});
    }
    for (value = local_min_y; value <= local_max_y; value += grid_step) {
        draw_line_3d(
            dc, client, camera,
            (KartVec3){local_min_x, value, 0.0f},
            (KartVec3){local_max_x, value, 0.0f});
    }
    SelectObject(dc, wall_pen);
    draw_line_3d(dc, client, camera,
        (KartVec3){-track_half_width, -track_half_height, 0},
        (KartVec3){ track_half_width, -track_half_height, 0});
    draw_line_3d(dc, client, camera,
        (KartVec3){ track_half_width, -track_half_height, 0},
        (KartVec3){ track_half_width,  track_half_height, 0});
    draw_line_3d(dc, client, camera,
        (KartVec3){ track_half_width,  track_half_height, 0},
        (KartVec3){-track_half_width,  track_half_height, 0});
    draw_line_3d(dc, client, camera,
        (KartVec3){-track_half_width,  track_half_height, 0},
        (KartVec3){-track_half_width, -track_half_height, 0});
    draw_line_3d(dc, client, camera,
        (KartVec3){-track_half_width, -track_half_height, 0.8f},
        (KartVec3){ track_half_width, -track_half_height, 0.8f});
    draw_line_3d(dc, client, camera,
        (KartVec3){ track_half_width, -track_half_height, 0.8f},
        (KartVec3){ track_half_width,  track_half_height, 0.8f});
    draw_line_3d(dc, client, camera,
        (KartVec3){ track_half_width,  track_half_height, 0.8f},
        (KartVec3){-track_half_width,  track_half_height, 0.8f});
    draw_line_3d(dc, client, camera,
        (KartVec3){-track_half_width,  track_half_height, 0.8f},
        (KartVec3){-track_half_width, -track_half_height, 0.8f});
    SelectObject(dc, old_pen);
    DeleteObject(grid_pen);
    DeleteObject(wall_pen);
}

static void draw_track_minimap(
    HDC dc,
    RECT client,
    const Demo3DState *demo)
{
    const int panel_width = 220;
    const int panel_height = 248;
    const int margin = 16;
    const float track_width = kart_demo_track_width(demo->track_spec);
    const float track_length = kart_demo_track_length(demo->track_spec);
    RECT panel = {
        client.right - panel_width - margin,
        client.top + margin,
        client.right - margin,
        client.top + panel_height + margin,
    };
    const float available_width = (float)panel_width - 20.0f;
    const float available_height = (float)panel_height - 58.0f;
    const float scale_x = available_width / track_width;
    const float scale_y = available_height / track_length;
    const float scale = scale_x < scale_y ? scale_x : scale_y;
    const float center_x = (float)(panel.left + panel.right) * 0.5f;
    const float center_y = (float)(panel.top + 42 + panel.bottom - 10) * 0.5f;
    const int half_width = (int)(track_width * scale * 0.5f);
    const int half_height = (int)(track_length * scale * 0.5f);
    RECT boundary = {
        (LONG)center_x - half_width,
        (LONG)center_y - half_height,
        (LONG)center_x + half_width,
        (LONG)center_y + half_height,
    };
    const RECT image_rect = {
        panel.left + 15,
        panel.top + 42,
        panel.right - 15,
        panel.bottom - 16,
    };
    const KartDemoMinimap *minimap = kart_demo_minimap_for_track(
        &demo->minimaps, demo->track_spec);
    KartVec3 right;
    KartVec3 forward;
    KartVec3 up;
    POINT kart_point;
    POINT kart_triangle[3];
    HBRUSH panel_brush = CreateSolidBrush(RGB(15, 19, 26));
    HBRUSH kart_brush = CreateSolidBrush(RGB(255, 180, 45));
    HPEN panel_pen = CreatePen(PS_SOLID, 1, RGB(54, 64, 73));
    HPEN grid_pen = CreatePen(PS_SOLID, 1, RGB(86, 98, 108));
    HPEN wall_pen = CreatePen(PS_SOLID, 3, RGB(75, 220, 255));
    HPEN kart_pen = CreatePen(PS_SOLID, 1, RGB(255, 235, 175));
    HGDIOBJ old_brush = SelectObject(dc, panel_brush);
    HGDIOBJ old_pen = SelectObject(dc, panel_pen);
    int division;
    static const char label[] = "TRACK MAP";
    char kart_size[96];

    Rectangle(dc, panel.left, panel.top, panel.right, panel.bottom);
    if (minimap != NULL) {
        boundary = image_rect;
        kart_demo_draw_minimap_bitmap(dc, image_rect, minimap);
    } else {
        SelectObject(dc, grid_pen);
        for (division = 1; division < 4; ++division) {
            const int x = boundary.left +
                (boundary.right - boundary.left) * division / 4;
            const int y = boundary.top +
                (boundary.bottom - boundary.top) * division / 4;
            MoveToEx(dc, x, boundary.top, NULL);
            LineTo(dc, x, boundary.bottom);
            MoveToEx(dc, boundary.left, y, NULL);
            LineTo(dc, boundary.right, y);
        }
    }
    SelectObject(dc, wall_pen);
    SelectObject(dc, GetStockObject(NULL_BRUSH));
    Rectangle(dc, boundary.left, boundary.top, boundary.right, boundary.bottom);

    orientation_axes(demo->kart.orientation, &right, &forward, &up);
    (void)right;
    (void)up;
    if (minimap != NULL) {
        kart_point = kart_demo_minimap_kart_point(
            image_rect, minimap, demo->track_spec, demo->kart.position);
    } else {
        kart_point = kart_demo_minimap_bounds_point(
            boundary, demo->track_spec, demo->kart.position);
    }
    right = kart_demo_minimap_direction(demo->track_spec, right);
    forward = kart_demo_minimap_direction(demo->track_spec, forward);
    kart_triangle[0] = (POINT){
        kart_point.x + (LONG)(forward.x * 9.0f),
        kart_point.y + (LONG)(forward.y * 9.0f),
    };
    kart_triangle[1] = (POINT){
        kart_point.x - (LONG)(forward.x * 6.0f) + (LONG)(right.x * 5.0f),
        kart_point.y - (LONG)(forward.y * 6.0f) + (LONG)(right.y * 5.0f),
    };
    kart_triangle[2] = (POINT){
        kart_point.x - (LONG)(forward.x * 6.0f) - (LONG)(right.x * 5.0f),
        kart_point.y - (LONG)(forward.y * 6.0f) - (LONG)(right.y * 5.0f),
    };
    SelectObject(dc, kart_brush);
    SelectObject(dc, kart_pen);
    Polygon(dc, kart_triangle, 3);

    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, RGB(180, 205, 215));
    TextOutA(dc, panel.left + 8, panel.top + 6, label, (int)strlen(label));
    snprintf(
        kart_size,
        sizeof(kart_size),
        "KART %s  %.3f x %.3f",
        demo->kart_spec->asset_name,
        demo->kart.geometry.half_width * 2.0f,
        demo->kart.geometry.half_length * 2.0f);
    TextOutA(
        dc, panel.left + 8, panel.top + 22,
        kart_size, (int)strlen(kart_size));
    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
    DeleteObject(panel_brush);
    DeleteObject(kart_brush);
    DeleteObject(panel_pen);
    DeleteObject(grid_pen);
    DeleteObject(wall_pen);
    DeleteObject(kart_pen);
}

static void draw_skid_marks(
    HDC dc,
    RECT client,
    Camera3D camera,
    const Demo3DState *demo)
{
    HPEN fresh_pen = CreatePen(PS_SOLID, 4, RGB(8, 10, 12));
    HPEN middle_pen = CreatePen(PS_SOLID, 3, RGB(23, 26, 29));
    HPEN old_pen_color = CreatePen(PS_SOLID, 2, RGB(43, 47, 51));
    HPEN selected_pen = NULL;
    HGDIOBJ original_pen = NULL;
    unsigned int i;

    for (i = 0; i < demo->skid_count; ++i) {
        const unsigned int oldest =
            (demo->skid_head + MAX_SKID_SEGMENTS - demo->skid_count) %
            MAX_SKID_SEGMENTS;
        const SkidSegment3D *segment =
            &demo->skid_segments[(oldest + i) % MAX_SKID_SEGMENTS];
        const unsigned int age = demo->simulation_time_ms - segment->created_ms;
        HPEN wanted_pen;
        if (age > SKID_LIFETIME_MS) {
            continue;
        }
        wanted_pen = age < 4000
            ? fresh_pen
            : (age < 9000 ? middle_pen : old_pen_color);
        if (wanted_pen != selected_pen) {
            if (selected_pen == NULL) {
                original_pen = SelectObject(dc, wanted_pen);
            } else {
                SelectObject(dc, wanted_pen);
            }
            selected_pen = wanted_pen;
        }
        draw_line_3d(
            dc, client, camera,
            (KartVec3){segment->left_x0, segment->left_y0, 0.025f},
            (KartVec3){segment->left_x1, segment->left_y1, 0.025f});
        draw_line_3d(
            dc, client, camera,
            (KartVec3){segment->right_x0, segment->right_y0, 0.025f},
            (KartVec3){segment->right_x1, segment->right_y1, 0.025f});
    }
    if (selected_pen != NULL) {
        SelectObject(dc, original_pen);
    }
    DeleteObject(fresh_pen);
    DeleteObject(middle_pen);
    DeleteObject(old_pen_color);
}

static void draw_boost_effect(
    HDC dc,
    RECT client,
    Camera3D camera,
    const Demo3DState *demo)
{
    KartVec3 right;
    KartVec3 forward;
    KartVec3 up;
    KartVec3 rear;
    KartVec3 flame[3];
    ProjectedPoint projected[3];
    POINT polygon[3];
    HBRUSH flame_brush;
    HPEN flame_pen;
    HGDIOBJ old_brush;
    HGDIOBJ old_pen;
    unsigned int i;

    if (!demo->boost_active) {
        return;
    }
    orientation_axes(demo->kart.orientation, &right, &forward, &up);
    rear = vec_add(
        demo->kart.position,
        vec_add(
            vec_scale(forward, -demo->kart.geometry.half_length),
            vec_scale(up, 0.35f)));
    flame[0] = vec_add(rear, vec_scale(right, demo->kart.geometry.half_width * 0.42f));
    flame[1] = vec_add(rear, vec_scale(right, -demo->kart.geometry.half_width * 0.42f));
    flame[2] = vec_add(
        rear,
        vec_add(vec_scale(forward, -2.8f), vec_scale(up, -0.08f)));
    for (i = 0; i < 3; ++i) {
        projected[i] = project_point(client, camera, flame[i]);
        if (!projected[i].visible) {
            return;
        }
        polygon[i] = projected[i].point;
    }
    flame_brush = CreateSolidBrush(RGB(255, 198, 35));
    flame_pen = CreatePen(PS_SOLID, 2, RGB(90, 225, 255));
    old_brush = SelectObject(dc, flame_brush);
    old_pen = SelectObject(dc, flame_pen);
    Polygon(dc, polygon, 3);
    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
    DeleteObject(flame_brush);
    DeleteObject(flame_pen);
}

static void draw_motion_vectors(
    HDC dc,
    RECT client,
    Camera3D camera,
    const KartSimulationState *kart,
    KartVec3 acceleration)
{
    KartVec3 right;
    KartVec3 forward;
    KartVec3 up;
    KartVec3 origin;
    KartVec3 velocity_direction;
    float speed;
    float arrow_length;
    HPEN heading_pen;
    HPEN velocity_pen;
    HGDIOBJ old_pen;

    orientation_axes(kart->orientation, &right, &forward, &up);
    (void)right;
    origin = vec_add(kart->position, vec_scale(up, 1.0f));
    heading_pen = CreatePen(PS_SOLID, 2, RGB(255, 245, 180));
    old_pen = SelectObject(dc, heading_pen);
    draw_line_3d(dc, client, camera, origin, vec_add(origin, vec_scale(forward, 4.0f)));
    SelectObject(dc, old_pen);
    DeleteObject(heading_pen);

    speed = sqrtf(
        kart->linear_velocity.x * kart->linear_velocity.x +
        kart->linear_velocity.y * kart->linear_velocity.y +
        kart->linear_velocity.z * kart->linear_velocity.z);
    if (speed <= 0.01f) {
        return;
    }
    velocity_direction = vec_scale(kart->linear_velocity, 1.0f / speed);
    arrow_length = fminf(speed * 0.18f, 9.0f);
    velocity_pen = CreatePen(PS_SOLID, 3, RGB(80, 230, 255));
    old_pen = SelectObject(dc, velocity_pen);
    draw_line_3d(
        dc, client, camera, origin,
        vec_add(origin, vec_scale(velocity_direction, arrow_length)));
    SelectObject(dc, old_pen);
    DeleteObject(velocity_pen);

    /* Velocity split along the body axes, so the lateral leg is the drift, and
       the acceleration the last step actually produced. */
    {
        const float forward_speed = vec_dot(kart->linear_velocity, forward);
        const float lateral_speed = vec_dot(kart->linear_velocity, right);
        const float scale = arrow_length / speed;
        HPEN forward_pen = CreatePen(PS_SOLID, 2, RGB(150, 205, 165));
        HPEN lateral_pen = CreatePen(PS_SOLID, 2, RGB(255, 120, 170));
        const KartVec3 forward_leg =
            vec_add(origin, vec_scale(forward, forward_speed * scale));
        old_pen = SelectObject(dc, forward_pen);
        draw_line_3d(dc, client, camera, origin, forward_leg);
        SelectObject(dc, lateral_pen);
        draw_line_3d(
            dc, client, camera, forward_leg,
            vec_add(forward_leg, vec_scale(right, lateral_speed * scale)));
        SelectObject(dc, old_pen);
        DeleteObject(forward_pen);
        DeleteObject(lateral_pen);
    }
    {
        const float magnitude = sqrtf(
            acceleration.x * acceleration.x +
            acceleration.y * acceleration.y +
            acceleration.z * acceleration.z);
        if (magnitude > 0.05f) {
            HPEN accel_pen = CreatePen(PS_SOLID, 2, RGB(255, 175, 110));
            const float length = fminf(magnitude * 0.12f, 6.0f) / magnitude;
            old_pen = SelectObject(dc, accel_pen);
            draw_line_3d(
                dc, client, camera, origin,
                vec_add(origin, vec_scale(acceleration, length)));
            SelectObject(dc, old_pen);
            DeleteObject(accel_pen);
        }
    }
}

static KartVec3 kart_vertex(
    const KartSimulationState *kart,
    KartVec3 right,
    KartVec3 forward,
    KartVec3 up,
    float x,
    float y,
    float z)
{
    KartVec3 result = kart->position;
    result = vec_add(result, vec_scale(right, x));
    result = vec_add(result, vec_scale(forward, y));
    result = vec_add(result, vec_scale(up, z + 0.15f));
    return result;
}

/* Model space to world.

   The model's own axes are not the simulation's, and nothing in the assets
   labels them. Three independent details agree that the model's -y is the
   front, so that is the end put along the simulation's +forward:

     - the 9-vertex 8-triangle disc at y -0.36 is a steering wheel, and its
       mean normal is (0, +0.65, +0.76): it faces up and toward +y, which is
       where the driver has to be sitting
     - the wheels at +y are the larger pair, and a kart carries its bigger
       wheels at the back
     - the body is 0.69 tall at the +y end against 0.50 at the -y end, the
       usual low-nose, high-tail profile

   Turning it round is a rotation, not a mirror, so x is negated with y. Getting
   that wrong would be invisible on these near-symmetric hulls right up until
   the moment it made a left-hand detail come out on the right. */
static KartVec3 kart_model_vertex(
    const KartSimulationState *kart,
    KartVec3 right,
    KartVec3 forward,
    KartVec3 up,
    const KartTrackSceneVertex *vertex)
{
    return kart_vertex(
        kart, right, forward, up, -vertex->x, -vertex->y, vertex->z);
}

/* One triangle of the model, ready to sort. */
typedef struct KartModelFace {
    float depth;
    float shade;
    bool wheel;
    KartVec3 vertices[3];
} KartModelFace;

/* cotten5 is the heaviest model at 645 triangles. */
#define KART_MODEL_MAX_FACES 1024

static int compare_model_faces(const void *left, const void *right)
{
    const KartModelFace *a = (const KartModelFace *)left;
    const KartModelFace *b = (const KartModelFace *)right;
    /* Far to near: the painter's order for a convex-ish hull with no z-buffer. */
    if (a->depth > b->depth) return -1;
    if (a->depth < b->depth) return 1;
    return 0;
}

static COLORREF scale_color(COLORREF color, float shade)
{
    const float r = (float)GetRValue(color) * shade;
    const float g = (float)GetGValue(color) * shade;
    const float b = (float)GetBValue(color) * shade;
    return RGB(
        (BYTE)(r > 255.0f ? 255.0f : r),
        (BYTE)(g > 255.0f ? 255.0f : g),
        (BYTE)(b > 255.0f ? 255.0f : b));
}

/* Draws the recovered mesh: every submesh, depth sorted, flat shaded.

   The paint is the demo's state colour rather than the original skin. That is
   not a shortcut waiting to be finished - the skins carry no colour to use.
   All 26 are achromatic apart from a 900-texel marker patch, so a textured
   kart would come out grey. See docs/KART_MODEL_CATALOG.md. */
static void draw_kart_model(
    HDC dc,
    RECT client,
    Camera3D camera,
    const KartSimulationState *kart,
    const KartTrackScene *model,
    const KartModelParts *parts,
    bool boost_active)
{
    /* Fixed world-space key light, so the shading reads as the kart turning
       under a light rather than the light turning with the kart. */
    const KartVec3 light = {0.38f, -0.52f, 0.76f};
    const COLORREF body_color =
        boost_active
            ? RGB(255, 165, 35)
            : (drift_visual_active(kart)
                ? RGB(65, 205, 255)
                : RGB(235, 65, 80));
    /* Dark enough to read as tyres, light enough that the shading on them is
       still visible against the track fill. */
    const COLORREF wheel_color = RGB(96, 100, 112);
    static KartModelFace faces[KART_MODEL_MAX_FACES];
    KartVec3 body_right;
    KartVec3 body_forward;
    KartVec3 body_up;
    HGDIOBJ old_pen;
    HGDIOBJ old_brush;
    size_t face_count = 0;
    size_t i;
    uint32_t mesh_index;

    orientation_axes(kart->orientation, &body_right, &body_forward, &body_up);

    for (mesh_index = 0; mesh_index < model->mesh_count; ++mesh_index) {
        const KartTrackSceneMesh *mesh = &model->meshes[mesh_index];
        const bool wheel = kart_model_is_wheel(parts, mesh_index);
        const uint32_t triangle_count = mesh->index_count / 3u;
        uint32_t triangle;
        for (triangle = 0; triangle < triangle_count; ++triangle) {
            const uint32_t base = triangle * 3u;
            KartModelFace *face;
            KartVec3 normal;
            KartVec3 centre;
            float facing;
            if (face_count == KART_MODEL_MAX_FACES) break;
            face = &faces[face_count];
            face->wheel = wheel;
            face->vertices[0] = kart_model_vertex(
                kart, body_right, body_forward, body_up,
                &mesh->vertices[mesh->indices[base]]);
            face->vertices[1] = kart_model_vertex(
                kart, body_right, body_forward, body_up,
                &mesh->vertices[mesh->indices[base + 1u]]);
            face->vertices[2] = kart_model_vertex(
                kart, body_right, body_forward, body_up,
                &mesh->vertices[mesh->indices[base + 2u]]);
            centre = vec_scale(
                vec_add(vec_add(face->vertices[0], face->vertices[1]),
                        face->vertices[2]),
                1.0f / 3.0f);
            face->depth = vec_dot(
                vec_sub(centre, camera.position), camera.forward);
            if (face->depth <= 0.25f) continue;
            normal = vec_normalize(vec_cross(
                vec_sub(face->vertices[1], face->vertices[0]),
                vec_sub(face->vertices[2], face->vertices[0])));
            /* The exports do not agree on winding between submeshes, so the
               normal is folded to whichever side faces the camera instead of
               being used to cull. Nothing here is a closed hull anyway. */
            facing = vec_dot(normal, light);
            if (facing < 0.0f) facing = -facing;
            face->shade = 0.42f + 0.58f * facing;
            ++face_count;
        }
    }
    if (face_count == 0) {
        return;
    }
    qsort(faces, face_count, sizeof(faces[0]), compare_model_faces);

    old_pen = SelectObject(dc, GetStockObject(NULL_PEN));
    old_brush = SelectObject(dc, GetStockObject(NULL_BRUSH));
    for (i = 0; i < face_count; ++i) {
        const KartModelFace *face = &faces[i];
        KartVec3 clipped[8];
        POINT points[8];
        HBRUSH brush;
        HGDIOBJ previous;
        int clipped_count = clip_to_near_plane(
            face->vertices, 3, camera, 0.25f, clipped);
        int point;
        if (clipped_count < 3) continue;
        for (point = 0; point < clipped_count; ++point) {
            const ProjectedPoint projected =
                project_point(client, camera, clipped[point]);
            points[point].x = clamp_device_coordinate(projected.point.x);
            points[point].y = clamp_device_coordinate(projected.point.y);
        }
        brush = CreateSolidBrush(scale_color(
            face->wheel ? wheel_color : body_color, face->shade));
        previous = SelectObject(dc, brush);
        Polygon(dc, points, clipped_count);
        SelectObject(dc, previous);
        DeleteObject(brush);
    }
    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
}

/* One of the three model-space boxes, as a wireframe in the kart's frame. */
static void draw_kart_model_box(
    HDC dc,
    RECT client,
    Camera3D camera,
    const KartSimulationState *kart,
    const KartModelBox *box,
    COLORREF color)
{
    static const unsigned int edges[12][2] = {
        {0,1},{1,2},{2,3},{3,0}, {4,5},{5,6},{6,7},{7,4},
        {0,4},{1,5},{2,6},{3,7},
    };
    KartVec3 body_right;
    KartVec3 body_forward;
    KartVec3 body_up;
    ProjectedPoint projected[8];
    HPEN pen;
    HGDIOBJ old_pen;
    unsigned int i;
    if (!box->valid) {
        return;
    }
    orientation_axes(kart->orientation, &body_right, &body_forward, &body_up);
    for (i = 0; i < 8; ++i) {
        /* 0-3 walk the bottom face in order and 4-7 the top, which is what the
           edge table above expects. Deriving the corners from the bits of i
           instead would put 1 and 2 diagonally opposite and draw an X across
           each face. */
        static const unsigned char corners[8][3] = {
            {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
            {0,0,1}, {1,0,1}, {1,1,1}, {0,1,1},
        };
        const KartTrackSceneVertex corner = {
            corners[i][0] ? box->maximum[0] : box->minimum[0],
            corners[i][1] ? box->maximum[1] : box->minimum[1],
            corners[i][2] ? box->maximum[2] : box->minimum[2],
            0.0f,
            0.0f,
        };
        projected[i] = project_point(
            client, camera,
            kart_model_vertex(
                kart, body_right, body_forward, body_up, &corner));
    }
    pen = CreatePen(PS_SOLID, 1, color);
    old_pen = SelectObject(dc, pen);
    for (i = 0; i < 12; ++i) {
        const unsigned int a = edges[i][0];
        const unsigned int b = edges[i][1];
        if (!projected[a].visible || !projected[b].visible) continue;
        MoveToEx(dc, projected[a].point.x, projected[a].point.y, NULL);
        LineTo(dc, projected[b].point.x, projected[b].point.y);
    }
    SelectObject(dc, old_pen);
    DeleteObject(pen);
}

/* full yellow, body green, wheels magenta - the same three volumes the
   catalogue tabulates, drawn so the wheels-wider-than-body gap is visible. */
static void draw_kart_model_bounds(
    HDC dc,
    RECT client,
    Camera3D camera,
    const KartSimulationState *kart,
    const KartModelParts *parts)
{
    if (parts == NULL || !parts->valid) {
        return;
    }
    draw_kart_model_box(
        dc, client, camera, kart, &parts->full, RGB(235, 215, 90));
    draw_kart_model_box(
        dc, client, camera, kart, &parts->wheels, RGB(225, 105, 220));
    draw_kart_model_box(
        dc, client, camera, kart, &parts->body, RGB(110, 235, 140));
}

static void draw_kart_box(
    HDC dc,
    RECT client,
    Camera3D camera,
    const KartSimulationState *kart,
    const KartDemoKartSpec *spec,
    bool boost_active)
{
    static const unsigned int edges[12][2] = {
        {0,1},{1,2},{2,3},{3,0}, {4,5},{5,6},{6,7},{7,4},
        {0,4},{1,5},{2,6},{3,7},
    };
    static const unsigned int top_face[4] = {4, 5, 6, 7};
    KartVec3 body_right;
    KartVec3 body_forward;
    KartVec3 body_up;
    KartVec3 vertices[8];
    ProjectedPoint projected[8];
    HBRUSH body_brush;
    HPEN body_pen;
    HGDIOBJ old_brush;
    HGDIOBJ old_pen;
    POINT polygon[4];
    unsigned int i;
    const float half_width = kart->geometry.half_width;
    const float half_length = kart->geometry.half_length;
    const float roof_width = half_width * 0.8f;
    const float roof_length = half_length * 0.75f;
    const float model_height = spec->model_height;

    orientation_axes(kart->orientation, &body_right, &body_forward, &body_up);
    vertices[0] = kart_vertex(kart, body_right, body_forward, body_up, -half_width, -half_length, 0.0f);
    vertices[1] = kart_vertex(kart, body_right, body_forward, body_up,  half_width, -half_length, 0.0f);
    vertices[2] = kart_vertex(kart, body_right, body_forward, body_up,  half_width,  half_length, 0.0f);
    vertices[3] = kart_vertex(kart, body_right, body_forward, body_up, -half_width,  half_length, 0.0f);
    vertices[4] = kart_vertex(kart, body_right, body_forward, body_up, -roof_width, -roof_length, model_height);
    vertices[5] = kart_vertex(kart, body_right, body_forward, body_up,  roof_width, -roof_length, model_height);
    vertices[6] = kart_vertex(kart, body_right, body_forward, body_up,  roof_width,  roof_length, model_height);
    vertices[7] = kart_vertex(kart, body_right, body_forward, body_up, -roof_width,  roof_length, model_height);
    for (i = 0; i < 8; ++i) {
        projected[i] = project_point(client, camera, vertices[i]);
    }

    body_brush = CreateSolidBrush(
        boost_active
            ? RGB(255, 165, 35)
            : (drift_visual_active(kart)
                ? RGB(65, 205, 255)
                : RGB(235, 65, 80)));
    body_pen = CreatePen(PS_SOLID, 2, RGB(255, 230, 220));
    old_brush = SelectObject(dc, body_brush);
    old_pen = SelectObject(dc, body_pen);
    for (i = 0; i < 4; ++i) {
        polygon[i] = projected[top_face[i]].point;
    }
    if (projected[4].visible && projected[5].visible &&
        projected[6].visible && projected[7].visible) {
        Polygon(dc, polygon, 4);
    }
    for (i = 0; i < 12; ++i) {
        const unsigned int a = edges[i][0];
        const unsigned int b = edges[i][1];
        if (projected[a].visible && projected[b].visible) {
            MoveToEx(dc, projected[a].point.x, projected[a].point.y, NULL);
            LineTo(dc, projected[b].point.x, projected[b].point.y);
        }
    }
    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
    DeleteObject(body_brush);
    DeleteObject(body_pen);
}

/* Draws whichever hull the driver asked for, then the boxes over it.

   The box is still here and still correct: it is what the demo drew before the
   models were recovered, and it is the shape the physics actually reasons
   about. M switches between them so the two can be compared directly, and a
   build without embedded models simply always shows the box. */
static void draw_kart(
    HDC dc,
    RECT client,
    Camera3D camera,
    const Demo3DState *demo)
{
    const KartModelParts *parts = NULL;
    const KartTrackScene *model = active_kart_model(demo, &parts);
    if (demo->show_model && model != NULL && parts != NULL && parts->valid) {
        draw_kart_model(
            dc, client, camera, &demo->kart, model, parts, demo->boost_active);
    } else {
        draw_kart_box(
            dc, client, camera, &demo->kart, demo->kart_spec,
            demo->boost_active);
    }
    if (demo->show_model_bounds) {
        draw_kart_model_bounds(dc, client, camera, &demo->kart, parts);
    }
}

/* World to screen for the top-down view. World +Y points up the screen and
   world +X points left, matching the original minimap artwork's orientation.
   Both axes are negated together, which is a rotation rather than a mirror, so
   steering handedness and the Track Map panel stay consistent. */
static POINT world_to_screen(
    RECT client,
    float camera_x,
    float camera_y,
    float x,
    float y)
{
    const float margin = 45.0f;
    const float width = (float)(client.right - client.left) - margin * 2.0f;
    const float height = (float)(client.bottom - client.top) - margin * 2.0f;
    const float sx = width / (VIEW_HALF_WIDTH * 2.0f);
    const float sy = height / (VIEW_HALF_HEIGHT * 2.0f);
    const float scale = sx < sy ? sx : sy;
    const POINT point = {
        (LONG)((client.right + client.left) * 0.5f - (x - camera_x) * scale),
        (LONG)((client.bottom + client.top) * 0.5f - (y - camera_y) * scale),
    };
    return point;
}

/* The flat forward vector the top-down view draws with. */
static KartVec3 kart_forward(KartQuat q)
{
    return (KartVec3){
        -2.0f * (q.x * q.y - q.w * q.z),
        -(1.0f - 2.0f * (q.x * q.x + q.z * q.z)),
        -2.0f * (q.y * q.z + q.w * q.x),
    };
}

static void draw_scene_topdown(HDC buffer, RECT client, const Demo3DState *demo)
{
    HBRUSH track_brush;
    HPEN wall_pen;
    HPEN grid_pen;
    HPEN heading_pen;
    HPEN velocity_pen;
    HPEN skid_fresh_pen;
    HPEN skid_middle_pen;
    HPEN skid_old_pen;
    HBRUSH kart_brush;
    HGDIOBJ old_brush;
    HGDIOBJ old_pen;
    POINT track_min;
    POINT track_max;
    POINT center;
    POINT nose;
    POINT kart_points[4];
    POINT boost_points[3];
    const KartVec3 forward = kart_forward(demo->kart.orientation);
    const float side_x = -forward.y;
    const float side_y = forward.x;
    const float camera_x = demo->kart.position.x;
    const float camera_y = demo->kart.position.y;
    const float track_half_width =
        kart_demo_track_width(demo->track_spec) * 0.5f;
    const float track_half_height =
        kart_demo_track_length(demo->track_spec) * 0.5f;
    float speed;

    track_min = world_to_screen(
        client, camera_x, camera_y, -track_half_width, -track_half_height);
    track_max = world_to_screen(
        client, camera_x, camera_y, track_half_width, track_half_height);
    {
        RECT track = {track_min.x, track_min.y, track_max.x, track_max.y};
        track_brush = CreateSolidBrush(RGB(50, 58, 65));
        wall_pen = CreatePen(PS_SOLID, 4, RGB(100, 210, 255));
        old_brush = SelectObject(buffer, track_brush);
        old_pen = SelectObject(buffer, wall_pen);
        Rectangle(buffer, track.left, track.top, track.right, track.bottom);
        SelectObject(buffer, old_brush);
        SelectObject(buffer, old_pen);
        DeleteObject(track_brush);
        DeleteObject(wall_pen);
    }

    grid_pen = CreatePen(PS_SOLID, 1, RGB(66, 75, 82));
    old_pen = SelectObject(buffer, grid_pen);
    {
        float grid_x;
        float grid_y;
        const float grid_size = 10.0f;
        const float min_x = camera_x - VIEW_HALF_WIDTH - grid_size;
        const float max_x = camera_x + VIEW_HALF_WIDTH + grid_size;
        const float min_y = camera_y - VIEW_HALF_HEIGHT - grid_size;
        const float max_y = camera_y + VIEW_HALF_HEIGHT + grid_size;
        for (grid_x = floorf(min_x / grid_size) * grid_size;
             grid_x <= max_x;
             grid_x += grid_size) {
            POINT a = world_to_screen(client, camera_x, camera_y, grid_x, min_y);
            POINT b = world_to_screen(client, camera_x, camera_y, grid_x, max_y);
            MoveToEx(buffer, a.x, a.y, NULL);
            LineTo(buffer, b.x, b.y);
        }
        for (grid_y = floorf(min_y / grid_size) * grid_size;
             grid_y <= max_y;
             grid_y += grid_size) {
            POINT a = world_to_screen(client, camera_x, camera_y, min_x, grid_y);
            POINT b = world_to_screen(client, camera_x, camera_y, max_x, grid_y);
            MoveToEx(buffer, a.x, a.y, NULL);
            LineTo(buffer, b.x, b.y);
        }
    }
    SelectObject(buffer, old_pen);
    DeleteObject(grid_pen);

    skid_fresh_pen = CreatePen(PS_SOLID, 3, RGB(12, 14, 16));
    skid_middle_pen = CreatePen(PS_SOLID, 3, RGB(25, 28, 31));
    skid_old_pen = CreatePen(PS_SOLID, 2, RGB(39, 44, 48));
    {
        unsigned int i;
        HPEN selected_pen = NULL;
        for (i = 0; i < demo->skid_count; ++i) {
            const unsigned int oldest =
                (demo->skid_head + MAX_SKID_SEGMENTS - demo->skid_count) %
                MAX_SKID_SEGMENTS;
            const SkidSegment3D *segment =
                &demo->skid_segments[(oldest + i) % MAX_SKID_SEGMENTS];
            const unsigned int age =
                demo->simulation_time_ms - segment->created_ms;
            HPEN wanted_pen;
            POINT a;
            POINT b;
            if (age > SKID_LIFETIME_MS) {
                continue;
            }
            wanted_pen = age < 4000
                ? skid_fresh_pen
                : (age < 9000 ? skid_middle_pen : skid_old_pen);
            if (wanted_pen != selected_pen) {
                if (selected_pen == NULL) {
                    old_pen = SelectObject(buffer, wanted_pen);
                } else {
                    SelectObject(buffer, wanted_pen);
                }
                selected_pen = wanted_pen;
            }
            a = world_to_screen(
                client, camera_x, camera_y, segment->left_x0, segment->left_y0);
            b = world_to_screen(
                client, camera_x, camera_y, segment->left_x1, segment->left_y1);
            MoveToEx(buffer, a.x, a.y, NULL);
            LineTo(buffer, b.x, b.y);
            a = world_to_screen(
                client, camera_x, camera_y, segment->right_x0, segment->right_y0);
            b = world_to_screen(
                client, camera_x, camera_y, segment->right_x1, segment->right_y1);
            MoveToEx(buffer, a.x, a.y, NULL);
            LineTo(buffer, b.x, b.y);
        }
        if (selected_pen != NULL) {
            SelectObject(buffer, old_pen);
        }
    }
    DeleteObject(skid_fresh_pen);
    DeleteObject(skid_middle_pen);
    DeleteObject(skid_old_pen);

    center = world_to_screen(
        client, camera_x, camera_y,
        demo->kart.position.x, demo->kart.position.y);
    nose = world_to_screen(
        client,
        camera_x,
        camera_y,
        demo->kart.position.x + forward.x * demo->kart.geometry.half_length,
        demo->kart.position.y + forward.y * demo->kart.geometry.half_length);
    kart_points[0] = world_to_screen(
        client,
        camera_x,
        camera_y,
        demo->kart.position.x + forward.x * demo->kart.geometry.half_length +
            side_x * demo->kart.geometry.half_width,
        demo->kart.position.y + forward.y * demo->kart.geometry.half_length +
            side_y * demo->kart.geometry.half_width);
    kart_points[1] = world_to_screen(
        client,
        camera_x,
        camera_y,
        demo->kart.position.x + forward.x * demo->kart.geometry.half_length -
            side_x * demo->kart.geometry.half_width,
        demo->kart.position.y + forward.y * demo->kart.geometry.half_length -
            side_y * demo->kart.geometry.half_width);
    kart_points[2] = world_to_screen(
        client,
        camera_x,
        camera_y,
        demo->kart.position.x - forward.x * demo->kart.geometry.half_length -
            side_x * demo->kart.geometry.half_width,
        demo->kart.position.y - forward.y * demo->kart.geometry.half_length -
            side_y * demo->kart.geometry.half_width);
    kart_points[3] = world_to_screen(
        client,
        camera_x,
        camera_y,
        demo->kart.position.x - forward.x * demo->kart.geometry.half_length +
            side_x * demo->kart.geometry.half_width,
        demo->kart.position.y - forward.y * demo->kart.geometry.half_length +
            side_y * demo->kart.geometry.half_width);

    if (demo->boost_active) {
        HBRUSH flame_brush = CreateSolidBrush(RGB(255, 205, 45));
        boost_points[0] = world_to_screen(
            client,
            camera_x,
            camera_y,
            demo->kart.position.x - forward.x * 3.2f,
            demo->kart.position.y - forward.y * 3.2f);
        boost_points[1] = world_to_screen(
            client,
            camera_x,
            camera_y,
            demo->kart.position.x - forward.x * 0.7f + side_x * 0.48f,
            demo->kart.position.y - forward.y * 0.7f + side_y * 0.48f);
        boost_points[2] = world_to_screen(
            client,
            camera_x,
            camera_y,
            demo->kart.position.x - forward.x * 0.7f - side_x * 0.48f,
            demo->kart.position.y - forward.y * 0.7f - side_y * 0.48f);
        old_brush = SelectObject(buffer, flame_brush);
        Polygon(buffer, boost_points, 3);
        SelectObject(buffer, old_brush);
        DeleteObject(flame_brush);
    }

    kart_brush = CreateSolidBrush(
        demo->boost_active
            ? RGB(255, 170, 45)
            : (drift_visual_active(&demo->kart)
                ? RGB(65, 205, 255)
                : RGB(255, 80, 95)));
    old_brush = SelectObject(buffer, kart_brush);
    Polygon(buffer, kart_points, 4);
    SelectObject(buffer, old_brush);
    DeleteObject(kart_brush);

    heading_pen = CreatePen(PS_SOLID, 2, RGB(255, 245, 180));
    old_pen = SelectObject(buffer, heading_pen);
    MoveToEx(buffer, center.x, center.y, NULL);
    LineTo(buffer, nose.x, nose.y);
    SelectObject(buffer, old_pen);
    DeleteObject(heading_pen);

    speed = sqrtf(
        demo->kart.linear_velocity.x * demo->kart.linear_velocity.x +
        demo->kart.linear_velocity.y * demo->kart.linear_velocity.y);
    if (!demo->show_vectors) {
        return;
    }
    if (speed > 0.01f) {
        const float arrow_length = fminf(speed * 0.18f, 9.0f);
        POINT velocity_end = world_to_screen(
            client,
            camera_x,
            camera_y,
            demo->kart.position.x +
                demo->kart.linear_velocity.x / speed * arrow_length,
            demo->kart.position.y +
                demo->kart.linear_velocity.y / speed * arrow_length);
        velocity_pen = CreatePen(PS_SOLID, 3, RGB(80, 230, 255));
        old_pen = SelectObject(buffer, velocity_pen);
        MoveToEx(buffer, center.x, center.y, NULL);
        LineTo(buffer, velocity_end.x, velocity_end.y);
        SelectObject(buffer, old_pen);
        DeleteObject(velocity_pen);

        /* The same arrow split along the body axes: the lateral leg is what
           the kart is sliding sideways. */
        {
            const float forward_speed =
                demo->kart.linear_velocity.x * forward.x +
                demo->kart.linear_velocity.y * forward.y;
            const float lateral_speed =
                demo->kart.linear_velocity.x * side_x +
                demo->kart.linear_velocity.y * side_y;
            const float scale = arrow_length / speed;
            HPEN forward_pen = CreatePen(PS_SOLID, 2, RGB(150, 205, 165));
            HPEN lateral_pen = CreatePen(PS_SOLID, 2, RGB(255, 120, 170));
            const POINT forward_leg = world_to_screen(
                client, camera_x, camera_y,
                demo->kart.position.x + forward.x * forward_speed * scale,
                demo->kart.position.y + forward.y * forward_speed * scale);
            const POINT lateral_leg = world_to_screen(
                client, camera_x, camera_y,
                demo->kart.position.x + forward.x * forward_speed * scale +
                    side_x * lateral_speed * scale,
                demo->kart.position.y + forward.y * forward_speed * scale +
                    side_y * lateral_speed * scale);
            old_pen = SelectObject(buffer, forward_pen);
            MoveToEx(buffer, center.x, center.y, NULL);
            LineTo(buffer, forward_leg.x, forward_leg.y);
            SelectObject(buffer, lateral_pen);
            MoveToEx(buffer, forward_leg.x, forward_leg.y, NULL);
            LineTo(buffer, lateral_leg.x, lateral_leg.y);
            SelectObject(buffer, old_pen);
            DeleteObject(forward_pen);
            DeleteObject(lateral_pen);
        }
    }
    {
        /* The acceleration the last step produced. */
        const float magnitude = sqrtf(
            demo->acceleration.x * demo->acceleration.x +
            demo->acceleration.y * demo->acceleration.y);
        if (magnitude > 0.05f) {
            const float length = fminf(magnitude * 0.12f, 6.0f) / magnitude;
            HPEN accel_pen = CreatePen(PS_SOLID, 2, RGB(255, 175, 110));
            const POINT accel_end = world_to_screen(
                client, camera_x, camera_y,
                demo->kart.position.x + demo->acceleration.x * length,
                demo->kart.position.y + demo->acceleration.y * length);
            old_pen = SelectObject(buffer, accel_pen);
            MoveToEx(buffer, center.x, center.y, NULL);
            LineTo(buffer, accel_end.x, accel_end.y);
            SelectObject(buffer, old_pen);
            DeleteObject(accel_pen);
        }
    }
}

static const char *drift_phase_name(const KartSimulationState *kart)
{
    if (kart->drift.trigger_active) return "TRIGGER";
    if (kart->drift.slip_detected) return "SLIP";
    if (kart->drift.input_active) return "DRIFT";
    return "GRIP";
}

/* The values the HUD lines have no room for: the whole rigid body state, the
   drift and boost timers, the suspension contacts and what the last step
   resolved. Fixed pitch so the columns line up as the numbers move. */
static void draw_telemetry(HDC dc, RECT client, const Demo3DState *demo)
{
    const KartSimulationState *kart = &demo->kart;
    const int line_height = 15;
    const int rows = 14;
    const int panel_width = 396;
    const int margin = 16;
    RECT panel = {
        client.left + margin,
        client.bottom - margin - (rows * line_height + 14),
        client.left + margin + panel_width,
        client.bottom - margin,
    };
    HBRUSH panel_brush = CreateSolidBrush(RGB(15, 19, 26));
    HPEN panel_pen = CreatePen(PS_SOLID, 1, RGB(54, 64, 73));
    HFONT font = CreateFontA(
        -12, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        ANTIALIASED_QUALITY, FIXED_PITCH | FF_MODERN, "Consolas");
    HGDIOBJ old_brush = SelectObject(dc, panel_brush);
    HGDIOBJ old_pen = SelectObject(dc, panel_pen);
    HGDIOBJ old_font;
    KartVec3 right;
    KartVec3 forward;
    KartVec3 up;
    float speed;
    float acceleration;
    float forward_speed;
    float lateral_speed;
    float slip_angle;
    int y = panel.top + 7;
    int line = 0;
    char text[160];

    Rectangle(dc, panel.left, panel.top, panel.right, panel.bottom);
    old_font = SelectObject(dc, font);
    SetBkMode(dc, TRANSPARENT);

    orientation_axes(kart->orientation, &right, &forward, &up);
    speed = sqrtf(
        kart->linear_velocity.x * kart->linear_velocity.x +
        kart->linear_velocity.y * kart->linear_velocity.y +
        kart->linear_velocity.z * kart->linear_velocity.z);
    acceleration = sqrtf(
        demo->acceleration.x * demo->acceleration.x +
        demo->acceleration.y * demo->acceleration.y +
        demo->acceleration.z * demo->acceleration.z);
    forward_speed = vec_dot(kart->linear_velocity, forward);
    lateral_speed = vec_dot(kart->linear_velocity, right);
    slip_angle = atan2f(fabsf(lateral_speed), fabsf(forward_speed)) *
                 (180.0f / 3.14159265358979323846f);

#define TELEMETRY_LINE(colour, ...)                                           \
    do {                                                                      \
        snprintf(text, sizeof(text), __VA_ARGS__);                            \
        SetTextColor(dc, colour);                                             \
        TextOutA(dc, panel.left + 9, y + line * line_height, text,            \
                 (int)strlen(text));                                          \
        line += 1;                                                            \
    } while (0)

    TELEMETRY_LINE(
        kart->drift.trigger_active ? RGB(255, 210, 90) :
        (kart->drift.slip_detected ? RGB(255, 140, 120) :
         (kart->drift.input_active ? RGB(255, 185, 55) : RGB(150, 205, 165))),
        "PHASE   %-8s beta %5.1f deg  (auto slip at 50)",
        drift_phase_name(kart), slip_angle);
    TELEMETRY_LINE(
        RGB(200, 212, 220),
        "POS     %8.2f %8.2f %8.2f", kart->position.x, kart->position.y,
        kart->position.z);
    TELEMETRY_LINE(
        RGB(120, 215, 245),
        "VEL     %8.2f %8.2f %8.2f  |v| %6.2f",
        kart->linear_velocity.x, kart->linear_velocity.y,
        kart->linear_velocity.z, speed);
    TELEMETRY_LINE(
        RGB(255, 175, 110),
        "ACC     %8.2f %8.2f %8.2f  |a| %6.2f",
        demo->acceleration.x, demo->acceleration.y, demo->acceleration.z,
        acceleration);
    TELEMETRY_LINE(
        RGB(200, 212, 220),
        "OMEGA   %8.3f %8.3f %8.3f  yaw %6.1f d/s",
        kart->angular_velocity.x, kart->angular_velocity.y,
        kart->angular_velocity.z,
        kart->angular_velocity.z * (180.0f / 3.14159265358979323846f));
    TELEMETRY_LINE(
        RGB(200, 212, 220),
        "AXIS    vf %6.2f  vs %6.2f  up_z %5.3f", forward_speed, lateral_speed,
        up.z);
    TELEMETRY_LINE(
        kart->grounded ? RGB(150, 205, 165) : RGB(255, 140, 120),
        "GROUND  %-3s contacts %u  (loads in the wheel panel)",
        kart->grounded ? "yes" : "AIR", demo->last_step.wheel_contacts);
    TELEMETRY_LINE(
        RGB(200, 212, 220),
        "STEER   in %5.2f  applied %6.2f deg  hyst %6.2f",
        demo->steering.value,
        kart->previous_steer_angle_rad * (180.0f / 3.14159265358979323846f),
        kart->config.max_steer_angle_deg);
    TELEMETRY_LINE(
        RGB(200, 212, 220),
        "DRIFT   linger %5.2f  trigger %5.2f  entry %s",
        kart->drift.linger_timer, kart->drift.trigger_timer,
        kart->drift.entry_was_forward ? "fwd" : "-");
    TELEMETRY_LINE(
        demo->boost_active ? RGB(75, 225, 255) : RGB(200, 212, 220),
        "BOOST   item %5.2fs  inst %-3s %5.2fs  opp %5.2fs",
        (float)kart->timed_boost.remaining_ms * 0.001f,
        kart->instant_boost.active ? "ON" : "off",
        kart->instant_boost.active_timer,
        kart->instant_boost.opportunity_timer);
    TELEMETRY_LINE(
        RGB(200, 212, 220),
        "FORCES  fwd %6.0f  brake %6.0f  Gf %4.2f  Gr %4.2f",
        kart->config.forward_accel_force, kart->config.grip_brake_force,
        kart->config.front_grip_factor, kart->config.rear_grip_factor);
    TELEMETRY_LINE(
        RGB(200, 212, 220),
        "DRAG    scale x%4.2f  air %5.2f  ground %5.3f  m %5.1f",
        kart->grounded_drag_scale, kart->config.air_friction,
        kart->config.drag_factor, kart->config.mass);
    TELEMETRY_LINE(
        demo->gauge.rate > 0.0f ? RGB(255, 210, 90) : RGB(200, 212, 220),
        "GAUGE   %-17s %5.1f%%  rate %6.2f  W %4.2f",
        kart_gauge_model_name(demo->gauge.model),
        demo->gauge_config.full_value > 0.0f
            ? demo->gauge.value / demo->gauge_config.full_value * 100.0f : 0.0f,
        demo->gauge.rate, demo->gauge.contact_weight);
    TELEMETRY_LINE(
        demo->last_step.body_contacts != 0 ? RGB(255, 140, 120)
                                           : RGB(150, 165, 180),
        "STEP    sub %2u  wheels %u  body %u  wall %4.1f  gnd %4.1f",
        demo->last_step.substeps, demo->last_step.wheel_contacts,
        demo->last_step.body_contacts, demo->last_step.wall_impact_speed,
        demo->last_step.ground_impact_speed);

#undef TELEMETRY_LINE

    SelectObject(dc, old_font);
    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
    DeleteObject(panel_brush);
    DeleteObject(panel_pen);
    DeleteObject(font);
}

static void draw_scene(HWND window, HDC target, const Demo3DState *demo)
{
    const bool topdown = demo->view_mode == DEMO_VIEW_TOPDOWN;
    RECT client;
    HDC buffer;
    HBITMAP bitmap;
    HGDIOBJ old_bitmap;
    HBRUSH sky;
    Camera3D camera;
    KartVec3 body_right;
    KartVec3 body_forward;
    KartVec3 body_up;
    float speed;
    float forward_speed;
    float lateral_speed;
    float slip_angle;
    int speedometer_kmh;
    char status[384];
    char help[256];

    GetClientRect(window, &client);
    buffer = CreateCompatibleDC(target);
    bitmap = CreateCompatibleBitmap(target, client.right, client.bottom);
    old_bitmap = SelectObject(buffer, bitmap);
    sky = CreateSolidBrush(topdown ? RGB(22, 25, 31) : RGB(19, 24, 33));
    FillRect(buffer, &client, sky);
    DeleteObject(sky);

    camera = make_chase_camera(&demo->camera_pose, client);
    if (topdown) {
        draw_scene_topdown(buffer, client, demo);
    } else {
        draw_track(buffer, client, camera, demo->track_spec);
        draw_track_scene(
            buffer, client, camera, demo->track_spec, active_track_scene(demo));
        draw_skid_marks(buffer, client, camera, demo);
        draw_boost_effect(buffer, client, camera, demo);
        draw_kart(buffer, client, camera, demo);
        if (demo->show_vectors) {
            draw_motion_vectors(
                buffer, client, camera, &demo->kart, demo->acceleration);
        }
    }
    draw_track_minimap(buffer, client, demo);

    orientation_axes(
        demo->kart.orientation, &body_right, &body_forward, &body_up);
    (void)body_up;
    speed = sqrtf(
        demo->kart.linear_velocity.x * demo->kart.linear_velocity.x +
        demo->kart.linear_velocity.y * demo->kart.linear_velocity.y +
        demo->kart.linear_velocity.z * demo->kart.linear_velocity.z);
    forward_speed = vec_dot(demo->kart.linear_velocity, body_forward);
    lateral_speed = vec_dot(demo->kart.linear_velocity, body_right);
    slip_angle = atan2f(fabsf(lateral_speed), fabsf(forward_speed)) *
                 (180.0f / 3.14159265358979323846f);
    speedometer_kmh = kart_speedometer_kmh(demo->kart.linear_velocity);
    SetBkMode(buffer, TRANSPARENT);
    SetTextColor(buffer, RGB(235, 240, 245));
    snprintf(
        status,
        sizeof(status),
        "speed %.2f m/s | slip %.1f deg | vf %.1f vs %.1f | AUTO %s",
        speed,
        slip_angle,
        forward_speed,
        lateral_speed,
        demo->kart.drift.slip_detected ? "ON" : "off");
    TextOutA(buffer, 16, 12, status, (int)strlen(status));
    snprintf(
        help,
        sizeof(help),
        "Arrows: drive  Shift/W: drift  Ctrl/D: boost  C: camera  V: %s vectors  P: parameters  K: kart  T: track  F: drag trigger  S: screenshot  R: reset  M: %s  B: %s bounds",
        demo->show_vectors ? "hide" : "show",
        demo->show_model ? "box hull" : "mesh hull",
        demo->show_model_bounds ? "hide" : "show");
    TextOutA(buffer, 16, 32, help, (int)strlen(help));
    /* The inferred layers, kept on their own line so they read as what they
       are rather than as part of the recovered demo. */
    snprintf(
        help,
        sizeof(help),
        "(experimental)  E: gear model [%s]  G: gauge model [%s]",
        demo->gearbox.mode == KART_GEAR_MULTI ? "multi" : "single",
        kart_gauge_model_name(demo->gauge.model));
    SetTextColor(buffer, RGB(150, 165, 180));
    TextOutA(buffer, 16, 52, help, (int)strlen(help));
    SetTextColor(buffer, RGB(235, 240, 245));
    snprintf(
        status,
        sizeof(status),
        "%s | %s (%s) %.1f x %.1f | scene %s | kart %s %.3f x %.3f | h %.2f",
        topdown ? "TOP-DOWN" : "CHASE",
        demo->track_spec->display_name,
        demo->track_spec->asset_name,
        kart_demo_track_width(demo->track_spec),
        kart_demo_track_length(demo->track_spec),
        active_track_scene(demo) != NULL ? "KTRK+collision" : "bounds",
        demo->kart_spec->asset_name,
        demo->kart.geometry.half_width * 2.0f,
        demo->kart.geometry.half_length * 2.0f,
        demo->kart.position.z);
    SetTextColor(
        buffer,
        demo->kart.drift.slip_detected ? RGB(255, 185, 55) : RGB(180, 195, 205));
    kart_demo_text_out_utf8(buffer, 16, 72, status);
    snprintf(
        status,
        sizeof(status),
        "DRIFT %s | ITEM %s %.2fs | INSTANT READY %.2fs | INSTANT %s | drag x%.2f | skids %u",
        drift_visual_active(&demo->kart) ? "ON" : "off",
        demo->kart.timed_boost.active ? "ON" : "off",
        (float)demo->kart.timed_boost.remaining_ms * 0.001f,
        demo->kart.instant_boost.opportunity_timer,
        demo->kart.instant_boost.active ? "ON" : "off",
        demo->kart.grounded_drag_scale,
        demo->skid_count);
    SetTextColor(
        buffer,
        demo->boost_active ? RGB(75, 225, 255) :
        (drift_visual_active(&demo->kart) ? RGB(255, 185, 55) : RGB(180, 195, 205)));
    TextOutA(buffer, 16, 92, status, (int)strlen(status));
    if (demo->respawn_notice_ms != 0) {
        static const char notice[] = "FELL THROUGH THE TRACK - RESPAWNED";
        SetTextColor(buffer, RGB(255, 120, 120));
        TextOutA(buffer, 16, 112, notice, (int)strlen(notice));
    }
    /* 3, 2, 1 as the deadline approaches, then START; the recovered cues fire at
       the same thresholds. */
    kart_demo_draw_countdown(
        buffer, client, demo->countdown_remaining_ms,
        (unsigned int)demo->start_notice_ms);
    if (demo->shot_notice_ms != 0) {
        char notice[96];
        snprintf(notice, sizeof(notice), "SAVED %s", demo->shot_name);
        SetTextColor(buffer, RGB(140, 235, 255));
        TextOutA(buffer, 16, 132, notice, (int)strlen(notice));
    }
    draw_telemetry(buffer, client, demo);
    kart_demo_draw_gauge(
        buffer, client,
        demo->gauge_config.full_value > 0.0f
            ? demo->gauge.value / demo->gauge_config.full_value : 0.0f,
        demo->gauge.boosters, demo->gauge.rate > 0.0f,
        kart_gauge_model_name(demo->gauge.model));
    kart_demo_draw_wheel_load(
        buffer, client, demo->kart.wheels.compression, demo->kart.grounded);
    kart_demo_draw_tachometer(
        buffer, client,
        demo->gearbox.mode == KART_GEAR_MULTI
            ? demo->gearbox.pitch : demo->sound.driver.motor_pitch,
        demo->sound.driver.motor_volume,
        /* The uncapped recovered ramp is only a reference for SINGLE. */
        demo->gearbox.mode == KART_GEAR_MULTI
            ? 0.0f : speed * KART_SOUND_MOTOR_SLOPE + KART_SOUND_MOTOR_BASE,
        demo->gearbox.mode == KART_GEAR_MULTI ? demo->gearbox.gear : 0);
    kart_demo_draw_speedometer(
        buffer, client, speedometer_kmh, demo->boost_active);
    if (topdown) {
        /* world_to_screen maps +X to screen left and +Y to screen up, and the
           view looks straight down, so +Z points at the viewer. */
        static const float axis_x[3] = {-1.0f, 0.0f, 0.0f};
        static const float axis_y[3] = {0.0f, -1.0f, 0.0f};
        static const float axis_toward[3] = {0.0f, 0.0f, 1.0f};
        kart_demo_draw_axis_gizmo(buffer, client, axis_x, axis_y, axis_toward);
    } else {
        /* Project each world axis onto the chase camera's basis. Screen Y grows
           downward and camera.forward points into the screen, so both are
           negated to get screen-space directions. */
        static const KartVec3 WORLD_AXES[3] = {
            {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        float axis_x[3];
        float axis_y[3];
        float axis_toward[3];
        int axis;
        for (axis = 0; axis < 3; ++axis) {
            axis_x[axis] = vec_dot(WORLD_AXES[axis], camera.right);
            axis_y[axis] = -vec_dot(WORLD_AXES[axis], camera.up);
            axis_toward[axis] = -vec_dot(WORLD_AXES[axis], camera.forward);
        }
        kart_demo_draw_axis_gizmo(buffer, client, axis_x, axis_y, axis_toward);
    }

    BitBlt(target, 0, 0, client.right, client.bottom, buffer, 0, 0, SRCCOPY);
    SelectObject(buffer, old_bitmap);
    DeleteObject(bitmap);
    DeleteDC(buffer);
}

static LRESULT CALLBACK window_proc(HWND window, UINT message, WPARAM wparam, LPARAM lparam)
{
    Demo3DState *demo = (Demo3DState *)GetWindowLongPtr(window, GWLP_USERDATA);
    switch (message) {
    case WM_CREATE: {
        CREATESTRUCT *create = (CREATESTRUCT *)lparam;
        demo = (Demo3DState *)create->lpCreateParams;
        SetWindowLongPtr(window, GWLP_USERDATA, (LONG_PTR)demo);
        g_input_window = window;
        load_track_scenes(create->hInstance, demo);
        load_kart_models(create->hInstance, demo);
        kart_demo_sound_start(create->hInstance, &demo->sound);
        kart_demo_minimap_set_load(create->hInstance, &demo->minimaps);
        reset_kart(demo);
        SetTimer(window, 1, 16, NULL);
        return 0;
    }
    case WM_KEYDOWN:
    case WM_KEYUP:
        if (demo != NULL && (wparam == VK_LEFT || wparam == VK_RIGHT)) {
            kart_steering_key_event(
                &demo->steering,
                wparam == VK_LEFT ? KART_STEERING_LEFT : KART_STEERING_RIGHT,
                message == WM_KEYDOWN);
            return 0;
        }
        return DefWindowProc(window, message, wparam, lparam);
    case WM_KILLFOCUS:
        if (demo != NULL) {
            kart_steering_input_reset(&demo->steering);
        }
        return 0;
    case WM_TIMER: {
        DWORD now;
        DWORD elapsed;
        KartSimulationStepResult step = {0};
        KartSimulationControls controls = {0};
        const KartSimulationWorld world = {
            .query_ground = query_track_ground,
            .query_body_collisions = query_track_walls,
            .user_data = demo,
        };
        if (demo == NULL) {
            return 0;
        }
        now = GetTickCount();
        elapsed = now - demo->previous_tick;
        demo->previous_tick = now;
        if (elapsed > 50) {
            elapsed = 50;
        }
        controls.forward_input = key_down(VK_UP) ? 1.0f : 0.0f;
        controls.reverse_input = key_down(VK_DOWN) ? 1.0f : 0.0f;
        /* The camera and bounds map now use the same handedness as top-down. */
        controls.steering_input = demo->steering.value;
        controls.reverse_steering = false;
        controls.drift_input = key_down(VK_SHIFT) || key_down('W');
        {
            /* The gauge models spend a charge on the press that starts a
               booster; the infinite model lets every press through. */
            const bool boost_down = key_down(VK_CONTROL) || key_down('D');
            if (!boost_down) {
                demo->boost_press_allowed = false;
            } else if (!demo->boost_press_allowed &&
                       !demo->kart.timed_boost.active &&
                       controls.forward_input != 0.0f) {
                /* The engine only starts an item boost while accelerating, so
                   the charge is spent at that moment rather than on a press
                   that cannot fire. */
                demo->boost_press_allowed = kart_gauge_take_booster(&demo->gauge);
            }
            controls.boost_active = boost_down && demo->boost_press_allowed;
        }
        {
            const bool kart_key_down = key_down('K') != 0;
            const bool track_key_down = key_down('T') != 0;
            const bool drag_key_down = key_down('F') != 0;
            const bool gear_key_down = key_down('E') != 0;
            const bool gauge_key_down = key_down('G') != 0;
            if (gear_key_down && !demo->gear_key_was_down) {
                demo->gearbox.mode =
                    demo->gearbox.mode == KART_GEAR_SINGLE
                        ? KART_GEAR_MULTI : KART_GEAR_SINGLE;
                demo->gearbox.gear = 1;
            }
            demo->gear_key_was_down = gear_key_down;
            if (gauge_key_down && !demo->gauge_key_was_down) {
                /* Cycles infinite booster -> slip integral -> slip x
                   suspension, so the three can be compared back to back. */
                demo->gauge.model = (KartGaugeModel)
                    ((demo->gauge.model + 1) % KART_GAUGE_MODEL_COUNT);
                demo->gauge.value = 0.0f;
                demo->gauge.boosters = 0;
            }
            demo->gauge_key_was_down = gauge_key_down;
            const bool shot_key_down = key_down('S') != 0;
            const bool view_key_down = key_down('C') != 0;
            const bool vector_key_down = key_down('V') != 0;
            const bool param_key_down = key_down('P') != 0;
            const bool model_key_down = key_down('M') != 0;
            const bool bounds_key_down = key_down('B') != 0;
            if (model_key_down && !demo->model_key_was_down) {
                demo->show_model = !demo->show_model;
            }
            demo->model_key_was_down = model_key_down;
            if (bounds_key_down && !demo->bounds_key_was_down) {
                demo->show_model_bounds = !demo->show_model_bounds;
            }
            demo->bounds_key_was_down = bounds_key_down;
            if (vector_key_down && !demo->vector_key_was_down) {
                demo->show_vectors = !demo->show_vectors;
            }
            demo->vector_key_was_down = vector_key_down;
            if (param_key_down && !demo->param_key_was_down) {
                kart_demo_params_open(
                    (HINSTANCE)GetWindowLongPtr(window, GWLP_HINSTANCE),
                    window, &demo->kart.config, &demo->kart_spec->dynamics,
                    &demo->gauge_config);
            }
            demo->param_key_was_down = param_key_down;
            if (view_key_down && !demo->view_key_was_down) {
                demo->view_mode = demo->view_mode == DEMO_VIEW_CHASE
                    ? DEMO_VIEW_TOPDOWN
                    : DEMO_VIEW_CHASE;
            }
            demo->view_key_was_down = view_key_down;
            if (kart_key_down && !demo->kart_key_was_down) {
                const KartDemoKartSpec *selected =
                    kart_demo_popup_select_kart(window, demo->kart_spec);
                if (selected != demo->kart_spec) {
                    /* Swap the kart under the driver: tuning and hull change,
                       the pose, velocity and boost state carry over. */
                    demo->kart_spec = selected;
                    demo->kart.config = selected->dynamics;
                    demo->kart.geometry = selected->geometry;
                    demo->kart.grounded_drag_scale =
                        selected->geometry.grounded_drag_scale *
                        (demo->drag_trigger_active ? 4.0f : 1.0f);
                }
                demo->previous_tick = GetTickCount();
            }
            if (track_key_down && !demo->track_key_was_down) {
                const KartDemoTrackSpec *selected =
                    kart_demo_popup_select_track(window, demo->track_spec);
                if (selected != demo->track_spec) {
                    demo->track_spec = selected;
                    /* A new track is a new race, so the lights run again. */
                    demo->countdown.armed = false;
                    reset_kart(demo);
                }
                demo->previous_tick = GetTickCount();
            }
            if (drag_key_down && !demo->drag_key_was_down) {
                kart_simulation_multiply_grounded_drag_scale(
                    &demo->kart, demo->drag_trigger_active ? 0.25f : 4.0f);
                demo->drag_trigger_active = !demo->drag_trigger_active;
            }
            demo->kart_key_was_down = kart_key_down;
            demo->track_key_was_down = track_key_down;
            if (shot_key_down && !demo->shot_key_was_down) {
                if (kart_demo_save_screenshot(
                        window, demo->shot_name, sizeof(demo->shot_name))) {
                    demo->shot_notice_ms = 2000;
                } else {
                    snprintf(demo->shot_name, sizeof(demo->shot_name),
                             "screenshot failed");
                    demo->shot_notice_ms = 2000;
                }
            }
            demo->shot_key_was_down = shot_key_down;
            demo->drag_key_was_down = drag_key_down;
        }
        /* Escape only dismisses the K and T popups, which TrackPopupMenu
           already handles. Closing the window is left to the usual Alt+F4 and
           title-bar routes. */
        {
            /* Recovered countdown: cues at T-3000/-2000/-1000 and release at T.
               The kart is held at the line until GO. */
            const KartCountdownCues cues =
                kart_countdown_update(&demo->countdown, now);
            const bool forward_down = controls.forward_input != 0.0f;
            if (cues.play_three) kart_demo_sound_play_count(&demo->sound, 0);
            if (cues.play_two) kart_demo_sound_play_count(&demo->sound, 1);
            if (cues.play_one) kart_demo_sound_play_count(&demo->sound, 2);
            if (cues.play_go) {
                kart_demo_sound_play_count(&demo->sound, 3);
                demo->start_notice_ms = 1500;
            }
            demo->countdown_remaining_ms = cues.remaining_ms;

            /* The start boost is granted on the press itself, measured against
               the same deadline the countdown uses. */
            if (forward_down && !demo->forward_key_was_down &&
                kart_countdown_start_boost_granted(&demo->countdown, now)) {
                kart_timed_boost_start(
                    &demo->kart.timed_boost, 1.0f,
                    KART_START_BOOST_DURATION_MS);
            }
            demo->forward_key_was_down = forward_down;

            /* Holding drift with the throttle before GO revs the booster: the
               idle loop plays and the kart reads as boosting until either key
               is let go. No other booster works before the line drops. */
            demo->countdown_rev_active =
                !cues.released && forward_down && controls.drift_input;
            kart_demo_sound_set_booster_idle(
                &demo->sound, demo->countdown_rev_active);

            if (!cues.released) {
                /* Before GO the throttle does nothing but the engine still
                   idles, which is what the countdown is for. */
                controls.forward_input = 0.0f;
                controls.reverse_input = 0.0f;
                controls.boost_active = false;
            }
        }
        if (key_down('R')) {
            reset_kart(demo);
        } else if (elapsed != 0) {
            step = kart_simulate_milliseconds(&demo->kart, &controls, &world, elapsed);
            /* Telemetry: the step's own report, and the acceleration it came
               out with, differenced over the same clock the step used. */
            demo->last_step = step;
            demo->acceleration = vec_scale(
                vec_sub(demo->kart.linear_velocity, demo->previous_velocity),
                1000.0f / (float)elapsed);
            demo->previous_velocity = demo->kart.linear_velocity;
            {
                /* The gauge integrates the same rear-axle slip the tire model
                   uses, so it needs the body-axis split of the velocity. */
                KartVec3 body_right;
                KartVec3 body_forward;
                KartVec3 body_up;
                orientation_axes(
                    demo->kart.orientation, &body_right, &body_forward,
                    &body_up);
                kart_gauge_update(
                    &demo->gauge, &demo->gauge_config, &demo->kart,
                    vec_dot(demo->kart.linear_velocity, body_forward),
                    vec_dot(demo->kart.linear_velocity, body_right),
                    drift_visual_active(&demo->kart),
                    (float)elapsed * 0.001f);
            }
        }
        if (demo->kart.position.z <
            kart_demo_track_fall_limit(demo->track_spec)) {
            reset_kart(demo);
            demo->respawn_notice_ms = 1500;
        } else if (demo->respawn_notice_ms > elapsed) {
            demo->respawn_notice_ms -= elapsed;
        } else {
            demo->respawn_notice_ms = 0;
        }
        if (demo->shot_notice_ms > elapsed) {
            demo->shot_notice_ms -= elapsed;
        } else {
            demo->shot_notice_ms = 0;
        }
        if (demo->start_notice_ms > elapsed) {
            demo->start_notice_ms -= elapsed;
        } else {
            demo->start_notice_ms = 0;
        }
        demo->boost_active = kart_any_boost_active(
                                 &demo->kart.timed_boost,
                                 &demo->kart.instant_boost) ||
                             demo->countdown_rev_active;
        {
            /* The original reads the kart's velocity magnitude and its
               booster-like flag, then advances the whole camera together. */
            const KartVec3 v = demo->kart.linear_velocity;
            const float speed = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
            demo->camera_pose = kart_chase_camera_update(
                &demo->camera_follow, demo->kart.position,
                demo->kart.orientation, speed,
                /* Revving on the line only looks like a boost; the camera is
                   left alone until a real one fires. */
                demo->boost_active && !demo->countdown_rev_active, elapsed,
                KART_CHASE_FOLLOW_OVERHEAD_MS);
        }
        demo->simulation_time_ms += elapsed;
        kart_demo_sound_update(
            &demo->sound, &demo->kart,
            step.wall_impact_speed, step.ground_impact_speed, now);
        {
            /* The gearbox only re-pitches the engine loop the sound driver
               already opened, so SINGLE leaves the recovered note untouched and
               MULTI replaces it with its own sawtooth. */
            const KartVec3 v = demo->kart.linear_velocity;
            kart_gearbox_update(
                &demo->gearbox, sqrtf(v.x * v.x + v.y * v.y + v.z * v.z),
                (float)elapsed * 0.001f);
            if (demo->gearbox.mode == KART_GEAR_MULTI &&
                demo->sound.audio != NULL && demo->sound.motor_voice >= 0) {
                kart_audio_set_voice(
                    demo->sound.audio, demo->sound.motor_voice,
                    demo->sound.driver.motor_volume, demo->gearbox.pitch);
            }
        }
        update_skid_marks(demo);
        InvalidateRect(window, NULL, FALSE);
        return 0;
    }
    case WM_PAINT: {
        PAINTSTRUCT paint;
        HDC dc = BeginPaint(window, &paint);
        if (demo != NULL) {
            draw_scene(window, dc, demo);
        }
        EndPaint(window, &paint);
        return 0;
    }
    case WM_DESTROY:
        KillTimer(window, 1);
        if (demo != NULL) {
            kart_demo_sound_stop(&demo->sound);
            free_track_scenes(demo);
            kart_demo_minimap_set_free(&demo->minimaps);
        }
        PostQuitMessage(0);
        return 0;
    default:
        return DefWindowProc(window, message, wparam, lparam);
    }
}

int WINAPI WinMain(HINSTANCE instance, HINSTANCE previous, LPSTR command_line, int show)
{
    static const char CLASS_NAME[] = "KartPhysicsWindow";
    WNDCLASSA window_class = {0};
    Demo3DState demo = {0};
    HWND window;
    MSG message;
    (void)previous;
    (void)command_line;
    (void)show;

    demo.gauge_config = kart_gauge_default_config();
    /* The recovered hull is the better default now that there is one; M falls
       back to the box the physics constants describe. */
    demo.show_model = true;
    window_class.lpfnWndProc = window_proc;
    window_class.hInstance = instance;
    window_class.lpszClassName = CLASS_NAME;
    window_class.hCursor = LoadCursor(NULL, IDC_ARROW);
    window_class.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    if (!RegisterClassA(&window_class)) {
        return 1;
    }
    window = CreateWindowExA(
        0,
        CLASS_NAME,
        "KartRider Demo Physics",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        1280,
        820,
        NULL,
        NULL,
        instance,
        &demo);
    if (window == NULL) {
        return 1;
    }
    ShowWindow(window, SW_SHOW);
    UpdateWindow(window);
    while (GetMessage(&message, NULL, 0, 0) > 0) {
        TranslateMessage(&message);
        DispatchMessage(&message);
    }
    return (int)message.wParam;
}
