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
#include "kart_course.h"
#include "kart_sound_win32.h"
#include "kart_track_collision.h"
#include "kart_track_scene.h"
#include "kart_skydome_resources.h"
#include "kart_track_scene_resources.h"
#include "kart_track_texture.h"
#include "kart_track_texture_resources.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SKID_MARK_POOL_SIZE 50u
#define SKID_MARK_MAX_CROSS_SECTIONS 44u
#define SKID_MARK_SIDE_COUNT 2u
#define SKID_MARK_WIDTH 0.28f
#define SKID_MARK_SURFACE_BIAS 0.02f
#define SKID_MARK_LOCAL_X 0.61f
#define SKID_MARK_LOCAL_Y 0.5f

/* Half-extents of the top-down view's window onto the world, in metres. */
static const float VIEW_HALF_WIDTH = 55.0f;
static const float VIEW_HALF_HEIGHT = 38.0f;

/* The two demos were the same simulation behind two renderers, so they are one
   executable now and V switches between them. */
typedef enum DemoViewMode {
    DEMO_VIEW_CHASE = 0,
    DEMO_VIEW_TOPDOWN = 1
} DemoViewMode;

typedef enum DemoTrackRenderMode {
    DEMO_TRACK_RENDER_ALPHA_85 = 0,
    DEMO_TRACK_RENDER_DEPTH_ALPHA_190 = 1,
    DEMO_TRACK_RENDER_TEXTURED = 2,
    DEMO_TRACK_RENDER_MODE_COUNT = 3
} DemoTrackRenderMode;

typedef struct SkidMarkCrossSection {
    KartVec3 edge[2];
    float texture_v;
} SkidMarkCrossSection;

typedef struct SkidMarkStrip {
    SkidMarkCrossSection sections[SKID_MARK_MAX_CROSS_SECTIONS];
    unsigned int section_count;
    unsigned int sequence;
    bool building;
} SkidMarkStrip;

typedef struct Demo3DState {
    KartSimulationState kart;
    const KartDemoKartSpec *kart_spec;
    const KartDemoTrackSpec *track_spec;
    DWORD previous_tick;
    unsigned int simulation_time_ms;
    SkidMarkStrip skid_marks[SKID_MARK_SIDE_COUNT][SKID_MARK_POOL_SIZE];
    unsigned int skid_next[SKID_MARK_SIDE_COUNT];
    unsigned int skid_sequence;
    int skid_building[SKID_MARK_SIDE_COUNT];
    KartTrackTextureImage skid_texture;
    DemoViewMode view_mode;
    bool view_key_was_down;
    bool vector_key_was_down;
    bool param_key_was_down;
    bool gauge_key_was_down;
    bool booster_storage_key_was_down;
    bool instant_model_key_was_down;
    bool boost_cutoff_key_was_down;
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
    /* Completed WM_PAINT frames measured with the performance counter. */
    LARGE_INTEGER fps_frequency;
    LARGE_INTEGER fps_sample_start;
    unsigned int fps_sample_frames;
    double fps;
    /* Drift plus throttle held on the line: booster idle loop and boost look. */
    bool countdown_rev_active;
    bool forward_key_was_down;
    KartTrackScene scenes[KART_TRACK_SCENE_CAPACITY];
    /* The skydome that ships beside each track, in the same order. Meshes with
       no exported dome stay empty. */
    KartTrackScene skydomes[KART_TRACK_SCENE_CAPACITY];
    /* The recovered kart models, in KARTS[] order. Loaded once up front: all 26
       together are about 340 KB of geometry, so there is nothing to gain from
       loading them on demand when the driver presses K. */
    KartTrackScene models[KART_MODEL_CAPACITY];
    KartModelParts model_parts[KART_MODEL_CAPACITY];
    bool show_model_bounds;
    bool bounds_key_was_down;
    /* Draws the course's gate quads over the track so the checkpoints the
       progress code tests against can be seen. Off by default. */
    bool show_gates;
    bool gate_key_was_down;
    /* The tracks' texture assets, one shared table for all 13. `textured`
       picks the solid renderer that samples them over the wireframe, and
       starts off: the mesh is what the rest of the demo is about. */
    KartTrackTextureTable textures;
    bool textures_ready;
    DemoTrackRenderMode track_render_mode;
    bool texture_key_was_down;
    /* The kart's own skin, on its own key: it only applies in the textured
       renderer, and the state colour is still the more readable of the two. */
    bool kart_textured;
    bool kart_texture_key_was_down;
    /* The colortable index, and what the skin was last repainted from: the
       repaint is in place, so the asset's own texels are kept to redo it. */
    unsigned int kart_colour;
    bool kart_colour_key_was_down;
    const KartTrackTextureImage *painted_skin;
    unsigned int painted_colour;
    uint16_t *skin_texels;
    size_t skin_texel_count;
    KartDemoMinimapSet minimaps;
    /* The original's checkpoint graph for the selected track, rebuilt whenever
       the track changes. `course_ready` is false for the synthetic flat track,
       which has no course asset. */
    KartCourse course;
    KartCourseProgress progress;
    bool course_ready;
    KartVec3 previous_position;
    /* 0x00458000 arms a respawn and only moves the kart 500 ms later, both for
       a fall out of the world and for the original's own reset command. */
    DWORD respawn_arm_ms;
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

/* The domes are ordinary KTRK exports, so the same loader reads them. */
static void load_skydomes(HINSTANCE instance, Demo3DState *demo)
{
#if defined(KART_EMBED_SKYDOMES)
    unsigned int i;
    const unsigned int count = kart_demo_track_count();
    for (i = 0; i < count && i < KART_TRACK_SCENE_CAPACITY; ++i) {
        if (KART_SKYDOME_RESOURCE_IDS[i] == 0) continue;
        load_scene_resource(
            instance, KART_SKYDOME_RESOURCE_IDS[i], &demo->skydomes[i]);
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

/* The texture table is one resource shared by every track, so it is loaded once
   and looked up by the theme the track belongs to. */
static void load_track_textures(HINSTANCE instance, Demo3DState *demo)
{
#if defined(KART_EMBED_TRACK_TEXTURES)
    HRSRC resource = FindResourceA(
        instance, MAKEINTRESOURCEA(IDR_TRACK_TEXTURES), RT_RCDATA);
    HGLOBAL loaded;
    const void *data;
    DWORD size;
    if (resource == NULL) {
        return;
    }
    loaded = LoadResource(instance, resource);
    size = SizeofResource(instance, resource);
    data = loaded != NULL ? LockResource(loaded) : NULL;
    demo->textures_ready = data != NULL && size != 0 &&
        kart_track_texture_load_compressed(&demo->textures, data, (size_t)size);
#else
    (void)instance;
    (void)demo;
#endif
}

/* `effect.rho/skidmark/skidmark.tga`, loaded without image-library conversion
   so the executable samples the exact 64x32 BGRA pixels shipped by the demo. */
static void load_skidmark_texture(HINSTANCE instance, Demo3DState *demo)
{
    HRSRC resource = FindResourceA(
        instance, MAKEINTRESOURCEA(IDR_SKIDMARK_TEXTURE), RT_RCDATA);
    HGLOBAL loaded;
    const unsigned char *bytes;
    DWORD size;
    uint32_t width;
    uint32_t height;
    size_t texel_count;
    size_t mask_bytes;
    uint32_t y;
    if (resource == NULL) return;
    loaded = LoadResource(instance, resource);
    size = SizeofResource(instance, resource);
    bytes = loaded != NULL ? (const unsigned char *)LockResource(loaded) : NULL;
    if (bytes == NULL || size < 18u || bytes[2] != 2u || bytes[16] != 32u) return;
    width = (uint32_t)bytes[12] | ((uint32_t)bytes[13] << 8);
    height = (uint32_t)bytes[14] | ((uint32_t)bytes[15] << 8);
    texel_count = (size_t)width * (size_t)height;
    if (width != 64u || height != 32u || 18u + texel_count * 4u > (size_t)size) return;
    mask_bytes = (texel_count + 7u) / 8u;
    demo->skid_texture.texels = (uint16_t *)malloc(
        texel_count * sizeof(*demo->skid_texture.texels));
    demo->skid_texture.mask = (uint8_t *)calloc(mask_bytes, 1u);
    demo->skid_texture.alpha = (uint8_t *)malloc(texel_count);
    if (demo->skid_texture.texels == NULL || demo->skid_texture.mask == NULL ||
        demo->skid_texture.alpha == NULL) {
        free(demo->skid_texture.texels);
        free(demo->skid_texture.mask);
        free(demo->skid_texture.alpha);
        memset(&demo->skid_texture, 0, sizeof(demo->skid_texture));
        return;
    }
    demo->skid_texture.width = width;
    demo->skid_texture.height = height;
    demo->skid_texture.flags =
        KART_TRACK_TEXTURE_MASKED | KART_TRACK_TEXTURE_ALPHA8 |
        KART_TRACK_TEXTURE_BLEND_ALPHA8;
    for (y = 0; y < height; ++y) {
        const uint32_t source_y =
            (bytes[17] & 0x20u) != 0u ? y : height - 1u - y;
        uint32_t x;
        for (x = 0; x < width; ++x) {
            const size_t destination = (size_t)y * width + x;
            const size_t source = 18u + ((size_t)source_y * width + x) * 4u;
            const uint8_t b = bytes[source];
            const uint8_t g = bytes[source + 1u];
            const uint8_t r = bytes[source + 2u];
            const uint8_t a = bytes[source + 3u];
            demo->skid_texture.texels[destination] = (uint16_t)(
                ((uint16_t)(r >> 3) << 11) |
                ((uint16_t)(g >> 2) << 5) |
                (uint16_t)(b >> 3));
            demo->skid_texture.alpha[destination] = a;
            if (a != 0u) {
                demo->skid_texture.mask[destination >> 3] |=
                    (uint8_t)(1u << (destination & 7u));
            }
        }
    }
}

static void free_skidmark_texture(Demo3DState *demo)
{
    free(demo->skid_texture.texels);
    free(demo->skid_texture.mask);
    free(demo->skid_texture.alpha);
    memset(&demo->skid_texture, 0, sizeof(demo->skid_texture));
}

/* The theme half of a texture key. Track asset names are "<theme>_<code>", the
   same split DeveloperTools/AssetPipeline/pack_track_textures.py makes on the mesh file names. */
static void track_theme(const KartDemoTrackSpec *track, char *out, size_t size)
{
    const char *name = track != NULL ? track->asset_name : NULL;
    const char *separator = name != NULL ? strchr(name, '_') : NULL;
    size_t length;
    if (separator == NULL) {
        out[0] = '\0';
        return;
    }
    length = (size_t)(separator - name);
    if (length >= size) {
        length = size - 1u;
    }
    memcpy(out, name, length);
    out[length] = '\0';
}

static void free_track_scenes(Demo3DState *demo)
{
    unsigned int i;
    for (i = 0; i < KART_TRACK_SCENE_CAPACITY; ++i) {
        kart_track_scene_free(&demo->scenes[i]);
        kart_track_scene_free(&demo->skydomes[i]);
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

static const KartTrackScene *active_skydome(const Demo3DState *demo)
{
    unsigned int i;
    const unsigned int count = kart_demo_track_count();
    for (i = 0; i < count && i < KART_TRACK_SCENE_CAPACITY; ++i) {
        if (kart_demo_track_at(i) == demo->track_spec) {
            return demo->skydomes[i].mesh_count != 0 ? &demo->skydomes[i] : NULL;
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

static KartVec3 vec_rotate_z(KartVec3 value, float radians)
{
    const float cosine = cosf(radians);
    const float sine = sinf(radians);
    return (KartVec3){
        cosine * value.x - sine * value.y,
        sine * value.x + cosine * value.y,
        value.z};
}

static KartQuat quat_pre_rotate_z(KartQuat value, float radians)
{
    const float half = radians * 0.5f;
    const KartQuat turn = {cosf(half), 0.0f, 0.0f, sinf(half)};
    return (KartQuat){
        turn.w * value.w - turn.x * value.x - turn.y * value.y - turn.z * value.z,
        turn.w * value.x + turn.x * value.w + turn.y * value.z - turn.z * value.y,
        turn.w * value.y - turn.x * value.z + turn.y * value.w + turn.z * value.x,
        turn.w * value.z + turn.x * value.y - turn.y * value.x + turn.z * value.w};
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

/* Rebuilds the checkpoint graph for the selected track. The flat reference
   track has no scene and no course asset, so it simply has no checkpoints. */
static void load_course(Demo3DState *demo)
{
    const KartCourseAsset *asset =
        demo->track_spec != NULL
            ? kart_course_find_asset(demo->track_spec->asset_name) : NULL;
    kart_course_free(&demo->course);
    memset(&demo->progress, 0, sizeof demo->progress);
    demo->course_ready = asset != NULL && kart_course_build(&demo->course, asset);
    if (demo->course_ready) {
        /* 0x004247e0. Three laps is what the demo's own race is; nothing in
           the track asset carries it. */
        kart_course_set_lap_count(&demo->course, 3);
    }
}

/* The ground snap 0x004260e0 finishes the start grid with: a ray from 10 above
   the placed position, 100 down. */
static void snap_to_ground(Demo3DState *demo, KartVec3 *position)
{
    KartGroundHit hit;
    const KartVec3 above = {position->x, position->y, position->z + 10.0f};
    const KartVec3 down = {0.0f, 0.0f, -100.0f};
    if (query_track_ground(demo, above, down, &hit)) *position = hit.point;
}

static void reset_kart(Demo3DState *demo)
{
    const bool stored_instant_model = demo->kart.instant_boost.stored_model;
    const bool reverse_input_ends_boost = demo->kart.reverse_input_ends_boost;
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
    demo->kart.instant_boost.stored_model = stored_instant_model;
    demo->kart.reverse_input_ends_boost = reverse_input_ends_boost;
    /* 0x00424530: the start grid, taken from the course itself. Slot 0 sits on
       the course's own start pose, which the track asset fixes exactly -
       including which way round the lap is driven, something the start-line
       mesh alone never said. The measured start line is only the fallback for
       the synthetic track, which has no course. */
    if (demo->course_ready) {
        kart_course_start_pose(
            &demo->course, 0u, &start_position, &start_orientation);
        snap_to_ground(demo, &start_position);
        demo->kart.position = start_position;
        demo->kart.orientation = start_orientation;
        kart_course_progress_init(&demo->course, &demo->progress, start_position);
    } else {
        if (kart_demo_track_start_position(demo->track_spec, &start_position)) {
            demo->kart.position = start_position;
        }
        if (kart_demo_track_start_orientation(demo->track_spec, &start_orientation)) {
            demo->kart.orientation = start_orientation;
        }
    }
    demo->previous_position = demo->kart.position;
    demo->respawn_arm_ms = 0;
    demo->previous_tick = GetTickCount();
    demo->simulation_time_ms = 0;
    memset(demo->skid_marks, 0, sizeof(demo->skid_marks));
    memset(demo->skid_next, 0, sizeof(demo->skid_next));
    demo->skid_sequence = 0;
    demo->skid_building[0] = -1;
    demo->skid_building[1] = -1;
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

static unsigned int skid_mark_segment_count(const Demo3DState *demo)
{
    unsigned int count = 0;
    unsigned int side;
    for (side = 0; side < SKID_MARK_SIDE_COUNT; ++side) {
        unsigned int mark;
        for (mark = 0; mark < SKID_MARK_POOL_SIZE; ++mark) {
            const unsigned int sections = demo->skid_marks[side][mark].section_count;
            if (sections > 1u) count += sections - 1u;
        }
    }
    return count;
}

static void update_skid_marks(Demo3DState *demo)
{
    static const float rear_wheel_x[SKID_MARK_SIDE_COUNT] = {-0.8f, 0.8f};
    KartVec3 right;
    KartVec3 forward;
    KartVec3 up;
    const float local_x[SKID_MARK_SIDE_COUNT] = {
        -SKID_MARK_LOCAL_X, SKID_MARK_LOCAL_X};
    KartGroundHit hits[SKID_MARK_SIDE_COUNT];
    bool contacts[SKID_MARK_SIDE_COUNT] = {false, false};
    bool active;
    unsigned int side;

    orientation_axes(demo->kart.orientation, &right, &forward, &up);
    for (side = 0; side < SKID_MARK_SIDE_COUNT; ++side) {
        KartVec3 ray_start = demo->kart.position;
        const KartVec3 ray_delta = vec_scale(
            up, -2.0f * demo->kart.geometry.suspension_range);
        /* The original gates emission on wheel contacts 2 and 3. Reuse the
           recovered wheel query's exact rear-wheel locations for that test. */
        ray_start = vec_add(ray_start, vec_scale(
            right, demo->kart.geometry.half_width * rear_wheel_x[side]));
        ray_start = vec_add(ray_start, vec_scale(
            forward, -demo->kart.geometry.half_length * 0.8f));
        ray_start = vec_add(
            ray_start, vec_scale(up, demo->kart.geometry.suspension_range));
        contacts[side] = query_track_ground(
            demo, ray_start, ray_delta, &hits[side]);
    }
    active = drift_visual_active(&demo->kart) && contacts[0] && contacts[1];

    if (!active) {
        for (side = 0; side < SKID_MARK_SIDE_COUNT; ++side) {
            if (demo->skid_building[side] >= 0) {
                demo->skid_marks[side][demo->skid_building[side]].building = false;
                demo->skid_building[side] = -1;
            }
        }
        demo->previous_skid_active = false;
        return;
    }

    for (side = 0; side < SKID_MARK_SIDE_COUNT; ++side) {
        SkidMarkStrip *strip;
        KartVec3 centre = demo->kart.position;
        KartVec3 lateral;
        SkidMarkCrossSection *section;
        if (demo->skid_building[side] < 0) {
            const unsigned int index = demo->skid_next[side];
            strip = &demo->skid_marks[side][index];
            memset(strip, 0, sizeof(*strip));
            strip->sequence = ++demo->skid_sequence;
            strip->building = true;
            demo->skid_building[side] = (int)index;
            demo->skid_next[side] = (index + 1u) % SKID_MARK_POOL_SIZE;
        } else {
            strip = &demo->skid_marks[side][demo->skid_building[side]];
        }
        if (strip->section_count >= SKID_MARK_MAX_CROSS_SECTIONS) {
            const unsigned int index = demo->skid_next[side];
            strip->building = false;
            strip = &demo->skid_marks[side][index];
            memset(strip, 0, sizeof(*strip));
            strip->sequence = ++demo->skid_sequence;
            strip->building = true;
            demo->skid_building[side] = (int)index;
            demo->skid_next[side] = (index + 1u) % SKID_MARK_POOL_SIZE;
        }
        centre = vec_add(centre, vec_scale(right, local_x[side]));
        centre = vec_add(centre, vec_scale(forward, -SKID_MARK_LOCAL_Y));
        centre = vec_sub(centre, vec_scale(
            hits[side].normal,
            vec_dot(vec_sub(centre, hits[side].point), hits[side].normal)));
        /* KartRider.exe 0x00428D30 builds the second frame used by SkidMark
           from -linearVelocity and chassis column 2. 0x00470110/0x00470470
           then transforms (-1,0,0) by that frame, dots it with the same axis
           from the chassis frame, clamps the dot to 0.7, and finally applies
           width * 0.5. Keeping this velocity frame is what prevents steering
           angle from directly turning every ribbon cross-section. */
        if (vec_dot(demo->kart.linear_velocity, demo->kart.linear_velocity) == 0.0f) {
            lateral = vec_scale(right, -1.0f);
        } else {
            const KartVec3 backward = vec_normalize(
                vec_scale(demo->kart.linear_velocity, -1.0f));
            lateral = vec_cross(up, backward);
        }
        {
            float frame_dot = vec_dot(lateral, vec_scale(right, -1.0f));
            if (frame_dot < 0.7f) frame_dot = 0.7f;
            lateral = vec_scale(
                lateral, frame_dot * SKID_MARK_WIDTH * 0.5f);
        }
        section = &strip->sections[strip->section_count];
        section->edge[0] = vec_add(
            vec_sub(centre, lateral),
            vec_scale(hits[side].normal, SKID_MARK_SURFACE_BIAS));
        section->edge[1] = vec_add(
            vec_add(centre, lateral),
            vec_scale(hits[side].normal, SKID_MARK_SURFACE_BIAS));
        if (strip->section_count != 0u) {
            const SkidMarkCrossSection *previous =
                &strip->sections[strip->section_count - 1u];
            const KartVec3 centre_previous = vec_scale(
                vec_add(previous->edge[0], previous->edge[1]), 0.5f);
            const KartVec3 centre_current = vec_scale(
                vec_add(section->edge[0], section->edge[1]), 0.5f);
            const KartVec3 delta = vec_sub(centre_current, centre_previous);
            section->texture_v = previous->texture_v +
                sqrtf(vec_dot(delta, delta));
        }
        strip->section_count += 1u;
    }
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

/* --- the textured pass ---------------------------------------------------

   The wireframe and tinted-face passes draw through GDI, which has no depth
   buffer; they get away with it because they are line art over a translucent
   wash. Sampling the tracks' own textures needs a real one, so this pass
   rasterizes into the frame's DIB directly with a 1/z buffer.

   Nothing here is shaded. A KTRK mesh carries positions, UVs and the texture
   name track.1s gave its material, and no normals at all, so a texel is written
   exactly as the asset stored it; any lighting term would be invention. Meshes
   whose texture the demo's archives never shipped keep the floor/wall tint the
   untextured pass uses, so they still read as surfaces and still occlude. */

typedef struct SoftwareTarget {
    /* 32-bit BGRX, top row first, one uint32 per pixel. */
    uint32_t *pixels;
    /* 1/z, so a larger value is nearer and an untouched pixel is 0. */
    float *depth;
    int width;
    int height;
    bool depth_only;
    bool shade_depth_equal;
} SoftwareTarget;

/* Camera space: x right, y up, z forward. */
typedef struct RasterVertex {
    float x;
    float y;
    float z;
    float u;
    float v;
} RasterVertex;

typedef struct ScreenVertex {
    float x;
    float y;
    float inverse_z;
    /* u/z and v/z, which is what interpolates linearly in screen space. */
    float u_over_z;
    float v_over_z;
} ScreenVertex;

static int clip_raster_near(
    const RasterVertex *input,
    int count,
    float near_depth,
    RasterVertex *output)
{
    int out_count = 0;
    int i;
    for (i = 0; i < count; ++i) {
        const RasterVertex current = input[i];
        const RasterVertex next = input[(i + 1) % count];
        const float depth_current = current.z - near_depth;
        const float depth_next = next.z - near_depth;
        if (depth_current >= 0.0f) {
            output[out_count++] = current;
        }
        if ((depth_current >= 0.0f) != (depth_next >= 0.0f)) {
            const float t = depth_current / (depth_current - depth_next);
            RasterVertex split;
            split.x = current.x + (next.x - current.x) * t;
            split.y = current.y + (next.y - current.y) * t;
            split.z = current.z + (next.z - current.z) * t;
            split.u = current.u + (next.u - current.u) * t;
            split.v = current.v + (next.v - current.v) * t;
            output[out_count++] = split;
        }
    }
    return out_count;
}

static ScreenVertex project_raster_vertex(
    RECT client,
    Camera3D camera,
    RasterVertex vertex)
{
    /* The same projection project_point applies, kept here in camera space so
       the reciprocal is computed once per vertex. */
    const float inverse_z = 1.0f / vertex.z;
    ScreenVertex out;
    out.x = (float)(client.right + client.left) * 0.5f +
            camera.focal_length * vertex.x * inverse_z;
    out.y = (float)(client.bottom + client.top) * 0.52f -
            camera.focal_length * vertex.y * inverse_z;
    out.inverse_z = inverse_z;
    out.u_over_z = vertex.u * inverse_z;
    out.v_over_z = vertex.v * inverse_z;
    return out;
}

static uint32_t rgb565_to_bgrx(uint16_t texel)
{
    const uint32_t r = (uint32_t)((texel >> 11) & 0x1Fu);
    const uint32_t g = (uint32_t)((texel >> 5) & 0x3Fu);
    const uint32_t b = (uint32_t)(texel & 0x1Fu);
    const uint32_t red = (r << 3) | (r >> 2);
    const uint32_t green = (g << 2) | (g >> 4);
    const uint32_t blue = (b << 3) | (b >> 2);
    return (red << 16) | (green << 8) | blue;
}

static void raster_triangle(
    SoftwareTarget *target,
    ScreenVertex a,
    ScreenVertex b,
    ScreenVertex c,
    const KartTrackTextureImage *image,
    uint32_t flat_colour,
    unsigned int flat_alpha)
{
    const float area =
        (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    float inverse_area;
    int minimum_x;
    int maximum_x;
    int minimum_y;
    int maximum_y;
    int x;
    int y;
    /* The exports do not agree on winding between submeshes, so both are drawn
       and the sign is folded into the barycentric weights instead. */
    if (area > -0.0001f && area < 0.0001f) {
        return;
    }
    inverse_area = 1.0f / area;

    minimum_x = (int)floorf(a.x < b.x ? (a.x < c.x ? a.x : c.x) : (b.x < c.x ? b.x : c.x));
    maximum_x = (int)ceilf(a.x > b.x ? (a.x > c.x ? a.x : c.x) : (b.x > c.x ? b.x : c.x));
    minimum_y = (int)floorf(a.y < b.y ? (a.y < c.y ? a.y : c.y) : (b.y < c.y ? b.y : c.y));
    maximum_y = (int)ceilf(a.y > b.y ? (a.y > c.y ? a.y : c.y) : (b.y > c.y ? b.y : c.y));
    if (minimum_x < 0) minimum_x = 0;
    if (minimum_y < 0) minimum_y = 0;
    if (maximum_x > target->width - 1) maximum_x = target->width - 1;
    if (maximum_y > target->height - 1) maximum_y = target->height - 1;

    for (y = minimum_y; y <= maximum_y; ++y) {
        const float py = (float)y + 0.5f;
        uint32_t *row = target->pixels + (size_t)y * (size_t)target->width;
        float *depth_row = target->depth != NULL
            ? target->depth + (size_t)y * (size_t)target->width
            : NULL;
        for (x = minimum_x; x <= maximum_x; ++x) {
            const float px = (float)x + 0.5f;
            const float w0 =
                ((b.x - px) * (c.y - py) - (b.y - py) * (c.x - px)) * inverse_area;
            const float w1 =
                ((c.x - px) * (a.y - py) - (c.y - py) * (a.x - px)) * inverse_area;
            const float w2 = 1.0f - w0 - w1;
            float inverse_z;
            uint32_t colour;
            if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) {
                continue;
            }
            inverse_z =
                w0 * a.inverse_z + w1 * b.inverse_z + w2 * c.inverse_z;
            if (inverse_z <= 0.0f) {
                continue;
            }
            if (depth_row != NULL) {
                if (target->shade_depth_equal) {
                    if (fabsf(inverse_z - depth_row[x]) > 0.000001f) continue;
                } else if (inverse_z <= depth_row[x]) {
                    continue;
                }
            }
            if (target->depth_only) {
                if (depth_row != NULL) depth_row[x] = inverse_z;
                continue;
            }
            colour = flat_colour;
            if (image != NULL) {
                const float u = (w0 * a.u_over_z + w1 * b.u_over_z +
                                 w2 * c.u_over_z) / inverse_z;
                const float v = (w0 * a.v_over_z + w1 * b.v_over_z +
                                 w2 * c.v_over_z) / inverse_z;
                /* The UVs tile well past 0..1, and the packer only ever emits
                   power-of-two sizes, so the wrap is a mask. */
                const int width_mask = (int)image->width - 1;
                const int height_mask = (int)image->height - 1;
                const int tx = (int)floorf(u * (float)image->width) & width_mask;
                const int ty = (int)floorf(v * (float)image->height) & height_mask;
                const size_t texel = (size_t)ty * (size_t)image->width + (size_t)tx;
                if ((image->flags & KART_TRACK_TEXTURE_MASKED) != 0u &&
                    (image->mask[texel >> 3] & (1u << (texel & 7u))) == 0u) {
                    continue;
                }
                colour = rgb565_to_bgrx(image->texels[texel]);
                if ((image->flags & KART_TRACK_TEXTURE_BLEND_ALPHA8) != 0u &&
                    image->alpha != NULL && image->alpha[texel] < 255u) {
                    const uint32_t alpha = image->alpha[texel];
                    const uint32_t inverse_alpha = 255u - alpha;
                    const uint32_t destination = row[x];
                    const uint32_t red =
                        (((colour >> 16) & 0xffu) * alpha +
                         ((destination >> 16) & 0xffu) * inverse_alpha) / 255u;
                    const uint32_t green =
                        (((colour >> 8) & 0xffu) * alpha +
                         ((destination >> 8) & 0xffu) * inverse_alpha) / 255u;
                    const uint32_t blue =
                        ((colour & 0xffu) * alpha +
                         (destination & 0xffu) * inverse_alpha) / 255u;
                    colour = (red << 16) | (green << 8) | blue;
                }
            } else if (flat_alpha < 255u) {
                const uint32_t inverse_alpha = 255u - flat_alpha;
                const uint32_t destination = row[x];
                const uint32_t red =
                    (((colour >> 16) & 0xffu) * flat_alpha +
                     ((destination >> 16) & 0xffu) * inverse_alpha) / 255u;
                const uint32_t green =
                    (((colour >> 8) & 0xffu) * flat_alpha +
                     ((destination >> 8) & 0xffu) * inverse_alpha) / 255u;
                const uint32_t blue =
                    ((colour & 0xffu) * flat_alpha +
                     (destination & 0xffu) * inverse_alpha) / 255u;
                colour = (red << 16) | (green << 8) | blue;
            }
            if (depth_row != NULL && !target->shade_depth_equal) {
                depth_row[x] = inverse_z;
            }
            row[x] = colour;
        }
    }
}

/* The depth buffer for one frame, cleared and handed back. It is kept across
   frames because it is the one big allocation the renderer needs and the window
   only rarely changes size. Returns NULL if it cannot be sized. */
static float *frame_depth_buffer(int width, int height)
{
    static float *buffer = NULL;
    static size_t capacity = 0;
    const size_t needed = (size_t)width * (size_t)height;
    if (width <= 0 || height <= 0) {
        return NULL;
    }
    if (needed > capacity) {
        float *grown = (float *)realloc(buffer, needed * sizeof(*buffer));
        if (grown == NULL) {
            return NULL;
        }
        buffer = grown;
        capacity = needed;
    }
    memset(buffer, 0, needed * sizeof(*buffer));
    return buffer;
}

/* One world-space triangle through the same pipeline, which is what everything
   that is not track geometry needs: the kart's faces, the boost flame, a skid
   mark's quad. `image` NULL fills with `colour` (0x00RRGGBB); otherwise `uv`
   supplies the three texture coordinates and `colour` is ignored. */
static void raster_world_triangle(
    SoftwareTarget *target,
    RECT client,
    Camera3D camera,
    KartVec3 a,
    KartVec3 b,
    KartVec3 c,
    const float *uv,
    const KartTrackTextureImage *image,
    uint32_t colour)
{
    const float near_depth = 0.2f;
    const KartVec3 world[3] = {a, b, c};
    RasterVertex source[3];
    RasterVertex clipped[8];
    ScreenVertex screen[8];
    int clipped_count;
    int corner;
    for (corner = 0; corner < 3; ++corner) {
        const KartVec3 relative = vec_sub(world[corner], camera.position);
        source[corner].x = vec_dot(relative, camera.right);
        source[corner].y = vec_dot(relative, camera.up);
        source[corner].z = vec_dot(relative, camera.forward);
        source[corner].u = uv != NULL ? uv[corner * 2] : 0.0f;
        source[corner].v = uv != NULL ? uv[corner * 2 + 1] : 0.0f;
    }
    clipped_count = clip_raster_near(source, 3, near_depth, clipped);
    if (clipped_count < 3) {
        return;
    }
    for (corner = 0; corner < clipped_count; ++corner) {
        screen[corner] = project_raster_vertex(client, camera, clipped[corner]);
    }
    for (corner = 1; corner < clipped_count - 1; ++corner) {
        raster_triangle(
            target, screen[0], screen[corner], screen[corner + 1], image, colour,
            255u);
    }
}

static void raster_flat_triangle(
    SoftwareTarget *target,
    RECT client,
    Camera3D camera,
    KartVec3 a,
    KartVec3 b,
    KartVec3 c,
    uint32_t colour)
{
    raster_world_triangle(target, client, camera, a, b, c, NULL, NULL, colour);
}

static uint32_t colorref_to_bgrx(COLORREF colour)
{
    return ((uint32_t)GetRValue(colour) << 16) |
           ((uint32_t)GetGValue(colour) << 8) |
           (uint32_t)GetBValue(colour);
}

/* Rasterizes the whole scene. `theme` keys the texture lookup; passing an empty
   table simply draws every mesh with its floor/wall tint. */
static void draw_track_scene_textured(
    SoftwareTarget *target,
    RECT client,
    Camera3D camera,
    const KartDemoTrackSpec *track,
    const KartTrackScene *scene,
    const KartTrackTextureTable *textures,
    const char *theme,
    float draw_radius,
    unsigned int flat_alpha)
{
    /* The dome is thousands of units across and has to be drawn whole, so a
       radius of zero means no distance cull at all. */
    const float draw_radius_squared =
        draw_radius > 0.0f ? draw_radius * draw_radius : 0.0f;
    const float near_depth = 0.2f;
    uint32_t mesh_index;

    for (mesh_index = 0; mesh_index < scene->mesh_count; ++mesh_index) {
        const KartTrackSceneMesh *mesh = &scene->meshes[mesh_index];
        const KartTrackTextureImage *image =
            kart_track_texture_find(textures, theme, mesh->texture);
        const uint32_t triangle_count = mesh->index_count / 3u;
        uint32_t triangle;
        for (triangle = 0; triangle < triangle_count; ++triangle) {
            const uint32_t base = triangle * 3u;
            RasterVertex source[3];
            RasterVertex clipped[8];
            ScreenVertex screen[8];
            float center_x = 0.0f;
            float center_y = 0.0f;
            KartVec3 world[3];
            uint32_t flat_colour;
            int clipped_count;
            int corner;
            for (corner = 0; corner < 3; ++corner) {
                const KartTrackSceneVertex *vertex =
                    &mesh->vertices[mesh->indices[base + (uint32_t)corner]];
                const KartVec3 point =
                    kart_track_scene_world_vertex(vertex, track);
                const KartVec3 relative = vec_sub(point, camera.position);
                world[corner] = point;
                center_x += point.x;
                center_y += point.y;
                source[corner].x = vec_dot(relative, camera.right);
                source[corner].y = vec_dot(relative, camera.up);
                source[corner].z = vec_dot(relative, camera.forward);
                source[corner].u = vertex->u;
                source[corner].v = vertex->v;
            }
            center_x = center_x / 3.0f - camera.position.x;
            center_y = center_y / 3.0f - camera.position.y;
            if (draw_radius_squared > 0.0f &&
                center_x * center_x + center_y * center_y >
                    draw_radius_squared) {
                continue;
            }
            clipped_count =
                clip_raster_near(source, 3, near_depth, clipped);
            if (clipped_count < 3) {
                continue;
            }
            if (image == NULL) {
                /* Same split as fill_track_scene_faces, and the same
                   thresholds kart_track_collision.c uses. */
                const KartVec3 normal = vec_normalize(vec_cross(
                    vec_sub(world[1], world[0]),
                    vec_sub(world[2], world[0])));
                const float normal_z = normal.z < 0.0f ? -normal.z : normal.z;
                flat_colour = normal_z >= 0.20f ? 0x00C4B078u : 0x006096D2u;
            } else {
                flat_colour = 0;
            }
            for (corner = 0; corner < clipped_count; ++corner) {
                screen[corner] =
                    project_raster_vertex(client, camera, clipped[corner]);
            }
            for (corner = 1; corner < clipped_count - 1; ++corner) {
                raster_triangle(
                    target, screen[0], screen[corner], screen[corner + 1],
                    image, flat_colour, flat_alpha);
            }
        }
    }
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
    const KartTrackScene *scene,
    BYTE alpha)
{
    const int width = client.right - client.left;
    const int height = client.bottom - client.top;
    BLENDFUNCTION blend = {AC_SRC_OVER, 0, alpha, 0};
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
    draw_track_scene_faces(dc, client, camera, track, scene, 85u);
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

static COLORREF kart_model_base_colour(unsigned int colour_index);

static void draw_track_minimap(
    HDC dc,
    RECT client,
    Demo3DState *demo)
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
    KartDemoMinimap *minimap = kart_demo_minimap_for_track(
        &demo->minimaps, demo->track_spec);
    KartVec3 right;
    KartVec3 forward;
    KartVec3 up;
    POINT kart_point;
    POINT kart_triangle[3];
    const COLORREF marker_colour = kart_model_base_colour(demo->kart_colour);
    HBRUSH panel_brush = CreateSolidBrush(RGB(15, 19, 26));
    HBRUSH kart_brush = CreateSolidBrush(marker_colour);
    HPEN panel_pen = CreatePen(PS_SOLID, 1, RGB(54, 64, 73));
    HPEN grid_pen = CreatePen(PS_SOLID, 1, RGB(86, 98, 108));
    HPEN wall_pen = CreatePen(PS_SOLID, 3, RGB(75, 220, 255));
    HPEN kart_pen = CreatePen(PS_SOLID, 1, marker_colour);
    HGDIOBJ old_brush = SelectObject(dc, panel_brush);
    HGDIOBJ old_pen = SelectObject(dc, panel_pen);
    int division;
    static const char label[] = "TRACK MAP";
    char kart_size[96];

    if (minimap != NULL) {
        /* The minimap texture itself supplies the verified 0.3 blend.  Do not
           put an opaque simulator
           panel behind it; retain only the header fill and outer outline. */
        RECT header = {panel.left, panel.top, panel.right, image_rect.top};
        FillRect(dc, &header, panel_brush);
        SelectObject(dc, GetStockObject(NULL_BRUSH));
        Rectangle(dc, panel.left, panel.top, panel.right, panel.bottom);
        boundary = image_rect;
        kart_demo_draw_original_minimap_camera(
            dc, image_rect, minimap, demo->track_spec, demo->kart.position,
            demo->kart.orientation, demo->simulation_time_ms, marker_colour);
    } else {
        SelectObject(dc, panel_brush);
        Rectangle(dc, panel.left, panel.top, panel.right, panel.bottom);
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
    (void)up;
    if (minimap == NULL) {
        static const float marker_vertices[3][2] = {
            {16.587f, -16.396f}, {0.274f, 21.517f}, {-16.587f, -16.396f},
        };
        float heading_length;
        int marker_index;
        kart_point = kart_demo_minimap_bounds_point(
            boundary, demo->track_spec, demo->kart.position);
        forward = kart_demo_minimap_direction(demo->track_spec, forward);
        forward.x = -forward.x;
        forward.z = 0.0f;
        heading_length = sqrtf(forward.x * forward.x + forward.y * forward.y);
        if (heading_length > 0.0f) {
            forward.x /= heading_length;
            forward.y /= heading_length;
        }
        right = (KartVec3){forward.y, -forward.x, 0.0f};
        for (marker_index = 0; marker_index < 3; ++marker_index) {
            kart_triangle[marker_index] = (POINT){
                kart_point.x + (LONG)(KART_DEMO_MINIMAP_MARKER_SCALE *
                    (right.x * marker_vertices[marker_index][0] +
                     forward.x * marker_vertices[marker_index][1])),
                kart_point.y + (LONG)(KART_DEMO_MINIMAP_MARKER_SCALE *
                    (right.y * marker_vertices[marker_index][0] +
                     forward.y * marker_vertices[marker_index][1])),
            };
        }
        SelectObject(dc, kart_brush);
        SelectObject(dc, kart_pen);
        Polygon(dc, kart_triangle, 3);
    }

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
    HPEN pen = CreatePen(PS_SOLID, 4, RGB(8, 10, 12));
    HGDIOBJ old_pen = SelectObject(dc, pen);
    unsigned int side;
    for (side = 0; side < SKID_MARK_SIDE_COUNT; ++side) {
        unsigned int mark;
        for (mark = 0; mark < SKID_MARK_POOL_SIZE; ++mark) {
            const SkidMarkStrip *strip = &demo->skid_marks[side][mark];
            unsigned int section;
            for (section = 1; section < strip->section_count; ++section) {
                const KartVec3 a = vec_scale(
                    vec_add(strip->sections[section - 1u].edge[0],
                            strip->sections[section - 1u].edge[1]), 0.5f);
                const KartVec3 b = vec_scale(
                    vec_add(strip->sections[section].edge[0],
                            strip->sections[section].edge[1]), 0.5f);
                draw_line_3d(dc, client, camera, a, b);
            }
        }
    }
    SelectObject(dc, old_pen);
    DeleteObject(pen);
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

/* The flame triangle the GDI pass builds, through the rasterizer. Kept beside
   it so the two stay the same shape. */
static void raster_boost_effect(
    SoftwareTarget *target,
    RECT client,
    Camera3D camera,
    const Demo3DState *demo)
{
    KartVec3 right;
    KartVec3 forward;
    KartVec3 up;
    KartVec3 rear;
    KartVec3 flame[3];

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
    raster_flat_triangle(
        target, client, camera, flame[0], flame[1], flame[2], 0x00FFC623u);
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

/* One triangle of the model, ready to sort. `uv` is the asset's own texture
   coordinate per corner, which only the textured path reads. */
typedef struct KartModelFace {
    float depth;
    float shade;
    bool wheel;
    KartVec3 vertices[3];
    float uv[6];
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

/* Fixed world-space key light, so the shading reads as the kart turning under a
   light rather than the light turning with the kart. */
static const KartVec3 KART_MODEL_LIGHT = {0.38f, -0.52f, 0.76f};

/* Dark enough to read as tyres, light enough that the shading on them is still
   visible against the track fill. */
#define KART_MODEL_WHEEL_COLOR RGB(96, 100, 112)

/* The kart's paint.

   0x00417160 repaints the skin when the kart is built: it walks the atlas texel
   by texel looking for two key colours and fills a rectangle at each hit.

     key `cyan`  (DAT_005b1930) -> rect (x-5, y-8)..(x+5, y+9), filled with the
                                   colortable entry's `base`
     key `blue`  (DAT_005b1924) -> rect (x, y)..(x+0x2d, y+0x14), filled from a
                                   different slot - that one is the number plate

   So the colour the driver picks lands on the cyan-anchored patch, not on the
   900-texel blue block at the back. `kartColor` in riderData.1s is the index.

   The values are the demo's own, read from Data/etc.rho's colortable.xml; a
   copy of that file is at Assets/Models/Karts/demo_etc/colortable.xml. The
   names come from riderData.1s's editor enum. */
#define KART_SKIN_KEY_CYAN 0x07FFu    /* (0, 255, 255) in RGB565 */
#define KART_SKIN_KEY_BLUE 0x001Fu    /* (0, 0, 255) */
#define KART_SKIN_KEY_MAGENTA 0xF81Fu /* (255, 0, 255), the atlas filler */

static const struct {
    const char *name;
    unsigned char base[3];
    unsigned char high[3];
} KART_COLOURSETS[] = {
    {"red",    {232,  39,   6}, {255, 186,   0}},
    {"yellow", {255, 186,   0}, {255, 255,   0}},
    {"orange", {255, 130,   0}, {255, 186,  82}},
    {"green",  { 58, 174,  25}, {210, 255,   0}},
    {"teal",   {  0, 199, 206}, {255, 255, 255}},
    {"blue",   { 19, 121, 219}, {  0, 252, 255}},
    {"purple", {140,  56, 239}, {255, 255, 255}},
    {"black",  { 40,  40,  40}, {150, 150, 150}},
    {"pink",   {248,   1, 122}, {255, 190, 190}},
    {"white",  {243, 243, 243}, {255, 255, 255}},
};
#define KART_COLOURSET_COUNT \
    (unsigned int)(sizeof(KART_COLOURSETS) / sizeof(KART_COLOURSETS[0]))
/* riderData.1s ships 8 = pink on this profile. */
#define KART_COLOURSET_DEFAULT 8u
#define KART_COLOUR_MENU_BASE 3000u

static COLORREF kart_model_base_colour(unsigned int colour_index)
{
    if (colour_index >= KART_COLOURSET_COUNT) colour_index = 0u;
    return RGB(
        KART_COLOURSETS[colour_index].base[0],
        KART_COLOURSETS[colour_index].base[1],
        KART_COLOURSETS[colour_index].base[2]);
}

static unsigned int kart_demo_popup_select_colour(
    HWND window,
    unsigned int current)
{
    HMENU menu = CreatePopupMenu();
    RECT window_rect;
    unsigned int i;
    UINT command;
    if (menu == NULL) return current;
    AppendMenuA(menu, MF_STRING | MF_DISABLED, 0, "SELECT KART COLOUR");
    AppendMenuA(menu, MF_SEPARATOR, 0, NULL);
    for (i = 0; i < KART_COLOURSET_COUNT; ++i) {
        UINT flags = MF_STRING;
        char label[96];
        if (i == current) flags |= MF_CHECKED;
        snprintf(
            label, sizeof(label), "%s  Base #%02X%02X%02X",
            KART_COLOURSETS[i].name,
            KART_COLOURSETS[i].base[0],
            KART_COLOURSETS[i].base[1],
            KART_COLOURSETS[i].base[2]);
        AppendMenuA(menu, flags, KART_COLOUR_MENU_BASE + i, label);
    }
    GetWindowRect(window, &window_rect);
    SetForegroundWindow(window);
    command = TrackPopupMenu(
        menu,
        TPM_RETURNCMD | TPM_LEFTALIGN | TPM_TOPALIGN | TPM_RIGHTBUTTON,
        window_rect.left + 520,
        window_rect.top + 72,
        0,
        window,
        NULL);
    DestroyMenu(menu);
    PostMessage(window, WM_NULL, 0, 0);
    if (command >= KART_COLOUR_MENU_BASE &&
        command < KART_COLOUR_MENU_BASE + KART_COLOURSET_COUNT) {
        return command - KART_COLOUR_MENU_BASE;
    }
    return current;
}

static COLORREF kart_model_body_color(
    const KartSimulationState *kart,
    bool boost_active,
    unsigned int colour_index)
{
    return boost_active
        ? RGB(255, 165, 35)
        : (drift_visual_active(kart)
            ? RGB(65, 205, 255)
            : kart_model_base_colour(colour_index));
}

/* Builds the model's world-space faces with their shade. Shared by the GDI
   painter's-order path and the rasterized one, which only differ in how they
   resolve occlusion. Returns how many faces were written. */
static size_t collect_kart_model_faces(
    Camera3D camera,
    const KartSimulationState *kart,
    const KartTrackScene *model,
    const KartModelParts *parts,
    KartModelFace *faces,
    size_t capacity)
{
    KartVec3 body_right;
    KartVec3 body_forward;
    KartVec3 body_up;
    size_t face_count = 0;
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
            if (face_count == capacity) break;
            face = &faces[face_count];
            face->wheel = wheel;
            {
                unsigned int corner;
                for (corner = 0; corner < 3u; ++corner) {
                    const KartTrackSceneVertex *vertex =
                        &mesh->vertices[mesh->indices[base + corner]];
                    face->vertices[corner] = kart_model_vertex(
                        kart, body_right, body_forward, body_up, vertex);
                    face->uv[corner * 2u] = vertex->u;
                    face->uv[corner * 2u + 1u] = vertex->v;
                }
            }
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
            facing = vec_dot(normal, KART_MODEL_LIGHT);
            if (facing < 0.0f) facing = -facing;
            face->shade = 0.42f + 0.58f * facing;
            ++face_count;
        }
    }
    return face_count;
}

static void draw_kart_model(
    HDC dc,
    RECT client,
    Camera3D camera,
    const KartSimulationState *kart,
    const KartTrackScene *model,
    const KartModelParts *parts,
    bool boost_active,
    unsigned int colour_index)
{
    const COLORREF body_color =
        kart_model_body_color(kart, boost_active, colour_index);
    const COLORREF wheel_color = KART_MODEL_WHEEL_COLOR;
    static KartModelFace faces[KART_MODEL_MAX_FACES];
    HGDIOBJ old_pen;
    HGDIOBJ old_brush;
    size_t i;
    const size_t face_count = collect_kart_model_faces(
        camera, kart, model, parts, faces, KART_MODEL_MAX_FACES);
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
    bool boost_active,
    unsigned int colour_index)
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
                : kart_model_base_colour(colour_index)));
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

/* Draws the recovered mesh, then the boxes over it.

   The box is only the fallback for a build without embedded models: it is what
   the demo drew before the models were recovered, and it is the shape the
   physics actually reasons about. */
static void draw_kart(
    HDC dc,
    RECT client,
    Camera3D camera,
    const Demo3DState *demo)
{
    const KartModelParts *parts = NULL;
    const KartTrackScene *model = active_kart_model(demo, &parts);
    if (model != NULL && parts != NULL && parts->valid) {
        draw_kart_model(
            dc, client, camera, &demo->kart, model, parts, demo->boost_active,
            demo->kart_colour);
    } else {
        draw_kart_box(
            dc, client, camera, &demo->kart, demo->kart_spec,
            demo->boost_active, demo->kart_colour);
    }
    if (demo->show_model_bounds) {
        draw_kart_model_bounds(dc, client, camera, &demo->kart, parts);
    }
}

/* Rebuilds the active kart's skin for the chosen colour, the way 0x00417160
   does. Three things happen to the atlas:

   1. The body. 0x004a6eb0 composites "0" and "1" (the two PNGs beside model.1s)
      and tints by luminance: a texel of "0" darker than 0x80 in every channel
      takes `base`, otherwise `high`. Every kart's 0.png is 256x128 of
      (255, 255, 255, alpha 0) with two opaque white texels and no dark texel at
      all, so the tint is uniformly `high` and "1" is alpha-composited over it.
      Pure magenta in "1" is the colour key and comes out transparent.
   2. The number. Each `cyan` texel anchors a 10x17 rectangle - one digit of the
      100x17 number.png - blended in with `base` through the digit's alpha
      (0x00416ff0). The demo builds every kart with digit 0.
   3. The plate. Each `blue` texel is the top-left of a 45x20 rectangle that
      plate.png is copied straight into.

   This simulator-side display override uses `base` over the whole body as
   requested. The racing number already uses `base`; the 900-texel blue block
   at the back is the plate and takes no colour at all.

   The table's images are shared, so this starts from a copy of the asset's own
   texels rather than from whatever the last colour left behind. */
static void paint_kart_skin(Demo3DState *demo)
{
    /* The table is the demo's own; find() is const only because callers that
       sample it have no business writing to it. */
    KartTrackTextureImage *image = (KartTrackTextureImage *)
        kart_track_texture_find(
            &demo->textures, "kart", demo->kart_spec->asset_name);
    const KartTrackTextureImage *plate =
        kart_track_texture_find(&demo->textures, "kart", "@plate");
    const KartTrackTextureImage *number =
        kart_track_texture_find(&demo->textures, "kart", "@number");
    const unsigned char *base_rgb;
    const int width = image != NULL ? (int)image->width : 0;
    const int height = image != NULL ? (int)image->height : 0;
    size_t texel;
    int x;
    int y;

    if (image == NULL || image->alpha == NULL) {
        demo->painted_skin = NULL;
        return;
    }
    if (demo->painted_skin == image &&
        demo->painted_colour == demo->kart_colour) {
        return;
    }
    if (demo->painted_skin != image) {
        const size_t count = (size_t)width * (size_t)height;
        uint16_t *copy = (uint16_t *)realloc(
            demo->skin_texels, count * sizeof(*copy));
        if (copy == NULL) {
            return;
        }
        memcpy(copy, image->texels, count * sizeof(*copy));
        demo->skin_texels = copy;
        demo->skin_texel_count = count;
    }
    base_rgb = KART_COLOURSETS[demo->kart_colour].base;

    /* 1. body: "1" over a solid `base`, magenta keyed out. */
    for (texel = 0; texel < demo->skin_texel_count; ++texel) {
        const uint16_t source = demo->skin_texels[texel];
        const unsigned int alpha = image->alpha[texel];
        unsigned int channel[3];
        unsigned int i;
        if (source == KART_SKIN_KEY_MAGENTA) {
            image->mask[texel >> 3] &= (uint8_t)~(1u << (texel & 7u));
            continue;
        }
        channel[0] = (unsigned int)((source >> 11) & 0x1Fu) << 3;
        channel[1] = (unsigned int)((source >> 5) & 0x3Fu) << 2;
        channel[2] = (unsigned int)(source & 0x1Fu) << 3;
        for (i = 0; i < 3u; ++i) {
            const unsigned int blended =
                ((255u - alpha) * base_rgb[i] + channel[i] * alpha) / 255u;
            channel[i] = blended > 255u ? 255u : blended;
        }
        image->texels[texel] = (uint16_t)(
            ((channel[0] >> 3) << 11) | ((channel[1] >> 2) << 5) |
            (channel[2] >> 3));
        image->mask[texel >> 3] |= (uint8_t)(1u << (texel & 7u));
    }

    /* 2 and 3: the two stamps, placed off the key texels.

       The scan reads the image it is writing into, which is what 0x00417160
       does - it walks the composited image in place. That is not incidental:
       the plate's key is a solid 45x20 block of blue, so the first hit at its
       top-left corner overwrites all 900 of them and the scan finds no more.
       Reading a frozen copy instead stamps the plate 900 times. */
    for (y = 0; y < height; ++y) {
        for (x = 0; x < width; ++x) {
            const uint16_t key =
                image->texels[(size_t)y * (size_t)width + (size_t)x];
            int stamp_x;
            int stamp_y;
            if (key == KART_SKIN_KEY_CYAN && number != NULL &&
                number->alpha != NULL) {
                /* Digit 0 of the strip, blended in with `base`. */
                for (stamp_y = 0; stamp_y < 17; ++stamp_y) {
                    const int target_y = y - 8 + stamp_y;
                    if (target_y < 0 || target_y >= height) continue;
                    for (stamp_x = 0; stamp_x < 10; ++stamp_x) {
                        const int target_x = x - 5 + stamp_x;
                        const size_t from = (size_t)stamp_y *
                            (size_t)number->width + (size_t)stamp_x;
                        const size_t to =
                            (size_t)target_y * (size_t)width + (size_t)target_x;
                        const unsigned int alpha = number->alpha[from];
                        unsigned int i;
                        unsigned int channel[3];
                        if (target_x < 0 || target_x >= width) continue;
                        if ((int)number->width <= stamp_x ||
                            (int)number->height <= stamp_y) continue;
                        channel[0] = (unsigned int)((image->texels[to] >> 11) & 0x1Fu) << 3;
                        channel[1] = (unsigned int)((image->texels[to] >> 5) & 0x3Fu) << 2;
                        channel[2] = (unsigned int)(image->texels[to] & 0x1Fu) << 3;
                        for (i = 0; i < 3u; ++i) {
                            channel[i] = (channel[i] * (255u - alpha) +
                                          (unsigned int)base_rgb[i] * alpha) / 255u;
                        }
                        image->texels[to] = (uint16_t)(
                            ((channel[0] >> 3) << 11) |
                            ((channel[1] >> 2) << 5) | (channel[2] >> 3));
                        image->mask[to >> 3] |= (uint8_t)(1u << (to & 7u));
                    }
                }
            } else if (key == KART_SKIN_KEY_BLUE && plate != NULL) {
                for (stamp_y = 0; stamp_y < (int)plate->height; ++stamp_y) {
                    const int target_y = y + stamp_y;
                    if (target_y < 0 || target_y >= height) continue;
                    for (stamp_x = 0; stamp_x < (int)plate->width; ++stamp_x) {
                        const int target_x = x + stamp_x;
                        const size_t to =
                            (size_t)target_y * (size_t)width + (size_t)target_x;
                        if (target_x < 0 || target_x >= width) continue;
                        image->texels[to] = plate->texels[
                            (size_t)stamp_y * (size_t)plate->width +
                            (size_t)stamp_x];
                        image->mask[to >> 3] |= (uint8_t)(1u << (to & 7u));
                    }
                }
            }
        }
    }
    demo->painted_skin = image;
    demo->painted_colour = demo->kart_colour;
}

/* The kart, its skid marks and the boost flame through the rasterizer instead
   of GDI, so the track's depth buffer decides what is in front of what. Without
   this the kart shows through a wall the moment the track rises between it and
   the camera. Drawn after the track, testing and writing the same buffer. */
static void raster_kart(
    SoftwareTarget *target,
    RECT client,
    Camera3D camera,
    const Demo3DState *demo)
{
    static KartModelFace faces[KART_MODEL_MAX_FACES];
    const KartModelParts *parts = NULL;
    const KartTrackScene *model = active_kart_model(demo, &parts);
    const COLORREF body_color =
        kart_model_body_color(
            &demo->kart, demo->boost_active, demo->kart_colour);
    /* The kart's own skin. A kart mesh names no material - the exporter finds
       none on a ReToonRigid node - so the lookup is by the kart's asset name
       under the "kart" theme, which is where the packer put its 1.png. */
    const KartTrackTextureImage *skin =
        demo->kart_textured
            ? kart_track_texture_find(
                  &demo->textures, "kart", demo->kart_spec->asset_name)
            : NULL;
    size_t face_count;
    size_t i;

    if (model == NULL || parts == NULL || !parts->valid) {
        return;
    }
    face_count = collect_kart_model_faces(
        camera, &demo->kart, model, parts, faces, KART_MODEL_MAX_FACES);
    /* No sort: the depth buffer resolves the order the qsort stood in for. */
    for (i = 0; i < face_count; ++i) {
        const KartModelFace *face = &faces[i];
        raster_world_triangle(
            target, client, camera,
            face->vertices[0], face->vertices[1], face->vertices[2],
            skin != NULL ? face->uv : NULL, skin,
            colorref_to_bgrx(scale_color(
                face->wheel ? KART_MODEL_WHEEL_COLOR : body_color,
                face->shade)));
    }
}

static void raster_skid_marks(
    SoftwareTarget *target,
    RECT client,
    Camera3D camera,
    const Demo3DState *demo)
{
    const KartTrackTextureImage *texture =
        demo->skid_texture.texels != NULL ? &demo->skid_texture : NULL;
    unsigned int side;
    for (side = 0; side < SKID_MARK_SIDE_COUNT; ++side) {
        unsigned int mark;
        for (mark = 0; mark < SKID_MARK_POOL_SIZE; ++mark) {
            const SkidMarkStrip *strip = &demo->skid_marks[side][mark];
            unsigned int section;
            for (section = 1; section < strip->section_count; ++section) {
                const SkidMarkCrossSection *a = &strip->sections[section - 1u];
                const SkidMarkCrossSection *b = &strip->sections[section];
                const float uv0[6] = {
                    0.0f, a->texture_v, 0.0f, b->texture_v,
                    0.499999f, b->texture_v};
                const float uv1[6] = {
                    0.0f, a->texture_v, 0.499999f, b->texture_v,
                    0.499999f, a->texture_v};
                raster_world_triangle(
                    target, client, camera,
                    a->edge[0], b->edge[0], b->edge[1],
                    texture != NULL ? uv0 : NULL, texture, 0x00141414u);
                raster_world_triangle(
                    target, client, camera,
                    a->edge[0], b->edge[1], a->edge[1],
                    texture != NULL ? uv1 : NULL, texture, 0x00141414u);
            }
        }
    }
}

/* Draws every gate of the course graph as the quad it is: the two triangles
   kart_course_gate_crossing tests the trail segment against, outlined edge by
   edge. The final gate, the one the lap counter checks, is drawn apart. */
static void draw_course_gates(
    HDC dc,
    RECT client,
    Camera3D camera,
    const KartCourse *course)
{
    HPEN gate_pen = CreatePen(PS_SOLID, 2, RGB(90, 220, 255));
    HPEN final_pen = CreatePen(PS_SOLID, 2, RGB(255, 205, 80));
    HGDIOBJ old_pen = SelectObject(dc, gate_pen);
    unsigned int gate_index;

    for (gate_index = 0; gate_index < course->gate_count; ++gate_index) {
        const KartCourseGate *gate = &course->gates[gate_index];
        unsigned int triangle;
        SelectObject(dc, gate->is_final ? final_pen : gate_pen);
        for (triangle = 0; triangle < 2u; ++triangle) {
            unsigned int corner;
            for (corner = 0; corner < 3u; ++corner) {
                draw_line_3d(
                    dc, client, camera,
                    gate->face[triangle][corner],
                    gate->face[triangle][(corner + 1u) % 3u]);
            }
        }
    }
    SelectObject(dc, old_pen);
    DeleteObject(gate_pen);
    DeleteObject(final_pen);
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

static void raster_skid_marks_topdown(
    SoftwareTarget *target,
    RECT client,
    float camera_x,
    float camera_y,
    const Demo3DState *demo)
{
    const KartTrackTextureImage *texture =
        demo->skid_texture.texels != NULL ? &demo->skid_texture : NULL;
    unsigned int side;
    for (side = 0; side < SKID_MARK_SIDE_COUNT; ++side) {
        unsigned int mark;
        for (mark = 0; mark < SKID_MARK_POOL_SIZE; ++mark) {
            const SkidMarkStrip *strip = &demo->skid_marks[side][mark];
            unsigned int section;
            for (section = 1; section < strip->section_count; ++section) {
                const SkidMarkCrossSection *a = &strip->sections[section - 1u];
                const SkidMarkCrossSection *b = &strip->sections[section];
                const POINT p00 = world_to_screen(
                    client, camera_x, camera_y, a->edge[0].x, a->edge[0].y);
                const POINT p01 = world_to_screen(
                    client, camera_x, camera_y, a->edge[1].x, a->edge[1].y);
                const POINT p10 = world_to_screen(
                    client, camera_x, camera_y, b->edge[0].x, b->edge[0].y);
                const POINT p11 = world_to_screen(
                    client, camera_x, camera_y, b->edge[1].x, b->edge[1].y);
                const ScreenVertex v00 = {
                    (float)p00.x, (float)p00.y, 1.0f, 0.0f, a->texture_v};
                const ScreenVertex v01 = {
                    (float)p01.x, (float)p01.y, 1.0f, 0.499999f, a->texture_v};
                const ScreenVertex v10 = {
                    (float)p10.x, (float)p10.y, 1.0f, 0.0f, b->texture_v};
                const ScreenVertex v11 = {
                    (float)p11.x, (float)p11.y, 1.0f, 0.499999f, b->texture_v};
                raster_triangle(
                    target, v00, v10, v11, texture, 0x00141414u, 255u);
                raster_triangle(
                    target, v00, v11, v01, texture, 0x00141414u, 255u);
            }
        }
    }
}

static void draw_scene_topdown(
    HDC buffer,
    RECT client,
    const Demo3DState *demo,
    void *frame_pixels)
{
    HBRUSH track_brush;
    HPEN wall_pen;
    HPEN grid_pen;
    HPEN heading_pen;
    HPEN velocity_pen;
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

    if (frame_pixels != NULL && demo->skid_texture.texels != NULL) {
        SoftwareTarget frame;
        memset(&frame, 0, sizeof(frame));
        frame.pixels = (uint32_t *)frame_pixels;
        frame.width = client.right;
        frame.height = client.bottom;
        GdiFlush();
        raster_skid_marks_topdown(
            &frame, client, camera_x, camera_y, demo);
    }

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

static const char *jump_phase_name(KartJumpPhase phase)
{
    switch (phase) {
    case KART_JUMP_CROUCH: return "CROUCH";
    case KART_JUMP_PUSH: return "PUSH";
    case KART_JUMP_AIRBORNE: return "AIR";
    case KART_JUMP_LANDING: return "LAND";
    default: return "READY";
    }
}

/* The values the HUD lines have no room for: the whole rigid body state, the
   drift and boost timers, the suspension contacts and what the last step
   resolved. Fixed pitch so the columns line up as the numbers move. */
static void draw_telemetry(HDC dc, RECT client, const Demo3DState *demo)
{
    const KartSimulationState *kart = &demo->kart;
    const int line_height = 15;
    const int rows = 16;
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
        RGB(190, 165, 240),
        "INST    model %-6s stored %u  (Q: model, Up: use)",
        kart->instant_boost.stored_model ? "stored" : "window",
        kart->instant_boost.stored_count);
    TELEMETRY_LINE(
        kart->jump.phase == KART_JUMP_READY ? RGB(200, 212, 220)
                                            : RGB(255, 180, 235),
        "JUMP    %-6s gauge %.2f power %3.0f%% E %6.0fJ F %6.0fN h %.2fm",
        jump_phase_name(kart->jump.phase), kart->jump.gauge_position,
        kart->jump.jump_strength * 100.0f,
        kart->jump.stored_energy, kart->jump.applied_force,
        kart->jump.apex_height - kart->jump.takeoff_height);
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
        "GAUGE   %-17s %5.1f%%  rate %6.2f  W %4.2f  store %s %u/%u",
        kart_gauge_model_name(demo->gauge.model),
        demo->gauge_config.full_value > 0.0f
            ? demo->gauge.value / demo->gauge_config.full_value * 100.0f : 0.0f,
        demo->gauge.rate, demo->gauge.contact_weight,
        demo->gauge.unlimited_boosters ? "unlimited" : "capped",
        demo->gauge.boosters, demo->kart_spec->max_boosters);
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

static void draw_scene(HWND window, HDC target, Demo3DState *demo)
{
    const bool topdown = demo->view_mode == DEMO_VIEW_TOPDOWN;
    RECT client;
    HDC buffer;
    HBITMAP bitmap;
    HGDIOBJ old_bitmap;
    HBRUSH sky;
    /* The frame is a DIB section rather than a compatible bitmap so the
       textured pass can write pixels straight into it while GDI keeps drawing
       everything else onto the same surface. */
    BITMAPINFO frame_info;
    void *frame_pixels = NULL;
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
    memset(&frame_info, 0, sizeof(frame_info));
    frame_info.bmiHeader.biSize = sizeof(frame_info.bmiHeader);
    frame_info.bmiHeader.biWidth = client.right;
    /* Negative height puts row 0 at the top, which is the order the rasterizer
       indexes with. */
    frame_info.bmiHeader.biHeight = -client.bottom;
    frame_info.bmiHeader.biPlanes = 1;
    frame_info.bmiHeader.biBitCount = 32;
    frame_info.bmiHeader.biCompression = BI_RGB;
    bitmap = CreateDIBSection(
        target, &frame_info, DIB_RGB_COLORS, &frame_pixels, NULL, 0);
    if (bitmap == NULL) {
        bitmap = CreateCompatibleBitmap(target, client.right, client.bottom);
        frame_pixels = NULL;
    }
    old_bitmap = SelectObject(buffer, bitmap);
    sky = CreateSolidBrush(topdown ? RGB(22, 25, 31) : RGB(19, 24, 33));
    FillRect(buffer, &client, sky);
    DeleteObject(sky);

    camera = make_chase_camera(&demo->camera_pose, client);
    if (topdown) {
        draw_scene_topdown(buffer, client, demo, frame_pixels);
    } else {
        const KartTrackScene *scene = active_track_scene(demo);
        const bool depth_fill =
            demo->track_render_mode == DEMO_TRACK_RENDER_DEPTH_ALPHA_190 &&
            scene != NULL && frame_pixels != NULL;
        const bool textured =
            demo->track_render_mode == DEMO_TRACK_RENDER_TEXTURED &&
            scene != NULL && frame_pixels != NULL;
        /* The kart's skin is a separate switch, so Z alone rasterizes just the
           kart over the wireframe track. Its depth buffer is then empty, which
           is right: the wireframe is line art with no depth to test against. */
        const bool kart_only =
            !depth_fill && !textured && demo->kart_textured && frame_pixels != NULL;
        const bool skid_only =
            !textured && !kart_only && frame_pixels != NULL &&
            demo->skid_texture.texels != NULL;
        bool rasterized = false;
        bool skid_rasterized = false;
        bool wireframe_base_drawn = false;
        if (depth_fill || textured || kart_only || skid_only) {
            SoftwareTarget frame;
            char theme[32];
            memset(&frame, 0, sizeof(frame));
            frame.pixels = (uint32_t *)frame_pixels;
            frame.depth = frame_depth_buffer(client.right, client.bottom);
            frame.width = client.right;
            frame.height = client.bottom;
            if (kart_only || skid_only) {
                /* Everything the kart is drawn over has to be on the DIB first,
                   because the rasterizer writes pixels GDI will not touch
                   again. Gates included, so they read as annotation under it. */
                draw_track(buffer, client, camera, demo->track_spec);
                draw_track_scene(
                    buffer, client, camera, demo->track_spec, scene);
                if (demo->show_gates && demo->course_ready) {
                    draw_course_gates(buffer, client, camera, &demo->course);
                }
                wireframe_base_drawn = true;
            }
            /* GDI batches, so everything drawn so far has to have landed in the
               DIB before anything writes to it behind GDI's back. */
            GdiFlush();
            if (frame.depth != NULL && (kart_only || skid_only)) {
                raster_skid_marks(&frame, client, camera, demo);
                skid_rasterized = true;
            }
            if (kart_only || skid_only) {
                draw_boost_effect(buffer, client, camera, demo);
                GdiFlush();
            }
            if (frame.depth != NULL && kart_only) {
                rasterized = true;
                raster_kart(&frame, client, camera, demo);
            } else if (frame.depth != NULL && (depth_fill || textured)) {
                const KartTrackScene *dome = active_skydome(demo);
                rasterized = true;
                track_theme(demo->track_spec, theme, sizeof(theme));
                /* The dome first and with no distance cull: it is thousands of
                   units across and encloses the whole track, so it is drawn
                   where the asset puts it and the depth buffer keeps everything
                   else in front of it. */
                if (textured && dome != NULL) {
                    draw_track_scene_textured(
                        &frame, client, camera, demo->track_spec, dome,
                        &demo->textures, theme, 0.0f, 255u);
                }
                if (depth_fill) {
                    frame.depth_only = true;
                    draw_track_scene_textured(
                        &frame, client, camera, demo->track_spec, scene,
                        NULL, NULL, 210.0f, 255u);
                    frame.depth_only = false;
                    frame.shade_depth_equal = true;
                }
                draw_track_scene_textured(
                    &frame, client, camera, demo->track_spec, scene,
                    textured ? &demo->textures : NULL,
                    textured ? theme : NULL, 210.0f,
                    depth_fill ? 190u : 255u);
                frame.shade_depth_equal = false;
                /* Everything that used to be painted over the track goes
                   through the same depth buffer now. */
                raster_skid_marks(&frame, client, camera, demo);
                raster_boost_effect(&frame, client, camera, demo);
                raster_kart(&frame, client, camera, demo);
            }
        }
        if (!rasterized && !wireframe_base_drawn) {
            draw_track(buffer, client, camera, demo->track_spec);
            draw_track_scene(
                buffer, client, camera, demo->track_spec, scene);
        }
        if (demo->show_gates && demo->course_ready && !wireframe_base_drawn) {
            draw_course_gates(buffer, client, camera, &demo->course);
        }
        if (!rasterized) {
            if (!skid_rasterized) {
                draw_skid_marks(buffer, client, camera, demo);
            }
            draw_boost_effect(buffer, client, camera, demo);
            draw_kart(buffer, client, camera, demo);
        } else if (demo->show_model_bounds) {
            /* The bounds are wireframe annotation rather than geometry, so they
               stay on the GDI overlay in both modes. */
            const KartModelParts *parts = NULL;
            (void)active_kart_model(demo, &parts);
            draw_kart_model_bounds(buffer, client, camera, &demo->kart, parts);
        }
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
        "FPS %5.1f | speed %.2f m/s | slip %.1f deg | vf %.1f vs %.1f | AUTO %s",
        demo->fps,
        speed,
        slip_angle,
        forward_speed,
        lateral_speed,
        demo->kart.drift.slip_detected ? "ON" : "off");
    TextOutA(buffer, 16, 12, status, (int)strlen(status));
    {
        static const char driving[] =
            "Arrows: drive/instant  Space: cat jump  Shift/W: drift  Ctrl/D: boost  C: camera  "
            "P: parameters  K: kart  T: track  U: engine sound  F: drag trigger  "
            "S: screenshot  R: reset";
        TextOutA(buffer, 16, 32, driving, (int)(sizeof(driving) - 1));
    }
    /* The view toggles, on their own line: there are enough of them now that
       keeping them with the driving keys ran the line off the window. */
    snprintf(
        help,
        sizeof(help),
        "X: %s  Z: %s kart skin  L: colour %u [%s]  N: %s checkpoints  B: %s bounds  V: %s vectors",
        demo->track_render_mode == DEMO_TRACK_RENDER_ALPHA_85
            ? "depth fill"
            : (demo->track_render_mode == DEMO_TRACK_RENDER_DEPTH_ALPHA_190
                ? "textured" : "alpha 85"),
        demo->kart_textured ? "hide" : "show",
        demo->kart_colour,
        KART_COLOURSETS[demo->kart_colour].name,
        demo->show_gates ? "hide" : "show",
        demo->show_model_bounds ? "hide" : "show",
        demo->show_vectors ? "hide" : "show");
    TextOutA(buffer, 16, 52, help, (int)strlen(help));
    /* The inferred layers, kept on their own line so they read as what they
       are rather than as part of the recovered demo. */
    snprintf(
        help,
        sizeof(help),
        "(experimental) E: gear [%s] G: gauge [%s] H: storage [%s] Q: instant [%s, %u] M: stop [%s]",
        demo->gearbox.mode == KART_GEAR_MULTI ? "multi" : "single",
        kart_gauge_model_name(demo->gauge.model),
        demo->gauge.unlimited_boosters ? "unlimited" : "capped",
        demo->kart.instant_boost.stored_model ? "stored" : "window",
        demo->kart.instant_boost.stored_count,
        demo->kart.reverse_input_ends_boost ? "reverse" : "release");
    SetTextColor(buffer, RGB(190, 165, 240));
    TextOutA(buffer, 16, 72, help, (int)strlen(help));
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
    kart_demo_text_out_utf8(buffer, 16, 92, status);
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
        skid_mark_segment_count(demo));
    SetTextColor(
        buffer,
        demo->boost_active ? RGB(75, 225, 255) :
        (drift_visual_active(&demo->kart) ? RGB(255, 185, 55) : RGB(180, 195, 205)));
    TextOutA(buffer, 16, 112, status, (int)strlen(status));
    if (demo->course_ready) {
        /* The original's own progress record, field for field: the node the
           kart is in, how far into it, the accumulated start-gate crossings
           that gate the lap counter, and the wrong-way flag. */
        snprintf(
            status,
            sizeof(status),
            "LAP %u | node %u/%u %.0fm | advance %d%s%s",
            demo->progress.lap,
            demo->progress.node_id,
            demo->course.node_count,
            demo->progress.node_distance,
            demo->progress.advance,
            demo->progress.best_lap_ms != 0 ? " | best " : "",
            demo->progress.wrong_way ? " | WRONG WAY" : "");
        SetTextColor(
            buffer,
            demo->progress.wrong_way ? RGB(255, 120, 120) : RGB(180, 195, 205));
        TextOutA(buffer, 16, 132, status, (int)strlen(status));
        if (demo->progress.best_lap_ms != 0) {
            char best[32];
            snprintf(best, sizeof(best), "%.2fs",
                     (float)demo->progress.best_lap_ms * 0.001f);
            TextOutA(buffer, 16 + 7 * (int)strlen(status), 132, best,
                     (int)strlen(best));
        }
    }
    if (demo->respawn_notice_ms != 0) {
        static const char notice[] = "RESPAWNING ONTO THE COURSE";
        SetTextColor(buffer, RGB(255, 120, 120));
        TextOutA(buffer, 16, 152, notice, (int)strlen(notice));
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
        TextOutA(buffer, 16, 172, notice, (int)strlen(notice));
    }
    draw_telemetry(buffer, client, demo);
    kart_demo_draw_gauge(
        buffer, client,
        demo->gauge_config.full_value > 0.0f
            ? demo->gauge.value / demo->gauge_config.full_value : 0.0f,
        demo->gauge.boosters, demo->kart_spec->max_boosters,
        demo->gauge.unlimited_boosters, demo->gauge.rate > 0.0f,
        kart_gauge_model_name(demo->gauge.model));
    kart_demo_draw_jump_gauge(
        buffer, client, demo->kart.jump.gauge_position,
        demo->kart.jump.jump_strength,
        demo->kart.jump.phase == KART_JUMP_CROUCH);
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
    if (demo->fps_frequency.QuadPart > 0) {
        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        ++demo->fps_sample_frames;
        if (demo->fps_sample_start.QuadPart == 0) {
            demo->fps_sample_start = now;
            demo->fps_sample_frames = 0;
        } else {
            const LONGLONG ticks = now.QuadPart - demo->fps_sample_start.QuadPart;
            if (ticks * 2 >= demo->fps_frequency.QuadPart) {
                demo->fps = (double)demo->fps_sample_frames *
                    (double)demo->fps_frequency.QuadPart / (double)ticks;
                demo->fps_sample_start = now;
                demo->fps_sample_frames = 0;
            }
        }
    }
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
        load_skydomes(create->hInstance, demo);
        load_kart_models(create->hInstance, demo);
        load_track_textures(create->hInstance, demo);
        load_skidmark_texture(create->hInstance, demo);
        demo->kart_colour = KART_COLOURSET_DEFAULT;
        demo->kart_textured = true;
        QueryPerformanceFrequency(&demo->fps_frequency);
        kart_demo_sound_start(create->hInstance, &demo->sound);
        kart_demo_minimap_set_load(create->hInstance, &demo->minimaps);
        if (demo->track_spec == NULL) {
            demo->track_spec = kart_demo_find_track("village_R01");
        }
        load_course(demo);
        reset_kart(demo);
        SetTimer(window, 1, 16, NULL);
        return 0;
    }
    case WM_KEYDOWN:
    case WM_KEYUP:
        if (demo != NULL && message == WM_KEYDOWN && wparam == 'U' &&
            (lparam & (1L << 30)) == 0) {
            const unsigned int selected = kart_demo_popup_select_engine_sound(
                window, demo->sound.engine_preset);
            kart_demo_sound_select_engine(&demo->sound, selected);
            demo->previous_tick = GetTickCount();
            return 0;
        }
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
        controls.jump_input = key_down(VK_SPACE) != 0;
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
            const bool booster_storage_key_down = key_down('H') != 0;
            const bool instant_model_key_down = key_down('Q') != 0;
            const bool boost_cutoff_key_down = key_down('M') != 0;
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
            }
            demo->gauge_key_was_down = gauge_key_down;
            if (booster_storage_key_down &&
                !demo->booster_storage_key_was_down) {
                demo->gauge.unlimited_boosters =
                    !demo->gauge.unlimited_boosters;
                if (!demo->gauge.unlimited_boosters &&
                    demo->gauge.boosters > demo->kart_spec->max_boosters) {
                    demo->gauge.boosters = demo->kart_spec->max_boosters;
                }
            }
            demo->booster_storage_key_was_down = booster_storage_key_down;
            if (instant_model_key_down && !demo->instant_model_key_was_down) {
                demo->kart.instant_boost.stored_model =
                    !demo->kart.instant_boost.stored_model;
                demo->kart.instant_boost.opportunity_timer = 0.0f;
            }
            demo->instant_model_key_was_down = instant_model_key_down;
            if (boost_cutoff_key_down && !demo->boost_cutoff_key_was_down) {
                demo->kart.reverse_input_ends_boost =
                    !demo->kart.reverse_input_ends_boost;
            }
            demo->boost_cutoff_key_was_down = boost_cutoff_key_down;
            const bool shot_key_down = key_down('S') != 0;
            const bool view_key_down = key_down('C') != 0;
            const bool vector_key_down = key_down('V') != 0;
            const bool param_key_down = key_down('P') != 0;
            const bool gate_key_down = key_down('N') != 0;
            const bool bounds_key_down = key_down('B') != 0;
            const bool texture_key_down = key_down('X') != 0;
            const bool kart_texture_key_down = key_down('Z') != 0;
            if (texture_key_down && !demo->texture_key_was_down) {
                demo->track_render_mode = (DemoTrackRenderMode)(
                    (demo->track_render_mode + 1) % DEMO_TRACK_RENDER_MODE_COUNT);
            }
            demo->texture_key_was_down = texture_key_down;
            const bool kart_colour_key_down = key_down('L') != 0;
            if (kart_texture_key_down && !demo->kart_texture_key_was_down) {
                demo->kart_textured = !demo->kart_textured;
            }
            demo->kart_texture_key_was_down = kart_texture_key_down;
            if (kart_colour_key_down && !demo->kart_colour_key_was_down) {
                demo->kart_colour = kart_demo_popup_select_colour(
                    window, demo->kart_colour);
                demo->previous_tick = GetTickCount();
            }
            demo->kart_colour_key_was_down = kart_colour_key_down;
            /* Cheap once the skin is painted: it returns immediately unless the
               kart or the colour changed. */
            paint_kart_skin(demo);
            if (gate_key_down && !demo->gate_key_was_down) {
                demo->show_gates = !demo->show_gates;
            }
            demo->gate_key_was_down = gate_key_down;
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
                    if (!demo->gauge.unlimited_boosters &&
                        demo->gauge.boosters > selected->max_boosters) {
                        demo->gauge.boosters = selected->max_boosters;
                    }
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
                    load_course(demo);
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
                controls.jump_input = false;
            }
        }
        /* The original has no teleport-to-the-line key. Its reset command and
           a fall out of the world both go through the same path at 0x00458000:
           arm a timer, and 500 ms later put the kart back on the node it is
           already in (0x00424640). R does that here; without a course to
           respawn onto, it falls back to the start line. */
        if (key_down('R') && demo->respawn_arm_ms == 0) {
            if (demo->course_ready) {
                demo->respawn_arm_ms = now;
                demo->respawn_notice_ms = 2000;
            } else {
                reset_kart(demo);
            }
        }
        if (elapsed != 0) {
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
                    demo->kart_spec->max_boosters,
                    (float)elapsed * 0.001f);
            }
            /* 0x00426470 walks the kart's position trail one segment at a
               time; one simulation step is one such segment. */
            if (demo->course_ready) {
                KartVec3 warp_destination;
                float warp_yaw;
                if (kart_course_warp_next(
                        &demo->course,
                        demo->previous_position, demo->kart.position,
                        &warp_destination, &warp_yaw)) {
                    demo->kart.position = warp_destination;
                    demo->kart.linear_velocity =
                        vec_rotate_z(demo->kart.linear_velocity, warp_yaw);
                    demo->kart.angular_velocity =
                        vec_rotate_z(demo->kart.angular_velocity, warp_yaw);
                    demo->kart.orientation =
                        quat_pre_rotate_z(demo->kart.orientation, warp_yaw);
                    demo->previous_position = warp_destination;
                } else {
                    kart_course_progress_step(
                        &demo->course, &demo->progress, demo->previous_position,
                        demo->kart.position, demo->kart.orientation,
                        demo->kart.linear_velocity, demo->simulation_time_ms);
                }
            }
            demo->previous_position = demo->kart.position;
        }
        /* Falling out of the world arms the same respawn the reset command
           does, rather than teleporting on the spot. */
        if (demo->kart.position.z <
                kart_demo_track_fall_limit(demo->track_spec) &&
            demo->respawn_arm_ms == 0) {
            if (demo->course_ready) {
                demo->respawn_arm_ms = now;
                demo->respawn_notice_ms = 2000;
            } else {
                reset_kart(demo);
                demo->respawn_notice_ms = 1500;
            }
        }
        if (demo->respawn_arm_ms != 0 && now - demo->respawn_arm_ms >= 500) {
            KartVec3 respawn_position;
            KartQuat respawn_orientation;
            if (kart_course_respawn_pose(
                    &demo->course, &demo->progress, &respawn_position,
                    &respawn_orientation)) {
                /* 0x00428c40 writes the pose and zeroes the velocities with
                   it, which is why the kart lands stopped. */
                demo->kart.position = respawn_position;
                demo->kart.orientation = respawn_orientation;
                demo->kart.linear_velocity = (KartVec3){0.0f, 0.0f, 0.0f};
                demo->kart.angular_velocity = (KartVec3){0.0f, 0.0f, 0.0f};
                demo->previous_position = respawn_position;
                demo->previous_velocity = demo->kart.linear_velocity;
                kart_chase_camera_follow_reset(&demo->camera_follow);
            }
            demo->respawn_arm_ms = 0;
        }
        if (demo->respawn_notice_ms > elapsed) {
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
            kart_track_texture_free(&demo->textures);
            free_skidmark_texture(demo);
            free(demo->skin_texels);
            demo->skin_texels = NULL;
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
    demo.gauge.model = KART_GAUGE_SLIP;
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
