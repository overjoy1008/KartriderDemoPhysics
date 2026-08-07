#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "kart_axis_gizmo_win32.h"
#include "kart_demo_data.h"
#include "kart_demo_win32_ui.h"
#include "kart_input.h"
#include "kart_minimap_win32.h"
#include "kart_simulation.h"
#include "kart_sound_win32.h"
#include "kart_track_collision.h"
#include "kart_track_scene.h"
#include "kart_track_scene_resources.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static const float VIEW_HALF_WIDTH = 55.0f;
static const float VIEW_HALF_HEIGHT = 38.0f;
#define MAX_SKID_SEGMENTS 2048
#define SKID_LIFETIME_MS 15000

typedef struct SkidSegment {
    float left_x0;
    float left_y0;
    float left_x1;
    float left_y1;
    float right_x0;
    float right_y0;
    float right_x1;
    float right_y1;
    unsigned int created_ms;
} SkidSegment;

typedef struct DemoState {
    KartSimulationState kart;
    const KartDemoKartSpec *kart_spec;
    const KartDemoTrackSpec *track_spec;
    DWORD previous_tick;
    unsigned int simulation_time_ms;
    unsigned int skid_head;
    unsigned int skid_count;
    SkidSegment skid_segments[MAX_SKID_SEGMENTS];
    float previous_skid_left_x;
    float previous_skid_left_y;
    float previous_skid_right_x;
    float previous_skid_right_y;
    bool kart_key_was_down;
    bool track_key_was_down;
    bool drag_key_was_down;
    bool drag_trigger_active;
    bool previous_skid_active;
    bool boost_active;
    KartSteeringInputState steering;
    KartDemoMinimapSet minimaps;
    /* Counts down after an out-of-world respawn so the HUD can say why the
       kart moved. */
    DWORD respawn_notice_ms;
    KartTrackScene scenes[KART_TRACK_SCENE_CAPACITY];
    KartDemoSound sound;
} DemoState;

static bool load_track_scene_resource(
    HINSTANCE instance,
    int resource_id,
    KartTrackScene *scene)
{
#if defined(KART_EMBED_TRACK_SCENES)
    HRSRC resource = FindResourceA(
        instance, MAKEINTRESOURCEA(resource_id), RT_RCDATA);
    HGLOBAL loaded;
    const void *data;
    DWORD size;
    if (resource == NULL) return false;
    loaded = LoadResource(instance, resource);
    size = SizeofResource(instance, resource);
    data = loaded != NULL ? LockResource(loaded) : NULL;
    return data != NULL && size != 0 &&
           kart_track_scene_load_compressed(scene, data, (size_t)size);
#else
    (void)instance;
    (void)resource_id;
    (void)scene;
    return false;
#endif
}

static void load_track_scenes(HINSTANCE instance, DemoState *demo)
{
    unsigned int i;
    const unsigned int count = kart_demo_track_count();
    for (i = 0; i < count && i < KART_TRACK_SCENE_CAPACITY; ++i) {
        /* Id 0 marks a track with no mesh, such as the flat test track. */
        if (KART_TRACK_SCENE_RESOURCE_IDS[i] == 0) continue;
        load_track_scene_resource(
            instance, KART_TRACK_SCENE_RESOURCE_IDS[i], &demo->scenes[i]);
    }
}

static void free_track_scenes(DemoState *demo)
{
    unsigned int i;
    for (i = 0; i < KART_TRACK_SCENE_CAPACITY; ++i) {
        kart_track_scene_free(&demo->scenes[i]);
    }
}

static const KartTrackScene *active_track_scene(const DemoState *demo)
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
    const DemoState *demo = (const DemoState *)user_data;
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
    const DemoState *demo = (const DemoState *)user_data;
    const float track_half_width = kart_demo_track_width(demo->track_spec) * 0.5f;
    const float track_half_height = kart_demo_track_length(demo->track_spec) * 0.5f;
    const KartTrackScene *scene = active_track_scene(demo);
    unsigned int count = scene != NULL
        ? kart_track_scene_query_body_collisions(
            scene, demo->track_spec, state, contacts, capacity)
        : 0;

#define ADD_CONTACT(nx, ny) do { \
    if (count < capacity) { \
        contacts[count].normal = (KartVec3){(nx), (ny), 0.0f}; \
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

static void reset_kart(DemoState *demo)
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
    demo->drag_trigger_active = false;
}

static int key_down(int virtual_key)
{
    return (GetAsyncKeyState(virtual_key) & 0x8000) != 0;
}

static KartVec3 kart_forward(KartQuat q)
{
    return (KartVec3){
        -2.0f * (q.x * q.y - q.w * q.z),
        -(1.0f - 2.0f * (q.x * q.x + q.z * q.z)),
        -2.0f * (q.y * q.z + q.w * q.x),
    };
}

static bool drift_visual_active(const KartSimulationState *kart)
{
    return kart->drift.input_active || kart->drift.trigger_active ||
           kart->drift.slip_detected || kart->drift.linger_timer > 0.0f;
}

static void update_skid_marks(DemoState *demo)
{
    const KartVec3 forward = kart_forward(demo->kart.orientation);
    const float side_x = -forward.y;
    const float side_y = forward.x;
    const float speed = sqrtf(
        demo->kart.linear_velocity.x * demo->kart.linear_velocity.x +
        demo->kart.linear_velocity.y * demo->kart.linear_velocity.y);
    const float rear_offset = demo->kart.geometry.half_length * 0.8f;
    const float side_offset = demo->kart.geometry.half_width * 0.8f;
    const float rear_x = demo->kart.position.x - forward.x * rear_offset;
    const float rear_y = demo->kart.position.y - forward.y * rear_offset;
    const float left_x = rear_x + side_x * side_offset;
    const float left_y = rear_y + side_y * side_offset;
    const float right_x = rear_x - side_x * side_offset;
    const float right_y = rear_y - side_y * side_offset;
    const bool active = demo->kart.grounded && speed > 5.0f &&
                        drift_visual_active(&demo->kart);

    if (active && demo->previous_skid_active) {
        SkidSegment *segment = &demo->skid_segments[demo->skid_head];
        *segment = (SkidSegment){
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
    /* World +Y points up the screen and world +X points left, matching the
       original minimap artwork's orientation. Both axes are negated together,
       which is a rotation rather than a mirror, so steering handedness and the
       Track Map panel stay consistent with the main view. */
    const POINT point = {
        (LONG)((client.right + client.left) * 0.5f - (x - camera_x) * scale),
        (LONG)((client.bottom + client.top) * 0.5f - (y - camera_y) * scale),
    };
    return point;
}

static void draw_track_minimap(
    HDC dc,
    RECT client,
    const DemoState *demo)
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
    const KartVec3 forward = kart_forward(demo->kart.orientation);
    const float right_x = -forward.y;
    const float right_y = forward.x;
    const KartVec3 minimap_forward = kart_demo_minimap_direction(
        demo->track_spec, forward);
    const KartVec3 minimap_right = kart_demo_minimap_direction(
        demo->track_spec, (KartVec3){right_x, right_y, 0.0f});
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
        kart_point = kart_demo_minimap_kart_point(
            image_rect, minimap, demo->track_spec, demo->kart.position);
    } else {
        kart_point = kart_demo_minimap_bounds_point(
            boundary, demo->track_spec, demo->kart.position);
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

    kart_triangle[0] = (POINT){
        kart_point.x + (LONG)(minimap_forward.x * 9.0f),
        kart_point.y + (LONG)(minimap_forward.y * 9.0f),
    };
    kart_triangle[1] = (POINT){
        kart_point.x - (LONG)(minimap_forward.x * 6.0f) +
            (LONG)(minimap_right.x * 5.0f),
        kart_point.y - (LONG)(minimap_forward.y * 6.0f) +
            (LONG)(minimap_right.y * 5.0f),
    };
    kart_triangle[2] = (POINT){
        kart_point.x - (LONG)(minimap_forward.x * 6.0f) -
            (LONG)(minimap_right.x * 5.0f),
        kart_point.y - (LONG)(minimap_forward.y * 6.0f) -
            (LONG)(minimap_right.y * 5.0f),
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

static void draw_scene(HWND window, HDC target, const DemoState *demo)
{
    RECT client;
    HDC buffer;
    HBITMAP bitmap;
    HGDIOBJ old_bitmap;
    HBRUSH background;
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
    KartVec3 forward;
    float side_x;
    float side_y;
    float speed;
    float forward_speed;
    float lateral_speed;
    float slip_angle;
    float camera_x;
    float camera_y;
    float track_half_width;
    float track_half_height;
    int speedometer_kmh;
    char text[320];

    GetClientRect(window, &client);
    buffer = CreateCompatibleDC(target);
    bitmap = CreateCompatibleBitmap(target, client.right, client.bottom);
    old_bitmap = SelectObject(buffer, bitmap);

    background = CreateSolidBrush(RGB(22, 25, 31));
    FillRect(buffer, &client, background);
    DeleteObject(background);

    camera_x = demo->kart.position.x;
    camera_y = demo->kart.position.y;
    track_half_width = kart_demo_track_width(demo->track_spec) * 0.5f;
    track_half_height = kart_demo_track_length(demo->track_spec) * 0.5f;
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
            const SkidSegment *segment =
                &demo->skid_segments[(oldest + i) % MAX_SKID_SEGMENTS];
            const unsigned int age = demo->simulation_time_ms - segment->created_ms;
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

    forward = kart_forward(demo->kart.orientation);
    side_x = -forward.y;
    side_y = forward.x;
    center = world_to_screen(
        client, camera_x, camera_y, demo->kart.position.x, demo->kart.position.y);
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
            ? RGB(65, 205, 255)
            : (drift_visual_active(&demo->kart)
                ? RGB(255, 170, 45)
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
    speedometer_kmh = kart_speedometer_kmh(demo->kart.linear_velocity);
    if (speed > 0.01f) {
        const float arrow_length = fminf(speed * 0.18f, 9.0f);
        POINT velocity_end = world_to_screen(
            client,
            camera_x,
            camera_y,
            demo->kart.position.x + demo->kart.linear_velocity.x / speed * arrow_length,
            demo->kart.position.y + demo->kart.linear_velocity.y / speed * arrow_length);
        velocity_pen = CreatePen(PS_SOLID, 3, RGB(80, 230, 255));
        old_pen = SelectObject(buffer, velocity_pen);
        MoveToEx(buffer, center.x, center.y, NULL);
        LineTo(buffer, velocity_end.x, velocity_end.y);
        SelectObject(buffer, old_pen);
        DeleteObject(velocity_pen);
    }

    draw_track_minimap(buffer, client, demo);

    forward_speed =
        demo->kart.linear_velocity.x * forward.x + demo->kart.linear_velocity.y * forward.y;
    lateral_speed =
        demo->kart.linear_velocity.x * side_x + demo->kart.linear_velocity.y * side_y;
    slip_angle = atan2f(fabsf(lateral_speed), fabsf(forward_speed)) *
                 (180.0f / 3.14159265358979323846f);
    SetBkMode(buffer, TRANSPARENT);
    SetTextColor(buffer, RGB(235, 240, 245));
    snprintf(
        text,
        sizeof(text),
        "speed %.2f m/s | slip %.1f deg | vf %.1f vs %.1f | AUTO %s",
        speed,
        slip_angle,
        forward_speed,
        lateral_speed,
        demo->kart.drift.slip_detected ? "ON" : "off");
    TextOutA(buffer, 16, 12, text, (int)strlen(text));
    {
        const char *help =
            "Arrows: drive  Shift/W: drift  Ctrl/D: boost  K: kart list  T: track list  G: drag trigger  R: reset";
        TextOutA(buffer, 16, 32, help, (int)strlen(help));
    }
    snprintf(
        text,
        sizeof(text),
        "%s (%s) %.1f x %.1f | surface %s | kart %s %.3f x %.3f",
        demo->track_spec->display_name,
        demo->track_spec->asset_name,
        kart_demo_track_width(demo->track_spec),
        kart_demo_track_length(demo->track_spec),
        active_track_scene(demo) != NULL ? "KTRK collision" : "flat",
        demo->kart_spec->asset_name,
        demo->kart.geometry.half_width * 2.0f,
        demo->kart.geometry.half_length * 2.0f);
    SetTextColor(
        buffer,
        demo->kart.drift.slip_detected ? RGB(255, 185, 55) : RGB(180, 195, 205));
    kart_demo_text_out_utf8(buffer, 16, 52, text);
    snprintf(
        text,
        sizeof(text),
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
    TextOutA(buffer, 16, 72, text, (int)strlen(text));
    if (demo->respawn_notice_ms != 0) {
        static const char notice[] = "FELL THROUGH THE TRACK - RESPAWNED";
        SetTextColor(buffer, RGB(255, 120, 120));
        TextOutA(buffer, 16, 92, notice, (int)strlen(notice));
    }
    kart_demo_draw_speedometer(
        buffer, client, speedometer_kmh, demo->boost_active);
    {
        /* world_to_screen maps +X to screen left and +Y to screen up, and the
           view looks straight down, so +Z points at the viewer. */
        static const float axis_x[3] = {-1.0f, 0.0f, 0.0f};
        static const float axis_y[3] = {0.0f, -1.0f, 0.0f};
        static const float axis_toward[3] = {0.0f, 0.0f, 1.0f};
        kart_demo_draw_axis_gizmo(buffer, client, axis_x, axis_y, axis_toward);
    }

    BitBlt(target, 0, 0, client.right, client.bottom, buffer, 0, 0, SRCCOPY);
    SelectObject(buffer, old_bitmap);
    DeleteObject(bitmap);
    DeleteDC(buffer);
}

static LRESULT CALLBACK window_proc(HWND window, UINT message, WPARAM wparam, LPARAM lparam)
{
    DemoState *demo = (DemoState *)GetWindowLongPtr(window, GWLP_USERDATA);
    switch (message) {
    case WM_CREATE: {
        CREATESTRUCT *create = (CREATESTRUCT *)lparam;
        demo = (DemoState *)create->lpCreateParams;
        SetWindowLongPtr(window, GWLP_USERDATA, (LONG_PTR)demo);
        load_track_scenes(create->hInstance, demo);
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
        /* In this projection negative yaw is visually left. The recovered
           travel-direction term already preserves the expected reverse turn. */
        controls.steering_input = demo->steering.value;
        controls.reverse_steering = false;
        controls.drift_input = key_down(VK_SHIFT) || key_down('W');
        controls.boost_active = key_down(VK_CONTROL) || key_down('D');
        {
            const bool kart_key_down = key_down('K') != 0;
            const bool track_key_down = key_down('T') != 0;
            const bool drag_key_down = key_down('G') != 0;
            if (kart_key_down && !demo->kart_key_was_down) {
                const KartDemoKartSpec *selected =
                    kart_demo_popup_select_kart(window, demo->kart_spec);
                if (selected != demo->kart_spec) {
                    demo->kart_spec = selected;
                    reset_kart(demo);
                }
                demo->previous_tick = GetTickCount();
            }
            if (track_key_down && !demo->track_key_was_down) {
                const KartDemoTrackSpec *selected =
                    kart_demo_popup_select_track(window, demo->track_spec);
                if (selected != demo->track_spec) {
                    demo->track_spec = selected;
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
            demo->drag_key_was_down = drag_key_down;
        }
        if (key_down('R')) {
            reset_kart(demo);
        } else if (elapsed != 0) {
            step = kart_simulate_milliseconds(&demo->kart, &controls, &world, elapsed);
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
        demo->boost_active = kart_any_boost_active(
            &demo->kart.timed_boost, &demo->kart.instant_boost);
        demo->simulation_time_ms += elapsed;
        kart_demo_sound_update(
            &demo->sound, &demo->kart, demo->boost_active,
            step.wall_impact_speed, step.ground_impact_speed, now);
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
            kart_demo_minimap_set_free(&demo->minimaps);
            free_track_scenes(demo);
        }
        PostQuitMessage(0);
        return 0;
    default:
        return DefWindowProc(window, message, wparam, lparam);
    }
}

int WINAPI WinMain(HINSTANCE instance, HINSTANCE previous, LPSTR command_line, int show)
{
    static const char CLASS_NAME[] = "KartPhysicsTopDownWindow";
    WNDCLASSA window_class = {0};
    DemoState demo = {0};
    HWND window;
    MSG message;
    (void)previous;
    (void)command_line;

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
        "KartRider Demo Physics - Top-down 2D",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        1100,
        800,
        NULL,
        NULL,
        instance,
        &demo);
    if (window == NULL) {
        return 1;
    }
    (void)show;
    ShowWindow(window, SW_SHOW);
    UpdateWindow(window);

    while (GetMessage(&message, NULL, 0, 0) > 0) {
        TranslateMessage(&message);
        DispatchMessage(&message);
    }
    return (int)message.wParam;
}
