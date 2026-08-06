#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "kart_demo_data.h"
#include "kart_demo_win32_ui.h"
#include "kart_input.h"
#include "kart_simulation.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define MAX_SKID_SEGMENTS 2048
#define SKID_LIFETIME_MS 15000

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
    bool kart_key_was_down;
    bool track_key_was_down;
    bool drag_key_was_down;
    bool drag_trigger_active;
    bool previous_skid_active;
    bool boost_active;
    KartSteeringInputState steering;
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

static unsigned int query_track_walls(
    void *user_data,
    const KartSimulationState *state,
    KartBodyContact *contacts,
    unsigned int capacity)
{
    const Demo3DState *demo = (const Demo3DState *)user_data;
    const float track_half_width = kart_demo_track_width(demo->track_spec) * 0.5f;
    const float track_half_height = kart_demo_track_length(demo->track_spec) * 0.5f;
    unsigned int count = 0;
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

static int key_down(int virtual_key)
{
    return (GetAsyncKeyState(virtual_key) & 0x8000) != 0;
}

static void reset_kart(Demo3DState *demo)
{
    if (demo->kart_spec == NULL) {
        demo->kart_spec = kart_demo_default_kart();
    }
    if (demo->track_spec == NULL) {
        demo->track_spec = kart_demo_default_track();
    }
    kart_simulation_init(
        &demo->kart, &demo->kart_spec->dynamics, &demo->kart_spec->geometry);
    demo->previous_tick = GetTickCount();
    demo->simulation_time_ms = 0;
    demo->skid_head = 0;
    demo->skid_count = 0;
    demo->previous_skid_active = false;
    demo->boost_active = false;
    demo->drag_trigger_active = false;
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

static Camera3D make_chase_camera(const KartSimulationState *kart, RECT client)
{
    const KartVec3 world_up = {0.0f, 0.0f, 1.0f};
    KartVec3 body_right;
    KartVec3 body_forward;
    KartVec3 body_up;
    KartVec3 flat_forward;
    KartVec3 target;
    Camera3D camera;
    const float width = (float)(client.right - client.left);
    const float height = (float)(client.bottom - client.top);

    orientation_axes(kart->orientation, &body_right, &body_forward, &body_up);
    (void)body_right;
    (void)body_up;
    flat_forward = vec_normalize((KartVec3){body_forward.x, body_forward.y, 0.0f});
    if (vec_dot(flat_forward, flat_forward) == 0.0f) {
        flat_forward = (KartVec3){0.0f, -1.0f, 0.0f};
    }
    target = vec_add(kart->position, vec_add(vec_scale(flat_forward, 4.0f), (KartVec3){0, 0, 0.8f}));
    camera.position = vec_add(
        kart->position,
        vec_add(vec_scale(flat_forward, -11.0f), (KartVec3){0, 0, 7.0f}));
    camera.forward = vec_normalize(vec_sub(target, camera.position));
    /* Keep the chase view in the same screen handedness as the top-down view.
       The former forward x up construction mirrored world X and forced a
       compensating (and error-prone) steering-key reversal. */
    camera.right = vec_normalize(vec_cross(world_up, camera.forward));
    camera.up = vec_cross(camera.forward, camera.right);
    camera.focal_length = (width < height ? width : height) * 0.85f;
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
    const int panel_height = 190;
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
    static const char label[] = "TRACK BOUNDS";
    char kart_size[96];

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
    SelectObject(dc, wall_pen);
    SelectObject(dc, GetStockObject(NULL_BRUSH));
    Rectangle(dc, boundary.left, boundary.top, boundary.right, boundary.bottom);

    orientation_axes(demo->kart.orientation, &right, &forward, &up);
    (void)right;
    (void)up;
    kart_point = (POINT){
        (LONG)(center_x + demo->kart.position.x * scale),
        (LONG)(center_y + demo->kart.position.y * scale),
    };
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
    const KartSimulationState *kart)
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

static void draw_kart(
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
            ? RGB(65, 205, 255)
            : (drift_visual_active(kart)
                ? RGB(255, 165, 35)
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

static void draw_scene(HWND window, HDC target, const Demo3DState *demo)
{
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
    const char *help =
        "Arrows: drive  Shift/W: drift  Ctrl/D: boost  K: kart list  T: track list  G: drag trigger  R: reset  Esc: exit";

    GetClientRect(window, &client);
    buffer = CreateCompatibleDC(target);
    bitmap = CreateCompatibleBitmap(target, client.right, client.bottom);
    old_bitmap = SelectObject(buffer, bitmap);
    sky = CreateSolidBrush(RGB(19, 24, 33));
    FillRect(buffer, &client, sky);
    DeleteObject(sky);

    camera = make_chase_camera(&demo->kart, client);
    draw_track(buffer, client, camera, demo->track_spec);
    draw_skid_marks(buffer, client, camera, demo);
    draw_boost_effect(buffer, client, camera, demo);
    draw_kart(
        buffer, client, camera, &demo->kart, demo->kart_spec,
        demo->boost_active);
    draw_motion_vectors(buffer, client, camera, &demo->kart);
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
    TextOutA(buffer, 16, 32, help, (int)strlen(help));
    snprintf(
        status,
        sizeof(status),
        "%s (%s) %.1f x %.1f | kart %s %.3f x %.3f | h %.2f",
        demo->track_spec->display_name,
        demo->track_spec->asset_name,
        kart_demo_track_width(demo->track_spec),
        kart_demo_track_length(demo->track_spec),
        demo->kart_spec->asset_name,
        demo->kart.geometry.half_width * 2.0f,
        demo->kart.geometry.half_length * 2.0f,
        demo->kart.position.z);
    SetTextColor(
        buffer,
        demo->kart.drift.slip_detected ? RGB(255, 185, 55) : RGB(180, 195, 205));
    TextOutA(buffer, 16, 52, status, (int)strlen(status));
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
    TextOutA(buffer, 16, 72, status, (int)strlen(status));
    kart_demo_draw_speedometer(
        buffer, client, speedometer_kmh, demo->boost_active);

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
        KartSimulationControls controls = {0};
        const KartSimulationWorld world = {
            .query_ground = query_flat_ground,
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
        if (key_down(VK_ESCAPE)) {
            DestroyWindow(window);
        } else if (key_down('R')) {
            reset_kart(demo);
        } else if (elapsed != 0) {
            kart_simulate_milliseconds(&demo->kart, &controls, &world, elapsed);
        }
        demo->boost_active = kart_any_boost_active(
            &demo->kart.timed_boost, &demo->kart.instant_boost);
        demo->simulation_time_ms += elapsed;
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
        PostQuitMessage(0);
        return 0;
    default:
        return DefWindowProc(window, message, wparam, lparam);
    }
}

int WINAPI WinMain(HINSTANCE instance, HINSTANCE previous, LPSTR command_line, int show)
{
    static const char CLASS_NAME[] = "KartPhysics3DWindow";
    WNDCLASSA window_class = {0};
    Demo3DState demo = {0};
    HWND window;
    MSG message;
    (void)previous;
    (void)command_line;
    (void)show;

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
        "KartRider Demo Physics - 3D chase camera",
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
