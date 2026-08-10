#include "kart_camera.h"

#include <math.h>
#include <stddef.h>

/* 0x00482590. The two magic constants are read directly from .rdata:
     0x005758F0 = 0.8227968811988831f
     0x005758F4 = 0.5854921936988831f
   and the polynomial's 2.0f / 3.0f come from 0x00571948 / 0x005722C0.

     base = 1 - 0.82279688 * cosine
     k    = 0.58549219 * base * base
     w    = ((2t - 3) * k * t + 1 + k) * t

   At k = 0 this is the identity, so nearly-aligned quaternions blend linearly;
   the wider the angle, the more the curve bends. */
float kart_chase_camera_slerp_weight(float t, float cosine)
{
    const float base = 1.0f - 0.8227968811988831f * cosine;
    const float k = 0.5854921936988831f * base * base;
    return ((2.0f * t - 3.0f) * k * t + 1.0f + k) * t;
}

/* 0x00481CD0. The weight is evaluated on whichever half of the interval keeps
   t small, then the components are blended linearly and renormalized.

   The original normalizes with a fast reciprocal-square-root approximation
   (0x00482490) built on runtime-initialized tables. This uses the exact square
   root instead; that is a numerical difference of about 1e-7, not a behavioural
   one, and it is the only place this file departs from the recovered code. */
KartQuat kart_chase_camera_interpolate(KartQuat a, KartQuat b, float t)
{
    const float cosine = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    const float weight = t > 0.5f
        ? 1.0f - kart_chase_camera_slerp_weight(1.0f - t, cosine)
        : kart_chase_camera_slerp_weight(t, cosine);
    KartQuat result;
    float length;

    result.w = a.w + (b.w - a.w) * weight;
    result.x = a.x + (b.x - a.x) * weight;
    result.y = a.y + (b.y - a.y) * weight;
    result.z = a.z + (b.z - a.z) * weight;

    length = sqrtf(result.w * result.w + result.x * result.x +
                   result.y * result.y + result.z * result.z);
    if (length > 0.0f) {
        const float inverse = 1.0f / length;
        result.w *= inverse;
        result.x *= inverse;
        result.y *= inverse;
        result.z *= inverse;
    }
    return result;
}

void kart_chase_camera_follow_reset(KartChaseCameraFollow *state)
{
    if (state == NULL) return;
    state->orientation = (KartQuat){.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};
    state->filtered_speed = 0.0f;
    /* The reset path stores 0x42960000 into +0x34. */
    state->field_of_view = KART_CHASE_FOV_NARROW_DEGREES;
    state->previous_position_z = 0.0f;
    state->extra_pitch = 0.0f;
    state->initialized = false;
}

/* 0x00447510 */
static float follow_alpha(unsigned int elapsed_ms, float response_ms)
{
    float alpha;
    if (response_ms <= 0.0f) return 1.0f;
    alpha = (float)elapsed_ms / response_ms;
    return alpha > 1.0f ? 1.0f : alpha;
}

/* Columns of the rotation matrix 0x0042B640 builds from a quaternion, picked
   out by 0x0042B330. Column 0 is body right, column 2 is body up, and the
   engine's forward is the negated column 1 (0x00413650 applied to it). */
static void orientation_columns(
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
        1.0f - 2.0f * (yy + zz), 2.0f * (xy + wz), 2.0f * (xz - wy)};
    *forward = (KartVec3){
        -2.0f * (xy - wz), -(1.0f - 2.0f * (xx + zz)), -2.0f * (yz + wx)};
    *up = (KartVec3){
        2.0f * (xz + wy), 2.0f * (yz - wx), 1.0f - 2.0f * (xx + yy)};
}

float kart_chase_camera_focal_length(float field_of_view_degrees, float height)
{
    /* 0x004A98B1 multiplies the FOV by 0.00872664f (pi/360) before calling
       tangent, so the stored value is a full angle in degrees. */
    const float half = field_of_view_degrees * 0.00872664f;
    const float tangent = tanf(half);
    if (tangent <= 0.0f) return height;
    return height * 0.5f / tangent;
}

KartChaseCameraPose kart_chase_camera_update(
    KartChaseCameraFollow *state,
    KartVec3 kart_position,
    KartQuat kart_orientation,
    float kart_speed,
    bool wide_view,
    unsigned int elapsed_ms,
    float follow_ms)
{
    KartChaseCameraPose pose;
    KartVec3 right;
    KartVec3 forward;
    KartVec3 up;
    float pitch;
    float pitch_cos;
    float pitch_sin;
    float distance;
    float height;
    float filtered;
    const bool was_initialized = state != NULL && state->initialized;

    if (state == NULL) {
        pose.position = kart_position;
        pose.right = (KartVec3){1.0f, 0.0f, 0.0f};
        pose.forward = (KartVec3){0.0f, 1.0f, 0.0f};
        pose.up = (KartVec3){0.0f, 0.0f, 1.0f};
        pose.field_of_view_degrees = KART_CHASE_FOV_NARROW_DEGREES;
        return pose;
    }

    kart_chase_camera_follow(state, kart_orientation, elapsed_ms, follow_ms);

    if (!was_initialized) {
        /* The reset branch stores the sampled values straight through instead
           of filtering them. */
        state->filtered_speed = kart_speed;
        state->field_of_view = KART_CHASE_FOV_NARROW_DEGREES;
        state->extra_pitch = 0.0f;
    } else {
        /* Rising speed eases out over 10 s; falling speed pulls in over 100 ms. */
        const float speed_beta = follow_alpha(
            elapsed_ms,
            state->filtered_speed <= kart_speed ? KART_CHASE_SPEED_RISE_MS
                                                : KART_CHASE_SPEED_FALL_MS);
        const float fov_target = wide_view ? KART_CHASE_FOV_WIDE_DEGREES
                                           : KART_CHASE_FOV_NARROW_DEGREES;
        const float fov_gamma = follow_alpha(
            elapsed_ms,
            wide_view ? KART_CHASE_FOV_WIDE_MS : KART_CHASE_FOV_NARROW_MS);
        state->filtered_speed =
            kart_speed * speed_beta + (1.0f - speed_beta) * state->filtered_speed;
        state->field_of_view =
            fov_target * fov_gamma + (1.0f - fov_gamma) * state->field_of_view;
    }

    filtered = state->filtered_speed;
    orientation_columns(state->orientation, &right, &forward, &up);

    /* The pitch input is clamped at zero by 0x0042AFA0 before the divide. */
    pitch = (filtered > 0.0f ? filtered : 0.0f) / KART_CHASE_PITCH_DIVISOR +
            KART_CHASE_PITCH_BASE + state->extra_pitch;
    pitch_cos = cosf(pitch);
    pitch_sin = sinf(pitch);

    distance = filtered * KART_CHASE_DISTANCE_TERM_A + KART_CHASE_DISTANCE_BASE;
    distance = filtered * KART_CHASE_DISTANCE_TERM_B + distance;
    if (distance < KART_CHASE_DISTANCE_BASE) distance = KART_CHASE_DISTANCE_BASE;

    height = filtered / KART_CHASE_HEIGHT_DIVISOR + KART_CHASE_HEIGHT_BASE;

    /* out_orientation = camera_rotation * RotationX(pitch) (0x0042B370 with
       0x0047EE40). Taking that product's columns rotates forward and up in
       their own plane and leaves right alone, so a positive pitch tilts the
       view down toward the kart. */
    pose.right = right;
    pose.forward = (KartVec3){
        forward.x * pitch_cos - up.x * pitch_sin,
        forward.y * pitch_cos - up.y * pitch_sin,
        forward.z * pitch_cos - up.z * pitch_sin};
    pose.up = (KartVec3){
        forward.x * pitch_sin + up.x * pitch_cos,
        forward.y * pitch_sin + up.y * pitch_cos,
        forward.z * pitch_sin + up.z * pitch_cos};

    /* position = kart - forward * distance + up * height, using the unpitched
       axes, which is the order 0x00444C30 builds it in. */
    pose.position = (KartVec3){
        kart_position.x - forward.x * distance + up.x * height,
        kart_position.y - forward.y * distance + up.y * height,
        kart_position.z - forward.z * distance + up.z * height};

    /* Only Z is smoothed, and only once the camera has a previous frame. */
    if (was_initialized) {
        const float z_alpha = follow_alpha(elapsed_ms, KART_CHASE_POSITION_Z_MS);
        pose.position.z = pose.position.z * z_alpha +
                          (1.0f - z_alpha) * state->previous_position_z;
    }
    state->previous_position_z = pose.position.z;

    pose.field_of_view_degrees = state->field_of_view;
    return pose;
}

KartQuat kart_chase_camera_follow(
    KartChaseCameraFollow *state,
    KartQuat kart_orientation,
    unsigned int elapsed_ms,
    float follow_ms)
{
    float alpha;
    float cosine;

    if (state == NULL) return kart_orientation;
    if (!state->initialized || follow_ms <= 0.0f) {
        state->orientation = kart_orientation;
        state->initialized = true;
        return state->orientation;
    }

    /* q and -q are the same orientation. The original picks the same hemisphere
       first so the blend takes the shorter arc. This is not an angle limit. */
    cosine = state->orientation.w * kart_orientation.w +
             state->orientation.x * kart_orientation.x +
             state->orientation.y * kart_orientation.y +
             state->orientation.z * kart_orientation.z;
    if (cosine < 0.0f) {
        state->orientation.w = -state->orientation.w;
        state->orientation.x = -state->orientation.x;
        state->orientation.y = -state->orientation.y;
        state->orientation.z = -state->orientation.z;
    }

    /* 0x00447510: alpha = min(elapsed / follow_ms, 1). Only the upper bound is
       clamped, so a long stall snaps the camera rather than overshooting. */
    alpha = (float)elapsed_ms / follow_ms;
    if (alpha > 1.0f) alpha = 1.0f;

    state->orientation =
        kart_chase_camera_interpolate(state->orientation, kart_orientation, alpha);
    return state->orientation;
}
