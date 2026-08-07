#ifndef KART_CAMERA_H
#define KART_CAMERA_H

#include "kart_simulation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Chase-camera orientation follow recovered from the original demo.

   ChaseCameraman::Update is vftable slot 8 at 0x00444C30. It does not read the
   kart's angular velocity. It samples the kart's current orientation each
   update and lets a persistent camera quaternion approach it, so a fast-turning
   kart stays visibly rotated relative to the view until the camera catches up.

   The follow time is pushed as an immediate at the call site:
     0x00444DAF  PUSH 0x43C80000   400.0f  Overhead Chase Cameraman
     0x00445244  PUSH 0x42C80000   100.0f  Front Chase Cameraman */
#define KART_CHASE_FOLLOW_OVERHEAD_MS 400.0f
#define KART_CHASE_FOLLOW_FRONT_MS 100.0f

/* Speed filter, ChaseCameraman+0x24. The response is asymmetric: the geometry
   eases outward slowly as speed rises and snaps back quickly as it falls. */
#define KART_CHASE_SPEED_RISE_MS 10000.0f
#define KART_CHASE_SPEED_FALL_MS 100.0f

/* Field of view, ChaseCameraman+0x34, in degrees. The wide value is selected by
   the kart's vftable slot 26 (booster-like runtime flag), not by speed. */
#define KART_CHASE_FOV_NARROW_DEGREES 75.0f
#define KART_CHASE_FOV_WIDE_DEGREES 110.0f
#define KART_CHASE_FOV_NARROW_MS 1500.0f
#define KART_CHASE_FOV_WIDE_MS 1000.0f

/* Chase geometry constants, read from .rdata:
     0x005712C0 = 0.25f   base pitch
     0x00572680 = 400.0f  pitch divisor
     0x00572684 = 5.5f    base and minimum rear distance
     0x00572688 = 0.015f  first distance term
     0x0057267C = 0.03f   second distance term
     0x00572678 = 60.0f   height divisor
     0x005722C0 = 3.0f    base height */
#define KART_CHASE_PITCH_BASE 0.25f
#define KART_CHASE_PITCH_DIVISOR 400.0f
#define KART_CHASE_DISTANCE_BASE 5.5f
#define KART_CHASE_DISTANCE_TERM_A 0.015f
#define KART_CHASE_DISTANCE_TERM_B 0.03f
#define KART_CHASE_HEIGHT_DIVISOR 60.0f
#define KART_CHASE_HEIGHT_BASE 3.0f
/* The final position's Z is smoothed on its own; X and Y are copied. */
#define KART_CHASE_POSITION_Z_MS 100.0f

typedef struct KartChaseCameraFollow {
    KartQuat orientation;
    /* ChaseCameraman+0x24 */
    float filtered_speed;
    /* ChaseCameraman+0x34, degrees */
    float field_of_view;
    /* ChaseCameraman+0x30, the previous frame's camera Z */
    float previous_position_z;
    /* ChaseCameraman+0x3C. Reset to zero; which code writes it is not traced,
       so it stays an input the caller may leave at zero. */
    float extra_pitch;
    /* Mirrors the original reset flag at ChaseCameraman+0x08: while set, the
       next update snaps to the kart instead of interpolating. */
    bool initialized;
} KartChaseCameraFollow;

/* One update's worth of camera placement, all in world space. */
typedef struct KartChaseCameraPose {
    KartVec3 position;
    KartVec3 right;
    KartVec3 up;
    KartVec3 forward;
    float field_of_view_degrees;
} KartChaseCameraPose;

/* Makes the next follow step snap, as the original does after a reset. */
void kart_chase_camera_follow_reset(KartChaseCameraFollow *state);

/* One update of 0x00444C30's orientation path. Returns the camera orientation. */
KartQuat kart_chase_camera_follow(
    KartChaseCameraFollow *state,
    KartQuat kart_orientation,
    unsigned int elapsed_ms,
    float follow_ms);

/* The whole mode-0 update of 0x00444C30: orientation follow, speed filter, FOV
   state, chase pitch/distance/height, and the Z-only position smoothing.

   kart_speed is the magnitude of the kart's linear velocity, as the original
   takes it from kart+0x5C. wide_view is the kart's booster-like flag, which the
   original reads through its vftable slot 26. */
KartChaseCameraPose kart_chase_camera_update(
    KartChaseCameraFollow *state,
    KartVec3 kart_position,
    KartQuat kart_orientation,
    float kart_speed,
    bool wide_view,
    unsigned int elapsed_ms,
    float follow_ms);

/* Focal length in pixels for a viewport height, matching the original's
   projection which takes tan(fov/2) at 0x004A98B1. */
float kart_chase_camera_focal_length(float field_of_view_degrees, float height);

/* The original's interpolation weight, 0x00482590. Not a true slerp: it is a
   polynomial approximation whose curvature depends on the angle between the
   quaternions. Exposed for tests. */
float kart_chase_camera_slerp_weight(float t, float cosine);

/* The original's quaternion interpolation, 0x00481CD0: weight, componentwise
   blend, then normalize. Exposed for tests. */
KartQuat kart_chase_camera_interpolate(KartQuat a, KartQuat b, float t);

#ifdef __cplusplus
}
#endif

#endif
