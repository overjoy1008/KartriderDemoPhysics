/* Chase-camera orientation follow recovered from KartRider.exe 0x00444C30.

   These pin the recovered constants and the shape of the curve, so a later
   edit cannot quietly turn the follow into an invented smoothing. */

#include "kart_camera.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static int near(float actual, float expected, float epsilon)
{
    return fabsf(actual - expected) <= epsilon;
}

static KartQuat yaw_quat(float radians)
{
    return (KartQuat){
        .w = cosf(radians * 0.5f), .x = 0.0f, .y = 0.0f,
        .z = sinf(radians * 0.5f)};
}

/* Signed Z rotation of a quaternion that only rotates about Z, in (-pi, pi].
   q and -q are the same rotation, so the sign is canonicalized first; without
   that the hemisphere test below would measure the wrong branch. */
static float yaw_of(KartQuat q)
{
    if (q.w < 0.0f) {
        q.w = -q.w;
        q.z = -q.z;
    }
    return 2.0f * atan2f(q.z, q.w);
}

static void test_slerp_weight_matches_recovered_polynomial(void)
{
    /* 0x00482590: base = 1 - 0.82279688*cos, k = 0.58549219*base^2,
       w = ((2t - 3)*k*t + 1 + k)*t */
    const float cosine = 0.5f;
    const float base = 1.0f - 0.8227968811988831f * cosine;
    const float k = 0.5854921936988831f * base * base;
    const float t = 0.25f;
    const float expected = ((2.0f * t - 3.0f) * k * t + 1.0f + k) * t;
    assert(near(kart_chase_camera_slerp_weight(t, cosine), expected, 1e-6f));

    /* Endpoints are exact for any angle. */
    assert(near(kart_chase_camera_slerp_weight(0.0f, cosine), 0.0f, 1e-6f));
    assert(near(kart_chase_camera_slerp_weight(1.0f, cosine), 1.0f, 1e-6f));

    /* The midpoint is a fixed point for every angle: the k terms cancel. */
    assert(near(kart_chase_camera_slerp_weight(0.5f, 1.0f), 0.5f, 1e-6f));
    assert(near(kart_chase_camera_slerp_weight(0.5f, 0.0f), 0.5f, 1e-6f));

    /* Below the midpoint the weight runs ahead of t, which is what lets a
       normalized linear blend approximate constant angular speed, and the
       wider the angle the further ahead it runs. */
    assert(kart_chase_camera_slerp_weight(0.25f, 1.0f) > 0.25f);
    assert(kart_chase_camera_slerp_weight(0.25f, 0.0f) >
           kart_chase_camera_slerp_weight(0.25f, 1.0f));
    /* Near-aligned quaternions are almost a plain lerp: k is about 0.018. */
    assert(near(kart_chase_camera_slerp_weight(0.25f, 1.0f), 0.25f, 0.01f));
}

static void test_interpolation_endpoints_and_normalization(void)
{
    const KartQuat a = yaw_quat(0.0f);
    const KartQuat b = yaw_quat(1.2f);
    KartQuat mid;
    assert(near(yaw_of(kart_chase_camera_interpolate(a, b, 0.0f)), 0.0f, 1e-4f));
    assert(near(yaw_of(kart_chase_camera_interpolate(a, b, 1.0f)), 1.2f, 1e-4f));
    mid = kart_chase_camera_interpolate(a, b, 0.5f);
    assert(near(mid.w * mid.w + mid.x * mid.x + mid.y * mid.y + mid.z * mid.z,
                1.0f, 1e-5f));
    /* Strictly between the endpoints, and moving the right way. */
    assert(yaw_of(mid) > 0.0f && yaw_of(mid) < 1.2f);
}

static void test_first_update_snaps(void)
{
    /* Mirrors the reset flag at ChaseCameraman+0x08. */
    KartChaseCameraFollow state;
    KartQuat result;
    kart_chase_camera_follow_reset(&state);
    result = kart_chase_camera_follow(
        &state, yaw_quat(2.0f), 16, KART_CHASE_FOLLOW_OVERHEAD_MS);
    assert(near(yaw_of(result), 2.0f, 1e-4f));
}

static void test_follow_lags_then_converges(void)
{
    KartChaseCameraFollow state;
    KartQuat camera;
    const KartQuat target = yaw_quat(1.0f);
    int step;

    kart_chase_camera_follow_reset(&state);
    kart_chase_camera_follow(&state, yaw_quat(0.0f), 0,
                             KART_CHASE_FOLLOW_OVERHEAD_MS);

    /* One 16 ms update at 400 ms moves only a small fraction of the way. */
    camera = kart_chase_camera_follow(&state, target, 16,
                                      KART_CHASE_FOLLOW_OVERHEAD_MS);
    assert(yaw_of(camera) > 0.0f);
    assert(yaw_of(camera) < 0.10f);

    /* It keeps closing and never overshoots. */
    for (step = 0; step < 200; ++step) {
        const float before = yaw_of(camera);
        camera = kart_chase_camera_follow(&state, target, 16,
                                          KART_CHASE_FOLLOW_OVERHEAD_MS);
        assert(yaw_of(camera) >= before - 1e-5f);
        assert(yaw_of(camera) <= 1.0f + 1e-4f);
    }
    assert(near(yaw_of(camera), 1.0f, 1e-3f));
}

static void test_front_mode_is_faster_than_overhead(void)
{
    /* 400 ms overhead vs 100 ms front: the same elapsed time must close more. */
    KartChaseCameraFollow overhead;
    KartChaseCameraFollow front;
    const KartQuat start = yaw_quat(0.0f);
    const KartQuat target = yaw_quat(1.0f);
    KartQuat a;
    KartQuat b;
    kart_chase_camera_follow_reset(&overhead);
    kart_chase_camera_follow_reset(&front);
    kart_chase_camera_follow(&overhead, start, 0, KART_CHASE_FOLLOW_OVERHEAD_MS);
    kart_chase_camera_follow(&front, start, 0, KART_CHASE_FOLLOW_FRONT_MS);
    a = kart_chase_camera_follow(&overhead, target, 16,
                                 KART_CHASE_FOLLOW_OVERHEAD_MS);
    b = kart_chase_camera_follow(&front, target, 16, KART_CHASE_FOLLOW_FRONT_MS);
    assert(yaw_of(b) > yaw_of(a));
}

static void test_long_stall_snaps(void)
{
    /* 0x00447510 clamps alpha at 1, so a stall longer than the follow time
       lands exactly on the kart rather than overshooting. */
    KartChaseCameraFollow state;
    KartQuat camera;
    kart_chase_camera_follow_reset(&state);
    kart_chase_camera_follow(&state, yaw_quat(0.0f), 0,
                             KART_CHASE_FOLLOW_OVERHEAD_MS);
    camera = kart_chase_camera_follow(&state, yaw_quat(1.0f), 5000,
                                      KART_CHASE_FOLLOW_OVERHEAD_MS);
    assert(near(yaw_of(camera), 1.0f, 1e-4f));
}

static void test_hemisphere_correction_takes_the_short_arc(void)
{
    /* q and -q are the same orientation; without the dot < 0 flip the blend
       would swing the long way around. */
    KartChaseCameraFollow state;
    KartQuat target = yaw_quat(0.2f);
    KartQuat camera;
    kart_chase_camera_follow_reset(&state);
    kart_chase_camera_follow(&state, yaw_quat(0.0f), 0,
                             KART_CHASE_FOLLOW_OVERHEAD_MS);
    target.w = -target.w;
    target.x = -target.x;
    target.y = -target.y;
    target.z = -target.z;
    camera = kart_chase_camera_follow(&state, target, 16,
                                      KART_CHASE_FOLLOW_OVERHEAD_MS);
    /* Small positive step toward 0.2, not a large step the other way. */
    assert(fabsf(yaw_of(camera)) < 0.05f);
}

/* Runs the camera to a steady state at a constant speed. */
static KartChaseCameraPose settle(
    KartChaseCameraFollow *state, float speed, bool wide_view, int seconds)
{
    const KartQuat level = {.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};
    const KartVec3 origin = {0.0f, 0.0f, 0.0f};
    KartChaseCameraPose pose;
    int step;
    kart_chase_camera_follow_reset(state);
    pose = kart_chase_camera_update(state, origin, level, speed, wide_view, 0,
                                    KART_CHASE_FOLLOW_OVERHEAD_MS);
    for (step = 0; step < seconds * 62; ++step) {
        pose = kart_chase_camera_update(state, origin, level, speed, wide_view,
                                        16, KART_CHASE_FOLLOW_OVERHEAD_MS);
    }
    return pose;
}

static void test_speed_filter_is_asymmetric(void)
{
    /* 10 s rising, 100 ms falling: one second of acceleration moves the
       filtered speed only a little, but one second of deceleration is done. */
    KartChaseCameraFollow state;
    const KartQuat level = {.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};
    const KartVec3 origin = {0.0f, 0.0f, 0.0f};
    int step;

    kart_chase_camera_follow_reset(&state);
    kart_chase_camera_update(&state, origin, level, 0.0f, false, 0,
                             KART_CHASE_FOLLOW_OVERHEAD_MS);
    for (step = 0; step < 62; ++step) {
        kart_chase_camera_update(&state, origin, level, 50.0f, false, 16,
                                 KART_CHASE_FOLLOW_OVERHEAD_MS);
    }
    assert(state.filtered_speed > 0.0f);
    assert(state.filtered_speed < 10.0f);

    for (step = 0; step < 62; ++step) {
        kart_chase_camera_update(&state, origin, level, 0.0f, false, 16,
                                 KART_CHASE_FOLLOW_OVERHEAD_MS);
    }
    assert(state.filtered_speed < 0.01f);
}

static void test_geometry_grows_with_filtered_speed(void)
{
    /* distance = 5.5 + f*0.045, height = f/60 + 3, pitch = f/400 + 0.25.
       Compared at the same filtered speed by settling for long enough. */
    KartChaseCameraFollow slow;
    KartChaseCameraFollow fast;
    const KartChaseCameraPose a = settle(&slow, 0.0f, false, 2);
    const KartChaseCameraPose b = settle(&fast, 60.0f, false, 60);

    /* At rest: exactly the base distance and height behind and above. */
    assert(near(a.position.z, KART_CHASE_HEIGHT_BASE, 0.01f));
    assert(near(a.position.y, KART_CHASE_DISTANCE_BASE, 0.01f));

    assert(b.position.y > a.position.y);
    assert(b.position.z > a.position.z);
    /* Faster means a steeper downward tilt, so forward's Z is more negative. */
    assert(b.forward.z < a.forward.z);

    {
        const float f = fast.filtered_speed;
        assert(near(b.position.y,
                    KART_CHASE_DISTANCE_BASE +
                        f * (KART_CHASE_DISTANCE_TERM_A + KART_CHASE_DISTANCE_TERM_B),
                    0.05f));
        assert(near(b.position.z, f / KART_CHASE_HEIGHT_DIVISOR + KART_CHASE_HEIGHT_BASE,
                    0.05f));
    }
}

static void test_camera_basis_stays_orthonormal_and_right_handed(void)
{
    KartChaseCameraFollow state;
    const KartChaseCameraPose pose = settle(&state, 40.0f, false, 30);
    const KartVec3 r = pose.right;
    const KartVec3 u = pose.up;
    const KartVec3 f = pose.forward;
    const KartVec3 cross = {
        r.y * u.z - r.z * u.y, r.z * u.x - r.x * u.z, r.x * u.y - r.y * u.x};
    assert(near(r.x * r.x + r.y * r.y + r.z * r.z, 1.0f, 1e-4f));
    assert(near(u.x * u.x + u.y * u.y + u.z * u.z, 1.0f, 1e-4f));
    assert(near(f.x * f.x + f.y * f.y + f.z * f.z, 1.0f, 1e-4f));
    assert(near(r.x * u.x + r.y * u.y + r.z * u.z, 0.0f, 1e-4f));
    assert(near(r.x * f.x + r.y * f.y + r.z * f.z, 0.0f, 1e-4f));
    /* right x up == forward, the handedness the projection assumes. */
    assert(near(cross.x, f.x, 1e-4f));
    assert(near(cross.y, f.y, 1e-4f));
    assert(near(cross.z, f.z, 1e-4f));
}

static void test_field_of_view_tracks_the_boost_flag(void)
{
    KartChaseCameraFollow narrow;
    KartChaseCameraFollow wide;
    const KartChaseCameraPose a = settle(&narrow, 0.0f, false, 6);
    const KartChaseCameraPose b = settle(&wide, 0.0f, true, 6);
    assert(near(a.field_of_view_degrees, KART_CHASE_FOV_NARROW_DEGREES, 0.5f));
    assert(near(b.field_of_view_degrees, KART_CHASE_FOV_WIDE_DEGREES, 0.5f));
    /* Wider view means a shorter focal length. */
    assert(kart_chase_camera_focal_length(b.field_of_view_degrees, 800.0f) <
           kart_chase_camera_focal_length(a.field_of_view_degrees, 800.0f));
    /* 0x004A98B1 takes tan(fov/2); 90 degrees must give exactly half height. */
    assert(near(kart_chase_camera_focal_length(90.0f, 800.0f), 400.0f, 0.5f));
}

static void test_position_z_is_smoothed_but_xy_is_not(void)
{
    /* Only the Z component gets the 100 ms filter; X and Y are copied. */
    KartChaseCameraFollow state;
    const KartQuat level = {.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};
    KartChaseCameraPose pose;
    kart_chase_camera_follow_reset(&state);
    kart_chase_camera_update(&state, (KartVec3){0.0f, 0.0f, 0.0f}, level, 0.0f,
                             false, 0, KART_CHASE_FOLLOW_OVERHEAD_MS);
    pose = kart_chase_camera_update(&state, (KartVec3){0.0f, 0.0f, 100.0f},
                                    level, 0.0f, false, 16,
                                    KART_CHASE_FOLLOW_OVERHEAD_MS);
    /* A 100 unit jump in Z is followed only partway... */
    assert(pose.position.z > KART_CHASE_HEIGHT_BASE);
    assert(pose.position.z < 100.0f);
    /* ...while the same jump in X lands immediately. */
    kart_chase_camera_follow_reset(&state);
    kart_chase_camera_update(&state, (KartVec3){0.0f, 0.0f, 0.0f}, level, 0.0f,
                             false, 0, KART_CHASE_FOLLOW_OVERHEAD_MS);
    pose = kart_chase_camera_update(&state, (KartVec3){100.0f, 0.0f, 0.0f},
                                    level, 0.0f, false, 16,
                                    KART_CHASE_FOLLOW_OVERHEAD_MS);
    assert(near(pose.position.x, 100.0f, 1e-3f));
}

int main(void)
{
    test_slerp_weight_matches_recovered_polynomial();
    test_interpolation_endpoints_and_normalization();
    test_first_update_snaps();
    test_follow_lags_then_converges();
    test_front_mode_is_faster_than_overhead();
    test_long_stall_snaps();
    test_hemisphere_correction_takes_the_short_arc();
    test_speed_filter_is_asymmetric();
    test_geometry_grows_with_filtered_speed();
    test_camera_basis_stays_orthonormal_and_right_handed();
    test_field_of_view_tracks_the_boost_flag();
    test_position_z_is_smoothed_but_xy_is_not();
    printf("camera follow tests passed\n");
    return 0;
}
