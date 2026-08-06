#include "kart_dynamics.h"

#include <stdio.h>

static float dot(KartVec3 a, KartVec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static void axes(KartQuat q, KartVec3 *right, KartVec3 *forward)
{
    const float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    const float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    const float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
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
}

int main(void)
{
    /* Original oracle-trajectory.csv immediately after frame 60: the trigger
       has ended, manual drift remains held, and one steer sign was used. */
    const KartVec3 velocity = {-0.0446146168f, -19.2336655f, 0.0f};
    const KartQuat orientation = {
        0.999973416f, 0.0f, 0.0f, 0.00732006598f,
    };
    const float yaw_rate = 0.264597356f;
    const KartDynamicsConfig config = kart_dynamics_default_config();
    KartVec3 right, forward;
    float forward_velocity, lateral_velocity;
    int steer;

    axes(orientation, &right, &forward);
    forward_velocity = dot(velocity, forward);
    lateral_velocity = dot(velocity, right);

    puts("steer,forward_velocity,lateral_velocity,steer_angle,front_slip,rear_slip,front_force,rear_force,lateral_force,yaw_torque");
    for (steer = -1; steer <= 1; ++steer) {
        const KartLateralInput input = {
            .forward_velocity = forward_velocity,
            .lateral_velocity = lateral_velocity,
            .yaw_lever_velocity = yaw_rate,
            .steering_input = (float)steer,
            .forward_input = 1.0f,
            .previous_steer_angle_rad = kart_steer_angle_rad(
                &config, forward_velocity, 1.0f, false),
            .drift_input_active = true,
            .mode = KART_LATERAL_DRIFT,
        };
        const KartLateralOutput out =
            kart_compute_lateral_response(&config, &input);
        printf(
            "%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
            steer, forward_velocity, lateral_velocity,
            out.steer_angle_rad, out.front_slip, out.rear_slip,
            out.front_force, out.rear_force,
            out.local_lateral_force, out.local_yaw_torque);
    }
    return 0;
}
