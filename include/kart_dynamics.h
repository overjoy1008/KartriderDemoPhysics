#ifndef KART_DYNAMICS_H
#define KART_DYNAMICS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct KartDynamicsConfig {
    float mass;
    float air_friction;
    float drag_factor;
    float forward_accel_force;
    float backward_accel_force;
    float grip_brake_force;
    float slip_brake_force;
    float max_steer_angle_deg;
    float steer_constraint;
    float front_grip_factor;
    float rear_grip_factor;
    float drift_trigger_factor;
    float drift_trigger_time;
    float drift_slip_factor;
    float drift_escape_force;
    float corner_draw_factor;
    float drift_lean_factor;
    float steer_lean_factor;
} KartDynamicsConfig;

typedef struct KartVec3 {
    float x;
    float y;
    float z;
} KartVec3;

typedef struct KartMat3 {
    float m[3][3];
} KartMat3;

typedef struct KartQuat {
    float w;
    float x;
    float y;
    float z;
} KartQuat;

typedef struct KartPoseInput {
    KartVec3 position;
    KartQuat orientation;
    KartVec3 linear_velocity;
    KartVec3 angular_velocity;
    float dt;
} KartPoseInput;

typedef struct KartPoseOutput {
    KartVec3 position;
    KartQuat orientation;
    KartVec3 angular_velocity;
    float up_z;
    unsigned int tilt_retries;
    bool tilt_clamped;
} KartPoseOutput;

typedef struct KartSuspensionContact {
    bool active;
    KartVec3 normal;
    float compression;
    float compression_delta;
} KartSuspensionContact;

typedef struct KartSuspensionInput {
    float dt;
    float half_width;
    float half_length;
    KartVec3 chassis_up;
    KartSuspensionContact contacts[4];
} KartSuspensionInput;

typedef struct KartSuspensionOutput {
    KartVec3 world_force;
    KartVec3 local_torque;
    float contact_force[4];
    unsigned int active_contacts;
} KartSuspensionOutput;

typedef struct KartDriftState {
    bool input_active;
    bool slip_detected;
    bool trigger_active;
    bool entry_was_forward;
    float trigger_timer;
    float linger_timer;
} KartDriftState;

typedef struct KartDragInput {
    KartVec3 linear_velocity;
    KartVec3 angular_velocity;
    bool grounded;
    float grounded_drag_scale;
} KartDragInput;

typedef struct KartDragOutput {
    KartVec3 force;
    KartVec3 torque;
} KartDragOutput;

typedef struct KartCollisionInput {
    KartVec3 velocity;
    KartVec3 angular_velocity;
    KartVec3 normal;
    KartVec3 body_right;
    KartVec3 body_forward;
    KartVec3 body_up;
    /* Carried in the original's contact record and passed through for
       completeness; the response branch does not read it. */
    float sweep_fraction;
} KartCollisionInput;

typedef struct KartCollisionOutput {
    KartVec3 velocity;
    KartVec3 angular_velocity;
    bool incoming;
    /* True when the contact took the wall branch, i.e. normal.z <= 0.65. */
    bool wall_contact;
    float normal_speed;
    /* Wall branch only. */
    float tangential_speed_removed;
    float wall_yaw_kick;
} KartCollisionOutput;

typedef enum KartLongitudinalMode {
    KART_LONGITUDINAL_IDLE = 0,
    KART_LONGITUDINAL_FORWARD = 1,
    KART_LONGITUDINAL_BRAKE = 2,
    KART_LONGITUDINAL_REVERSE = 3,
    KART_LONGITUDINAL_STOPPED = 4
} KartLongitudinalMode;

typedef struct KartLongitudinalState {
    float reverse_timer;
} KartLongitudinalState;

typedef struct KartInstantBoostState {
    float opportunity_timer;
    float active_timer;
    bool active;
} KartInstantBoostState;

typedef struct KartTimedBoostState {
    unsigned int remaining_ms;
    bool active;
} KartTimedBoostState;

typedef struct KartLongitudinalInput {
    KartVec3 velocity;
    KartVec3 forward_axis;
    float forward_velocity;
    float lateral_velocity;
    float dt;
    float forward_input;
    float reverse_input;
    bool drive_disabled;
    bool drift_input_active;
    bool drift_slip_detected;
    bool boost_active;
} KartLongitudinalInput;

typedef struct KartLongitudinalOutput {
    KartVec3 force;
    KartVec3 velocity;
    KartLongitudinalMode mode;
    bool velocity_overridden;
} KartLongitudinalOutput;

void kart_instant_boost_step_timers(
    KartInstantBoostState *state,
    float dt);

void kart_instant_boost_press_forward(KartInstantBoostState *state);

void kart_instant_boost_update_drift_exit(
    KartInstantBoostState *boost,
    KartDriftState *drift,
    bool was_drifting);

bool kart_timed_boost_start(
    KartTimedBoostState *state,
    float forward_input,
    unsigned int duration_ms);

void kart_timed_boost_step_milliseconds(
    KartTimedBoostState *state,
    unsigned int elapsed_ms);

bool kart_any_boost_active(
    const KartTimedBoostState *timed,
    const KartInstantBoostState *instant);

typedef enum KartLateralMode {
    KART_LATERAL_GRIP = 0,
    KART_LATERAL_DRIFT = 1,
    KART_LATERAL_DRIFT_TRIGGER = 2
} KartLateralMode;

typedef struct KartLateralInput {
    float forward_velocity;
    float lateral_velocity;
    float yaw_lever_velocity;
    float steering_input;
    float forward_input;
    float previous_steer_angle_rad;
    bool reverse_steering;
    bool drift_input_active;
    KartLateralMode mode;
} KartLateralInput;

typedef struct KartLateralOutput {
    float speed;
    float steer_angle_rad;
    float next_previous_steer_angle_rad;
    float front_slip;
    float rear_slip;
    float front_force;
    float rear_force;
    float local_lateral_force;
    float local_forward_force;
    float local_roll_torque;
    float local_yaw_torque;
} KartLateralOutput;

KartDynamicsConfig kart_dynamics_default_config(void);

float kart_speed_kmh(KartVec3 linear_velocity);

int kart_speedometer_kmh(KartVec3 linear_velocity);

float kart_steer_angle_rad(
    const KartDynamicsConfig *config,
    float forward_velocity,
    float steering_input,
    bool reverse_steering);

KartLateralOutput kart_compute_lateral_response(
    const KartDynamicsConfig *config,
    const KartLateralInput *input);

KartSuspensionOutput kart_compute_suspension_response(
    const KartDynamicsConfig *config,
    const KartSuspensionInput *input);

void kart_drift_set_input(
    KartDriftState *state,
    bool pressed,
    float forward_velocity);

void kart_drift_step_trigger(
    KartDriftState *state,
    const KartDynamicsConfig *config,
    float dt);

void kart_drift_clear_for_low_speed(KartDriftState *state);

void kart_drift_update_slip_detection(
    KartDriftState *state,
    float speed,
    float forward_velocity,
    float lateral_velocity);

void kart_drift_step_linger(KartDriftState *state, float dt);

KartDragOutput kart_compute_drag_response(
    const KartDynamicsConfig *config,
    const KartDragInput *input);

KartVec3 kart_integrate_linear_velocity(
    KartVec3 velocity,
    KartVec3 accumulated_force,
    float mass,
    float dt);

KartMat3 kart_default_inverse_inertia(float mass);

KartVec3 kart_integrate_angular_velocity(
    KartVec3 angular_velocity,
    KartVec3 accumulated_torque,
    KartMat3 inverse_inertia,
    float dt);

KartPoseOutput kart_integrate_pose(const KartPoseInput *input);

KartCollisionOutput kart_resolve_linear_collision(
    const KartCollisionInput *input);

KartVec3 kart_compute_forward_drive_force(
    const KartDynamicsConfig *config,
    KartVec3 forward_axis,
    float input_amount,
    bool drift_slip_detected,
    bool boost_active);

KartVec3 kart_compute_reverse_drive_force(
    const KartDynamicsConfig *config,
    KartVec3 forward_axis,
    float input_amount);

KartVec3 kart_compute_directional_brake_force(
    const KartDynamicsConfig *config,
    KartVec3 velocity,
    KartVec3 forward_axis);

KartLongitudinalOutput kart_step_longitudinal(
    const KartDynamicsConfig *config,
    KartLongitudinalState *state,
    const KartLongitudinalInput *input);

#ifdef __cplusplus
}
#endif

#endif
