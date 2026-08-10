#include "kart_dynamics.h"

#include <limits.h>
#include <math.h>

/* Constants read from KartRider.exe .rdata at 0x00571d20-0x00571d38. */
static const float KART_PI = 3.1415927410125732f;
static const float KART_DEGREES_PER_HALF_TURN = 180.0f;
static const float KART_GRAVITY = 9.800000190734863f;
static const float KART_LOW_SPEED_DENOMINATOR = 5.0f;
static const float KART_STEER_ACTIVE_SPEED = 0.5f;
static const float KART_LEAN_SPEED_THRESHOLD = 10.0f;
static const float KART_WORLD_GRAVITY = -58.79999923706055f;
static const float KART_SUSPENSION_REBOUND_RATIO = 0.20000000298023224f;
static const float KART_SUSPENSION_TORQUE_SCALE = 0.1f;

static KartVec3 vec3_add(KartVec3 a, KartVec3 b)
{
    const KartVec3 result = {a.x + b.x, a.y + b.y, a.z + b.z};
    return result;
}

static KartVec3 vec3_scale(KartVec3 value, float scale)
{
    const KartVec3 result = {value.x * scale, value.y * scale, value.z * scale};
    return result;
}

static float vec3_dot(KartVec3 a, KartVec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static KartVec3 vec3_cross(KartVec3 a, KartVec3 b)
{
    const KartVec3 result = {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
    return result;
}

static KartVec3 vec3_normalize(KartVec3 value)
{
    const float length = sqrtf(vec3_dot(value, value));
    if (length <= 0.0f) {
        const KartVec3 zero = {0};
        return zero;
    }
    return vec3_scale(value, 1.0f / length);
}

static KartVec3 mat3_multiply_vec3(KartMat3 matrix, KartVec3 value)
{
    const KartVec3 result = {
        matrix.m[0][0] * value.x + matrix.m[0][1] * value.y + matrix.m[0][2] * value.z,
        matrix.m[1][0] * value.x + matrix.m[1][1] * value.y + matrix.m[1][2] * value.z,
        matrix.m[2][0] * value.x + matrix.m[2][1] * value.y + matrix.m[2][2] * value.z,
    };
    return result;
}

static KartQuat quat_integrate_local(KartQuat q, KartVec3 omega, float dt)
{
    KartQuat result;
    float length;
    const float half_dt = dt * 0.5f;

    result.w = q.w + (-q.x * omega.x - q.y * omega.y - q.z * omega.z) * half_dt;
    result.x = q.x + (q.y * omega.z + q.w * omega.x - q.z * omega.y) * half_dt;
    result.y = q.y + (q.z * omega.x + q.w * omega.y - q.x * omega.z) * half_dt;
    result.z = q.z + (q.x * omega.y + q.w * omega.z - q.y * omega.x) * half_dt;

    length = sqrtf(
        result.w * result.w + result.x * result.x +
        result.y * result.y + result.z * result.z);
    if (length > 0.0f) {
        result.w /= length;
        result.x /= length;
        result.y /= length;
        result.z /= length;
    }
    return result;
}

static float quat_up_z(KartQuat q)
{
    return 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
}

KartDynamicsConfig kart_dynamics_default_config(void)
{
    /*
     * Recovered from KartRider.exe function 0x0042e190.
     * The original function loads these keys from the "Dynamics" section and
     * uses the values below when a key is absent.
     */
    const KartDynamicsConfig result = {
        .mass = 100.0f,
        .air_friction = 3.0f,
        .drag_factor = 0.5f,
        .forward_accel_force = 3000.0f,
        .backward_accel_force = 2000.0f,
        .grip_brake_force = 2000.0f,
        .slip_brake_force = 1500.0f,
        .max_steer_angle_deg = 10.0f,
        .steer_constraint = 30.0f,
        .front_grip_factor = 5.0f,
        .rear_grip_factor = 5.0f,
        .drift_trigger_factor = 0.05f,
        .drift_trigger_time = 0.1f,
        .drift_slip_factor = 0.2f,
        .drift_escape_force = 5000.0f,
        .corner_draw_factor = 0.0f,
        .drift_lean_factor = 0.07f,
        .steer_lean_factor = 0.01f,
        .jump_spring_million_per_m = 1.2f,
        .jump_max_crouch_distance = 0.18f,
        .jump_gauge_sweep_time = 0.75f,
        .jump_push_duration = 0.09f,
        .jump_min_efficiency = 0.20f,
        .jump_max_efficiency = 1.0f,
        .jump_velocity_direction_bias = 0.12f,
        .jump_body_up_blend = 0.25f,
        .jump_torque_scale = 0.02f,
        .jump_max_slope_deg = 45.0f,
        .jump_landing_cooldown = 0.12f,
        .jump_landing_damping = 1200.0f,
    };
    return result;
}

float kart_speed_kmh(KartVec3 linear_velocity)
{
    /* HUD path 0x00450ABD-0x00450AC6: |linear velocity| * 3.6f. */
    const float speed = sqrtf(vec3_dot(linear_velocity, linear_velocity));
    return speed * 3.5999999046325684f;
}

int kart_speedometer_kmh(KartVec3 linear_velocity)
{
    /* The original sends the float value to the gauge and its x87 integer
       conversion to the L"%03d" numeric display. */
    return (int)lrintf(kart_speed_kmh(linear_velocity));
}

float kart_steer_angle_rad(
    const KartDynamicsConfig *config,
    float forward_velocity,
    float steering_input,
    bool reverse_steering)
{
    /* Recovered from 0x0042fcde-0x0042fd42. */
    const float direction = reverse_steering ? -1.0f : 1.0f;
    const float maximum =
        (KART_PI * config->max_steer_angle_deg) /
        KART_DEGREES_PER_HALF_TURN;
    const float attenuation =
        expf(-fabsf(forward_velocity / config->steer_constraint));

    return maximum * direction * steering_input * attenuation;
}

KartLateralOutput kart_compute_lateral_response(
    const KartDynamicsConfig *config,
    const KartLateralInput *input)
{
    KartLateralOutput out = {0};
    const float travel_direction = input->forward_velocity <= 0.0f ? -1.0f : 1.0f;
    const float steering_direction = input->reverse_steering ? -1.0f : 1.0f;
    const float maximum_steer =
        (KART_PI * config->max_steer_angle_deg) /
        KART_DEGREES_PER_HALF_TURN * steering_direction * input->steering_input;
    float filtered_steer = kart_steer_angle_rad(
        config,
        input->forward_velocity,
        input->steering_input,
        input->reverse_steering);
    float denominator;
    float steer_for_slip;

    out.speed = sqrtf(
        input->forward_velocity * input->forward_velocity +
        input->lateral_velocity * input->lateral_velocity);
    if (input->forward_input != 0.0f &&
        ((input->previous_steer_angle_rad > 0.0f && filtered_steer > 0.0f) ||
         (input->previous_steer_angle_rad < 0.0f && filtered_steer < 0.0f)) &&
        fabsf(input->previous_steer_angle_rad) < fabsf(filtered_steer)) {
        filtered_steer = input->previous_steer_angle_rad;
        out.next_previous_steer_angle_rad = input->previous_steer_angle_rad;
    } else {
        out.next_previous_steer_angle_rad = filtered_steer;
    }
    out.steer_angle_rad = filtered_steer;

    denominator = out.speed <= KART_LOW_SPEED_DENOMINATOR
        ? KART_LOW_SPEED_DENOMINATOR
        : out.speed;
    steer_for_slip = out.speed >= KART_STEER_ACTIVE_SPEED
        ? travel_direction *
            ((out.speed > KART_LOW_SPEED_DENOMINATOR &&
              input->mode == KART_LATERAL_DRIFT && input->drift_input_active)
                ? maximum_steer
                : filtered_steer)
        : 0.0f;

    /* Recovered from 0x0042fe1c-0x00430588. */
    out.front_slip =
        steer_for_slip -
        input->lateral_velocity / denominator -
        (input->yaw_lever_velocity * 0.5f) / denominator;
    out.rear_slip =
        -input->lateral_velocity / denominator +
        (input->yaw_lever_velocity * 0.5f) / denominator;

    /* 0x0042FEF8 enters a dedicated <=5 branch before any drift mode branch.
       It clears drift state in the owner and computes ordinary tire forces
       with the fixed denominator above. */
    if (out.speed > KART_LOW_SPEED_DENOMINATOR &&
        input->mode == KART_LATERAL_DRIFT_TRIGGER) {
        out.front_force = 0.0f;
        out.rear_force =
            maximum_steer * config->drift_trigger_factor *
            -(KART_GRAVITY * config->mass) * config->front_grip_factor;
    } else {
        out.front_force =
            out.front_slip * KART_GRAVITY * config->mass *
            config->front_grip_factor;
        out.rear_force =
            out.rear_slip * KART_GRAVITY * config->mass *
            config->rear_grip_factor;

        if (out.speed > KART_LOW_SPEED_DENOMINATOR &&
            input->mode == KART_LATERAL_DRIFT) {
            out.front_force *= config->drift_slip_factor;
            out.rear_force *= config->drift_slip_factor;
        }
    }

    out.local_lateral_force = out.front_force + out.rear_force;
    out.local_yaw_torque = 0.5f * out.front_force - 0.5f * out.rear_force;

    /* 0x00430311-0x0043033A stores -abs(lateral force)*factor in the
       executable's local Y axis. Body forward is -local-Y, so expose the
       equivalent positive body-forward force here. */
    if (out.speed > KART_LOW_SPEED_DENOMINATOR &&
        input->mode == KART_LATERAL_GRIP) {
        out.local_forward_force =
            fabsf(out.local_lateral_force) * config->corner_draw_factor;
    }

    if (out.speed > KART_LOW_SPEED_DENOMINATOR &&
        input->mode == KART_LATERAL_DRIFT) {
        const float low_speed_scale =
            out.speed <= KART_LEAN_SPEED_THRESHOLD ? 0.5f : 1.0f;
        out.local_roll_torque =
            -out.local_lateral_force * config->drift_lean_factor * low_speed_scale;
    } else if (input->mode == KART_LATERAL_GRIP &&
               out.speed > KART_LOW_SPEED_DENOMINATOR) {
        out.local_roll_torque =
            -out.local_lateral_force * config->steer_lean_factor;
    }

    return out;
}

KartSuspensionOutput kart_compute_suspension_response(
    const KartDynamicsConfig *config,
    const KartSuspensionInput *input)
{
    /*
     * Recovered from 0x0042f460. The contact generation that precedes this
     * calculation is at 0x0042ef90.
     */
    static const float corner_x[4] = {1.0f, -1.0f, 1.0f, -1.0f};
    static const float corner_y[4] = {1.0f, 1.0f, -1.0f, -1.0f};
    KartSuspensionOutput out = {0};
    const float static_force =
        fabsf(KART_WORLD_GRAVITY) * config->mass * 0.5f;
    const float compression_damping = fmaxf(input->compression_damping, 0.0f);
    const float rebound_damping =
        static_force * KART_SUSPENSION_REBOUND_RATIO;
    unsigned int i;

    out.world_force.z = KART_WORLD_GRAVITY * config->mass;

    for (i = 0; i < 4; ++i) {
        const KartSuspensionContact *contact = &input->contacts[i];
        float force;
        float damping;
        KartVec3 lever;
        KartVec3 local_force;
        KartVec3 torque;

        if (!contact->active || input->dt <= 0.0f) {
            continue;
        }

        damping = contact->compression_delta <= 0.0f
            ? rebound_damping
            : compression_damping;
        force =
            (contact->compression_delta / input->dt) * damping +
            static_force * contact->compression;

        if (force <= 0.0f) {
            continue;
        }

        force *= vec3_dot(contact->normal, input->chassis_up);
        out.contact_force[i] = force;
        out.active_contacts += 1;
        out.world_force = vec3_add(
            out.world_force,
            vec3_scale(input->chassis_up, force));

        lever.x = input->half_width * corner_x[i];
        lever.y = -input->half_length * corner_y[i];
        lever.z = 0.0f;
        local_force.x = 0.0f;
        local_force.y = 0.0f;
        local_force.z = force;
        torque = vec3_scale(
            vec3_cross(lever, local_force),
            KART_SUSPENSION_TORQUE_SCALE);
        out.local_torque = vec3_add(out.local_torque, torque);
    }

    return out;
}

void kart_drift_set_input(
    KartDriftState *state,
    bool pressed,
    float forward_velocity)
{
    /* Recovered from 0x00431a30. */
    if (!pressed) {
        state->input_active = false;
        return;
    }

    if (state->linger_timer <= 0.0f) {
        state->input_active = true;
        state->trigger_active = true;
        state->entry_was_forward = forward_velocity > 0.0f;
    }
}

void kart_drift_step_trigger(
    KartDriftState *state,
    const KartDynamicsConfig *config,
    float dt)
{
    /* Recovered from 0x0042fff0-0x00430076. */
    if (!state->trigger_active) {
        return;
    }

    if (state->trigger_timer <= 0.0f) {
        state->trigger_timer = config->drift_trigger_time;
        state->linger_timer = config->drift_trigger_time * 2.0f;
        return;
    }

    state->trigger_timer -= dt;
    if (state->trigger_timer <= 0.0f) {
        state->trigger_timer = 0.0f;
        state->trigger_active = false;
    }
}

void kart_drift_clear_for_low_speed(KartDriftState *state)
{
    /* Recovered from 0x0042fef8-0x0042ff0c. */
    state->trigger_active = false;
    state->input_active = false;
    state->slip_detected = false;
}

void kart_drift_update_slip_detection(
    KartDriftState *state,
    float speed,
    float forward_velocity,
    float lateral_velocity)
{
    /* Recovered from 0x0042fef8-0x0042ff58. */
    static const float slip_ratio = 1.2000000476837158f;

    state->slip_detected = false;
    if (speed <= KART_LOW_SPEED_DENOMINATOR) {
        kart_drift_clear_for_low_speed(state);
        return;
    }

    if (!state->input_active && !state->trigger_active) {
        state->slip_detected =
            fabsf(lateral_velocity) > fabsf(forward_velocity) * slip_ratio;
    }
}

void kart_drift_step_linger(KartDriftState *state, float dt)
{
    /* Recovered from 0x00430244-0x0043026e. */
    state->linger_timer = fmaxf(state->linger_timer - dt, 0.0f);
}

KartDragOutput kart_compute_drag_response(
    const KartDynamicsConfig *config,
    const KartDragInput *input)
{
    /* Recovered from 0x00430640. */
    KartDragOutput out;
    const float speed = sqrtf(vec3_dot(input->linear_velocity, input->linear_velocity));

    out.force = vec3_scale(input->linear_velocity, -config->air_friction);
    out.torque = vec3_scale(input->angular_velocity, -config->air_friction);

    if (input->grounded) {
        const KartVec3 quadratic_drag = vec3_scale(
            input->linear_velocity,
            -(speed * config->drag_factor * input->grounded_drag_scale));
        out.force = vec3_add(out.force, quadratic_drag);
    }
    return out;
}

KartVec3 kart_integrate_linear_velocity(
    KartVec3 velocity,
    KartVec3 accumulated_force,
    float mass,
    float dt)
{
    /* Linear half of 0x00430740: v += (force / mass) * dt. */
    if (mass <= 0.0f || dt <= 0.0f) {
        return velocity;
    }
    return vec3_add(velocity, vec3_scale(accumulated_force, dt / mass));
}

KartMat3 kart_default_inverse_inertia(float mass)
{
    /* Recovered from 0x0042e5d5-0x0042e623. */
    KartMat3 result = {{{0}}};
    if (mass > 0.0f) {
        const float diagonal = 12.0f / mass;
        result.m[0][0] = diagonal;
        result.m[1][1] = diagonal;
        result.m[2][2] = diagonal;
    }
    return result;
}

KartVec3 kart_integrate_angular_velocity(
    KartVec3 angular_velocity,
    KartVec3 accumulated_torque,
    KartMat3 inverse_inertia,
    float dt)
{
    /* Recovered from the angular half of 0x00430740. */
    const KartVec3 inertia_velocity =
        mat3_multiply_vec3(inverse_inertia, angular_velocity);
    const KartVec3 gyroscopic = vec3_cross(angular_velocity, inertia_velocity);
    const KartVec3 effective_torque =
        vec3_add(accumulated_torque, vec3_scale(gyroscopic, -1.0f));
    const KartVec3 angular_acceleration =
        mat3_multiply_vec3(inverse_inertia, effective_torque);

    if (dt <= 0.0f) {
        return angular_velocity;
    }
    return vec3_add(angular_velocity, vec3_scale(angular_acceleration, dt));
}

KartPoseOutput kart_integrate_pose(const KartPoseInput *input)
{
    /* Recovered from 0x00430ed0 and quaternion helper 0x0042da70. */
    static const float minimum_up_z = 0.5f;
    static const float retry_damping = 0.1f;
    KartPoseOutput out = {0};
    unsigned int retry;

    out.position = vec3_add(
        input->position,
        vec3_scale(input->linear_velocity, input->dt));
    out.angular_velocity = input->angular_velocity;
    out.orientation = quat_integrate_local(
        input->orientation, out.angular_velocity, input->dt);
    out.up_z = quat_up_z(out.orientation);

    for (retry = 0; retry < 3 && out.up_z < minimum_up_z; ++retry) {
        out.angular_velocity.x *= retry_damping;
        out.angular_velocity.y *= retry_damping;
        out.orientation = quat_integrate_local(
            input->orientation, out.angular_velocity, input->dt);
        out.up_z = quat_up_z(out.orientation);
        out.tilt_retries += 1;
    }

    if (out.up_z < minimum_up_z) {
        out.angular_velocity.x = 0.0f;
        out.angular_velocity.y = 0.0f;
        out.orientation = quat_integrate_local(
            input->orientation, out.angular_velocity, input->dt);
        out.up_z = quat_up_z(out.orientation);
        out.tilt_clamped = true;
    }
    return out;
}

KartCollisionOutput kart_resolve_linear_collision(
    const KartCollisionInput *input)
{
    /* Linear-velocity portion of 0x00430830.

       The branch is on the contact normal's Z, not on the sweep fraction:
         00430A3B  FLD  float ptr [EBP + -0x50]     ; contact normal .z
         00430A3E  FCOMP float ptr [0x00571D48]     ; 0.65f
       0x00426C20 initializes the contact record as two 12-byte vectors, so the
       local at record+0xC+8 is normal.z while the sweep fraction is the next
       slot. A steep face takes the wall branch; a shallow one takes the
       ground branch. */
    static const float wall_normal_z_limit = 0.6499999761581421f;
    static const float hard_normal_impulse = 1.5f;
    static const float hard_tangent_limit = 0.6000000238418579f;
    static const float soft_restitution = 0.20000000298023224f;
    KartCollisionOutput out = {0};
    const float signed_normal_speed = vec3_dot(input->normal, input->velocity);
    KartVec3 normal_velocity;
    KartVec3 tangent_velocity;

    out.velocity = input->velocity;
    out.angular_velocity = input->angular_velocity;
    if (signed_normal_speed >= 0.0f) {
        return out;
    }

    out.incoming = true;
    out.normal_speed = -signed_normal_speed;
    normal_velocity = vec3_scale(input->normal, signed_normal_speed);
    tangent_velocity = vec3_add(input->velocity, vec3_scale(normal_velocity, -1.0f));

    if (input->normal.z <= wall_normal_z_limit) {
        const float tangent_speed = sqrtf(vec3_dot(tangent_velocity, tangent_velocity));
        const float normal_forward = vec3_dot(input->normal, input->body_forward);
        const float normal_right = vec3_dot(input->normal, input->body_right);
        const float turn_speed = fminf(fmaxf(out.normal_speed, 1.0f), 30.0f);
        KartVec3 tangent_direction = {0};
        KartVec3 correction;

        out.wall_contact = true;
        out.tangential_speed_removed = fminf(
            out.normal_speed * hard_normal_impulse,
            tangent_speed * hard_tangent_limit);
        if (tangent_speed > 0.0f) {
            tangent_direction = vec3_scale(tangent_velocity, 1.0f / tangent_speed);
        }

        correction = vec3_add(
            vec3_scale(normal_velocity, -hard_normal_impulse),
            vec3_scale(tangent_direction, -out.tangential_speed_removed));
        /* The original removes the vertical component of the hard correction. */
        correction.z = 0.0f;
        out.velocity = vec3_add(input->velocity, correction);

        if (fabsf(normal_forward) <= fabsf(normal_right)) {
            const float side_sign = normal_right <= 0.0f ? 1.0f : -1.0f;
            out.wall_yaw_kick = normal_forward * side_sign * turn_speed;
        } else {
            const float forward_sign = normal_forward <= 0.0f ? -1.0f : 1.0f;
            out.wall_yaw_kick = normal_right * forward_sign * turn_speed;
        }

        /* The candidate is accepted unless existing same-direction spin is strong. */
        if (out.wall_yaw_kick * out.angular_velocity.z <= 1.0f) {
            out.angular_velocity.z += out.wall_yaw_kick;
        } else {
            out.wall_yaw_kick = 0.0f;
        }
    } else {
        const KartVec3 wall_turn_axis = vec3_cross(input->normal, input->body_up);
        out.velocity = vec3_add(
            tangent_velocity,
            vec3_scale(normal_velocity, -soft_restitution));

        /* Late-contact rotational correction at 0x00430df2-0x00430e73. */
        out.angular_velocity.x -=
            vec3_dot(wall_turn_axis, input->body_right) * 0.1f;
        out.angular_velocity.y +=
            vec3_dot(wall_turn_axis, input->body_forward) * 0.1f;
    }

    return out;
}

KartVec3 kart_compute_forward_drive_force(
    const KartDynamicsConfig *config,
    KartVec3 forward_axis,
    float input_amount,
    bool drift_slip_detected,
    bool boost_active)
{
    /* Main forward branch of 0x0042f6c0. */
    const float base_force = drift_slip_detected
        ? config->drift_escape_force
        : config->forward_accel_force;
    const float boost = boost_active ? 1.5f : 1.0f;
    return vec3_scale(forward_axis, input_amount * base_force * boost);
}

void kart_instant_boost_step_timers(
    KartInstantBoostState *state,
    float dt)
{
    /* 0x0042eda0: the 0x2e0 opportunity window is clamped at zero. */
    if (state->opportunity_timer > 0.0f) {
        state->opportunity_timer =
            fmaxf(state->opportunity_timer - dt, 0.0f);
    }

    /* 0x0042eda0: 0x2d4 remains active while the 0x2d8 timer is positive. */
    if (state->active) {
        state->active_timer -= dt;
        if (state->active_timer <= 0.0f) {
            state->active_timer = 0.0f;
            state->active = false;
        }
    }
}

void kart_instant_boost_press_forward(KartInstantBoostState *state)
{
    /* GoKart::SetAccel(true), 0x00431960. */
    if (!state->stored_model && state->opportunity_timer > 0.0f) {
        state->opportunity_timer = 0.0f;
        state->active_timer = 0.5f;
        state->active = true;
        state->activation_count += 1;
    } else if (state->stored_model) {
        kart_instant_boost_use_stored(state);
    }
}

bool kart_instant_boost_use_stored(KartInstantBoostState *state)
{
    if (!state->stored_model || state->stored_count == 0) {
        return false;
    }
    state->stored_count -= 1;
    state->active_timer = 0.5f;
    state->active = true;
    state->activation_count += 1;
    return true;
}

void kart_instant_boost_update_drift_exit(
    KartInstantBoostState *boost,
    KartDriftState *drift,
    bool was_drifting)
{
    /* Tail of 0x0042fc40. A forward drift ending without remaining manual or
       automatic slip opens a one-shot 0.5 second accelerator-input window. */
    if (drift->entry_was_forward && was_drifting &&
        !drift->input_active && !drift->slip_detected) {
        drift->entry_was_forward = false;
        if (boost->stored_model) {
            if (boost->stored_count != UINT_MAX) {
                boost->stored_count += 1;
            }
        } else if (boost->opportunity_timer == 0.0f) {
            boost->opportunity_timer = 0.5f;
        }
    }
}

bool kart_timed_boost_start(
    KartTimedBoostState *state,
    float forward_input,
    unsigned int duration_ms)
{
    /* GoKart::StartTimedBoost at 0x00431ab0. */
    if (forward_input != 0.0f) {
        state->remaining_ms = duration_ms;
        state->active = true;
    }
    return state->active;
}

void kart_timed_boost_step_milliseconds(
    KartTimedBoostState *state,
    unsigned int elapsed_ms)
{
    /* Tail of 0x0042e750: subtract min(remaining, frame milliseconds). */
    if (state->remaining_ms != 0) {
        const unsigned int consumed =
            state->remaining_ms < elapsed_ms ? state->remaining_ms : elapsed_ms;
        state->remaining_ms -= consumed;
        if (state->remaining_ms == 0) {
            state->active = false;
        }
    }
}

bool kart_any_boost_active(
    const KartTimedBoostState *timed,
    const KartInstantBoostState *instant)
{
    /* GoKart::IsBoosting at 0x00431b00. */
    return timed->active || instant->active;
}

KartVec3 kart_compute_reverse_drive_force(
    const KartDynamicsConfig *config,
    KartVec3 forward_axis,
    float input_amount)
{
    /* Reverse-acceleration branch at 0x0042fa98-0x0042fadb. */
    return vec3_scale(forward_axis, -input_amount * config->backward_accel_force);
}

KartVec3 kart_compute_directional_brake_force(
    const KartDynamicsConfig *config,
    KartVec3 velocity,
    KartVec3 forward_axis)
{
    /* Brake selection at 0x0042fb6f-0x0042fbcd. */
    static const float grip_alignment_threshold = 0.800000011920929f;
    const KartVec3 direction = vec3_normalize(velocity);
    const float alignment = vec3_dot(direction, forward_axis);
    const float magnitude = alignment <= grip_alignment_threshold
        ? config->slip_brake_force
        : config->grip_brake_force;
    return vec3_scale(direction, -magnitude);
}

KartLongitudinalOutput kart_step_longitudinal(
    const KartDynamicsConfig *config,
    KartLongitudinalState *state,
    const KartLongitudinalInput *input)
{
    /* Full input/timer state machine recovered from 0x0042f6c0. */
    static const float direction_threshold = 0.5f;
    static const float reverse_delay = 0.20000000298023224f;
    static const float lateral_stop_threshold = 0.20000000298023224f;
    KartLongitudinalOutput out = {0};
    bool should_brake = false;

    out.velocity = input->velocity;
    out.mode = KART_LONGITUDINAL_IDLE;

    if (input->forward_input != 0.0f && !input->drive_disabled) {
        const float speed = sqrtf(vec3_dot(input->velocity, input->velocity));
        out.force = kart_compute_forward_drive_force(
            config,
            input->forward_axis,
            input->forward_input,
            input->drift_slip_detected,
            input->boost_active);

        /* Extra recovery force opposes residual reverse travel. */
        if (input->forward_velocity < 0.0f) {
            const float recovery_speed =
                (input->drift_input_active || input->drift_slip_detected)
                    ? speed
                    : fminf(speed, KART_LOW_SPEED_DENOMINATOR);
            out.force = vec3_add(
                out.force,
                vec3_scale(
                    input->forward_axis,
                    recovery_speed * config->mass * KART_GRAVITY));
        }

        state->reverse_timer = 0.0f;
        out.mode = KART_LONGITUDINAL_FORWARD;
        return out;
    }

    if (input->reverse_input == 0.0f && !input->drive_disabled) {
        if (input->forward_velocity <= direction_threshold &&
            input->forward_velocity >= -direction_threshold) {
            state->reverse_timer += input->dt;
        }
        return out;
    }

    should_brake = true;
    if (input->forward_velocity <= direction_threshold) {
        state->reverse_timer += input->dt;
        if (input->forward_velocity <= -direction_threshold) {
            state->reverse_timer = 1.0f;
        }

        if (state->reverse_timer <= reverse_delay) {
            if (fabsf(input->lateral_velocity) <= lateral_stop_threshold) {
                out.velocity = (KartVec3){0};
                out.velocity_overridden = true;
                out.mode = KART_LONGITUDINAL_STOPPED;
                should_brake = false;
            }
        } else if (input->drive_disabled) {
            if (input->forward_velocity <= -direction_threshold) {
                out.force = vec3_scale(
                    vec3_normalize(input->velocity),
                    -config->grip_brake_force);
                out.mode = KART_LONGITUDINAL_BRAKE;
            } else {
                out.velocity = (KartVec3){0};
                out.velocity_overridden = true;
                out.mode = KART_LONGITUDINAL_STOPPED;
            }
            should_brake = false;
        } else {
            out.force = kart_compute_reverse_drive_force(
                config,
                input->forward_axis,
                input->reverse_input);
            out.mode = KART_LONGITUDINAL_REVERSE;
            should_brake = false;
        }
    }

    if (should_brake) {
        out.force = kart_compute_directional_brake_force(
            config, input->velocity, input->forward_axis);
        out.mode = KART_LONGITUDINAL_BRAKE;
    }
    return out;
}
