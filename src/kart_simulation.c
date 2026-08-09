#include "kart_simulation.h"

#include <math.h>
#include <string.h>

static const float WORLD_GRAVITY = -58.79999923706055f;
static const float FIXED_STEP_SECONDS_PER_MS = 0.0010000000474974513f;
static const unsigned int MAX_SUBSTEP_MS = 5;

static KartVec3 add(KartVec3 a, KartVec3 b)
{
    const KartVec3 value = {a.x + b.x, a.y + b.y, a.z + b.z};
    return value;
}

static KartVec3 scale(KartVec3 value, float amount)
{
    const KartVec3 result = {value.x * amount, value.y * amount, value.z * amount};
    return result;
}

static float dot(KartVec3 a, KartVec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
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
    /* The executable defines forward as the negated second matrix column. */
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

KartSimulationGeometry kart_simulation_default_geometry(void)
{
    /* Body dimensions are model inputs; suspension range defaults to 0.5. */
    const KartSimulationGeometry result = {1.0f, 1.0f, 0.5f, 1.0f};
    return result;
}

void kart_simulation_init(
    KartSimulationState *state,
    const KartDynamicsConfig *config,
    const KartSimulationGeometry *geometry)
{
    memset(state, 0, sizeof(*state));
    state->config = config != NULL ? *config : kart_dynamics_default_config();
    state->geometry = geometry != NULL ? *geometry : kart_simulation_default_geometry();
    state->grounded_drag_scale = state->geometry.grounded_drag_scale;
    state->orientation.w = 1.0f;
}

void kart_simulation_set_grounded_drag_scale(
    KartSimulationState *state,
    float scale)
{
    if (state != NULL) {
        state->grounded_drag_scale = scale;
    }
}

void kart_simulation_multiply_grounded_drag_scale(
    KartSimulationState *state,
    float multiplier)
{
    if (state != NULL) {
        /* Original trigger callbacks 0x00441A10/0x00441AA0 multiply the
           runtime field by 4.0 on entry and 0.25 on exit. */
        state->grounded_drag_scale *= multiplier;
    }
}

KartWheelQueryOutput kart_query_wheel_contacts(
    KartWheelContactState *state,
    const KartWheelQueryInput *input)
{
    static const float right_sign[KART_WHEEL_COUNT] = {1.0f, -1.0f, 1.0f, -1.0f};
    static const float forward_sign[KART_WHEEL_COUNT] = {1.0f, 1.0f, -1.0f, -1.0f};
    static const float inset = 0.800000011920929f;
    KartWheelQueryOutput out = {0};
    unsigned int i;

    for (i = 0; i < KART_WHEEL_COUNT; ++i) {
        const float old_compression = state->compression[i];
        KartVec3 start = input->position;
        KartVec3 delta = scale(input->body_up, -2.0f * input->geometry.suspension_range);
        KartGroundHit hit = {0};
        float compression = 0.0f;

        start = add(start, scale(
            input->body_right,
            input->geometry.half_width * right_sign[i] * inset));
        start = add(start, scale(
            input->body_forward,
            input->geometry.half_length * forward_sign[i] * inset));
        start = add(start, scale(input->body_up, input->geometry.suspension_range));

        if (input->query != NULL && input->query(input->user_data, start, delta, &hit)) {
            const float bottom_height =
                dot(input->position, input->body_up) - input->geometry.suspension_range;
            const float raw = dot(hit.point, input->body_up) - bottom_height;
            const float maximum = 2.0f * input->geometry.suspension_range;

            compression = fmaxf(0.0f, fminf(raw, maximum));
            out.contacts[i].active = true;
            out.contacts[i].normal = hit.normal;
            out.contacts[i].compression = compression;
            out.contacts[i].compression_delta = compression - old_compression;
            out.contact_points[i] = hit.point;
            out.average_normal = add(out.average_normal, hit.normal);
            out.surface_id = hit.surface_id;
            out.active_contacts += 1;
        }
        state->compression[i] = compression;
    }

    out.grounded = out.active_contacts != 0;
    out.landed_this_step = out.grounded && !state->grounded;
    if (out.active_contacts != 0) {
        out.average_normal = scale(out.average_normal, 1.0f / (float)out.active_contacts);
    }
    state->grounded = out.grounded;
    return out;
}

static void simulate_substep(
    KartSimulationState *state,
    const KartSimulationControls *controls,
    const KartSimulationWorld *world,
    float dt,
    KartSimulationStepResult *result)
{
    KartVec3 right;
    KartVec3 forward;
    KartVec3 up;
    KartVec3 force = {0};
    KartVec3 torque = {0};
    KartWheelQueryInput wheel_input;
    KartWheelQueryOutput wheel;
    KartDragInput drag_input;
    KartDragOutput drag;
    float forward_velocity;
    float lateral_velocity;
    float speed;
    bool was_drifting;

    {
        const bool forward_pressed = controls->forward_input != 0.0f;
        if (forward_pressed && !state->previous_forward_input) {
            kart_instant_boost_press_forward(&state->instant_boost);
        }
        state->previous_forward_input = forward_pressed;
        kart_instant_boost_step_timers(&state->instant_boost, dt);
    }

    orientation_axes(state->orientation, &right, &forward, &up);
    forward_velocity = dot(state->linear_velocity, forward);
    lateral_velocity = dot(state->linear_velocity, right);
    speed = sqrtf(dot(state->linear_velocity, state->linear_velocity));

    wheel_input = (KartWheelQueryInput){
        .position = state->position,
        .body_right = right,
        .body_forward = forward,
        .body_up = up,
        .geometry = state->geometry,
        .query = world != NULL ? world->query_ground : NULL,
        .user_data = world != NULL ? world->user_data : NULL,
    };
    wheel = kart_query_wheel_contacts(&state->wheels, &wheel_input);
    state->grounded = wheel.grounded;
    result->wheel_contacts += wheel.active_contacts;
    result->landed = result->landed || wheel.landed_this_step;

    if (wheel.grounded) {
        KartSuspensionInput suspension_input = {
            .dt = dt,
            .half_width = state->geometry.half_width,
            .half_length = state->geometry.half_length,
            .chassis_up = up,
        };
        KartSuspensionOutput suspension;
        KartLongitudinalInput longitudinal_input;
        KartLongitudinalOutput longitudinal;
        KartLateralInput lateral_input;
        KartLateralOutput lateral;
        KartLateralMode lateral_mode;
        unsigned int i;

        for (i = 0; i < KART_WHEEL_COUNT; ++i) {
            suspension_input.contacts[i] = wheel.contacts[i];
        }
        suspension = kart_compute_suspension_response(&state->config, &suspension_input);
        force = add(force, suspension.world_force);
        torque = add(torque, suspension.local_torque);

        longitudinal_input = (KartLongitudinalInput){
            .velocity = state->linear_velocity,
            .forward_axis = forward,
            .forward_velocity = forward_velocity,
            .lateral_velocity = lateral_velocity,
            .dt = dt,
            .forward_input = controls->forward_input,
            .reverse_input = controls->reverse_input,
            .drive_disabled = controls->drive_disabled,
            .drift_input_active = state->drift.input_active,
            .drift_slip_detected = state->drift.slip_detected,
            .boost_active = kart_any_boost_active(
                &state->timed_boost, &state->instant_boost),
        };
        longitudinal = kart_step_longitudinal(
            &state->config, &state->longitudinal, &longitudinal_input);
        if (longitudinal.velocity_overridden) {
            state->linear_velocity = longitudinal.velocity;
        }
        force = add(force, longitudinal.force);

    if (controls->drift_input != state->previous_drift_input) {
        kart_drift_set_input(
            &state->drift, controls->drift_input, forward_velocity);
        state->previous_drift_input = controls->drift_input;
    }
        was_drifting = state->drift.input_active || state->drift.slip_detected;
        kart_drift_update_slip_detection(
            &state->drift, speed, forward_velocity, lateral_velocity);
        if (state->drift.trigger_active) {
            lateral_mode = KART_LATERAL_DRIFT_TRIGGER;
            kart_drift_step_trigger(&state->drift, &state->config, dt);
        } else if (state->drift.input_active || state->drift.slip_detected ||
                   state->drift.linger_timer > 0.0f) {
            lateral_mode = KART_LATERAL_DRIFT;
            kart_drift_step_linger(&state->drift, dt);
        } else {
            lateral_mode = KART_LATERAL_GRIP;
        }

        lateral_input = (KartLateralInput){
            .forward_velocity = forward_velocity,
            .lateral_velocity = lateral_velocity,
            .yaw_lever_velocity = state->angular_velocity.z,
            .steering_input = controls->steering_input,
            .forward_input = controls->forward_input,
            .previous_steer_angle_rad = state->previous_steer_angle_rad,
            .reverse_steering = controls->reverse_steering,
            .drift_input_active = state->drift.input_active,
            .mode = lateral_mode,
        };
        lateral = kart_compute_lateral_response(&state->config, &lateral_input);
        kart_instant_boost_update_drift_exit(
            &state->instant_boost, &state->drift, was_drifting);
        state->previous_steer_angle_rad = lateral.next_previous_steer_angle_rad;
        force = add(force, scale(right, lateral.local_lateral_force));
        force = add(force, scale(forward, lateral.local_forward_force));
        torque.y += lateral.local_roll_torque;
        torque.z += lateral.local_yaw_torque;
    } else {
        state->drift.input_active = false;
        state->drift.slip_detected = false;
        state->drift.trigger_active = false;
        force.z += WORLD_GRAVITY * state->config.mass;
        torque = add(torque, scale(state->angular_velocity, -30.0f));
    }

    drag_input = (KartDragInput){
        .linear_velocity = state->linear_velocity,
        .angular_velocity = state->angular_velocity,
        .grounded = wheel.grounded,
        .grounded_drag_scale = state->grounded_drag_scale,
    };
    drag = kart_compute_drag_response(&state->config, &drag_input);
    force = add(force, drag.force);
    torque = add(torque, drag.torque);

    state->linear_velocity = kart_integrate_linear_velocity(
        state->linear_velocity, force, state->config.mass, dt);
    state->angular_velocity = kart_integrate_angular_velocity(
        state->angular_velocity,
        torque,
        kart_default_inverse_inertia(state->config.mass),
        dt);

    {
        const KartPoseInput pose_input = {
            .position = state->position,
            .orientation = state->orientation,
            .linear_velocity = state->linear_velocity,
            .angular_velocity = state->angular_velocity,
            .dt = dt,
        };
        const KartPoseOutput pose = kart_integrate_pose(&pose_input);
        state->position = pose.position;
        state->orientation = pose.orientation;
        state->angular_velocity = pose.angular_velocity;
    }

    if (world != NULL && world->query_body_collisions != NULL) {
        KartBodyContact contacts[KART_MAX_BODY_CONTACTS];
        unsigned int count = world->query_body_collisions(
            world->user_data, state, contacts, KART_MAX_BODY_CONTACTS);
        unsigned int i;
        if (count > KART_MAX_BODY_CONTACTS) {
            count = KART_MAX_BODY_CONTACTS;
        }
        orientation_axes(state->orientation, &right, &forward, &up);
        for (i = 0; i < count; ++i) {
            const KartCollisionInput collision_input = {
                .velocity = state->linear_velocity,
                .angular_velocity = state->angular_velocity,
                .normal = contacts[i].normal,
                .body_right = right,
                .body_forward = forward,
                .body_up = up,
                .sweep_fraction = contacts[i].sweep_fraction,
            };
            const KartCollisionOutput collision =
                kart_resolve_linear_collision(&collision_input);
            state->linear_velocity = collision.velocity;
            state->angular_velocity = collision.angular_velocity;
            if (collision.incoming) {
                float *strongest = collision.wall_contact
                    ? &result->wall_impact_speed
                    : &result->ground_impact_speed;
                if (collision.normal_speed > *strongest) {
                    *strongest = collision.normal_speed;
                }
            }
        }
        result->body_contacts += count;
    }
}

KartSimulationStepResult kart_simulate_milliseconds(
    KartSimulationState *state,
    const KartSimulationControls *controls,
    const KartSimulationWorld *world,
    unsigned int elapsed_ms)
{
    KartSimulationStepResult result = {0};
    const KartSimulationControls zero_controls = {0};
    const KartSimulationControls *active_controls =
        controls != NULL ? controls : &zero_controls;
    const unsigned int frame_elapsed_ms = elapsed_ms;
    const bool boost_pressed = active_controls->boost_active;

    /* Item input caller at 0x00457ac0 first checks IsBoosting, then starts a
       3000 ms timed boost. A held key cannot retrigger after expiry.

       Simulator-side rule: only the timed boost blocks a new one, so an item
       boost can be started while the instant boost is still running and the two
       overlap. Reading IsBoosting as covering both would swallow the press
       instead. The forward-force multiplier is unchanged either way — it is
       already keyed off "any boost active" and does not stack. */
    if (boost_pressed && !state->previous_boost_input &&
        !state->timed_boost.active) {
        kart_timed_boost_start(
            &state->timed_boost,
            active_controls->forward_input,
            KART_ITEM_BOOST_DURATION_MS);
    }
    state->previous_boost_input = boost_pressed;

    while (elapsed_ms != 0) {
        const unsigned int step_ms =
            elapsed_ms > MAX_SUBSTEP_MS ? MAX_SUBSTEP_MS : elapsed_ms;
        simulate_substep(
            state,
            active_controls,
            world,
            (float)step_ms * FIXED_STEP_SECONDS_PER_MS,
            &result);
        elapsed_ms -= step_ms;
        result.substeps += 1;
    }
    kart_timed_boost_step_milliseconds(&state->timed_boost, frame_elapsed_ms);
    /* Two simulator-side cutoff models can be compared: the current model ends
       both boosts on throttle release, while the alternate model keeps them
       alive until reverse is pressed. */
    if ((!state->reverse_input_ends_boost &&
         active_controls->forward_input == 0.0f) ||
        (state->reverse_input_ends_boost &&
         active_controls->reverse_input != 0.0f)) {
        if (state->timed_boost.active) {
            state->timed_boost.remaining_ms = 0;
            state->timed_boost.active = false;
        }
        /* The selected cutoff is shared by the item and instant boosts. */
        if (state->instant_boost.active) {
            state->instant_boost.active_timer = 0.0f;
            state->instant_boost.active = false;
        }
    }
    result.grounded = state->grounded;
    return result;
}
