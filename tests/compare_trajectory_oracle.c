#include "kart_dynamics.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct TrajectoryRow {
    unsigned int step;
    float steer;
    unsigned int drift_pressed;
    KartVec3 position;
    KartVec3 velocity;
    KartVec3 omega;
    KartQuat orientation;
    unsigned int drift_input;
    unsigned int slip;
    unsigned int trigger;
    float trigger_timer;
    float linger_timer;
} TrajectoryRow;

typedef struct ReplayState {
    KartVec3 position;
    KartVec3 velocity;
    KartVec3 omega;
    KartQuat orientation;
    KartDriftState drift;
    float previous_steer_angle_rad;
} ReplayState;

static KartVec3 add(KartVec3 a, KartVec3 b)
{
    return (KartVec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

static KartVec3 scale(KartVec3 value, float amount)
{
    return (KartVec3){value.x * amount, value.y * amount, value.z * amount};
}

static float dot(KartVec3 a, KartVec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static void axes(KartQuat q, KartVec3 *right, KartVec3 *forward, KartVec3 *up)
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
    *up = (KartVec3){
        2.0f * (xz + wy),
        2.0f * (yz - wx),
        1.0f - 2.0f * (xx + yy),
    };
}

static int parse_row(const char *line, TrajectoryRow *row)
{
    return sscanf(
        line,
        "%u,%f,%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%u,%u,%u,%f,%f",
        &row->step, &row->steer, &row->drift_pressed,
        &row->position.x, &row->position.y, &row->position.z,
        &row->velocity.x, &row->velocity.y, &row->velocity.z,
        &row->omega.x, &row->omega.y, &row->omega.z,
        &row->orientation.w, &row->orientation.x,
        &row->orientation.y, &row->orientation.z,
        &row->drift_input, &row->slip, &row->trigger,
        &row->trigger_timer, &row->linger_timer) == 21;
}

static float similarity(float actual, float expected)
{
    const float denominator = fmaxf(1.0f, fabsf(expected));
    return 1.0f - fminf(fabsf(actual - expected) / denominator, 1.0f);
}

static void replay_step(
    ReplayState *state,
    const KartDynamicsConfig *config,
    const TrajectoryRow *input)
{
    KartVec3 right, forward, up;
    KartVec3 force = {0};
    KartVec3 torque = {0};
    KartLateralInput lateral_input;
    KartLateralOutput lateral;
    KartLateralMode mode;
    KartDragInput drag_input;
    KartDragOutput drag;
    float forward_velocity;
    float lateral_velocity;
    float speed;

    if (input->step == 40) {
        state->drift.input_active = true;
        state->drift.trigger_active = true;
        state->drift.entry_was_forward = true;
    } else if (input->step == 100) {
        state->drift.input_active = false;
    }

    axes(state->orientation, &right, &forward, &up);
    (void)up;
    forward_velocity = dot(state->velocity, forward);
    lateral_velocity = dot(state->velocity, right);
    speed = sqrtf(
        forward_velocity * forward_velocity +
        lateral_velocity * lateral_velocity);
    kart_drift_update_slip_detection(
        &state->drift, speed, forward_velocity, lateral_velocity);
    if (state->drift.trigger_active) {
        mode = KART_LATERAL_DRIFT_TRIGGER;
    } else if (state->drift.input_active || state->drift.slip_detected ||
               state->drift.linger_timer > 0.0f) {
        mode = KART_LATERAL_DRIFT;
    } else {
        mode = KART_LATERAL_GRIP;
    }
    lateral_input = (KartLateralInput){
        .forward_velocity = forward_velocity,
        .lateral_velocity = lateral_velocity,
        .yaw_lever_velocity = state->omega.z,
        .steering_input = input->steer,
        .forward_input = 1.0f,
        .previous_steer_angle_rad = state->previous_steer_angle_rad,
        .drift_input_active = state->drift.input_active,
        .mode = mode,
    };
    lateral = kart_compute_lateral_response(config, &lateral_input);
    state->previous_steer_angle_rad = lateral.next_previous_steer_angle_rad;
    if (mode == KART_LATERAL_DRIFT_TRIGGER) {
        kart_drift_step_trigger(&state->drift, config, 0.005f);
    } else if (mode == KART_LATERAL_DRIFT) {
        kart_drift_step_linger(&state->drift, 0.005f);
    }
    force = add(force, scale(right, lateral.local_lateral_force));
    torque.y += lateral.local_roll_torque;
    torque.z += lateral.local_yaw_torque;

    drag_input = (KartDragInput){
        .linear_velocity = state->velocity,
        .angular_velocity = state->omega,
        .grounded = true,
        .grounded_drag_scale = 1.0f,
    };
    drag = kart_compute_drag_response(config, &drag_input);
    force = add(force, drag.force);
    torque = add(torque, drag.torque);
    state->velocity = kart_integrate_linear_velocity(
        state->velocity, force, config->mass, 0.005f);
    state->omega = kart_integrate_angular_velocity(
        state->omega, torque, kart_default_inverse_inertia(config->mass), 0.005f);
    {
        const KartPoseInput pose_input = {
            .position = state->position,
            .orientation = state->orientation,
            .linear_velocity = state->velocity,
            .angular_velocity = state->omega,
            .dt = 0.005f,
        };
        const KartPoseOutput pose = kart_integrate_pose(&pose_input);
        state->position = pose.position;
        state->orientation = pose.orientation;
        state->omega = pose.angular_velocity;
    }
}

int main(int argc, char **argv)
{
    static const char *field_names[15] = {
        "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z",
        "omega_x", "omega_y", "omega_z", "q_w", "q_x", "q_y", "q_z",
        "trigger_timer", "linger_timer",
    };
    const KartDynamicsConfig config = kart_dynamics_default_config();
    ReplayState state = {
        .velocity = {0.0f, -20.0f, 0.0f},
        .orientation = {1.0f, 0.0f, 0.0f, 0.0f},
    };
    FILE *file;
    char line[2048];
    unsigned int rows = 0, count = 0, flag_mismatches = 0;
    float sum = 0.0f, worst = 1.0f;
    float field_sum[15] = {0};
    float field_worst[15];
    float field_worst_actual[15] = {0};
    float field_worst_expected[15] = {0};
    unsigned int field_worst_step[15] = {0};
    unsigned int field_index;

    for (field_index = 0; field_index < 15; ++field_index) {
        field_worst[field_index] = 1.0f;
    }

    if (argc != 2 || (file = fopen(argv[1], "r")) == NULL) {
        return 2;
    }
    if (fgets(line, sizeof(line), file) == NULL) {
        fclose(file);
        return 2;
    }
    while (fgets(line, sizeof(line), file) != NULL) {
        TrajectoryRow expected;
        float actual_values[15];
        float expected_values[15];
        unsigned int i;
        if (!parse_row(line, &expected)) {
            fclose(file);
            return 2;
        }
        replay_step(&state, &config, &expected);
        actual_values[0] = state.position.x;
        actual_values[1] = state.position.y;
        actual_values[2] = state.position.z;
        actual_values[3] = state.velocity.x;
        actual_values[4] = state.velocity.y;
        actual_values[5] = state.velocity.z;
        actual_values[6] = state.omega.x;
        actual_values[7] = state.omega.y;
        actual_values[8] = state.omega.z;
        actual_values[9] = state.orientation.w;
        actual_values[10] = state.orientation.x;
        actual_values[11] = state.orientation.y;
        actual_values[12] = state.orientation.z;
        actual_values[13] = state.drift.trigger_timer;
        actual_values[14] = state.drift.linger_timer;
        expected_values[0] = expected.position.x;
        expected_values[1] = expected.position.y;
        expected_values[2] = expected.position.z;
        expected_values[3] = expected.velocity.x;
        expected_values[4] = expected.velocity.y;
        expected_values[5] = expected.velocity.z;
        expected_values[6] = expected.omega.x;
        expected_values[7] = expected.omega.y;
        expected_values[8] = expected.omega.z;
        expected_values[9] = expected.orientation.w;
        expected_values[10] = expected.orientation.x;
        expected_values[11] = expected.orientation.y;
        expected_values[12] = expected.orientation.z;
        expected_values[13] = expected.trigger_timer;
        expected_values[14] = expected.linger_timer;
        for (i = 0; i < 15; ++i) {
            const float value = similarity(actual_values[i], expected_values[i]);
            sum += value;
            field_sum[i] += value;
            count += 1;
            if (value < worst) worst = value;
            if (value < field_worst[i]) {
                field_worst[i] = value;
                field_worst_step[i] = expected.step;
                field_worst_actual[i] = actual_values[i];
                field_worst_expected[i] = expected_values[i];
            }
        }
        if ((unsigned int)state.drift.input_active != expected.drift_input ||
            (unsigned int)state.drift.slip_detected != expected.slip ||
            (unsigned int)state.drift.trigger_active != expected.trigger) {
            flag_mismatches += 1;
        }
        rows += 1;
    }
    fclose(file);
    printf(
        "trajectory rows=%u scalar_similarity=%.6f worst_scalar=%.6f flag_mismatches=%u\n",
        rows, sum / (float)count, worst, flag_mismatches);
    for (field_index = 0; field_index < 15; ++field_index) {
        printf(
            "  %-14s average=%.6f worst=%.6f step=%u actual=%.7g expected=%.7g\n",
            field_names[field_index],
            field_sum[field_index] / (float)rows,
            field_worst[field_index],
            field_worst_step[field_index],
            field_worst_actual[field_index],
            field_worst_expected[field_index]);
    }
    return rows == 240 && sum / (float)count >= 0.95f && flag_mismatches == 0 ? 0 : 1;
}
