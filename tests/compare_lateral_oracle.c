#include "kart_dynamics.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct OracleRow {
    char scenario[64];
    unsigned int step;
    float force[3];
    float torque[3];
    unsigned int drift_input;
    unsigned int slip;
    unsigned int trigger;
    float trigger_timer;
    float linger_timer;
    float steer_angle;
} OracleRow;

typedef struct RecoveredScenario {
    KartDriftState drift;
} RecoveredScenario;

static int parse_row(const char *line, OracleRow *row)
{
    return sscanf(
        line,
        "%63[^,],%u,%f,%f,%f,%f,%f,%f,%u,%u,%u,%f,%f,%f",
        row->scenario,
        &row->step,
        &row->force[0], &row->force[1], &row->force[2],
        &row->torque[0], &row->torque[1], &row->torque[2],
        &row->drift_input, &row->slip, &row->trigger,
        &row->trigger_timer, &row->linger_timer, &row->steer_angle) == 14;
}

static float scalar_similarity(float actual, float expected)
{
    const float denominator = fmaxf(1.0f, fabsf(expected));
    return 1.0f - fminf(fabsf(actual - expected) / denominator, 1.0f);
}

static KartLateralOutput replay_row(
    const OracleRow *row,
    const KartDynamicsConfig *config,
    RecoveredScenario *scenario)
{
    KartLateralInput input = {0};
    KartLateralMode mode;

    if (row->step == 0) {
        memset(scenario, 0, sizeof(*scenario));
        if (strcmp(row->scenario, "drift_release") == 0) {
            scenario->drift.input_active = true;
            scenario->drift.trigger_active = true;
            scenario->drift.entry_was_forward = true;
        }
    }

    if (strcmp(row->scenario, "grip_turn") == 0) {
        input.forward_velocity = 20.0f;
        input.steering_input = 1.0f;
    } else if (strcmp(row->scenario, "drift_release") == 0) {
        input.forward_velocity = 20.0f;
        input.lateral_velocity = row->step < 30 ? 0.0f : 8.0f;
        input.yaw_lever_velocity = 0.2f;
        input.steering_input = 1.0f;
        if (row->step == 25) {
            scenario->drift.input_active = false;
        }
    } else {
        input.forward_velocity = 10.0f;
        input.lateral_velocity = row->step < 10 ? 5.0f : 13.0f;
    }

    kart_drift_update_slip_detection(
        &scenario->drift,
        sqrtf(
            input.forward_velocity * input.forward_velocity +
            input.lateral_velocity * input.lateral_velocity),
        input.forward_velocity,
        input.lateral_velocity);

    if (scenario->drift.trigger_active) {
        mode = KART_LATERAL_DRIFT_TRIGGER;
    } else if (scenario->drift.input_active || scenario->drift.slip_detected ||
               scenario->drift.linger_timer > 0.0f) {
        mode = KART_LATERAL_DRIFT;
    } else {
        mode = KART_LATERAL_GRIP;
    }
    input.mode = mode;
    input.drift_input_active = scenario->drift.input_active;

    {
        const KartLateralOutput output = kart_compute_lateral_response(config, &input);
        if (mode == KART_LATERAL_DRIFT_TRIGGER) {
            kart_drift_step_trigger(&scenario->drift, config, 0.005f);
        } else if (mode == KART_LATERAL_DRIFT) {
            kart_drift_step_linger(&scenario->drift, 0.005f);
        }
        return output;
    }
}

int main(int argc, char **argv)
{
    const KartDynamicsConfig config = kart_dynamics_default_config();
    RecoveredScenario scenario = {0};
    FILE *file;
    char line[1024];
    unsigned int rows = 0;
    unsigned int scalar_count = 0;
    unsigned int flag_mismatches = 0;
    float similarity_sum = 0.0f;
    float worst_similarity = 1.0f;

    if (argc != 2) {
        fprintf(stderr, "Usage: kart_oracle_compare <oracle-lateral.csv>\n");
        return 2;
    }
    file = fopen(argv[1], "r");
    if (file == NULL) {
        perror("fopen");
        return 2;
    }
    if (fgets(line, sizeof(line), file) == NULL) {
        fclose(file);
        return 2;
    }

    while (fgets(line, sizeof(line), file) != NULL) {
        OracleRow expected;
        KartLateralOutput actual;
        float actual_values[9];
        float expected_values[9];
        unsigned int i;
        if (!parse_row(line, &expected)) {
            fprintf(stderr, "Could not parse oracle row %u\n", rows + 2);
            fclose(file);
            return 2;
        }
        actual = replay_row(&expected, &config, &scenario);
        actual_values[0] = actual.local_lateral_force;
        actual_values[1] = 0.0f;
        actual_values[2] = 0.0f;
        actual_values[3] = 0.0f;
        actual_values[4] = actual.local_roll_torque;
        actual_values[5] = actual.local_yaw_torque;
        actual_values[6] = scenario.drift.trigger_timer;
        actual_values[7] = scenario.drift.linger_timer;
        actual_values[8] = actual.steer_angle_rad;
        memcpy(expected_values, expected.force, sizeof(expected.force));
        memcpy(expected_values + 3, expected.torque, sizeof(expected.torque));
        expected_values[6] = expected.trigger_timer;
        expected_values[7] = expected.linger_timer;
        expected_values[8] = expected.steer_angle;

        for (i = 0; i < 9; ++i) {
            const float similarity = scalar_similarity(actual_values[i], expected_values[i]);
            similarity_sum += similarity;
            scalar_count += 1;
            if (similarity < worst_similarity) {
                worst_similarity = similarity;
            }
        }
        if ((unsigned int)scenario.drift.input_active != expected.drift_input ||
            (unsigned int)scenario.drift.slip_detected != expected.slip ||
            (unsigned int)scenario.drift.trigger_active != expected.trigger) {
            fprintf(
                stderr,
                "Flag mismatch %s step %u: actual %u/%u/%u expected %u/%u/%u\n",
                expected.scenario,
                expected.step,
                scenario.drift.input_active,
                scenario.drift.slip_detected,
                scenario.drift.trigger_active,
                expected.drift_input,
                expected.slip,
                expected.trigger);
            flag_mismatches += 1;
        }
        rows += 1;
    }
    fclose(file);

    {
        const float average = scalar_count != 0
            ? similarity_sum / (float)scalar_count
            : 0.0f;
        printf(
            "oracle rows=%u scalar_similarity=%.6f worst_scalar=%.6f flag_mismatches=%u\n",
            rows,
            average,
            worst_similarity,
            flag_mismatches);
        if (rows != 140 || average < 0.95f || flag_mismatches != 0) {
            return 1;
        }
    }
    return 0;
}
