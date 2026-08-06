#include "kart_dynamics.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct OracleRow {
    char phase[16];
    unsigned int step;
    float opportunity_timer;
    float active_timer;
    unsigned int active;
    KartVec3 force;
    unsigned int entry_was_forward;
} OracleRow;

static int parse_row(const char *line, OracleRow *row)
{
    return sscanf(
        line,
        "%15[^,],%u,%f,%f,%u,%f,%f,%f,%u",
        row->phase,
        &row->step,
        &row->opportunity_timer,
        &row->active_timer,
        &row->active,
        &row->force.x,
        &row->force.y,
        &row->force.z,
        &row->entry_was_forward) == 9;
}

static float similarity(float actual, float expected)
{
    const float denominator = fmaxf(1.0f, fabsf(expected));
    return 1.0f - fminf(fabsf(actual - expected) / denominator, 1.0f);
}

int main(int argc, char **argv)
{
    const KartDynamicsConfig config = kart_dynamics_default_config();
    const KartVec3 forward = {0.0f, -1.0f, 0.0f};
    KartInstantBoostState boost = {0};
    KartDriftState drift = {
        .entry_was_forward = true,
        .slip_detected = false,
    };
    FILE *file;
    char line[512];
    unsigned int rows = 0;
    unsigned int flags = 0;
    float total = 0.0f;
    float worst = 1.0f;

    if (argc != 2 || (file = fopen(argv[1], "r")) == NULL) {
        return 2;
    }
    if (fgets(line, sizeof(line), file) == NULL) {
        fclose(file);
        return 2;
    }
    while (fgets(line, sizeof(line), file) != NULL) {
        OracleRow expected;
        KartVec3 force = {0};
        float values[5];
        float expected_values[5];
        unsigned int i;

        if (!parse_row(line, &expected)) {
            fclose(file);
            return 2;
        }
        if (strcmp(expected.phase, "exit") == 0) {
            kart_instant_boost_update_drift_exit(&boost, &drift, true);
        } else if (strcmp(expected.phase, "press") == 0) {
            kart_instant_boost_press_forward(&boost);
        } else if (strcmp(expected.phase, "tick") == 0) {
            kart_instant_boost_step_timers(&boost, 0.005f);
            force = kart_compute_forward_drive_force(
                &config, forward, 1.0f, false, boost.active);
        } else {
            fclose(file);
            return 2;
        }

        values[0] = boost.opportunity_timer;
        values[1] = boost.active_timer;
        values[2] = force.x;
        values[3] = force.y;
        values[4] = force.z;
        expected_values[0] = expected.opportunity_timer;
        expected_values[1] = expected.active_timer;
        expected_values[2] = expected.force.x;
        expected_values[3] = expected.force.y;
        expected_values[4] = expected.force.z;
        for (i = 0; i < 5; ++i) {
            const float score = similarity(values[i], expected_values[i]);
            total += score;
            if (score < worst) worst = score;
        }
        if ((unsigned int)boost.active != expected.active ||
            (unsigned int)drift.entry_was_forward != expected.entry_was_forward) {
            flags += 1;
        }
        rows += 1;
    }
    fclose(file);
    printf(
        "instant boost rows=%u scalar_similarity=%.6f worst_scalar=%.6f flag_mismatches=%u\n",
        rows,
        rows != 0 ? total / (float)(rows * 5) : 0.0f,
        worst,
        flags);
    return rows == 107 && worst > 0.99998f && flags == 0 ? 0 : 1;
}
