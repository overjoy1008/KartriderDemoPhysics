#include "kart_dynamics.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static float similarity(float actual, float expected)
{
    const float denominator = fmaxf(1.0f, fabsf(expected));
    return 1.0f - fminf(fabsf(actual - expected) / denominator, 1.0f);
}

int main(int argc, char **argv)
{
    const KartDynamicsConfig config = kart_dynamics_default_config();
    const KartVec3 forward = {0.0f, -1.0f, 0.0f};
    KartTimedBoostState timed = {0};
    KartInstantBoostState instant = {0};
    FILE *file;
    char line[512];
    unsigned int rows = 0, flag_mismatches = 0;
    float total = 0.0f, worst = 1.0f;

    if (argc != 2 || (file = fopen(argv[1], "r")) == NULL) return 2;
    if (fgets(line, sizeof(line), file) == NULL) {
        fclose(file);
        return 2;
    }
    while (fgets(line, sizeof(line), file) != NULL) {
        char phase[16];
        unsigned int expected_remaining, expected_timed, expected_instant, expected_any;
        KartVec3 expected_force;
        KartVec3 force;
        float scores[4];
        unsigned int i;
        if (sscanf(
                line,
                "%15[^,],%u,%u,%u,%u,%f,%f,%f",
                phase,
                &expected_remaining,
                &expected_timed,
                &expected_instant,
                &expected_any,
                &expected_force.x,
                &expected_force.y,
                &expected_force.z) != 8) {
            fclose(file);
            return 2;
        }
        if (strcmp(phase, "no_forward") == 0) {
            kart_timed_boost_start(&timed, 0.0f, 3000);
        } else if (strcmp(phase, "timed") == 0) {
            kart_timed_boost_start(&timed, 1.0f, 3000);
        } else if (strcmp(phase, "instant") == 0) {
            timed = (KartTimedBoostState){0};
            instant.active = true;
        } else {
            fclose(file);
            return 2;
        }
        force = kart_compute_forward_drive_force(
            &config,
            forward,
            strcmp(phase, "no_forward") == 0 ? 0.0f : 1.0f,
            false,
            kart_any_boost_active(&timed, &instant));
        scores[0] = similarity((float)timed.remaining_ms, (float)expected_remaining);
        scores[1] = similarity(force.x, expected_force.x);
        scores[2] = similarity(force.y, expected_force.y);
        scores[3] = similarity(force.z, expected_force.z);
        for (i = 0; i < 4; ++i) {
            total += scores[i];
            if (scores[i] < worst) worst = scores[i];
        }
        if ((unsigned int)timed.active != expected_timed ||
            (unsigned int)instant.active != expected_instant ||
            (unsigned int)kart_any_boost_active(&timed, &instant) != expected_any) {
            flag_mismatches += 1;
        }
        rows += 1;
    }
    fclose(file);
    printf(
        "timed boost rows=%u scalar_similarity=%.6f worst_scalar=%.6f flag_mismatches=%u\n",
        rows,
        rows != 0 ? total / (float)(rows * 4) : 0.0f,
        worst,
        flag_mismatches);
    return rows == 3 && worst > 0.99998f && flag_mismatches == 0 ? 0 : 1;
}
