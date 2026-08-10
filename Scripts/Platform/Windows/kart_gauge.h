#ifndef KART_GAUGE_H
#define KART_GAUGE_H

/* Drift gauge — a simulator-side layer, not recovered code.

   The original's charging function is not in the recovered set, so this is a
   test bench for the two standing hypotheses rather than a claim about it. Both
   integrate how fast the rear axle is sliding sideways, which the engine
   already computes for the rear tire:

     S_r = (-v_s + 0.5*w_z) / max(|v|, 5)
     dG  = Kg * |v| * |S_r|            (above 5 m/s this is |-v_s + 0.5*w_z|)

   Model SLIP stops there. Model SUSPENSION multiplies by a contact weight built
   from the left/right suspension compression difference, so a run where the
   load has moved to the outside of the slide charges faster. Holding the same
   |v|, v_s and w_z on flat ground and on a bank tells the two apart: if the
   bank charges faster with everything else matched, the charging function reads
   the contacts rather than only the slip.

   Model INFINITE keeps the pre-gauge feel without skipping the slot rule: any
   drift at all fills the gauge at once, so a booster is one flick away.

   In every model the full gauge is handed over as a booster when the drift
   ends, not the instant it fills, so it stays visibly full while the drift
   lasts. */

#include "kart_simulation.h"

#include <math.h>
#include <stdbool.h>
#include <limits.h>

typedef enum KartGaugeModel {
    KART_GAUGE_INFINITE = 0,
    KART_GAUGE_SLIP = 1,
    KART_GAUGE_SUSPENSION = 2
} KartGaugeModel;

#define KART_GAUGE_MODEL_COUNT 3

typedef struct KartGaugeConfig {
    /* Kg: gauge units per metre of rear-axle side travel. */
    float charge_factor;
    /* Gauge units that make one booster. */
    float full_value;
    /* Ks and its clamp, for the SUSPENSION model's contact weight. */
    float suspension_gain;
    float suspension_max;
} KartGaugeConfig;

typedef struct KartGaugeState {
    KartGaugeModel model;
    float value;
    unsigned int boosters;
    bool unlimited_boosters;
    /* Last update's charge rate and contact weight, for the telemetry line. */
    float rate;
    float contact_weight;
} KartGaugeState;

static KartGaugeConfig kart_gauge_default_config(void)
{
    KartGaugeConfig config;
    config.charge_factor = 4.0f;
    config.full_value = 200.0f;
    config.suspension_gain = 1.0f;
    config.suspension_max = 1.0f;
    return config;
}

static const char *kart_gauge_model_name(KartGaugeModel model)
{
    switch (model) {
    case KART_GAUGE_SLIP: return "SLIP INTEGRAL";
    case KART_GAUGE_SUSPENSION: return "SLIP x SUSPENSION";
    default: return "INFINITE BOOSTER";
    }
}

/* Load moved to the outside of the slide, as a 0..1 fraction. Wheel order is
   the simulation's: 0 front-right, 1 front-left, 2 rear-right, 3 rear-left. */
static float kart_gauge_contact_weight(
    const KartGaugeConfig *config,
    const KartSimulationState *kart,
    float lateral_speed)
{
    const float right_load =
        kart->wheels.compression[0] + kart->wheels.compression[2];
    const float left_load =
        kart->wheels.compression[1] + kart->wheels.compression[3];
    const float total = right_load + left_load;
    float favourable;
    if (total <= 0.0001f) return 1.0f;
    /* Sliding towards body right (v_s > 0) rolls the load onto the left. */
    favourable = (left_load - right_load) / total;
    if (lateral_speed < 0.0f) favourable = -favourable;
    if (favourable < 0.0f) favourable = 0.0f;
    if (favourable > config->suspension_max) favourable = config->suspension_max;
    return 1.0f + config->suspension_gain * favourable;
}

/* Call once per frame with the same elapsed clock the simulation used.
   drift_active is the demo's drift visual state; the gauge only charges while
   the kart is on the ground and actually sliding. */
static void kart_gauge_update(
    KartGaugeState *state,
    const KartGaugeConfig *config,
    const KartSimulationState *kart,
    float forward_speed,
    float lateral_speed,
    bool drift_active,
    unsigned int max_boosters,
    float dt)
{
    const KartVec3 v = kart->linear_velocity;
    const float speed = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    float slip;
    float amount;

    (void)forward_speed;
    state->rate = 0.0f;
    state->contact_weight = 1.0f;
    if (dt <= 0.0f || !drift_active || !kart->grounded || speed <= 5.0f) {
        /* The booster is handed over when the drift ends, not the moment the
           gauge fills. A part-charged gauge is kept as it is. Filling with both
           slots already taken throws the booster away. */
        if (state->value >= config->full_value) {
            state->value = 0.0f;
            if (state->unlimited_boosters) {
                if (state->boosters != UINT_MAX) state->boosters += 1;
            } else if (state->boosters < max_boosters) {
                state->boosters += 1;
            }
        }
        return;
    }
    slip = (-lateral_speed + 0.5f * kart->angular_velocity.z) / speed;
    amount = speed * fabsf(slip);
    if (state->model == KART_GAUGE_SUSPENSION) {
        state->contact_weight =
            kart_gauge_contact_weight(config, kart, lateral_speed);
        amount *= state->contact_weight;
    }
    state->rate = config->charge_factor * amount;
    if (state->model == KART_GAUGE_INFINITE) {
        /* Any drift at all is enough. */
        state->value = config->full_value;
    } else {
        state->value += state->rate * dt;
    }
    /* Holds visibly full for the rest of the drift; one booster per drift. */
    if (state->value > config->full_value) {
        state->value = config->full_value;
    }
}

/* True when the press may start a booster, spending one of the charges. */
static bool kart_gauge_take_booster(KartGaugeState *state)
{
    if (state->boosters == 0) return false;
    state->boosters -= 1;
    return true;
}

#endif
