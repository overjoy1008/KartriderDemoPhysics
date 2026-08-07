#ifndef KART_SIMULATION_H
#define KART_SIMULATION_H

#include "kart_dynamics.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KART_WHEEL_COUNT 4
#define KART_MAX_BODY_CONTACTS 16
#define KART_ITEM_BOOST_DURATION_MS 3000

typedef struct KartGroundHit {
    KartVec3 point;
    KartVec3 normal;
    unsigned int surface_id;
} KartGroundHit;

typedef bool (*KartGroundQueryFn)(
    void *user_data,
    KartVec3 ray_start,
    KartVec3 ray_delta,
    KartGroundHit *hit);

typedef struct KartBodyContact {
    KartVec3 normal;
    float sweep_fraction;
    unsigned int surface_id;
} KartBodyContact;

struct KartSimulationState;
typedef unsigned int (*KartBodyCollisionQueryFn)(
    void *user_data,
    const struct KartSimulationState *state,
    KartBodyContact *contacts,
    unsigned int capacity);

typedef struct KartSimulationWorld {
    KartGroundQueryFn query_ground;
    KartBodyCollisionQueryFn query_body_collisions;
    void *user_data;
} KartSimulationWorld;

typedef struct KartSimulationGeometry {
    float half_width;
    float half_length;
    float suspension_range;
    float grounded_drag_scale;
} KartSimulationGeometry;

typedef struct KartWheelContactState {
    float compression[KART_WHEEL_COUNT];
    bool grounded;
} KartWheelContactState;

typedef struct KartWheelQueryInput {
    KartVec3 position;
    KartVec3 body_right;
    KartVec3 body_forward;
    KartVec3 body_up;
    KartSimulationGeometry geometry;
    KartGroundQueryFn query;
    void *user_data;
} KartWheelQueryInput;

typedef struct KartWheelQueryOutput {
    KartSuspensionContact contacts[KART_WHEEL_COUNT];
    KartVec3 contact_points[KART_WHEEL_COUNT];
    KartVec3 average_normal;
    unsigned int surface_id;
    unsigned int active_contacts;
    bool grounded;
    bool landed_this_step;
} KartWheelQueryOutput;

typedef struct KartSimulationControls {
    float forward_input;
    float reverse_input;
    float steering_input;
    bool reverse_steering;
    bool drift_input;
    bool boost_active;
    bool drive_disabled;
} KartSimulationControls;

typedef struct KartSimulationState {
    KartDynamicsConfig config;
    KartSimulationGeometry geometry;
    KartVec3 position;
    KartQuat orientation;
    KartVec3 linear_velocity;
    KartVec3 angular_velocity;
    KartDriftState drift;
    KartLongitudinalState longitudinal;
    KartInstantBoostState instant_boost;
    KartTimedBoostState timed_boost;
    KartWheelContactState wheels;
    float previous_steer_angle_rad;
    float grounded_drag_scale;
    bool previous_forward_input;
    bool previous_drift_input;
    bool previous_boost_input;
    bool grounded;
} KartSimulationState;

typedef struct KartSimulationStepResult {
    unsigned int substeps;
    unsigned int wheel_contacts;
    unsigned int body_contacts;
    bool grounded;
    bool landed;
    /* Largest incoming normal speed resolved during the step, split the way
       0x00430830 caches it: the wall branch stores its magnitude at kart+0x2FC
       and the ground branch at kart+0x304. The original's crash and shock
       sounds scale their volume by these. Zero when nothing was hit. */
    float wall_impact_speed;
    float ground_impact_speed;
} KartSimulationStepResult;

KartSimulationGeometry kart_simulation_default_geometry(void);

void kart_simulation_init(
    KartSimulationState *state,
    const KartDynamicsConfig *config,
    const KartSimulationGeometry *geometry);

void kart_simulation_set_grounded_drag_scale(
    KartSimulationState *state,
    float scale);

void kart_simulation_multiply_grounded_drag_scale(
    KartSimulationState *state,
    float multiplier);

KartWheelQueryOutput kart_query_wheel_contacts(
    KartWheelContactState *state,
    const KartWheelQueryInput *input);

KartSimulationStepResult kart_simulate_milliseconds(
    KartSimulationState *state,
    const KartSimulationControls *controls,
    const KartSimulationWorld *world,
    unsigned int elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif
