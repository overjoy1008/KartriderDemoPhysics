#include "kart_dynamics.h"
#include "kart_demo_data.h"
#include "kart_input.h"
#include "kart_simulation.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int near(float actual, float expected, float epsilon)
{
    return fabsf(actual - expected) <= epsilon;
}

static void test_defaults(void)
{
    const KartDynamicsConfig config = kart_dynamics_default_config();
    unsigned int kart_index;
    assert(near(config.mass, 100.0f, 0.0001f));
    assert(near(config.forward_accel_force, 3000.0f, 0.0001f));
    assert(near(config.drift_escape_force, 5000.0f, 0.0001f));
    assert(near(config.drift_trigger_time, 0.1f, 0.0001f));
    for (kart_index = 0; kart_index < kart_demo_kart_count(); ++kart_index) {
        assert(kart_demo_kart_at(kart_index)->max_boosters == 2u);
    }
}

static void test_steering_attenuation(void)
{
    const KartDynamicsConfig config = kart_dynamics_default_config();
    const float at_rest = kart_steer_angle_rad(&config, 0.0f, 1.0f, false);
    const float at_speed = kart_steer_angle_rad(&config, 30.0f, 1.0f, false);
    const float reversed = kart_steer_angle_rad(&config, 0.0f, 1.0f, true);

    assert(near(at_rest, 0.17453294f, 0.00001f));
    assert(near(at_speed, at_rest * expf(-1.0f), 0.00001f));
    assert(near(reversed, -at_rest, 0.00001f));
}

static void test_symmetric_grip(void)
{
    const KartDynamicsConfig config = kart_dynamics_default_config();
    const KartLateralInput input = {
        .forward_velocity = 20.0f,
        .lateral_velocity = 0.0f,
        .yaw_lever_velocity = 0.0f,
        .steering_input = 0.0f,
        .reverse_steering = false,
        .mode = KART_LATERAL_GRIP,
    };
    const KartLateralOutput output = kart_compute_lateral_response(&config, &input);

    assert(near(output.front_force, 0.0f, 0.0001f));
    assert(near(output.rear_force, 0.0f, 0.0001f));
    assert(near(output.local_yaw_torque, 0.0f, 0.0001f));
}

static void test_original_low_speed_lateral_branch(void)
{
    KartDynamicsConfig config = kart_dynamics_default_config();
    KartLateralInput input = {
        .forward_velocity = 3.0f,
        .lateral_velocity = 2.0f,
        .steering_input = 1.0f,
        .drift_input_active = true,
        .mode = KART_LATERAL_GRIP,
    };
    const KartLateralOutput grip = kart_compute_lateral_response(&config, &input);
    KartLateralOutput drift;
    KartLateralOutput trigger;

    config.corner_draw_factor = 0.5f;
    input.mode = KART_LATERAL_DRIFT;
    drift = kart_compute_lateral_response(&config, &input);
    input.mode = KART_LATERAL_DRIFT_TRIGGER;
    trigger = kart_compute_lateral_response(&config, &input);

    assert(near(drift.front_force, grip.front_force, 0.001f));
    assert(near(drift.rear_force, grip.rear_force, 0.001f));
    assert(near(trigger.front_force, grip.front_force, 0.001f));
    assert(near(trigger.rear_force, grip.rear_force, 0.001f));
    assert(near(drift.local_forward_force, 0.0f, 0.001f));
    assert(near(trigger.local_forward_force, 0.0f, 0.001f));
}

static void test_corner_draw_force(void)
{
    KartDynamicsConfig config = kart_dynamics_default_config();
    KartLateralInput input = {
        .forward_velocity = 20.0f,
        .steering_input = 1.0f,
        .mode = KART_LATERAL_GRIP,
    };
    KartLateralOutput output;

    config.corner_draw_factor = 0.2f;
    output = kart_compute_lateral_response(&config, &input);
    assert(near(
        output.local_forward_force,
        fabsf(output.local_lateral_force) * 0.2f,
        0.001f));

    input.mode = KART_LATERAL_DRIFT;
    output = kart_compute_lateral_response(&config, &input);
    assert(near(output.local_forward_force, 0.0f, 0.001f));
}

static void test_speedometer_and_demo_assets(void)
{
    const KartVec3 velocity = {3.0f, 4.0f, 0.0f};
    const KartDemoKartSpec *burst = kart_demo_find_kart("burst3");
    const KartDemoKartSpec *cotten = kart_demo_find_kart("cotten5");
    const KartDemoKartSpec *saber = kart_demo_find_kart("saber5");
    const KartDemoKartSpec *marathon = kart_demo_find_kart("marathon5");
    const KartDemoTrackSpec *forest = kart_demo_find_track("forest_I01");
    const KartDemoTrackSpec *village = kart_demo_find_track("village_R01");
    KartVec3 start;
    KartQuat start_orientation;

    assert(near(kart_speed_kmh(velocity), 18.0f, 0.0001f));
    assert(kart_speedometer_kmh(velocity) == 18);
    assert(burst != NULL && strcmp(burst->asset_name, "burst3") == 0);
    assert(near(burst->dynamics.drag_factor, 0.725f, 0.0001f));
    assert(near(burst->dynamics.drift_trigger_time, 0.2f, 0.0001f));
    assert(near(burst->geometry.half_width, 0.8080695f, 0.0001f));
    assert(cotten != NULL && near(cotten->geometry.half_length, 1.13917575f, 0.0001f));
    /* Both demos open on this kart. It is a simulator-side choice: the demo's
       own kartlist.xml offers only burst3. */
    assert(kart_demo_default_kart() == cotten);
    assert(saber != NULL && near(saber->dynamics.grip_brake_force, 3000.0f, 0.0001f));
    assert(marathon != NULL && near(marathon->dynamics.corner_draw_factor, 0.2f, 0.0001f));
    assert(forest != NULL && near(kart_demo_track_width(forest), 896.391f, 0.01f));
    assert(village != NULL && near(kart_demo_track_length(village), 1712.1172f, 0.01f));
    assert(forest->difficulty == 1 && strcmp(forest->race_mode, "아이템") == 0);
    assert(village->difficulty == 2 && strcmp(village->race_mode, "스피드") == 0);
    assert(kart_demo_track_start_position(forest, &start));
    assert(kart_demo_track_mirror_x(forest));
    assert(near(start.x, -296.9286f, 0.001f));
    assert(near(start.y, 53.9684f, 0.001f));
    assert(near(kart_demo_track_scene_ground_z(forest), 27.07603f, 0.0001f));
    assert(kart_demo_track_start_orientation(forest, &start_orientation));
    assert(near(start_orientation.z, 1.0f, 0.0001f));
    assert(near(start_orientation.w, 0.0f, 0.0001f));
    assert(kart_demo_track_start_position(village, &start));
    assert(kart_demo_track_mirror_x(village));
    assert(near(start.x, 457.7419f, 0.001f));
    assert(near(start.y, -221.4766f, 0.001f));
    assert(near(kart_demo_track_scene_ground_z(village), 26.84984f, 0.0001f));
    assert(forest->start_kind == KART_TRACK_START_CONFIRMED);
    assert(village->start_kind == KART_TRACK_START_CONFIRMED);
}

/* Body forward for a start orientation, matching the demos' orientation_axes:
   the default basis points forward at -Y. */
static KartVec3 start_forward(KartQuat q)
{
    return (KartVec3){
        -2.0f * (q.x * q.y - q.w * q.z),
        -(1.0f - 2.0f * (q.x * q.x + q.z * q.z)),
        -2.0f * (q.y * q.z + q.w * q.x),
    };
}

/* The AABB, the embedded scene, the safety wall, the minimap normalization and
   the spawn all read the same track record, so a track whose scene metadata
   disagrees with its bounds would place the mesh away from its own walls. */
static void test_demo_track_table_is_consistent(void)
{
    const unsigned int count = kart_demo_track_count();
    const KartDemoTrackSpec *flat = kart_demo_find_track("flat_test");
    const KartDemoTrackSpec *forest = kart_demo_find_track("forest_I01");
    unsigned int i;
    unsigned int confirmed = 0;
    unsigned int without_scene = 0;
    assert(count == KART_DEMO_TRACK_COUNT);

    /* The demos open on the synthetic flat track, which keeps the flat ground
       and the AABB walls and matches Forest Log's footprint. */
    assert(flat != NULL && !flat->has_scene);
    assert(kart_demo_default_track() == flat);
    assert(forest != NULL);
    assert(near(kart_demo_track_width(flat), kart_demo_track_width(forest), 0.01f));
    assert(near(kart_demo_track_length(flat), kart_demo_track_length(forest), 0.01f));
    /* Centred on the world origin, so the axis gizmo reads against the walls. */
    assert(near(flat->minimum.x, -flat->maximum.x, 0.0001f));
    assert(near(flat->minimum.y, -flat->maximum.y, 0.0001f));
    assert(flat->start_kind == KART_TRACK_START_NONE);

    for (i = 0; i < count; ++i) {
        const KartDemoTrackSpec *track = kart_demo_track_at(i);
        KartVec3 start;
        KartQuat orientation;
        assert(track != NULL);
        assert(kart_demo_track_width(track) > 1.0f);
        assert(kart_demo_track_length(track) > 1.0f);
        assert(track->maximum.z > track->minimum.z);
        /* Every scene uses the same export, so the mirror is not per-track. */
        assert(kart_demo_track_mirror_x(track));
        assert(kart_demo_track_start_kind_label(track)[0] != '\0');

        /* The respawn floor must sit clear below every scene triangle, so a
           kart resting on the lowest road never trips it, and clear below the
           spawn plane at world z 0. */
        {
            const float fall_limit = kart_demo_track_fall_limit(track);
            const float scene_floor =
                track->minimum.z - kart_demo_track_scene_ground_z(track);
            assert(fall_limit < 0.0f);
            assert(fall_limit < scene_floor);
            assert(scene_floor - fall_limit >= 20.0f);
        }

        if (track->start_kind == KART_TRACK_START_NONE) {
            assert(track->start_axis == KART_TRACK_AXIS_NONE);
            /* Only the spawn position falls back to the bounds centre; the
               facing still matches every other track rather than pointing at
               -Y. */
            assert(!kart_demo_track_start_position(track, &start));
            assert(kart_demo_track_start_orientation(track, &orientation));
            assert(near(orientation.z, 1.0f, 0.0001f));
            assert(near(orientation.w, 0.0f, 0.0001f));
            /* Without a start quad the ground plane falls back to the bounds. */
            assert(near(kart_demo_track_scene_ground_z(track),
                        track->minimum.z, 0.0001f));
            continue;
        }

        assert(track->start_axis != KART_TRACK_AXIS_NONE);
        assert(kart_demo_track_start_position(track, &start));
        assert(kart_demo_track_start_orientation(track, &orientation));
        /* The start line is inside its own track bounds. */
        assert(track->start_line.x >= track->minimum.x);
        assert(track->start_line.x <= track->maximum.x);
        assert(track->start_line.y >= track->minimum.y);
        assert(track->start_line.y <= track->maximum.y);
        /* The spawn is bounds-relative, so it stays inside the safety walls. */
        assert(start.x >= -kart_demo_track_width(track) * 0.5f);
        assert(start.x <= kart_demo_track_width(track) * 0.5f);
        assert(start.y >= -kart_demo_track_length(track) * 0.5f);
        assert(start.y <= kart_demo_track_length(track) * 0.5f);
        assert(near(kart_demo_track_scene_ground_z(track),
                    track->start_line.z, 0.0001f));
        assert(near(orientation.w * orientation.w + orientation.z * orientation.z,
                    1.0f, 0.0001f));
        /* The assumed racing direction is the positive world axis in both
           cases, so a Y-axis start faces world +Y and an X-axis start faces
           world +X. */
        {
            const KartVec3 forward = start_forward(orientation);
            if (track->start_axis == KART_TRACK_AXIS_X) {
                assert(near(forward.x, 1.0f, 0.0001f));
                assert(near(forward.y, 0.0f, 0.0001f));
            } else {
                assert(near(forward.x, 0.0f, 0.0001f));
                assert(near(forward.y, 1.0f, 0.0001f));
            }
            assert(near(forward.z, 0.0f, 0.0001f));
        }
        if (track->start_kind == KART_TRACK_START_CONFIRMED) ++confirmed;
    }
    for (i = 0; i < count; ++i) {
        if (!kart_demo_track_at(i)->has_scene) ++without_scene;
    }
    /* Only the two tracks checked against the original game may claim it, and
       the flat test track is the only one without a decoded mesh. */
    assert(confirmed == 2);
    assert(without_scene == 1);
}

static void test_runtime_grounded_drag_scale(void)
{
    KartSimulationState state;
    kart_simulation_init(&state, NULL, NULL);
    assert(near(state.grounded_drag_scale, 1.0f, 0.0001f));
    kart_simulation_multiply_grounded_drag_scale(&state, 4.0f);
    assert(near(state.grounded_drag_scale, 4.0f, 0.0001f));
    kart_simulation_multiply_grounded_drag_scale(&state, 0.25f);
    assert(near(state.grounded_drag_scale, 1.0f, 0.0001f));
    kart_simulation_set_grounded_drag_scale(&state, 2.5f);
    assert(near(state.grounded_drag_scale, 2.5f, 0.0001f));
}

static void test_static_suspension_equilibrium(void)
{
    const KartDynamicsConfig config = kart_dynamics_default_config();
    KartSuspensionInput input = {
        .dt = 0.005f,
        .half_width = 0.8f,
        .half_length = 1.0f,
        .chassis_up = {0.0f, 0.0f, 1.0f},
    };
    KartSuspensionOutput output;
    unsigned int i;

    for (i = 0; i < 4; ++i) {
        input.contacts[i].active = true;
        input.contacts[i].normal = input.chassis_up;
        input.contacts[i].compression = 0.5f;
        input.contacts[i].compression_delta = 0.0f;
    }

    output = kart_compute_suspension_response(&config, &input);
    assert(output.active_contacts == 4);
    assert(near(output.world_force.x, 0.0f, 0.001f));
    assert(near(output.world_force.y, 0.0f, 0.001f));
    assert(near(output.world_force.z, 0.0f, 0.01f));
    assert(near(output.local_torque.x, 0.0f, 0.01f));
    assert(near(output.local_torque.y, 0.0f, 0.01f));
    assert(near(output.local_torque.z, 0.0f, 0.01f));
}

static void test_drift_trigger_timing(void)
{
    const KartDynamicsConfig config = kart_dynamics_default_config();
    KartDriftState state = {0};

    kart_drift_set_input(&state, true, 20.0f);
    assert(state.input_active);
    assert(state.trigger_active);
    assert(state.entry_was_forward);

    kart_drift_step_trigger(&state, &config, 0.005f);
    assert(near(state.trigger_timer, 0.1f, 0.0001f));
    assert(near(state.linger_timer, 0.2f, 0.0001f));

    while (state.trigger_active) {
        kart_drift_step_trigger(&state, &config, 0.005f);
    }
    assert(near(state.trigger_timer, 0.0f, 0.0001f));

    kart_drift_clear_for_low_speed(&state);
    assert(!state.input_active);
    assert(!state.slip_detected);
}

static void test_drift_slip_detection(void)
{
    KartDriftState state = {0};

    kart_drift_update_slip_detection(&state, 20.0f, 10.0f, 13.0f);
    assert(state.slip_detected);

    kart_drift_update_slip_detection(&state, 20.0f, 10.0f, 12.0f);
    assert(!state.slip_detected);

    state.linger_timer = 0.2f;
    kart_drift_step_linger(&state, 0.005f);
    assert(near(state.linger_timer, 0.195f, 0.0001f));
}

static void test_instant_boost_state_machine(void)
{
    KartDriftState drift = {
        .entry_was_forward = true,
    };
    KartInstantBoostState boost = {0};
    unsigned int steps = 0;

    kart_instant_boost_update_drift_exit(&boost, &drift, true);
    assert(!drift.entry_was_forward);
    assert(near(boost.opportunity_timer, 0.5f, 0.0001f));
    assert(!boost.active);

    kart_instant_boost_step_timers(&boost, 0.005f);
    assert(near(boost.opportunity_timer, 0.495f, 0.0001f));
    kart_instant_boost_press_forward(&boost);
    assert(near(boost.opportunity_timer, 0.0f, 0.0001f));
    assert(near(boost.active_timer, 0.5f, 0.0001f));
    assert(boost.active);

    while (boost.active && steps < 200) {
        kart_instant_boost_step_timers(&boost, 0.005f);
        steps += 1;
    }
    assert(steps >= 99 && steps <= 101);
    assert(near(boost.active_timer, 0.0f, 0.0001f));
    assert(!boost.active);
}

static void test_stored_instant_boost_state_machine(void)
{
    KartDriftState drift = {.entry_was_forward = true};
    KartInstantBoostState boost = {.stored_model = true};

    kart_instant_boost_update_drift_exit(&boost, &drift, true);
    assert(boost.stored_count == 1);
    assert(near(boost.opportunity_timer, 0.0f, 0.0001f));

    kart_instant_boost_press_forward(&boost);
    assert(boost.active);
    assert(boost.stored_count == 0);
    assert(near(boost.active_timer, 0.5f, 0.0001f));
    assert(boost.activation_count == 1);
    assert(!kart_instant_boost_use_stored(&boost));

    boost.stored_count = 2;
    assert(kart_instant_boost_use_stored(&boost));
    assert(boost.activation_count == 2);
    assert(kart_instant_boost_use_stored(&boost));
    assert(boost.activation_count == 3);
}

static void test_timed_boost_state_machine(void)
{
    KartTimedBoostState timed = {0};
    KartInstantBoostState instant = {0};
    KartVec3 force;

    assert(!kart_timed_boost_start(&timed, 0.0f, 3000));
    assert(!timed.active);
    assert(kart_timed_boost_start(&timed, 1.0f, 3000));
    assert(timed.active);
    assert(timed.remaining_ms == 3000);
    assert(kart_any_boost_active(&timed, &instant));

    force = kart_compute_forward_drive_force(
        &(KartDynamicsConfig){.forward_accel_force = 3000.0f},
        (KartVec3){0.0f, -1.0f, 0.0f},
        1.0f,
        false,
        kart_any_boost_active(&timed, &instant));
    assert(near(force.y, -4500.0f, 0.0001f));

    kart_timed_boost_step_milliseconds(&timed, 1000);
    assert(timed.active);
    assert(timed.remaining_ms == 2000);
    kart_timed_boost_step_milliseconds(&timed, 2500);
    assert(!timed.active);
    assert(timed.remaining_ms == 0);

    instant.active = true;
    assert(kart_any_boost_active(&timed, &instant));
}

static void test_drag_and_linear_integration(void)
{
    const KartDynamicsConfig config = kart_dynamics_default_config();
    const KartDragInput input = {
        .linear_velocity = {10.0f, 0.0f, 0.0f},
        .angular_velocity = {0.0f, 2.0f, 0.0f},
        .grounded = true,
        .grounded_drag_scale = 1.0f,
    };
    const KartDragOutput drag = kart_compute_drag_response(&config, &input);
    const KartVec3 velocity = kart_integrate_linear_velocity(
        input.linear_velocity, drag.force, config.mass, 0.005f);

    assert(near(drag.force.x, -80.0f, 0.001f));
    assert(near(drag.torque.y, -6.0f, 0.001f));
    assert(near(velocity.x, 9.996f, 0.0001f));
}

/* 0x00430830 splits on the contact normal's Z against 0.65 (0x00571D48):
   a steep face is a wall, a shallow one is ground. Expectations below are hand
   evaluated from the recovered formulas. */
static void test_collision_response(void)
{
    KartCollisionInput input = {
        .velocity = {-10.0f, 5.0f, 0.0f},
        .normal = {1.0f, 0.0f, 0.0f},
        .body_right = {1.0f, 0.0f, 0.0f},
        .body_forward = {0.0f, 1.0f, 0.0f},
        .body_up = {0.0f, 0.0f, 1.0f},
        .sweep_fraction = 0.5f,
    };
    KartCollisionOutput output = kart_resolve_linear_collision(&input);

    /* Wall. v_n = (-10,0,0), v_t = (0,5,0).
       loss = min(1.5*10, 0.6*5) = 3, dv = -1.5*v_n - normalize(v_t)*3. */
    assert(output.incoming);
    assert(output.wall_contact);
    assert(near(output.velocity.x, 5.0f, 0.0001f));
    assert(near(output.velocity.y, 2.0f, 0.0001f));
    assert(near(output.tangential_speed_removed, 3.0f, 0.0001f));
    /* Normal restitution is 0.5: -10 becomes +5. */
    assert(near(output.velocity.x, -0.5f * -10.0f, 0.0001f));

    /* The sweep fraction must not steer the branch. It used to, which put wall
       physics on shallow ground and ground physics on walls. */
    input.sweep_fraction = 0.8f;
    output = kart_resolve_linear_collision(&input);
    assert(output.wall_contact);
    assert(near(output.velocity.x, 5.0f, 0.0001f));
    assert(near(output.velocity.y, 2.0f, 0.0001f));
    input.sweep_fraction = 0.5f;

    /* The wall branch zeroes the correction's Z, so vertical speed survives a
       wall hit untouched. */
    input.velocity = (KartVec3){-10.0f, 5.0f, -3.0f};
    output = kart_resolve_linear_collision(&input);
    assert(output.wall_contact);
    assert(near(output.velocity.x, 5.0f, 0.0001f));
    assert(near(output.velocity.y, 2.0f, 0.0001f));
    assert(near(output.velocity.z, -3.0f, 0.0001f));

    /* Ground: normal.z = 0.8 > 0.65. v = v_t - 0.2*v_n, no tangential loss.
       dot(n,v) = -6, v_n = (-3.6,0,-4.8), v_t = (-6.4,5,4.8). */
    input.velocity = (KartVec3){-10.0f, 5.0f, 0.0f};
    input.normal = (KartVec3){0.6f, 0.0f, 0.8f};
    output = kart_resolve_linear_collision(&input);
    assert(output.incoming);
    assert(!output.wall_contact);
    assert(near(output.velocity.x, -5.68f, 0.0001f));
    assert(near(output.velocity.y, 5.0f, 0.0001f));
    assert(near(output.velocity.z, 5.76f, 0.0001f));
    assert(near(output.tangential_speed_removed, 0.0f, 0.0001f));
    assert(near(output.wall_yaw_kick, 0.0f, 0.0001f));
    /* w = cross(n, up) = (0,-0.6,0); x takes -dot(w,right)*0.1 = 0,
       y takes +dot(w,forward)*0.1 = -0.06. */
    assert(near(output.angular_velocity.x, 0.0f, 0.0001f));
    assert(near(output.angular_velocity.y, -0.06f, 0.0001f));
    assert(near(output.angular_velocity.z, 0.0f, 0.0001f));

    /* Wall yaw kick: |dot(n,forward)| = 0.8 > |dot(n,right)| = 0.6, so the kick
       uses the right term, scaled by clamp(|dot(n,v)|, 1, 30) = 10. */
    input.velocity = (KartVec3){-6.0f, -8.0f, 0.0f};
    input.normal = (KartVec3){0.6f, 0.8f, 0.0f};
    output = kart_resolve_linear_collision(&input);
    assert(output.wall_contact);
    assert(near(output.normal_speed, 10.0f, 0.0001f));
    assert(near(output.wall_yaw_kick, 6.0f, 0.0001f));
    assert(near(output.angular_velocity.z, 6.0f, 0.0001f));

    /* Same-direction spin already strong enough suppresses the kick. */
    input.angular_velocity = (KartVec3){0.0f, 0.0f, 1.0f};
    output = kart_resolve_linear_collision(&input);
    assert(near(output.wall_yaw_kick, 0.0f, 0.0001f));
    assert(near(output.angular_velocity.z, 1.0f, 0.0001f));
}

static void test_drive_and_brake_forces(void)
{
    const KartDynamicsConfig config = kart_dynamics_default_config();
    const KartVec3 forward = {1.0f, 0.0f, 0.0f};
    KartVec3 force = kart_compute_forward_drive_force(
        &config, forward, 1.0f, false, false);

    assert(near(force.x, 3000.0f, 0.001f));
    force = kart_compute_forward_drive_force(&config, forward, 1.0f, true, true);
    assert(near(force.x, 7500.0f, 0.001f));

    force = kart_compute_reverse_drive_force(&config, forward, 1.0f);
    assert(near(force.x, -2000.0f, 0.001f));

    force = kart_compute_directional_brake_force(
        &config, (KartVec3){10.0f, 0.0f, 0.0f}, forward);
    assert(near(force.x, -2000.0f, 0.001f));

    force = kart_compute_directional_brake_force(
        &config, (KartVec3){0.0f, 10.0f, 0.0f}, forward);
    assert(near(force.y, -1500.0f, 0.001f));
}

static void test_angular_integration(void)
{
    const KartMat3 inverse_inertia = kart_default_inverse_inertia(100.0f);
    const KartVec3 result = kart_integrate_angular_velocity(
        (KartVec3){0.0f, 0.0f, 0.0f},
        (KartVec3){0.0f, 10.0f, 0.0f},
        inverse_inertia,
        0.005f);

    assert(near(inverse_inertia.m[0][0], 0.12f, 0.0001f));
    assert(near(inverse_inertia.m[1][1], 0.12f, 0.0001f));
    assert(near(result.y, 0.006f, 0.0001f));
}

static void test_longitudinal_state_machine(void)
{
    const KartDynamicsConfig config = kart_dynamics_default_config();
    KartLongitudinalState state = {0};
    KartLongitudinalInput input = {
        .velocity = {10.0f, 0.0f, 0.0f},
        .forward_axis = {1.0f, 0.0f, 0.0f},
        .forward_velocity = 10.0f,
        .dt = 0.005f,
        .reverse_input = 1.0f,
    };
    KartLongitudinalOutput out = kart_step_longitudinal(&config, &state, &input);

    assert(out.mode == KART_LONGITUDINAL_BRAKE);
    assert(near(out.force.x, -2000.0f, 0.001f));

    input.velocity = (KartVec3){0.2f, 0.0f, 0.0f};
    input.forward_velocity = 0.2f;
    out = kart_step_longitudinal(&config, &state, &input);
    assert(out.mode == KART_LONGITUDINAL_STOPPED);
    assert(out.velocity_overridden);
    assert(near(out.velocity.x, 0.0f, 0.0001f));

    state.reverse_timer = 0.21f;
    out = kart_step_longitudinal(&config, &state, &input);
    assert(out.mode == KART_LONGITUDINAL_REVERSE);
    assert(near(out.force.x, -2000.0f, 0.001f));

    state.reverse_timer = 0.0f;
    input.reverse_input = 0.0f;
    input.velocity = (KartVec3){0};
    input.forward_velocity = 0.0f;
    out = kart_step_longitudinal(&config, &state, &input);
    assert(out.mode == KART_LONGITUDINAL_IDLE);
    assert(near(state.reverse_timer, 0.005f, 0.0001f));

    state.reverse_timer = 0.21f;
    input.drive_disabled = true;
    input.velocity = (KartVec3){0.2f, 0.0f, 0.0f};
    input.forward_velocity = 0.2f;
    out = kart_step_longitudinal(&config, &state, &input);
    assert(out.mode == KART_LONGITUDINAL_STOPPED);

    state.reverse_timer = 1.0f;
    input.drive_disabled = false;
    input.forward_input = 1.0f;
    input.velocity = (KartVec3){-2.0f, 0.0f, 0.0f};
    input.forward_velocity = -2.0f;
    out = kart_step_longitudinal(&config, &state, &input);
    assert(out.mode == KART_LONGITUDINAL_FORWARD);
    assert(near(out.force.x, 4960.0f, 0.01f));
    assert(near(state.reverse_timer, 0.0f, 0.0001f));
}

static void test_pose_integration_and_tilt_guard(void)
{
    KartPoseInput input = {
        .position = {1.0f, 2.0f, 3.0f},
        .orientation = {1.0f, 0.0f, 0.0f, 0.0f},
        .linear_velocity = {10.0f, 0.0f, 0.0f},
        .angular_velocity = {0.0f, 0.0f, 2.0f},
        .dt = 0.005f,
    };
    KartPoseOutput out = kart_integrate_pose(&input);

    assert(near(out.position.x, 1.05f, 0.0001f));
    assert(near(out.orientation.z, 0.00499994f, 0.00001f));
    assert(near(out.up_z, 1.0f, 0.0001f));
    assert(out.tilt_retries == 0);

    input.angular_velocity = (KartVec3){400.0f, 0.0f, 0.0f};
    out = kart_integrate_pose(&input);
    assert(out.tilt_retries == 1);
    assert(!out.tilt_clamped);
    assert(near(out.angular_velocity.x, 40.0f, 0.0001f));
    assert(out.up_z > 0.5f);
}

typedef struct FlatGroundContext {
    unsigned int queries;
} FlatGroundContext;

static bool query_flat_ground(
    void *user_data,
    KartVec3 start,
    KartVec3 delta,
    KartGroundHit *hit)
{
    FlatGroundContext *context = (FlatGroundContext *)user_data;
    float fraction;
    context->queries += 1;
    if (delta.z >= 0.0f || start.z < 0.0f || start.z + delta.z > 0.0f) {
        return false;
    }
    fraction = -start.z / delta.z;
    hit->point = (KartVec3){
        start.x + delta.x * fraction,
        start.y + delta.y * fraction,
        0.0f,
    };
    hit->normal = (KartVec3){0.0f, 0.0f, 1.0f};
    hit->surface_id = 7;
    return true;
}

static void test_wheel_contact_generation(void)
{
    KartWheelContactState state = {0};
    FlatGroundContext context = {0};
    const KartWheelQueryInput input = {
        .position = {0.0f, 0.0f, 0.0f},
        .body_right = {1.0f, 0.0f, 0.0f},
        .body_forward = {0.0f, -1.0f, 0.0f},
        .body_up = {0.0f, 0.0f, 1.0f},
        .geometry = {1.0f, 1.0f, 0.5f, 1.0f},
        .query = query_flat_ground,
        .user_data = &context,
    };
    KartWheelQueryOutput out = kart_query_wheel_contacts(&state, &input);
    unsigned int i;

    assert(context.queries == 4);
    assert(out.grounded);
    assert(out.landed_this_step);
    assert(out.active_contacts == 4);
    assert(out.surface_id == 7);
    assert(near(out.average_normal.z, 1.0f, 0.0001f));
    for (i = 0; i < 4; ++i) {
        assert(near(out.contacts[i].compression, 0.5f, 0.0001f));
        assert(near(out.contacts[i].compression_delta, 0.5f, 0.0001f));
    }

    out = kart_query_wheel_contacts(&state, &input);
    assert(!out.landed_this_step);
    for (i = 0; i < 4; ++i) {
        assert(near(out.contacts[i].compression_delta, 0.0f, 0.0001f));
    }
}

static void test_fixed_step_simulation(void)
{
    KartSimulationState state;
    FlatGroundContext context = {0};
    const KartSimulationWorld world = {
        .query_ground = query_flat_ground,
        .user_data = &context,
    };
    KartSimulationStepResult result;

    kart_simulation_init(&state, NULL, NULL);
    result = kart_simulate_milliseconds(&state, NULL, &world, 12);
    assert(result.substeps == 3);
    assert(result.wheel_contacts == 12);
    assert(result.grounded);
    assert(result.landed);
    assert(context.queries == 12);
    assert(near(state.position.z, 0.0f, 0.0001f));
    assert(near(state.linear_velocity.z, 0.0f, 0.0001f));

    {
        const KartSimulationControls controls = {.forward_input = 1.0f};
        result = kart_simulate_milliseconds(&state, &controls, &world, 100);
        assert(result.substeps == 20);
        assert(state.linear_velocity.y < -2.0f);
        assert(fabsf(state.linear_velocity.x) < 0.01f);
    }
}

static void test_integrated_instant_boost_input_edge(void)
{
    KartSimulationState state;
    FlatGroundContext context = {0};
    const KartSimulationWorld world = {
        .query_ground = query_flat_ground,
        .user_data = &context,
    };
    KartSimulationControls controls = {0};

    kart_simulation_init(&state, NULL, NULL);
    state.linear_velocity.y = -20.0f;
    state.drift.entry_was_forward = true;
    state.drift.slip_detected = true;
    kart_simulate_milliseconds(&state, &controls, &world, 5);
    assert(near(state.instant_boost.opportunity_timer, 0.5f, 0.0001f));
    assert(!state.instant_boost.active);

    kart_simulate_milliseconds(&state, &controls, &world, 5);
    assert(near(state.instant_boost.opportunity_timer, 0.495f, 0.0001f));
    controls.forward_input = 1.0f;
    kart_simulate_milliseconds(&state, &controls, &world, 1);
    assert(state.instant_boost.active);
    assert(near(state.instant_boost.active_timer, 0.499f, 0.0001f));
    assert(near(state.instant_boost.opportunity_timer, 0.0f, 0.0001f));
}

static void test_integrated_timed_boost_lockout(void)
{
    KartSimulationState state;
    FlatGroundContext context = {0};
    const KartSimulationWorld world = {
        .query_ground = query_flat_ground,
        .user_data = &context,
    };
    KartSimulationControls controls = {
        .forward_input = 1.0f,
        .boost_active = true,
    };

    kart_simulation_init(&state, NULL, NULL);
    kart_simulate_milliseconds(&state, &controls, &world, 1);
    assert(state.timed_boost.active);
    assert(state.timed_boost.remaining_ms == 2999);

    controls.boost_active = false;
    kart_simulate_milliseconds(&state, &controls, &world, 1000);
    assert(state.timed_boost.remaining_ms == 1999);
    controls.boost_active = true;
    kart_simulate_milliseconds(&state, &controls, &world, 1);
    assert(state.timed_boost.remaining_ms == 1998);

    kart_simulate_milliseconds(&state, &controls, &world, 1998);
    assert(!state.timed_boost.active);
    assert(state.timed_boost.remaining_ms == 0);
    kart_simulate_milliseconds(&state, &controls, &world, 10);
    assert(!state.timed_boost.active);

    controls.boost_active = false;
    kart_simulate_milliseconds(&state, &controls, &world, 1);
    controls.boost_active = true;
    kart_simulate_milliseconds(&state, &controls, &world, 1);
    assert(state.timed_boost.active);
    assert(state.timed_boost.remaining_ms == 2999);
}

static void test_reverse_input_boost_cutoff_model(void)
{
    KartSimulationState state;
    FlatGroundContext context = {0};
    const KartSimulationWorld world = {
        .query_ground = query_flat_ground,
        .user_data = &context,
    };
    KartSimulationControls controls = {0};

    kart_simulation_init(&state, NULL, NULL);
    state.reverse_input_ends_boost = true;
    state.timed_boost = (KartTimedBoostState){.remaining_ms = 3000, .active = true};
    state.instant_boost.active = true;
    state.instant_boost.active_timer = 0.5f;

    kart_simulate_milliseconds(&state, &controls, &world, 1);
    assert(state.timed_boost.active);
    assert(state.instant_boost.active);

    controls.reverse_input = 1.0f;
    kart_simulate_milliseconds(&state, &controls, &world, 1);
    assert(!state.timed_boost.active);
    assert(!state.instant_boost.active);
}

typedef struct CollisionContext {
    unsigned int calls;
} CollisionContext;

static unsigned int query_one_wall_collision(
    void *user_data,
    const struct KartSimulationState *state,
    KartBodyContact *contacts,
    unsigned int capacity)
{
    CollisionContext *context = (CollisionContext *)user_data;
    (void)state;
    context->calls += 1;
    if (context->calls != 1 || capacity == 0) {
        return 0;
    }
    contacts[0].normal = (KartVec3){1.0f, 0.0f, 0.0f};
    contacts[0].sweep_fraction = 0.5f;
    contacts[0].surface_id = 3;
    return 1;
}

static void test_airborne_and_integrated_collision(void)
{
    KartSimulationState state;
    CollisionContext context = {0};
    const KartSimulationWorld world = {
        .query_body_collisions = query_one_wall_collision,
        .user_data = &context,
    };
    KartSimulationStepResult result;

    kart_simulation_init(&state, NULL, NULL);
    state.linear_velocity.x = -10.0f;
    result = kart_simulate_milliseconds(&state, NULL, &world, 1);
    assert(result.substeps == 1);
    assert(!result.grounded);
    assert(result.body_contacts == 1);
    assert(state.linear_velocity.x > 4.9f);
    assert(state.linear_velocity.z < 0.0f);

    kart_simulation_init(&state, NULL, NULL);
    result = kart_simulate_milliseconds(&state, NULL, NULL, 5);
    assert(!result.grounded);
    assert(near(state.linear_velocity.z, -0.294f, 0.0001f));
}

static void test_event_ordered_new_cut_steering(void)
{
    KartSteeringInputState input = {0};

    assert(kart_steering_key_event(
        &input, KART_STEERING_LEFT, true));
    assert(near(input.value, -1.0f, 0.0f));

    /* A later opposite press owns steering; both held must not sum to zero. */
    assert(kart_steering_key_event(
        &input, KART_STEERING_RIGHT, true));
    assert(input.left_down && input.right_down);
    assert(near(input.value, 1.0f, 0.0f));

    /* Releasing the older direction leaves the newer owner untouched. */
    assert(kart_steering_key_event(
        &input, KART_STEERING_LEFT, false));
    assert(near(input.value, 1.0f, 0.0f));
    assert(kart_steering_key_event(
        &input, KART_STEERING_RIGHT, false));
    assert(near(input.value, 0.0f, 0.0f));

    /* Key-repeat messages are not new transitions. */
    assert(kart_steering_key_event(
        &input, KART_STEERING_RIGHT, true));
    assert(!kart_steering_key_event(
        &input, KART_STEERING_RIGHT, true));
    kart_steering_input_reset(&input);
    assert(!input.left_down && !input.right_down);
    assert(near(input.value, 0.0f, 0.0f));
}

static void test_integrated_drift_input_edge(void)
{
    KartSimulationState state;
    FlatGroundContext context = {0};
    const KartSimulationWorld world = {
        .query_ground = query_flat_ground,
        .user_data = &context,
    };
    KartSimulationControls controls = {
        .forward_input = 1.0f,
        .steering_input = -1.0f,
        .drift_input = true,
    };

    kart_simulation_init(&state, NULL, NULL);
    state.linear_velocity.y = -20.0f;

    /* One held key produces one trigger. It must not restart after the
       default 0.2 second linger lockout expires. */
    kart_simulate_milliseconds(&state, &controls, &world, 500);
    assert(state.drift.input_active);
    assert(!state.drift.trigger_active);
    assert(near(state.drift.trigger_timer, 0.0f, 0.0001f));
    assert(near(state.drift.linger_timer, 0.0f, 0.0001f));
    kart_simulate_milliseconds(&state, &controls, &world, 500);
    assert(!state.drift.trigger_active);
    assert(near(state.drift.trigger_timer, 0.0f, 0.0001f));

    controls.drift_input = false;
    kart_simulate_milliseconds(&state, &controls, &world, 5);
    assert(!state.drift.input_active);
    controls.drift_input = true;
    kart_simulate_milliseconds(&state, &controls, &world, 5);
    assert(state.drift.input_active);
    assert(state.drift.trigger_active);
}

int main(void)
{
    test_defaults();
    test_steering_attenuation();
    test_symmetric_grip();
    test_original_low_speed_lateral_branch();
    test_corner_draw_force();
    test_speedometer_and_demo_assets();
    test_demo_track_table_is_consistent();
    test_runtime_grounded_drag_scale();
    test_static_suspension_equilibrium();
    test_drift_trigger_timing();
    test_drift_slip_detection();
    test_instant_boost_state_machine();
    test_stored_instant_boost_state_machine();
    test_timed_boost_state_machine();
    test_drag_and_linear_integration();
    test_collision_response();
    test_drive_and_brake_forces();
    test_angular_integration();
    test_longitudinal_state_machine();
    test_pose_integration_and_tilt_guard();
    test_wheel_contact_generation();
    test_fixed_step_simulation();
    test_integrated_instant_boost_input_edge();
    test_integrated_timed_boost_lockout();
    test_reverse_input_boost_cutoff_model();
    test_airborne_and_integrated_collision();
    test_event_ordered_new_cut_steering();
    test_integrated_drift_input_edge();
    puts("kart_dynamics_tests: ok");
    return 0;
}
