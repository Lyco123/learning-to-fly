// Tests for the CTBR (Collective Thrust + Body Rates) control pipeline.
//
// Covers:
//   1. Rate controller: angular-rate error to torque output
//   2. Mixer: torque + thrust to normalized motor commands (Crazyflie X layout)
//   3. End-to-end multirotor step with CTBR interface
//   4. Observation / privileged-observation dimensions and packing

#include <rl_tools/operations/cpu.h>
#include <learning_to_fly/simulator/multirotor.h>
#include <learning_to_fly/simulator/operations_cpu.h>
#include <learning_to_fly/simulator/rate_controller.h>
#include <learning_to_fly/simulator/mixer.h>
#include "config/parameters.h"

#include <gtest/gtest.h>
#include <cmath>
#include <iostream>

namespace bpt = RL_TOOLS_NAMESPACE_WRAPPER::rl_tools;

using DEVICE = bpt::devices::DefaultCPU;
using T      = double;
using TI     = DEVICE::index_t;

// Physical constants for the Crazyflie (matches crazy_flie.h)
static constexpr T ARM      = 0.028;
static constexpr T KM       = 0.005964552;
static constexpr T K_THRUST = 3.16e-10;
static constexpr T MAX_RPM  = 21702.0;
static constexpr T MIN_RPM  = 0.0;
static constexpr T MASS     = 0.027;

// Use the full CTBR parameter set from the training configuration
using ENV_BUILDER = parameters::builder::environment<T, TI, parameters::DefaultAblationSpec>;
using ENVIRONMENT = ENV_BUILDER::ENVIRONMENT;
using ENV_PARAMS  = ENV_BUILDER::PARAMETERS;
static const ENV_PARAMS& get_env_params() { return ENV_BUILDER::parameters; }

// ─────────────────────────────────────────────
// 1. Rate Controller Tests
// ─────────────────────────────────────────────

TEST(CTBR_RateController, ZeroError_ZeroTorque) {
    bpt::rl::environments::multirotor::RateControllerParameters<T> p{0.001, 0.0005};
    T des[3]  = {1.0, 2.0, 3.0};
    T meas[3] = {1.0, 2.0, 3.0};
    T torque[3];
    bpt::rl::environments::multirotor::rate_controller(p, des, meas, torque);
    EXPECT_DOUBLE_EQ(torque[0], 0.0);
    EXPECT_DOUBLE_EQ(torque[1], 0.0);
    EXPECT_DOUBLE_EQ(torque[2], 0.0);
    std::cout << "[rate_ctrl] zero error -> torques = [0, 0, 0]  OK\n";
}

TEST(CTBR_RateController, ProportionalGain_XY) {
    bpt::rl::environments::multirotor::RateControllerParameters<T> p{0.002, 0.001};
    T des[3]  = {5.0, -3.0, 0.0};
    T meas[3] = {0.0,  0.0, 0.0};
    T torque[3];
    bpt::rl::environments::multirotor::rate_controller(p, des, meas, torque);
    EXPECT_NEAR(torque[0],  0.010, 1e-10); // 0.002 * 5.0
    EXPECT_NEAR(torque[1], -0.006, 1e-10); // 0.002 * (-3.0)
    EXPECT_DOUBLE_EQ(torque[2], 0.0);
    std::cout << "[rate_ctrl] xy P-gain: tau=[" << torque[0] << "," << torque[1] << "," << torque[2] << "]  OK\n";
}

TEST(CTBR_RateController, ProportionalGain_Z) {
    bpt::rl::environments::multirotor::RateControllerParameters<T> p{0.002, 0.001};
    T des[3]  = {0.0, 0.0,  4.0};
    T meas[3] = {0.0, 0.0, -1.0};
    T torque[3];
    bpt::rl::environments::multirotor::rate_controller(p, des, meas, torque);
    EXPECT_DOUBLE_EQ(torque[0], 0.0);
    EXPECT_DOUBLE_EQ(torque[1], 0.0);
    EXPECT_NEAR(torque[2], 0.005, 1e-10); // 0.001 * 5.0
    std::cout << "[rate_ctrl] z P-gain: tau_z=" << torque[2] << "  OK\n";
}

// ─────────────────────────────────────────────
// 2. Mixer Tests
// ─────────────────────────────────────────────

TEST(CTBR_Mixer, HoverThrust_AllMotorsEqual) {
    DEVICE device;
    T thrust_acc = 9.81;
    T torque_cmd[3] = {0.0, 0.0, 0.0};
    T cmds[4];
    bpt::rl::environments::multirotor::mixer(device, get_env_params(), thrust_acc, torque_cmd, cmds);

    EXPECT_NEAR(cmds[0], cmds[1], 1e-9);
    EXPECT_NEAR(cmds[1], cmds[2], 1e-9);
    EXPECT_NEAR(cmds[2], cmds[3], 1e-9);

    T F_per   = MASS * 9.81 / 4.0;
    T rpm_exp = std::sqrt(F_per / K_THRUST);
    T cmd_exp = (rpm_exp - (MIN_RPM + MAX_RPM) / 2.0) / ((MAX_RPM - MIN_RPM) / 2.0);
    EXPECT_NEAR(cmds[0], cmd_exp, 1e-6);
    std::cout << "[mixer] hover: cmd=" << cmds[0] << " (expected " << cmd_exp << ")  OK\n";
}

TEST(CTBR_Mixer, PositiveRollTorque_LeftMotorsHigher) {
    // Positive tau_x: left motors (2,3) should spin faster than right motors (0,1).
    // Motor layout: 0=front-right, 1=back-right, 2=back-left, 3=front-left
    DEVICE device;
    T thrust_acc = 9.81;
    T torque_cmd[3] = {0.001, 0.0, 0.0};
    T cmds[4];
    bpt::rl::environments::multirotor::mixer(device, get_env_params(), thrust_acc, torque_cmd, cmds);

    T hover = (cmds[0] + cmds[1] + cmds[2] + cmds[3]) / 4.0;
    EXPECT_GT(cmds[2], hover) << "back-left  M3 should be above hover";
    EXPECT_GT(cmds[3], hover) << "front-left M4 should be above hover";
    EXPECT_LT(cmds[0], hover) << "front-right M1 should be below hover";
    EXPECT_LT(cmds[1], hover) << "back-right  M2 should be below hover";
    std::cout << "[mixer] +tau_x: cmds=[" << cmds[0] << "," << cmds[1] << "," << cmds[2] << "," << cmds[3] << "]  OK\n";
}

TEST(CTBR_Mixer, PositivePitchTorque_BackMotorsHigher) {
    // Positive tau_y: back motors (1,2) should spin faster than front motors (0,3).
    DEVICE device;
    T thrust_acc = 9.81;
    T torque_cmd[3] = {0.0, 0.001, 0.0};
    T cmds[4];
    bpt::rl::environments::multirotor::mixer(device, get_env_params(), thrust_acc, torque_cmd, cmds);

    T hover = (cmds[0] + cmds[1] + cmds[2] + cmds[3]) / 4.0;
    EXPECT_GT(cmds[1], hover) << "back-right M2 should be above hover";
    EXPECT_GT(cmds[2], hover) << "back-left  M3 should be above hover";
    EXPECT_LT(cmds[0], hover) << "front-right M1 should be below hover";
    EXPECT_LT(cmds[3], hover) << "front-left  M4 should be below hover";
    std::cout << "[mixer] +tau_y: cmds=[" << cmds[0] << "," << cmds[1] << "," << cmds[2] << "," << cmds[3] << "]  OK\n";
}

TEST(CTBR_Mixer, PositiveYawTorque_CWMotorsHigher) {
    // Positive tau_z: CW motors (1=back-right, 3=front-left) should dominate.
    DEVICE device;
    T thrust_acc = 9.81;
    T torque_cmd[3] = {0.0, 0.0, 0.001};
    T cmds[4];
    bpt::rl::environments::multirotor::mixer(device, get_env_params(), thrust_acc, torque_cmd, cmds);

    T hover = (cmds[0] + cmds[1] + cmds[2] + cmds[3]) / 4.0;
    EXPECT_GT(cmds[1], hover) << "back-right CW  M2 should be above hover";
    EXPECT_GT(cmds[3], hover) << "front-left CW  M4 should be above hover";
    EXPECT_LT(cmds[0], hover) << "front-right CCW M1 should be below hover";
    EXPECT_LT(cmds[2], hover) << "back-left  CCW M3 should be below hover";
    std::cout << "[mixer] +tau_z: cmds=[" << cmds[0] << "," << cmds[1] << "," << cmds[2] << "," << cmds[3] << "]  OK\n";
}

TEST(CTBR_Mixer, RoundTrip_ThrustToMotorAndBack) {
    // Verify mixer inverse: pass result through forward model and recover wrench.
    DEVICE device;
    T thrust_acc = 15.0;
    T torque_in[3] = {0.0005, -0.0003, 0.0002};
    T cmds[4];
    bpt::rl::environments::multirotor::mixer(device, get_env_params(), thrust_acc, torque_in, cmds);

    T half_range = (MAX_RPM - MIN_RPM) / 2.0;
    T center     = MIN_RPM + half_range;
    T F_total = 0, tau_x = 0, tau_y = 0, tau_z = 0;
    T ry[4] = {-ARM, -ARM, +ARM, +ARM};   // y-position per motor
    T rx[4] = {+ARM, -ARM, -ARM, +ARM};   // x-position per motor
    T tz[4] = {-1.0, +1.0, -1.0, +1.0};  // torque dir z (CCW=-1, CW=+1)
    for(int i = 0; i < 4; i++){
        T rpm = cmds[i] * half_range + center;
        T Ti  = K_THRUST * rpm * rpm;
        F_total += Ti;
        tau_x   += ry[i] * Ti;
        tau_y   += -rx[i] * Ti;
        tau_z   += tz[i] * KM * Ti;
    }

    EXPECT_NEAR(F_total, thrust_acc * MASS, 1e-6);
    EXPECT_NEAR(tau_x,   torque_in[0],       1e-8);
    EXPECT_NEAR(tau_y,   torque_in[1],       1e-8);
    EXPECT_NEAR(tau_z,   torque_in[2],       1e-8);
    std::cout << "[mixer] round-trip: F=" << F_total
              << " tau=[" << tau_x << "," << tau_y << "," << tau_z << "]  OK\n";
}

// ─────────────────────────────────────────────
// 3. End-to-end CTBR step
// ─────────────────────────────────────────────

TEST(CTBR_EndToEnd, HoverCommand_StaysNearHover) {
    using STATE = ENVIRONMENT::State;
    DEVICE device;
    ENVIRONMENT env{ENV_BUILDER::parameters};
    auto rng = bpt::random::default_engine(DEVICE::SPEC::RANDOM{}, 42);

    STATE state;
    bpt::initial_state(device, env, state);

    bpt::MatrixDynamic<bpt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> action;
    bpt::malloc(device, action);

    T hover_norm = (9.81 / ENV_BUILDER::parameters.ctbr.max_thrust_acc) * 2.0 - 1.0;
    bpt::set(action, 0, 0, 0.0);
    bpt::set(action, 0, 1, 0.0);
    bpt::set(action, 0, 2, 0.0);
    bpt::set(action, 0, 3, hover_norm);

    STATE next_state;
    bpt::step(device, env, state, action, next_state, rng);

    for(int i = 0; i < 3; i++){
        EXPECT_NEAR(next_state.position[i], state.position[i], 0.05)
            << "Position[" << i << "] changed too much in one step";
    }
    std::cout << "[e2e] hover step: delta_pos=["
              << next_state.position[0]-state.position[0] << ","
              << next_state.position[1]-state.position[1] << ","
              << next_state.position[2]-state.position[2] << "]  OK\n";
    bpt::free(device, action);
}

TEST(CTBR_EndToEnd, RollCommand_ChangesAngularVelocity) {
    using STATE = ENVIRONMENT::State;
    DEVICE device;
    ENVIRONMENT env{ENV_BUILDER::parameters};
    auto rng = bpt::random::default_engine(DEVICE::SPEC::RANDOM{}, 43);

    STATE state;
    bpt::initial_state(device, env, state);

    bpt::MatrixDynamic<bpt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> action;
    bpt::malloc(device, action);

    T hover_norm = (9.81 / ENV_BUILDER::parameters.ctbr.max_thrust_acc) * 2.0 - 1.0;
    bpt::set(action, 0, 0, 1.0);  // max positive roll rate
    bpt::set(action, 0, 1, 0.0);
    bpt::set(action, 0, 2, 0.0);
    bpt::set(action, 0, 3, hover_norm);

    STATE next_state;
    for(int i = 0; i < 10; i++){
        bpt::step(device, env, state, action, next_state, rng);
        state = next_state;
    }

    EXPECT_GT(state.angular_velocity[0], 0.0)
        << "Positive roll-rate command should increase omega_x";
    std::cout << "[e2e] roll cmd: omega_x after 10 steps = "
              << state.angular_velocity[0] << " rad/s  OK\n";
    bpt::free(device, action);
}

TEST(CTBR_EndToEnd, ThrustAboveHover_ClimbsUp) {
    using STATE = ENVIRONMENT::State;
    DEVICE device;
    ENVIRONMENT env{ENV_BUILDER::parameters};
    auto rng = bpt::random::default_engine(DEVICE::SPEC::RANDOM{}, 44);

    STATE state;
    bpt::initial_state(device, env, state);

    bpt::MatrixDynamic<bpt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> action;
    bpt::malloc(device, action);

    bpt::set(action, 0, 0, 0.0);
    bpt::set(action, 0, 1, 0.0);
    bpt::set(action, 0, 2, 0.0);
    bpt::set(action, 0, 3, 1.0);  // max thrust

    T z0 = state.position[2];
    STATE next_state;
    for(int i = 0; i < 20; i++){
        bpt::step(device, env, state, action, next_state, rng);
        state = next_state;
    }

    EXPECT_GT(state.position[2], z0)
        << "Full thrust should produce net upward acceleration";
    std::cout << "[e2e] full thrust: z=" << state.position[2] << " (was " << z0 << ")  OK\n";
    bpt::free(device, action);
}

// ─────────────────────────────────────────────
// 4. Observation dimension and packing tests
// ─────────────────────────────────────────────

TEST(CTBR_Observations, ActorObservationDimension) {
    // Actor: position(3) + rotation_matrix(9) + linear_velocity(3) = 15
    constexpr TI EXPECTED = 3 + 9 + 3; // 15
    EXPECT_EQ(ENVIRONMENT::OBSERVATION_DIM, EXPECTED)
        << "Actor obs dim should be " << EXPECTED;
    std::cout << "[obs] OBSERVATION_DIM = " << ENVIRONMENT::OBSERVATION_DIM << "  OK\n";
}

TEST(CTBR_Observations, PrivilegedObservationDimension) {
    // Privileged: position(3) + rot_mat(9) + lin_vel(3) + rand_force(6) + ctbr_cmd(4) = 25
    constexpr TI EXPECTED = 3 + 9 + 3 + 6 + 4; // 25
    EXPECT_EQ(ENVIRONMENT::OBSERVATION_DIM_PRIVILEGED, EXPECTED)
        << "Privileged obs dim should be " << EXPECTED;
    std::cout << "[obs] OBSERVATION_DIM_PRIVILEGED = " << ENVIRONMENT::OBSERVATION_DIM_PRIVILEGED << "  OK\n";
}

TEST(CTBR_Observations, ObservePacking) {
    using STATE = ENVIRONMENT::State;
    DEVICE device;
    ENVIRONMENT env{ENV_BUILDER::parameters};
    auto rng = bpt::random::default_engine(DEVICE::SPEC::RANDOM{}, 10);

    STATE state;
    bpt::initial_state(device, env, state);

    bpt::MatrixDynamic<bpt::matrix::Specification<T, TI, 1, ENVIRONMENT::OBSERVATION_DIM>> obs;
    bpt::malloc(device, obs);
    bpt::observe(device, env, state, obs, rng);

    // First 3 elements should contain position (with small noise possible)
    EXPECT_NEAR(bpt::get(obs, 0, 0), state.position[0], 0.01);
    EXPECT_NEAR(bpt::get(obs, 0, 1), state.position[1], 0.01);
    EXPECT_NEAR(bpt::get(obs, 0, 2), state.position[2], 0.01);
    std::cout << "[obs] observe() packing check  OK\n";
    bpt::free(device, obs);
}

TEST(CTBR_Observations, PrivilegedObservePacking) {
    // After initial_state the action_history is all zeros, so the
    // last 4 elements of the privileged obs (CTBRCommand) must be 0.
    using STATE = ENVIRONMENT::State;
    DEVICE device;
    ENVIRONMENT env{ENV_BUILDER::parameters};
    auto rng = bpt::random::default_engine(DEVICE::SPEC::RANDOM{}, 11);

    STATE state;
    bpt::initial_state(device, env, state);

    bpt::MatrixDynamic<bpt::matrix::Specification<T, TI, 1, ENVIRONMENT::OBSERVATION_DIM_PRIVILEGED>> obs;
    bpt::malloc(device, obs);
    bpt::observe_privileged(device, env, state, obs, rng);

    constexpr TI PRIV_DIM = ENVIRONMENT::OBSERVATION_DIM_PRIVILEGED;
    for(TI i = PRIV_DIM - 4; i < PRIV_DIM; i++){
        EXPECT_NEAR(bpt::get(obs, 0, i), 0.0, 1e-9)
            << "Initial CTBR command at privileged obs index " << i << " should be 0";
    }
    std::cout << "[obs] observe_privileged() CTBR command check  OK\n";
    bpt::free(device, obs);
}
