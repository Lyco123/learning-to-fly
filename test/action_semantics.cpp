#include <rl_tools/operations/cpu.h>
#include <learning_to_fly/simulator/operations_cpu.h>
#include <learning_to_fly/simulator/rate_controller.h>
#include <learning_to_fly/simulator/mixer.h>

#include <gtest/gtest.h>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER::rl_tools;

TEST(LEARNING_TO_FLY_ACTION, CTBR_DIM_AND_ORDER) {
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;

    constexpr T max_rate_command = 12.0f;
    constexpr T max_thrust_acceleration = 15.0f;
    T action_ctbr[4] = {0.6f, -0.2f, 0.4f, 0.1f}; // [wx, wy, wz, thrust_acc]

    T commanded_rate[3] = {
        action_ctbr[0] * max_rate_command,
        action_ctbr[1] * max_rate_command,
        action_ctbr[2] * max_rate_command,
    };
    T measured_rate[3] = {0.0f, 0.0f, 0.0f};
    T torque[3] = {0.0f, 0.0f, 0.0f};

    const rlt::rl::environments::multirotor::RateControllerParameters<T> rate_params = {
        {0.08f, 0.08f, 0.04f},
        {0.0006f, 0.0006f, 0.0002f}
    };
    rlt::rl::environments::multirotor::rate_controller(rate_params, commanded_rate, measured_rate, torque);

    const rlt::rl::environments::multirotor::MixerParameters<T> mixer_params = {
        0.028f,
        0.005964552f,
        3.16e-10f,
        0.027f,
        0.0f,
        21702.0f
    };
    T motors[4] = {0, 0, 0, 0};
    const T thrust_acc_cmd = (action_ctbr[3] + 1.0f) * 0.5f * max_thrust_acceleration;
    rlt::rl::environments::multirotor::mixer(mixer_params, torque, thrust_acc_cmd, motors);

    for(int i = 0; i < 4; i++) {
        EXPECT_LE(motors[i], 1.0f);
        EXPECT_GE(motors[i], -1.0f);
    }

    // thrust channel influences all motors in same direction when torques are zero
    T action_thrust_only[4] = {0.0f, 0.0f, 0.0f, 0.5f};
    T thrust_only_torque[3] = {0.0f, 0.0f, 0.0f};
    T motors_thrust_only[4] = {0, 0, 0, 0};
    const T thrust_only_acc = (action_thrust_only[3] + 1.0f) * 0.5f * max_thrust_acceleration;
    rlt::rl::environments::multirotor::mixer(mixer_params, thrust_only_torque, thrust_only_acc, motors_thrust_only);
    EXPECT_NEAR(motors_thrust_only[0], motors_thrust_only[1], 1e-6f);
    EXPECT_NEAR(motors_thrust_only[1], motors_thrust_only[2], 1e-6f);
    EXPECT_NEAR(motors_thrust_only[2], motors_thrust_only[3], 1e-6f);

    // roll channel produces opposite deltas across rotor pairs
    T action_roll_only[4] = {0.4f, 0.0f, 0.0f, 0.0f};
    T roll_cmd_rate[3] = {action_roll_only[0] * max_rate_command, 0.0f, 0.0f};
    T roll_torque[3] = {0.0f, 0.0f, 0.0f};
    rlt::rl::environments::multirotor::rate_controller(rate_params, roll_cmd_rate, measured_rate, roll_torque);
    T motors_roll_only[4] = {0, 0, 0, 0};
    const T roll_only_acc = (action_roll_only[3] + 1.0f) * 0.5f * max_thrust_acceleration;
    rlt::rl::environments::multirotor::mixer(mixer_params, roll_torque, roll_only_acc, motors_roll_only);
    EXPECT_NE(motors_roll_only[0], motors_roll_only[2]);
    EXPECT_NE(motors_roll_only[1], motors_roll_only[3]);
}
