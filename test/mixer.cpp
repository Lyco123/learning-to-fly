#include <learning_to_fly/simulator/mixer.h>

#include <gtest/gtest.h>

TEST(LEARNING_TO_FLY_MIXER, PURE_THRUST_BALANCES_ALL_MOTORS) {
    using T = float;
    rl_tools::rl::environments::multirotor::MixerParameters<T> params{
        0.028f,
        0.005964552f,
        3.16e-10f,
        0.027f,
        0.0f,
        21702.0f,
    };

    const T torque[3] = {0.0f, 0.0f, 0.0f};
    T motors[4] = {0, 0, 0, 0};
    rl_tools::rl::environments::multirotor::mixer(params, torque, 9.81f, motors);

    EXPECT_NEAR(motors[0], motors[1], 1e-6f);
    EXPECT_NEAR(motors[1], motors[2], 1e-6f);
    EXPECT_NEAR(motors[2], motors[3], 1e-6f);
}

TEST(LEARNING_TO_FLY_MIXER, PURE_ROLL_TORQUE_CHANGES_LEFT_RIGHT_PAIR) {
    using T = float;
    rl_tools::rl::environments::multirotor::MixerParameters<T> params{0.028f, 0.005964552f, 3.16e-10f, 0.027f, 0.0f, 21702.0f};

    const T torque[3] = {0.0002f, 0.0f, 0.0f};
    T thrust[4] = {0, 0, 0, 0};
    T motors[4] = {0, 0, 0, 0};
    rl_tools::rl::environments::multirotor::mixer(params, torque, 9.81f, motors, thrust);

    EXPECT_LT(thrust[0], thrust[2]);
    EXPECT_LT(thrust[1], thrust[3]);
}

TEST(LEARNING_TO_FLY_MIXER, PURE_PITCH_TORQUE_CHANGES_FRONT_BACK_PAIR) {
    using T = float;
    rl_tools::rl::environments::multirotor::MixerParameters<T> params{0.028f, 0.005964552f, 3.16e-10f, 0.027f, 0.0f, 21702.0f};

    const T torque[3] = {0.0f, 0.0002f, 0.0f};
    T thrust[4] = {0, 0, 0, 0};
    T motors[4] = {0, 0, 0, 0};
    rl_tools::rl::environments::multirotor::mixer(params, torque, 9.81f, motors, thrust);

    EXPECT_LT(thrust[0], thrust[1]);
    EXPECT_LT(thrust[3], thrust[2]);
}

TEST(LEARNING_TO_FLY_MIXER, PURE_YAW_TORQUE_SPLITS_CCW_CW_MOTORS) {
    using T = float;
    rl_tools::rl::environments::multirotor::MixerParameters<T> params{0.028f, 0.005964552f, 3.16e-10f, 0.027f, 0.0f, 21702.0f};

    const T torque[3] = {0.0f, 0.0f, 0.00002f};
    T thrust[4] = {0, 0, 0, 0};
    T motors[4] = {0, 0, 0, 0};
    rl_tools::rl::environments::multirotor::mixer(params, torque, 9.81f, motors, thrust);

    EXPECT_LT(thrust[0], thrust[1]);
    EXPECT_LT(thrust[2], thrust[3]);
}

TEST(LEARNING_TO_FLY_MIXER, OUTPUT_STAYS_NORMALIZED) {
    using T = float;
    rl_tools::rl::environments::multirotor::MixerParameters<T> params{0.028f, 0.005964552f, 3.16e-10f, 0.027f, 0.0f, 21702.0f};

    const T torque[3] = {1.0f, -1.0f, 1.0f};
    T motors[4] = {0, 0, 0, 0};
    rl_tools::rl::environments::multirotor::mixer(params, torque, 100.0f, motors);

    for(int rotor = 0; rotor < 4; rotor++){
        EXPECT_LE(motors[rotor], 1.0f);
        EXPECT_GE(motors[rotor], -1.0f);
    }
}
