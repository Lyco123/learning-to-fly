#include <learning_to_fly/simulator/rate_controller.h>

#include <gtest/gtest.h>

TEST(LEARNING_TO_FLY_RATE_CONTROLLER, ZERO_ERROR_ZERO_TORQUE) {
    using T = float;

    rl_tools::rl::environments::multirotor::RateControllerParameters<T> params{
        {1.0f, 1.0f, 0.8f},
        {0.2f, 0.2f, 0.1f}
    };

    T cmd[3] = {0.1f, -0.2f, 0.3f};
    T meas[3] = {0.1f, -0.2f, 0.3f};
    T torque[3] = {0, 0, 0};

    rl_tools::rl::environments::multirotor::rate_controller(params, cmd, meas, torque);

    EXPECT_NEAR(torque[0], 0.0f, 1e-7f);
    EXPECT_NEAR(torque[1], 0.0f, 1e-7f);
    EXPECT_NEAR(torque[2], 0.0f, 1e-7f);
}

TEST(LEARNING_TO_FLY_RATE_CONTROLLER, POSITIVE_ROLL_ERROR_POSITIVE_TORQUE) {
    using T = float;

    rl_tools::rl::environments::multirotor::RateControllerParameters<T> params{
        {1.2f, 1.2f, 0.7f},
        {0.5f, 0.5f, 0.2f}
    };

    T cmd[3] = {0.4f, 0.0f, 0.0f};
    T meas[3] = {0.1f, 0.0f, 0.0f};
    T torque[3] = {0, 0, 0};

    rl_tools::rl::environments::multirotor::rate_controller(params, cmd, meas, torque);

    EXPECT_GT(torque[0], 0.0f);
}

TEST(LEARNING_TO_FLY_RATE_CONTROLLER, AXIS_INDEPENDENCE) {
    using T = float;

    rl_tools::rl::environments::multirotor::RateControllerParameters<T> params{
        {1.0f, 1.0f, 1.0f},
        {1.0f, 1.0f, 1.0f}
    };

    T cmd[3] = {0.3f, 0.0f, 0.0f};
    T meas[3] = {0.0f, 0.0f, 0.0f};
    T torque[3] = {0, 0, 0};

    rl_tools::rl::environments::multirotor::rate_controller(params, cmd, meas, torque);

    EXPECT_NEAR(torque[1], 0.0f, 1e-7f);
    EXPECT_NEAR(torque[2], 0.0f, 1e-7f);
}

TEST(LEARNING_TO_FLY_RATE_CONTROLLER, TORQUE_SATURATION) {
    using T = float;

    rl_tools::rl::environments::multirotor::RateControllerParameters<T> params{
        {10.0f, 10.0f, 10.0f},
        {0.15f, 0.15f, 0.05f}
    };

    T cmd[3] = {2.0f, -3.0f, 1.0f};
    T meas[3] = {0.0f, 0.0f, 0.0f};
    T torque[3] = {0, 0, 0};

    rl_tools::rl::environments::multirotor::rate_controller(params, cmd, meas, torque);

    EXPECT_NEAR(torque[0], 0.15f, 1e-7f);
    EXPECT_NEAR(torque[1], -0.15f, 1e-7f);
    EXPECT_NEAR(torque[2], 0.05f, 1e-7f);
}
