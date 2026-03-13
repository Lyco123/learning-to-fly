#include <rl_tools/operations/cpu.h>
#include <learning_to_fly/simulator/operations_cpu.h>

#include "../src/config/parameters.h"

#include <gtest/gtest.h>

TEST(LEARNING_TO_FLY_CONTROL_CHAIN, POSITIVE_ROLL_RATE_INCREASES_ROLL_RATE) {
    using DEVICE = rl_tools::devices::DefaultCPU;
    using TI = typename DEVICE::index_t;
    using T = float;

    using ABLATION_SPEC = parameters::DefaultAblationSpec;
    using ENV_BUILDER = parameters::environment<T, TI, ABLATION_SPEC>;
    using ENVIRONMENT = typename parameters::environment<T, TI, ABLATION_SPEC>::ENVIRONMENT;

    DEVICE device;
    auto rng = rl_tools::random::default_engine(DEVICE::SPEC::RANDOM{}, 7);

    ENVIRONMENT env({ENV_BUILDER::parameters});
    typename ENVIRONMENT::State state{};
    rl_tools::initial_state(device, env, state);

    rl_tools::MatrixDynamic<rl_tools::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> action;
    rl_tools::malloc(device, action);

    set(action, 0, 0, 0.6f);  // +roll rate command
    set(action, 0, 1, 0.0f);
    set(action, 0, 2, 0.0f);
    set(action, 0, 3, 0.6f);  // positive thrust acceleration command

    typename ENVIRONMENT::State current_state = state;
    typename ENVIRONMENT::State next_state = state;
    for(int i = 0; i < 20; i++){
        rl_tools::step(device, env, current_state, action, next_state, rng);
        current_state = next_state;
    }

    EXPECT_GT(next_state.angular_velocity[0], 0.0f);

    rl_tools::free(device, action);
}

TEST(LEARNING_TO_FLY_CONTROL_CHAIN, POSITIVE_YAW_RATE_INCREASES_YAW_RATE) {
    using DEVICE = rl_tools::devices::DefaultCPU;
    using TI = typename DEVICE::index_t;
    using T = float;

    using ABLATION_SPEC = parameters::DefaultAblationSpec;
    using ENV_BUILDER = parameters::environment<T, TI, ABLATION_SPEC>;
    using ENVIRONMENT = typename parameters::environment<T, TI, ABLATION_SPEC>::ENVIRONMENT;

    DEVICE device;
    auto rng = rl_tools::random::default_engine(DEVICE::SPEC::RANDOM{}, 11);

    ENVIRONMENT env({ENV_BUILDER::parameters});
    typename ENVIRONMENT::State state{};
    rl_tools::initial_state(device, env, state);

    rl_tools::MatrixDynamic<rl_tools::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> action;
    rl_tools::malloc(device, action);

    set(action, 0, 0, 0.0f);
    set(action, 0, 1, 0.0f);
    set(action, 0, 2, 0.8f);  // +yaw rate command
    set(action, 0, 3, 0.6f);  // positive thrust acceleration command

    typename ENVIRONMENT::State current_state = state;
    typename ENVIRONMENT::State next_state = state;
    for(int i = 0; i < 20; i++){
        rl_tools::step(device, env, current_state, action, next_state, rng);
        current_state = next_state;
    }

    EXPECT_GT(next_state.angular_velocity[2], 0.0f);

    rl_tools::free(device, action);
}

TEST(LEARNING_TO_FLY_CONTROL_CHAIN, POSITIVE_PITCH_RATE_INCREASES_PITCH_RATE) {
    using DEVICE = rl_tools::devices::DefaultCPU;
    using TI = typename DEVICE::index_t;
    using T = float;

    using ABLATION_SPEC = parameters::DefaultAblationSpec;
    using ENV_BUILDER = parameters::environment<T, TI, ABLATION_SPEC>;
    using ENVIRONMENT = typename parameters::environment<T, TI, ABLATION_SPEC>::ENVIRONMENT;

    DEVICE device;
    auto rng = rl_tools::random::default_engine(DEVICE::SPEC::RANDOM{}, 13);

    ENVIRONMENT env({ENV_BUILDER::parameters});
    typename ENVIRONMENT::State state{};
    rl_tools::initial_state(device, env, state);

    rl_tools::MatrixDynamic<rl_tools::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> action;
    rl_tools::malloc(device, action);

    set(action, 0, 0, 0.0f);
    set(action, 0, 1, 0.7f);  // +pitch rate command
    set(action, 0, 2, 0.0f);
    set(action, 0, 3, 0.6f);  // positive thrust acceleration command

    typename ENVIRONMENT::State current_state = state;
    typename ENVIRONMENT::State next_state = state;
    for(int i = 0; i < 20; i++){
        rl_tools::step(device, env, current_state, action, next_state, rng);
        current_state = next_state;
    }

    EXPECT_GT(next_state.angular_velocity[1], 0.0f);

    rl_tools::free(device, action);
}
