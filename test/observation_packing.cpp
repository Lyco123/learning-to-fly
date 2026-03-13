#include <rl_tools/operations/cpu.h>
#include <learning_to_fly/simulator/operations_cpu.h>

#include "../src/config/parameters.h"

#include <gtest/gtest.h>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER::rl_tools;

TEST(LEARNING_TO_FLY_OBSERVATION, PACKING_STEP1_LAYOUT) {
    using DEVICE = rlt::devices::DefaultCPU;
    using TI = typename DEVICE::index_t;
    using T = float;

    using ABLATION_SPEC = parameters::DefaultAblationSpec;
    using ENVIRONMENT = typename parameters::environment<T, TI, ABLATION_SPEC>::ENVIRONMENT;

    static_assert(ENVIRONMENT::OBSERVATION_DIM == 15, "Actor observation must be position + orientation + linear velocity");
    static_assert(ENVIRONMENT::OBSERVATION_DIM_PRIVILEGED == 25, "Privileged observation must include actor obs + cmd placeholders + disturbances");

    DEVICE device;
    auto rng = rlt::random::default_engine(DEVICE::SPEC::RANDOM{}, 0);

    ENVIRONMENT env;
    env.parameters.mdp.observation_noise.position = 0;
    env.parameters.mdp.observation_noise.orientation = 0;
    env.parameters.mdp.observation_noise.linear_velocity = 0;
    env.parameters.mdp.observation_noise.angular_velocity = 0;

    typename ENVIRONMENT::State state{};
    state.position[0] = 1.0f;
    state.position[1] = -2.0f;
    state.position[2] = 3.0f;

    state.orientation[0] = 1.0f;
    state.orientation[1] = 0.0f;
    state.orientation[2] = 0.0f;
    state.orientation[3] = 0.0f;

    state.linear_velocity[0] = 0.1f;
    state.linear_velocity[1] = -0.2f;
    state.linear_velocity[2] = 0.3f;

    state.force[0] = 0.4f;
    state.force[1] = -0.5f;
    state.force[2] = 0.6f;
    state.torque[0] = -0.7f;
    state.torque[1] = 0.8f;
    state.torque[2] = -0.9f;

    rlt::MatrixDynamic<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::OBSERVATION_DIM>> observation;
    rlt::MatrixDynamic<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::OBSERVATION_DIM_PRIVILEGED>> observation_privileged;
    rlt::malloc(device, observation);
    rlt::malloc(device, observation_privileged);

    rlt::observe(device, env, state, observation, rng);
    rlt::observe_privileged(device, env, state, observation_privileged, rng);

    EXPECT_FLOAT_EQ(rlt::get(observation, 0, 0), 1.0f);
    EXPECT_FLOAT_EQ(rlt::get(observation, 0, 1), -2.0f);
    EXPECT_FLOAT_EQ(rlt::get(observation, 0, 2), 3.0f);

    EXPECT_FLOAT_EQ(rlt::get(observation, 0, 3), 1.0f);
    EXPECT_FLOAT_EQ(rlt::get(observation, 0, 4), 0.0f);
    EXPECT_FLOAT_EQ(rlt::get(observation, 0, 5), 0.0f);
    EXPECT_FLOAT_EQ(rlt::get(observation, 0, 6), 0.0f);
    EXPECT_FLOAT_EQ(rlt::get(observation, 0, 7), 1.0f);
    EXPECT_FLOAT_EQ(rlt::get(observation, 0, 8), 0.0f);
    EXPECT_FLOAT_EQ(rlt::get(observation, 0, 9), 0.0f);
    EXPECT_FLOAT_EQ(rlt::get(observation, 0, 10), 0.0f);
    EXPECT_FLOAT_EQ(rlt::get(observation, 0, 11), 1.0f);

    EXPECT_FLOAT_EQ(rlt::get(observation, 0, 12), 0.1f);
    EXPECT_FLOAT_EQ(rlt::get(observation, 0, 13), -0.2f);
    EXPECT_FLOAT_EQ(rlt::get(observation, 0, 14), 0.3f);

    for(TI i = 0; i < ENVIRONMENT::OBSERVATION_DIM; i++) {
        EXPECT_FLOAT_EQ(rlt::get(observation_privileged, 0, i), rlt::get(observation, 0, i));
    }

    EXPECT_FLOAT_EQ(rlt::get(observation_privileged, 0, 15), 0.0f);
    EXPECT_FLOAT_EQ(rlt::get(observation_privileged, 0, 16), 0.0f);
    EXPECT_FLOAT_EQ(rlt::get(observation_privileged, 0, 17), 0.0f);

    EXPECT_FLOAT_EQ(rlt::get(observation_privileged, 0, 18), 0.0f);

    EXPECT_FLOAT_EQ(rlt::get(observation_privileged, 0, 19), 0.4f);
    EXPECT_FLOAT_EQ(rlt::get(observation_privileged, 0, 20), -0.5f);
    EXPECT_FLOAT_EQ(rlt::get(observation_privileged, 0, 21), 0.6f);

    EXPECT_FLOAT_EQ(rlt::get(observation_privileged, 0, 22), -0.7f);
    EXPECT_FLOAT_EQ(rlt::get(observation_privileged, 0, 23), 0.8f);
    EXPECT_FLOAT_EQ(rlt::get(observation_privileged, 0, 24), -0.9f);

    rlt::free(device, observation);
    rlt::free(device, observation_privileged);
}
