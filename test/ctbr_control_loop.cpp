#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <type_traits>

#include <rl_tools/operations/cpu_mux.h>

#include <learning_to_fly/simulator/mixer.h>
#include <learning_to_fly/simulator/operations_cpu.h>
#include <learning_to_fly/simulator/parameters/dynamics/crazy_flie_sim.h>
#include <learning_to_fly/simulator/parameters/init/default.h>
#include <learning_to_fly/simulator/parameters/reward_functions/default.h>
#include <learning_to_fly/simulator/parameters/termination/default.h>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER::rl_tools;

namespace{
    template<typename T, typename TI>
    struct TestStaticParameters{
        static constexpr TI ACTION_HISTORY_LENGTH = 0;
        using STATE_TYPE = rlt::rl::environments::multirotor::StateRotors<T, TI, rlt::rl::environments::multirotor::StateBase<T, TI>>;
        using OBSERVATION_TYPE = typename rlt::rl::environments::multirotor::StaticParametersDefault<T, TI>::OBSERVATION_TYPE;
        using OBSERVATION_TYPE_PRIVILEGED = rlt::rl::environments::multirotor::observation::NONE<TI>;
        static constexpr bool PRIVILEGED_OBSERVATION_NOISE = false;
    };
}

TEST(RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_CTBR, MIXER_YAW_DIRECTION){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TI = typename DEVICE::index_t;
    DEVICE device;

    constexpr auto reward_function = rlt::rl::environments::multirotor::parameters::reward_functions::reward_squared_position_only_torque<T>;
    using REWARD_FUNCTION = std::remove_cv_t<decltype(reward_function)>;
    using PARAMETERS = rlt::rl::environments::multirotor::ParametersBase<T, TI, 4, REWARD_FUNCTION>;
    PARAMETERS params = {
        rlt::rl::environments::multirotor::parameters::dynamics::crazy_flie<T, TI, REWARD_FUNCTION>,
        {0.01f},
        {
            rlt::rl::environments::multirotor::parameters::init::simple<T, TI, 4, REWARD_FUNCTION>,
            reward_function,
            {0, 0, 0, 0},
            {0},
            rlt::rl::environments::multirotor::parameters::termination::fast_learning<T, TI, 4, REWARD_FUNCTION>
        }
    };

    T torque_pos[3] = {0, 0, 0.002f};
    T rpm_pos[4];
    rlt::rl::environments::multirotor::mix_ctbr_to_rpm(device, params, 9.81f, torque_pos, rpm_pos);
    T thrust_pos[4];
    for(TI i = 0; i < 4; i++){
        thrust_pos[i] = rlt::rl::environments::multirotor::rpm_to_thrust(device, params.dynamics.thrust_constants, rpm_pos[i]);
    }
    T yaw_moment_pos = 0;
    for(TI i = 0; i < 4; i++){
        yaw_moment_pos += params.dynamics.rotor_torque_directions[i][2] * thrust_pos[i] * params.dynamics.torque_constant;
    }
    EXPECT_GT(yaw_moment_pos, 0);

    T torque_neg[3] = {0, 0, -0.002f};
    T rpm_neg[4];
    rlt::rl::environments::multirotor::mix_ctbr_to_rpm(device, params, 9.81f, torque_neg, rpm_neg);
    T thrust_neg[4];
    for(TI i = 0; i < 4; i++){
        thrust_neg[i] = rlt::rl::environments::multirotor::rpm_to_thrust(device, params.dynamics.thrust_constants, rpm_neg[i]);
    }
    T yaw_moment_neg = 0;
    for(TI i = 0; i < 4; i++){
        yaw_moment_neg += params.dynamics.rotor_torque_directions[i][2] * thrust_neg[i] * params.dynamics.torque_constant;
    }
    EXPECT_LT(yaw_moment_neg, 0);
}

TEST(RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_CTBR, RATE_TO_DYNAMICS_RESPONSE){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TI = typename DEVICE::index_t;
    DEVICE device;
    auto rng = rlt::random::default_engine(DEVICE::SPEC::RANDOM{}, 7);

    constexpr auto reward_function = rlt::rl::environments::multirotor::parameters::reward_functions::reward_squared_position_only_torque<T>;
    using REWARD_FUNCTION = std::remove_cv_t<decltype(reward_function)>;
    using PARAMETERS = rlt::rl::environments::multirotor::ParametersBase<T, TI, 4, REWARD_FUNCTION>;
    PARAMETERS params = {
        rlt::rl::environments::multirotor::parameters::dynamics::crazy_flie<T, TI, REWARD_FUNCTION>,
        {0.01f},
        {
            rlt::rl::environments::multirotor::parameters::init::simple<T, TI, 4, REWARD_FUNCTION>,
            reward_function,
            {0, 0, 0, 0},
            {0},
            rlt::rl::environments::multirotor::parameters::termination::fast_learning<T, TI, 4, REWARD_FUNCTION>
        }
    };

    using ENV_SPEC = rlt::rl::environments::multirotor::Specification<T, TI, PARAMETERS, TestStaticParameters<T, TI>>;
    using ENV = rlt::rl::environments::Multirotor<ENV_SPEC>;
    ENV env{params};

    using ACTION = rlt::MatrixDynamic<rlt::matrix::Specification<T, TI, 1, ENV::ACTION_DIM>>;
    ACTION action;
    rlt::malloc(device, action);

    typename ENV::State state;
    typename ENV::State next_state;
    rlt::initial_state(device, env, state);

    set(action, 0, 0, 0.7f);
    set(action, 0, 1, 0.0f);
    set(action, 0, 2, 0.0f);
    set(action, 0, 3, 0.0f);
    for(TI step_i = 0; step_i < 80; step_i++){
        rlt::step(device, env, state, action, next_state, rng);
        state = next_state;
    }
    EXPECT_GT(state.angular_velocity[0], 0.2f);
    EXPECT_LT(std::abs(state.angular_velocity[1]), 1.0f);
    EXPECT_LT(std::abs(state.angular_velocity[2]), 1.0f);

    rlt::initial_state(device, env, state);
    set(action, 0, 0, 0.0f);
    set(action, 0, 1, 0.0f);
    set(action, 0, 2, 0.8f);
    set(action, 0, 3, 0.0f);
    for(TI step_i = 0; step_i < 80; step_i++){
        rlt::step(device, env, state, action, next_state, rng);
        state = next_state;
    }
    EXPECT_GT(state.angular_velocity[2], 0.2f);

    rlt::free(device, action);
}

TEST(RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_CTBR, STEP_RESPONSE_TENSORBOARD_LOG){
    using Logger = rlt::devices::logging::CPU_TENSORBOARD<>;
    using DevSpec = rlt::devices::cpu::Specification<rlt::devices::math::CPU, rlt::devices::random::CPU, Logger>;
    using DEVICE = rlt::devices::CPU<DevSpec>;
    using T = float;
    using TI = typename DEVICE::index_t;
    DEVICE device;
    auto rng = rlt::random::default_engine(DEVICE::SPEC::RANDOM{}, 11);

    constexpr auto reward_function = rlt::rl::environments::multirotor::parameters::reward_functions::reward_squared_position_only_torque<T>;
    using REWARD_FUNCTION = std::remove_cv_t<decltype(reward_function)>;
    using PARAMETERS = rlt::rl::environments::multirotor::ParametersBase<T, TI, 4, REWARD_FUNCTION>;
    PARAMETERS params = {
        rlt::rl::environments::multirotor::parameters::dynamics::crazy_flie<T, TI, REWARD_FUNCTION>,
        {0.01f},
        {
            rlt::rl::environments::multirotor::parameters::init::simple<T, TI, 4, REWARD_FUNCTION>,
            reward_function,
            {0, 0, 0, 0},
            {0},
            rlt::rl::environments::multirotor::parameters::termination::fast_learning<T, TI, 4, REWARD_FUNCTION>
        }
    };

    using ENV_SPEC = rlt::rl::environments::multirotor::Specification<T, TI, PARAMETERS, TestStaticParameters<T, TI>>;
    using ENV = rlt::rl::environments::Multirotor<ENV_SPEC>;
    ENV env{params};

    rlt::construct(device, device.logger, std::string("logs"), std::string("ctbr_step_response_test"));

    using ACTION = rlt::MatrixDynamic<rlt::matrix::Specification<T, TI, 1, ENV::ACTION_DIM>>;
    ACTION action;
    rlt::malloc(device, action);

    typename ENV::State state;
    typename ENV::State next_state;

    constexpr TI n_axes = 3;
    const TI axis_channel[n_axes] = {0, 1, 2};
    const char* axis_name[n_axes] = {"roll", "pitch", "yaw"};
    const T step_amplitude[n_axes] = {0.01f, 0.01f, 0.01f};
    constexpr TI pre_steps = 50;
    constexpr TI post_steps = 350;
    constexpr TI total_steps = pre_steps + post_steps;

    TI global_step = 0;
    for(TI axis_i = 0; axis_i < n_axes; axis_i++){
        rlt::initial_state(device, env, state);
        for(TI step_i = 0; step_i < total_steps; step_i++){
            set(action, 0, 0, 0.0f);
            set(action, 0, 1, 0.0f);
            set(action, 0, 2, 0.0f);
            set(action, 0, 3, 0.0f); // hover thrust acceleration command (normalized)

            const T axis_cmd = step_i >= pre_steps ? step_amplitude[axis_i] : 0.0f;
            set(action, 0, axis_channel[axis_i], axis_cmd);

            rlt::step(device, env, state, action, next_state, rng);
            state = next_state;

            rlt::set_step(device, device.logger, global_step);
            rlt::add_scalar(device, device.logger, std::string("ctbr_step/") + axis_name[axis_i] + "/command", axis_cmd);
            rlt::add_scalar(device, device.logger, std::string("ctbr_step/") + axis_name[axis_i] + "/wx", state.angular_velocity[0]);
            rlt::add_scalar(device, device.logger, std::string("ctbr_step/") + axis_name[axis_i] + "/wy", state.angular_velocity[1]);
            rlt::add_scalar(device, device.logger, std::string("ctbr_step/") + axis_name[axis_i] + "/wz", state.angular_velocity[2]);
            rlt::add_scalar(device, device.logger, std::string("ctbr_step/") + axis_name[axis_i] + "/rpm_0", state.rpm[0]);
            rlt::add_scalar(device, device.logger, std::string("ctbr_step/") + axis_name[axis_i] + "/rpm_1", state.rpm[1]);
            rlt::add_scalar(device, device.logger, std::string("ctbr_step/") + axis_name[axis_i] + "/rpm_2", state.rpm[2]);
            rlt::add_scalar(device, device.logger, std::string("ctbr_step/") + axis_name[axis_i] + "/rpm_3", state.rpm[3]);
            global_step++;
        }
    }

    rlt::free(device, action);
    rlt::destruct(device, device.logger);
}
