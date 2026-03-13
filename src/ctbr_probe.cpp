#include <rl_tools/operations/cpu.h>
#include <learning_to_fly/simulator/operations_cpu.h>

#include "config/parameters.h"

#include <cstdlib>
#include <iostream>

int main(int argc, char** argv) {
    using DEVICE = rl_tools::devices::DefaultCPU;
    using TI = typename DEVICE::index_t;
    using T = float;

    using ABLATION_SPEC = parameters::DefaultAblationSpec;
    using ENV_BUILDER = parameters::environment<T, TI, ABLATION_SPEC>;
    using ENVIRONMENT = typename ENV_BUILDER::ENVIRONMENT;

    DEVICE device;
    auto rng = rl_tools::random::default_engine(DEVICE::SPEC::RANDOM{}, 123);

    ENVIRONMENT env({ENV_BUILDER::parameters});
    typename ENVIRONMENT::State state{};
    rl_tools::initial_state(device, env, state);

    T action_wx = 0.6f;
    T action_wy = 0.0f;
    T action_wz = 0.4f;
    T action_thrust = 0.6f;
    int n_steps = 20;

    if(argc >= 5) {
        action_wx = std::atof(argv[1]);
        action_wy = std::atof(argv[2]);
        action_wz = std::atof(argv[3]);
        action_thrust = std::atof(argv[4]);
    }
    if(argc >= 6) {
        n_steps = std::atoi(argv[5]);
    }

    rl_tools::MatrixDynamic<rl_tools::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> action;
    rl_tools::malloc(device, action);
    set(action, 0, 0, action_wx);
    set(action, 0, 1, action_wy);
    set(action, 0, 2, action_wz);
    set(action, 0, 3, action_thrust);

    const T max_rate_command = env.parameters.mdp.init.max_commanded_angular_velocity;
    const T max_thrust_acceleration_command = env.parameters.mdp.init.max_commanded_thrust_acceleration;

    const T wx_cmd = action_wx * max_rate_command;
    const T wy_cmd = action_wy * max_rate_command;
    const T wz_cmd = action_wz * max_rate_command;
    const T thrust_acc_cmd = (action_thrust + (T)1) * (T)0.5 * max_thrust_acceleration_command;

    std::cout << "CTBR probe" << std::endl;
    std::cout << "input action [wx, wy, wz, thrust] = ["
              << action_wx << ", " << action_wy << ", " << action_wz << ", " << action_thrust << "]" << std::endl;
    std::cout << "decoded commands [wx_cmd, wy_cmd, wz_cmd, thrust_acc_cmd] = ["
              << wx_cmd << ", " << wy_cmd << ", " << wz_cmd << ", " << thrust_acc_cmd << "]" << std::endl;

    for(int step_i = 0; step_i < n_steps; step_i++) {
        typename ENVIRONMENT::State next_state = state;
        rl_tools::step(device, env, state, action, next_state, rng);

        const T linear_acc_z = (next_state.linear_velocity[2] - state.linear_velocity[2]) / env.parameters.integration.dt;

        std::cout << "step " << step_i
                  << " | ang_vel = ["
                  << next_state.angular_velocity[0] << ", "
                  << next_state.angular_velocity[1] << ", "
                  << next_state.angular_velocity[2] << "]"
                  << " | approx_lin_acc_z = " << linear_acc_z
                  << std::endl;

        state = next_state;
    }

    rl_tools::free(device, action);
    return 0;
}
