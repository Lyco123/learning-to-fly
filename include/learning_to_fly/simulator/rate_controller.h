#ifndef LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_RATE_CONTROLLER_H
#define LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_RATE_CONTROLLER_H

#include "multirotor.h"

#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif

namespace rl_tools::rl::environments::multirotor {
    template<typename DEVICE, typename T, typename TI, typename PARAMETERS>
    RL_TOOLS_FUNCTION_PLACEMENT void body_rate_controller(
            DEVICE& device,
            const PARAMETERS& params,
            const StateBase<T, TI>& state,
            const T desired_angular_velocity[3],
            T desired_torque[3]
    ){
        for(TI i = 0; i < 3; i++){
            const T error = desired_angular_velocity[i] - state.angular_velocity[i];
            const T torque = params.dynamics.rate_controller.kp[i] * error;
            desired_torque[i] = math::clamp(device.math, torque, -params.dynamics.rate_controller.torque_limit[i], params.dynamics.rate_controller.torque_limit[i]);
        }
    }
}

#endif
