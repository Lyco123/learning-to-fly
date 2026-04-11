#ifndef LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_RATE_CONTROLLER_H
#define LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_RATE_CONTROLLER_H

#include "multirotor.h"

#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif

namespace rl_tools::rl::environments::multirotor {
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_body_rate_controller(
            DEVICE& device,
            rl::environments::Multirotor<SPEC>& env
    ){
        for(typename DEVICE::index_t i = 0; i < 3; i++){
            env.rate_controller_state.integral[i] = 0;
            env.rate_controller_state.prev_angular_velocity[i] = 0;
            env.rate_controller_state.prev_derivative[i] = 0;
        }
        env.rate_controller_state.initialized = false;
    }

    template<typename DEVICE, typename SPEC, typename T, typename TI>
    RL_TOOLS_FUNCTION_PLACEMENT void body_rate_controller(
            DEVICE& device,
            rl::environments::Multirotor<SPEC>& env,
            const StateBase<T, TI>& state,
            const T desired_angular_velocity[3],
            T desired_torque[3]
    ){
        const auto& params = env.parameters;
        T omega_J[3];
        omega_J[0] = params.dynamics.J[0][0] * state.angular_velocity[0] + params.dynamics.J[0][1] * state.angular_velocity[1] + params.dynamics.J[0][2] * state.angular_velocity[2];
        omega_J[1] = params.dynamics.J[1][0] * state.angular_velocity[0] + params.dynamics.J[1][1] * state.angular_velocity[1] + params.dynamics.J[1][2] * state.angular_velocity[2];
        omega_J[2] = params.dynamics.J[2][0] * state.angular_velocity[0] + params.dynamics.J[2][1] * state.angular_velocity[1] + params.dynamics.J[2][2] * state.angular_velocity[2];
        T gyro_coupling[3];
        gyro_coupling[0] = state.angular_velocity[1] * omega_J[2] - state.angular_velocity[2] * omega_J[1];
        gyro_coupling[1] = state.angular_velocity[2] * omega_J[0] - state.angular_velocity[0] * omega_J[2];
        gyro_coupling[2] = state.angular_velocity[0] * omega_J[1] - state.angular_velocity[1] * omega_J[0];
        const T gyroscopic_compensation_gain = params.dynamics.rate_controller.gyroscopic_compensation_gain;

        if(!env.rate_controller_state.initialized){
            for(TI i = 0; i < 3; i++){
                env.rate_controller_state.prev_angular_velocity[i] = state.angular_velocity[i];
            }
            env.rate_controller_state.initialized = true;
        }

        const T dt = params.integration.dt > (T)1e-9 ? params.integration.dt : (T)1e-9;
        for(TI i = 0; i < 3; i++){
            const T error = desired_angular_velocity[i] - state.angular_velocity[i];

            const T proportional = params.dynamics.rate_controller.kp[i] * error;

            env.rate_controller_state.integral[i] += error * dt;
            const T integral_limit = params.dynamics.rate_controller.integral_limit[i];
            if(integral_limit != 0){
                env.rate_controller_state.integral[i] = math::clamp(
                        device.math,
                        env.rate_controller_state.integral[i],
                        -integral_limit,
                        integral_limit
                );
            }
            const T integral = params.dynamics.rate_controller.ki[i] * env.rate_controller_state.integral[i];

            const T derivative_measurement_raw = -(state.angular_velocity[i] - env.rate_controller_state.prev_angular_velocity[i]) / dt;
            const T derivative_lpf_alpha = math::clamp(device.math, params.dynamics.rate_controller.derivative_lpf_alpha[i], (T)0, (T)1);
            const T derivative_measurement = derivative_lpf_alpha * env.rate_controller_state.prev_derivative[i] + ((T)1 - derivative_lpf_alpha) * derivative_measurement_raw;
            const T derivative = params.dynamics.rate_controller.kd[i] * derivative_measurement;

            const T feedforward = params.dynamics.rate_controller.kff[i] * desired_angular_velocity[i];

            const T torque = proportional + integral + derivative + feedforward + gyroscopic_compensation_gain * gyro_coupling[i];
            desired_torque[i] = torque; // No clamping as per requirement
            env.rate_controller_state.prev_angular_velocity[i] = state.angular_velocity[i];
            env.rate_controller_state.prev_derivative[i] = derivative_measurement;
        }
    }
}

#endif
