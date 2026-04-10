#ifndef LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_MIXER_H
#define LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_MIXER_H

#include "multirotor.h"

#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif

namespace rl_tools::rl::environments::multirotor {
    template<typename DEVICE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T rpm_to_thrust(DEVICE& device, const T thrust_constants[3], T rpm){
        const T thrust = thrust_constants[0] + thrust_constants[1] * rpm + thrust_constants[2] * rpm * rpm;
        return thrust > (T)0 ? thrust : (T)0;
    }

    template<typename DEVICE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T thrust_to_rpm(DEVICE& device, const T thrust_constants[3], T thrust){
        thrust = thrust > (T)0 ? thrust : (T)0;
        const T a = thrust_constants[2];
        const T b = thrust_constants[1];
        const T c = thrust_constants[0] - thrust;
        if(math::abs(device.math, a) < (T)1e-12){
            if(math::abs(device.math, b) < (T)1e-12){
                return 0;
            }
            const T rpm_linear = -c / b;
            return rpm_linear > (T)0 ? rpm_linear : (T)0;
        }
        const T discriminant = b*b - 4*a*c;
        if(discriminant <= (T)0){
            return 0;
        }
        const T sqrt_discriminant = math::sqrt(device.math, discriminant);
        const T rpm_1 = (-b + sqrt_discriminant) / (2*a);
        const T rpm_2 = (-b - sqrt_discriminant) / (2*a);
        const T rpm_1_valid = rpm_1 > (T)0 ? rpm_1 : (T)0;
        const T rpm_2_valid = rpm_2 > (T)0 ? rpm_2 : (T)0;
        return rpm_1_valid > rpm_2_valid ? rpm_1_valid : rpm_2_valid;
    }

    template<typename DEVICE, typename T, typename PARAMETERS>
    RL_TOOLS_FUNCTION_PLACEMENT void mix_ctbr_to_rpm(
            DEVICE& device,
            const PARAMETERS& params,
            T thrust_acceleration,
            const T desired_torque[3],
            T desired_rpm[4]
    ){
        T thrust_acceleration_clamped = math::clamp(
            device.math,
            thrust_acceleration,
            params.dynamics.control_limits.thrust_acceleration_min,
            params.dynamics.control_limits.thrust_acceleration_max
        );
        T b[4] = {
            thrust_acceleration_clamped * params.dynamics.mass,
            desired_torque[0],
            desired_torque[1],
            desired_torque[2]
        };

        T augmented[4][5];
        for(typename DEVICE::index_t i = 0; i < 4; i++){
            const T x = params.dynamics.rotor_positions[i][0];
            const T y = params.dynamics.rotor_positions[i][1];
            const T yaw_factor = params.dynamics.rotor_torque_directions[i][2] * params.dynamics.torque_constant;
            augmented[0][i] = 1;
            augmented[1][i] = y;
            augmented[2][i] = -x;
            augmented[3][i] = yaw_factor;
        }
        for(typename DEVICE::index_t i = 0; i < 4; i++){
            augmented[i][4] = b[i];
        }

        for(typename DEVICE::index_t pivot_i = 0; pivot_i < 4; pivot_i++){
            T pivot = augmented[pivot_i][pivot_i];
            if(math::abs(device.math, pivot) < (T)1e-9){
                for(typename DEVICE::index_t swap_i = pivot_i + 1; swap_i < 4; swap_i++){
                    if(math::abs(device.math, augmented[swap_i][pivot_i]) >= (T)1e-9){
                        for(typename DEVICE::index_t col_i = 0; col_i < 5; col_i++){
                            const T tmp = augmented[pivot_i][col_i];
                            augmented[pivot_i][col_i] = augmented[swap_i][col_i];
                            augmented[swap_i][col_i] = tmp;
                        }
                        pivot = augmented[pivot_i][pivot_i];
                        break;
                    }
                }
            }
            if(math::abs(device.math, pivot) < (T)1e-9){
                continue;
            }
            for(typename DEVICE::index_t col_i = pivot_i; col_i < 5; col_i++){
                augmented[pivot_i][col_i] /= pivot;
            }
            for(typename DEVICE::index_t row_i = 0; row_i < 4; row_i++){
                if(row_i == pivot_i){
                    continue;
                }
                const T factor = augmented[row_i][pivot_i];
                for(typename DEVICE::index_t col_i = pivot_i; col_i < 5; col_i++){
                    augmented[row_i][col_i] -= factor * augmented[pivot_i][col_i];
                }
            }
        }

        T desired_thrust[4];
        T min_thrust = augmented[0][4];
        T max_thrust = augmented[0][4];
        for(typename DEVICE::index_t rotor_i = 0; rotor_i < 4; rotor_i++){
            desired_thrust[rotor_i] = augmented[rotor_i][4];
            min_thrust = desired_thrust[rotor_i] < min_thrust ? desired_thrust[rotor_i] : min_thrust;
            max_thrust = desired_thrust[rotor_i] > max_thrust ? desired_thrust[rotor_i] : max_thrust;
        }

        const T rotor_thrust_min = 0;
        const T rotor_thrust_max = rpm_to_thrust(device, params.dynamics.thrust_constants, params.dynamics.action_limit.max);

        const T shift_lower = rotor_thrust_min - min_thrust;
        const T shift_upper = rotor_thrust_max - max_thrust;
        T collective_shift;
        if(shift_lower <= shift_upper){
            // PX4-style desaturation: apply a common shift so torque-producing differential terms are preserved.
            collective_shift = math::clamp(device.math, (T)0, shift_lower, shift_upper);
        }
        else{
            collective_shift = (shift_lower + shift_upper) / 2;
        }

        for(typename DEVICE::index_t rotor_i = 0; rotor_i < 4; rotor_i++){
            const T thrust = math::clamp(device.math, desired_thrust[rotor_i] + collective_shift, rotor_thrust_min, rotor_thrust_max);
            T rpm = thrust_to_rpm(device, params.dynamics.thrust_constants, thrust);
            desired_rpm[rotor_i] = math::clamp(device.math, rpm, params.dynamics.action_limit.min, params.dynamics.action_limit.max);
        }
    }
}

#endif
