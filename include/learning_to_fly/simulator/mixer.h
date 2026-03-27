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
    RL_TOOLS_FUNCTION_PLACEMENT void power_distribution_force_torque(
            DEVICE& device,
            const PARAMETERS& params,
            T thrust_acceleration,
            const T desired_torque[3],
            T desired_thrust_uncapped[4]
    ){
        const T thrust_acceleration_clamped = math::clamp(
            device.math,
            thrust_acceleration,
            params.dynamics.control_limits.thrust_acceleration_min,
            params.dynamics.control_limits.thrust_acceleration_max
        );
        const T total_thrust = thrust_acceleration_clamped * params.dynamics.mass;

        const T rotor_x = params.dynamics.rotor_positions[0][0];
        const T rotor_y = params.dynamics.rotor_positions[0][1];
        const T arm_length = math::sqrt(device.math, rotor_x * rotor_x + rotor_y * rotor_y);
        const T arm = (T)0.707106781 * arm_length;
        const T arm_safe = arm > (T)1e-9 ? arm : (T)1e-9;

        const T roll_part = ((T)0.25 / arm_safe) * desired_torque[0];
        const T pitch_part = ((T)0.25 / arm_safe) * desired_torque[1];
        const T thrust_part = (T)0.25 * total_thrust;
        const T torque_constant_safe = math::abs(device.math, params.dynamics.torque_constant) > (T)1e-12 ? params.dynamics.torque_constant : (T)1e-12;
        const T yaw_part = ((T)0.25 / torque_constant_safe) * desired_torque[2];

        desired_thrust_uncapped[0] = thrust_part - roll_part - pitch_part - yaw_part;
        desired_thrust_uncapped[1] = thrust_part - roll_part + pitch_part + yaw_part;
        desired_thrust_uncapped[2] = thrust_part + roll_part + pitch_part - yaw_part;
        desired_thrust_uncapped[3] = thrust_part + roll_part - pitch_part + yaw_part;
    }

    template<typename DEVICE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT void power_distribution_cap(
            DEVICE& device,
            const T desired_thrust_uncapped[4],
            T rotor_thrust_min,
            T rotor_thrust_max,
            T desired_thrust_capped[4]
    ){
        T highest_thrust_found = desired_thrust_uncapped[0];
        for(typename DEVICE::index_t rotor_i = 1; rotor_i < 4; rotor_i++){
            highest_thrust_found = desired_thrust_uncapped[rotor_i] > highest_thrust_found ? desired_thrust_uncapped[rotor_i] : highest_thrust_found;
        }

        const T reduction = highest_thrust_found > rotor_thrust_max ? (highest_thrust_found - rotor_thrust_max) : (T)0;
        for(typename DEVICE::index_t rotor_i = 0; rotor_i < 4; rotor_i++){
            desired_thrust_capped[rotor_i] = math::clamp(device.math, desired_thrust_uncapped[rotor_i] - reduction, rotor_thrust_min, rotor_thrust_max);
        }
    }

    template<typename DEVICE, typename T, typename PARAMETERS>
    RL_TOOLS_FUNCTION_PLACEMENT void mix_ctbr_to_rpm(
            DEVICE& device,
            const PARAMETERS& params,
            T thrust_acceleration,
            const T desired_torque[3],
            T desired_rpm[4]
    ){
        T desired_thrust_uncapped[4];
        power_distribution_force_torque(device, params, thrust_acceleration, desired_torque, desired_thrust_uncapped);

        const T rotor_thrust_min = 0;
        const T rotor_thrust_max = rpm_to_thrust(device, params.dynamics.thrust_constants, params.dynamics.action_limit.max);
        T desired_thrust_capped[4];
        power_distribution_cap(device, desired_thrust_uncapped, rotor_thrust_min, rotor_thrust_max, desired_thrust_capped);

        for(typename DEVICE::index_t rotor_i = 0; rotor_i < 4; rotor_i++){
            const T thrust = desired_thrust_capped[rotor_i];
            T rpm = thrust_to_rpm(device, params.dynamics.thrust_constants, thrust);
            desired_rpm[rotor_i] = math::clamp(device.math, rpm, params.dynamics.action_limit.min, params.dynamics.action_limit.max);
        }
    }
}

#endif
