#ifndef LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_MIXER_H
#define LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_MIXER_H

#include <cmath>

namespace rl_tools::rl::environments::multirotor {
    template<typename T>
    struct MixerParameters {
        T arm_length;
        T yaw_torque_constant;
        T thrust_coefficient;
        T mass;
        T min_rpm;
        T max_rpm;
    };

    template<typename T>
    constexpr T clamp(T value, T min_value, T max_value){
        if(value < min_value){
            return min_value;
        }
        if(value > max_value){
            return max_value;
        }
        return value;
    }

    template<typename T>
    constexpr T normalize_rpm(T rpm, T min_rpm, T max_rpm){
        const T half_range = (max_rpm - min_rpm) / (T)2;
        return (rpm - min_rpm) / half_range - (T)1;
    }

    template<typename T>
    void mixer(const MixerParameters<T>& parameters,
               const T* torque_command,
               T thrust_acceleration_command,
               T* normalized_motor_command,
               T* rotor_thrust = nullptr,
               T* rotor_rpm = nullptr){
        const T total_thrust = parameters.mass * thrust_acceleration_command;
        const T collective = total_thrust / (T)4;
        const T roll_term = torque_command[0] / ((T)4 * parameters.arm_length);
        const T pitch_term = torque_command[1] / ((T)4 * parameters.arm_length);
        const T yaw_term = torque_command[2] / ((T)4 * parameters.yaw_torque_constant);

        T thrust_per_rotor[4];
        thrust_per_rotor[0] = collective - roll_term - pitch_term - yaw_term;
        thrust_per_rotor[1] = collective - roll_term + pitch_term + yaw_term;
        thrust_per_rotor[2] = collective + roll_term + pitch_term - yaw_term;
        thrust_per_rotor[3] = collective + roll_term - pitch_term + yaw_term;

        for(int rotor = 0; rotor < 4; rotor++){
            const T clamped_thrust = clamp(thrust_per_rotor[rotor], (T)0, parameters.thrust_coefficient * parameters.max_rpm * parameters.max_rpm);
            const T rpm = std::sqrt(clamped_thrust / parameters.thrust_coefficient);
            const T clamped_rpm = clamp(rpm, parameters.min_rpm, parameters.max_rpm);
            normalized_motor_command[rotor] = clamp(normalize_rpm(clamped_rpm, parameters.min_rpm, parameters.max_rpm), (T)-1, (T)1);
            if(rotor_thrust != nullptr){
                rotor_thrust[rotor] = clamped_thrust;
            }
            if(rotor_rpm != nullptr){
                rotor_rpm[rotor] = clamped_rpm;
            }
        }
    }
}

#endif
