#ifndef LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_RATE_CONTROLLER_H
#define LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_RATE_CONTROLLER_H

namespace rl_tools::rl::environments::multirotor {
    template<typename T>
    struct RateControllerParameters {
        T k_p[3];          // [roll_rate, pitch_rate, yaw_rate]
        T max_torque[3];   // per-axis absolute torque limit
    };

    template<typename T>
    constexpr T clamp_symmetric(T value, T limit){
        if(value > limit){
            return limit;
        }
        if(value < -limit){
            return -limit;
        }
        return value;
    }

    template<typename T>
    void rate_controller(const RateControllerParameters<T>& parameters,
                         const T* commanded_angular_rate,
                         const T* measured_angular_rate,
                         T* torque_command){
        for(int axis = 0; axis < 3; axis++){
            const T rate_error = commanded_angular_rate[axis] - measured_angular_rate[axis];
            const T raw_torque = parameters.k_p[axis] * rate_error;
            torque_command[axis] = clamp_symmetric(raw_torque, parameters.max_torque[axis]);
        }
    }
}

#endif
