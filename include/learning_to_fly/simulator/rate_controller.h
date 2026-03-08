#ifndef LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_RATE_CONTROLLER_H
#define LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_RATE_CONTROLLER_H

// Simple proportional body-rate controller.
// Converts desired body angular rates (rad/s) and measured angular rates
// (from the simulator state) into body-frame torque commands (N·m).
//
// The controller uses two gains:
//   kp_xy  – proportional gain for roll and pitch axes
//   kp_z   – proportional gain for the yaw axis

namespace rl_tools::rl::environments::multirotor {

    template<typename T>
    struct RateControllerParameters {
        T kp_xy; // proportional gain roll/pitch (N·m per rad/s)
        T kp_z;  // proportional gain yaw (N·m per rad/s)
    };

    // Compute torque commands given desired and measured angular rates.
    // omega_des[3]  : desired body angular rates [rad/s]
    // omega_meas[3] : measured body angular rates [rad/s]
    // torque_cmd[3] : output body-frame torque commands [N·m]
    template<typename T>
    inline void rate_controller(const RateControllerParameters<T>& params,
                                const T omega_des[3],
                                const T omega_meas[3],
                                T torque_cmd[3]) {
        torque_cmd[0] = params.kp_xy * (omega_des[0] - omega_meas[0]); // roll
        torque_cmd[1] = params.kp_xy * (omega_des[1] - omega_meas[1]); // pitch
        torque_cmd[2] = params.kp_z  * (omega_des[2] - omega_meas[2]); // yaw
    }

} // namespace rl_tools::rl::environments::multirotor

#endif // LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_RATE_CONTROLLER_H
