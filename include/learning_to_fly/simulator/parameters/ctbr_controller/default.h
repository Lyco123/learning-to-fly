#ifndef LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_PARAMETERS_CTBR_CONTROLLER_H
#define LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_PARAMETERS_CTBR_CONTROLLER_H

// CTBR (Collective Thrust + Body Rates) controller parameters.
//
// These values are calibrated for the Crazyflie 2.x (27 g, arm ≈ 0.028 m).
// The rate controller uses a simple proportional law; gains are chosen so
// that at the maximum angular-rate error the full torque authority is used.
//
// Physical reference values for Crazyflie:
//   max torque (roll/pitch) ≈ 2 * k * max_rpm^2 * arm
//                           = 2 * 3.16e-10 * 21702^2 * 0.028 ≈ 8.3e-3 N·m
//   max torque (yaw)        = 2 * km * k * max_rpm^2
//                           ≈ 2 * 0.005964 * 0.148 ≈ 1.77e-3 N·m
//   max thrust acc          = 4 * k * max_rpm^2 / mass
//                           = 4 * 3.16e-10 * 21702^2 / 0.027 ≈ 22.0 m/s^2

namespace rl_tools::rl::environments::multirotor::parameters::ctbr_controller {

    template<typename T>
    struct CTBRControllerParameters {
        // P-gains for the rate controller
        T kp_xy;             // roll / pitch  [N·m / (rad/s)]
        T kp_z;              // yaw            [N·m / (rad/s)]

        // Command-space limits used for normalization
        T max_body_rate_xy;  // max commanded roll/pitch rate [rad/s]
        T max_body_rate_z;   // max commanded yaw rate        [rad/s]
        T max_thrust_acc;    // max collective thrust acc     [m/s^2]
    };

    // Default parameters for the Crazyflie 2.x
    template<typename T>
    constexpr CTBRControllerParameters<T> crazy_flie = {
        /*kp_xy          =*/ (T)0.001,    // ≈ max_torque / max_rate_xy = 8.3e-3 / 10.0
        /*kp_z           =*/ (T)0.0005,   // ≈ max_torque_yaw / max_rate_z = 1.77e-3 / 5.0
        /*max_body_rate_xy=*/ (T)10.0,    // ~573 deg/s; aggressive but reachable
        /*max_body_rate_z =*/ (T)5.0,     // ~286 deg/s
        /*max_thrust_acc  =*/ (T)22.0,    // slightly above 4g; Crazyflie physical limit
    };

} // namespace rl_tools::rl::environments::multirotor::parameters::ctbr_controller

#endif // LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_PARAMETERS_CTBR_CONTROLLER_H
