#ifndef LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_MIXER_H
#define LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_MIXER_H

// Crazyflie X-frame motor mixer.
//
// Motor layout (viewed from above, body frame: x = forward, y = left, z = up):
//
//       M1 (front-right, CCW)     M4 (front-left, CW)
//            (+x, -y)                  (+x, +y)
//
//       M2 (back-right,  CW)      M3 (back-left,  CCW)
//            (-x, -y)                  (-x, +y)
//
// Clockwise ordering starting from front-right: M1, M2, M3, M4
// Spin directions: CCW, CW, CCW, CW
//
// This matches the crazy_flie.h dynamics definition where index i corresponds
// to motor M(i+1):
//   index 0 → M1 front-right  CCW  torque_dir z = -1
//   index 1 → M2 back-right   CW   torque_dir z = +1
//   index 2 → M3 back-left    CCW  torque_dir z = -1
//   index 3 → M4 front-left   CW   torque_dir z = +1
//
// Forward wrench model (T_i = per-motor thrust, d = arm length, km = torque constant):
//   F_z = T0 + T1 + T2 + T3
//   τx  = d * (-T0 - T1 + T2 + T3)
//   τy  = d * (-T0 + T1 + T2 - T3)
//   τz  = km * (-T0 + T1 - T2 + T3)
//
// The analytical inverse (mixer) used here:
//   T0 = (F_z - τx/d - τy/d - τz/km) / 4
//   T1 = (F_z - τx/d + τy/d + τz/km) / 4
//   T2 = (F_z + τx/d + τy/d - τz/km) / 4
//   T3 = (F_z + τx/d - τy/d + τz/km) / 4
//
// Sign conventions:
//   positive τx → roll right  (left wing up,  right wing down)
//   positive τy → pitch down  (nose down, tail up) [x=forward,y=left,z=up frame]
//   positive τz → yaw left    (CCW when viewed from above)

#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif

namespace rl_tools::rl::environments::multirotor {

    // Mix torque + collective thrust into normalized per-motor commands.
    //
    // Parameters (read from dynamics):
    //   arm_length      : distance from CoM to each rotor in the x/y plane [m]
    //                     (|rotor_positions[0][0]|, assuming symmetric layout)
    //   torque_constant : ratio of rotor drag torque to thrust (dimensionless)
    //   thrust_constant : k in  T = k * rpm^2  (Pa·s^2, i.e., N / rpm^2)
    //   action_limit    : {min_rpm, max_rpm} for normalization
    //
    // Inputs:
    //   thrust_acc   [m/s^2]  : desired collective thrust acceleration
    //   mass         [kg]     : vehicle mass (to convert acc → force)
    //   torque_cmd[3][N·m]    : body-frame torque commands (roll, pitch, yaw)
    //
    // Output:
    //   normalized_cmds[4]    : motor commands in [-1, 1] (clamped)
    //                           using the same convention as action_limit.

    template<typename DEVICE, typename PARAMETERS, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT inline void mixer(
            DEVICE& device,
            const PARAMETERS& params,
            T thrust_acc,
            const T torque_cmd[3],
            T normalized_cmds[4])
    {
        using MATH = typename DEVICE::SPEC::MATH;

        const T mass             = params.dynamics.mass;
        const T km               = params.dynamics.torque_constant;
        const T k                = params.dynamics.thrust_constants[2]; // T = k * rpm^2
        const T min_rpm          = params.dynamics.action_limit.min;
        const T max_rpm          = params.dynamics.action_limit.max;

        // Arm length: use the magnitude of the x-component of the first rotor
        // position (symmetric layout assumed).
        const T arm = params.dynamics.rotor_positions[0][0] > 0
                    ? params.dynamics.rotor_positions[0][0]
                    : -params.dynamics.rotor_positions[0][0];

        // Collective thrust force [N]
        const T F = thrust_acc * mass;

        // Per-motor thrust [N] from analytical inverse mixer
        T per_motor[4];
        const T inv_4d  = (T)1 / ((T)4 * arm);
        const T inv_4km = (T)1 / ((T)4 * km);
        per_motor[0] = (T)0.25 * F - inv_4d * torque_cmd[0] - inv_4d * torque_cmd[1] - inv_4km * torque_cmd[2];
        per_motor[1] = (T)0.25 * F - inv_4d * torque_cmd[0] + inv_4d * torque_cmd[1] + inv_4km * torque_cmd[2];
        per_motor[2] = (T)0.25 * F + inv_4d * torque_cmd[0] + inv_4d * torque_cmd[1] - inv_4km * torque_cmd[2];
        per_motor[3] = (T)0.25 * F + inv_4d * torque_cmd[0] - inv_4d * torque_cmd[1] + inv_4km * torque_cmd[2];

        // Convert per-motor thrust to RPM, then normalize to [-1, 1]
        const T half_range    = (max_rpm - min_rpm) / (T)2;
        const T range_center  = min_rpm + half_range;

        for(int i = 0; i < 4; i++) {
            // Clamp thrust to non-negative before taking sqrt
            T Ti = per_motor[i] > (T)0 ? per_motor[i] : (T)0;
            T rpm = rl_tools::math::sqrt(MATH{}, Ti / k);
            // Clamp RPM to valid range
            if(rpm < min_rpm) rpm = min_rpm;
            if(rpm > max_rpm) rpm = max_rpm;
            // Normalize to [-1, 1]
            normalized_cmds[i] = (rpm - range_center) / half_range;
        }
    }

} // namespace rl_tools::rl::environments::multirotor

#endif // LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_MIXER_H
