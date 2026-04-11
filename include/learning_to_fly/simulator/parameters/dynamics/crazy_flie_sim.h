
#include "../../multirotor.h"

namespace rl_tools::rl::environments::multirotor::parameters::dynamics{
    template<typename T, typename TI, typename REWARD_FUNCTION>
    constexpr typename ParametersBase <T, TI, TI(4), REWARD_FUNCTION>::Dynamics crazy_flie_sim = {
            // Rotor positions in FLU body frame (x: forward, y: left, z: up).
            // Do not directly reuse PX4 FRD motor indexing here.
            {
                    {
                            0.031,
                            -0.031,
                            0

                    },
                    {
                            -0.031,
                            -0.031,
                            0

                    },
                    {
                            -0.031,
                            0.031,
                            0

                    },
                    {
                            0.031,
                            0.031,
                            0

                    },
            },
            // Rotor thrust directions
            {
                    {0, 0, 1},
                    {0, 0, 1},
                    {0, 0, 1},
                    {0, 0, 1},
            },
            // Rotor torque directions
            {
                    {0, 0, -1},
                    {0, 0, +1},
                    {0, 0, -1},
                    {0, 0, +1},
            },
            // thrust constants
            {
                    0,
                    0,
                    1.7965e-8
            },
            // pwm to thrust coefficients (thrust = a * pwm^2 + b * pwm)
            0.091492681f,
            0.067673604f,
            // torque constant
//            0.025126582278481014,
            0.005964552,
            // mass vehicle
            0.025,
            // gravity
            {0, 0, -9.81},
            // J
            {
                    {
                            0.000016572,
                            0.0000000000000000000000000000000000000000,
                            0.0000000000000000000000000000000000000000
                    },
                    {
                            0.0000000000000000000000000000000000000000,
                            0.000016656,
                            0.0000000000000000000000000000000000000000
                    },
                    {
                            0.0000000000000000000000000000000000000000,
                            0.0000000000000000000000000000000000000000,
                            0.000029262
                    }
            },
            // J_inv
            {
                    {
                            60342.7468,
                            0.0000000000000000000000000000000000000000,
                            0.0000000000000000000000000000000000000000
                    },
                    {
                            0.0000000000000000000000000000000000000000,
                            60038.4245,
                            0.0000000000000000000000000000000000000000
                    },
                    {
                            0.0000000000000000000000000000000000000000,
                            0.0000000000000000000000000000000000000000,
                            34174.0141

                    }
            },
            // T, RPM time constant
            0.025,
            // action limit
            {0, 3052},
            // control limits (ctbr)
            {4.0, 1.0, 0.0, 19.62},
            // rate controller
            {
                {0.001, 0.001, 0.0005},//P
                {0.0002, 0.0002, 0.0000},//i
                {0.00005, 0.00005, 0.0000},//d
                {0.0, 0.0, 0.0},//kff
                {0.08, 0.08, 0.20},//ilimit
                {0.0015, 0.0015, 0.0012},
                {0.85, 0.85, 0.90},
                0.8
            }
    };
}
