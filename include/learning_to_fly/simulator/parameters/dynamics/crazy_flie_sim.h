
#include "../../multirotor.h"

namespace rl_tools::rl::environments::multirotor::parameters::dynamics{
    template<typename T, typename TI, typename REWARD_FUNCTION>
    constexpr typename ParametersBase <T, TI, TI(4), REWARD_FUNCTION>::Dynamics crazy_flie = {
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
            0.05,
            // action limit
            {0, 3052},
            // control limits (ctbr)
            {10.0, 5.0, 0.0, 19.62},
            // rate controller
            {
                {0.025, 0.025, 0.015},
                {0.02, 0.02, 0.01}
            }
    };
}
