#ifndef LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_PARAMETERS_INIT_DEFAULT_H
#define LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_PARAMETERS_INIT_DEFAULT_H
#include "../../multirotor.h"

#define RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_POSITION (0.2)
#define RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_LINEAR_VELOCITY (1)
#define RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_ANGULAR_VELOCITY (1)

namespace rl_tools::rl::environments::multirotor::parameters::init{
    template<typename T, typename TI, TI ACTION_DIM, typename REWARD_FUNCTION>
    constexpr typename ParametersBase<T, TI, ACTION_DIM, REWARD_FUNCTION>::MDP::Initialization all_around = {
            0.1, // guidance
            0.3, // position
            3.14,   // orientation
            1,   // linear velocity
            10,  // angular velocity
            true,// relative rpm
            -1,  // min rpm
            +1,  // max rpm
    };
    template<typename T, typename TI, TI ACTION_DIM, typename REWARD_FUNCTION>
    constexpr typename ParametersBase<T, TI, ACTION_DIM, REWARD_FUNCTION>::MDP::Initialization all_around_2 = {
            0.1, // guidance
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_POSITION,   // position
            3.14,   // orientation
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_LINEAR_VELOCITY,   // linear velocity
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_ANGULAR_VELOCITY,   // angular velocity
            true,// relative rpm
            -0,  // min rpm
            +0,  // max rpm
    };
    template<typename T, typename TI, TI ACTION_DIM, typename REWARD_FUNCTION>
    constexpr typename ParametersBase<T, TI, ACTION_DIM, REWARD_FUNCTION>::MDP::Initialization orientation_all_around = {
            0.1, // guidance
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_POSITION,   // position
            3.14,   // orientation
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_LINEAR_VELOCITY,   // linear velocity
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_ANGULAR_VELOCITY,   // angular velocity
            true,// relative rpm
            0,  // min rpm
            0,  // max rpm
    };
    template<typename T, typename TI, TI ACTION_DIM, typename REWARD_FUNCTION>
    constexpr typename ParametersBase<T, TI, ACTION_DIM, REWARD_FUNCTION>::MDP::Initialization orientation_small_angle = {
            0.1, // guidance
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_POSITION,   // position
            10.0/180.0*3.14,   // orientation
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_LINEAR_VELOCITY,   // linear velocity
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_ANGULAR_VELOCITY,   // angular velocity
            true,// relative rpm
            0,  // min rpm
            0,  // max rpm
    };
    template<typename T, typename TI, TI ACTION_DIM, typename REWARD_FUNCTION>
    constexpr typename ParametersBase<T, TI, ACTION_DIM, REWARD_FUNCTION>::MDP::Initialization orientation_big_angle = {
            0.1, // guidance
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_POSITION,   // position
            20.0/180.0 * 3.14,   // orientation
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_LINEAR_VELOCITY,   // linear velocity
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_ANGULAR_VELOCITY,   // angular velocity
            true,// relative rpm
            0,  // min rpm
            0,  // max rpm
    };
    template<typename T, typename TI, TI ACTION_DIM, typename REWARD_FUNCTION>
    constexpr typename ParametersBase<T, TI, ACTION_DIM, REWARD_FUNCTION>::MDP::Initialization orientation_bigger_angle = {
            0.1, // guidance
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_POSITION,   // position
            45.0/180.0 * 3.14,   // orientation
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_LINEAR_VELOCITY,   // linear velocity
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_ANGULAR_VELOCITY,   // angular velocity
            true,// relative rpm
            0,  // min rpm
            0,  // max rpm
    };
    template<typename T, typename TI, TI ACTION_DIM, typename REWARD_FUNCTION>
    constexpr typename ParametersBase<T, TI, ACTION_DIM, REWARD_FUNCTION>::MDP::Initialization orientation_biggest_angle = {
            0.1, // guidance
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_POSITION,   // position
            90.0/180.0 * 3.14,   // orientation
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_LINEAR_VELOCITY,   // linear velocity
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_ANGULAR_VELOCITY,   // angular velocity
            true,// relative rpm
            0,  // min rpm
            0,  // max rpm
    };
    template<typename T, typename TI, TI ACTION_DIM, typename REWARD_FUNCTION>
    constexpr typename ParametersBase<T, TI, ACTION_DIM, REWARD_FUNCTION>::MDP::Initialization all_around_simplified = {
            0.1, // guidance
            0.3, // position
            0,   // orientation
            1,   // linear velocity
            10,  // angular velocity
            true,// relative rpm
            -1,  // min rpm
            +1,  // max rpm
    };
    template<typename T, typename TI, TI ACTION_DIM, typename REWARD_FUNCTION>
    constexpr typename ParametersBase<T, TI, ACTION_DIM, REWARD_FUNCTION>::MDP::Initialization simple = {
            0,   // guidance
            0,   // position
            0,   // orientation
            0,   // linear velocity
            0,   // angular velocity
            true,// relative rpm
            0,   // min rpm
            0,   // max rpm
    };
    template<typename T, typename TI, TI ACTION_DIM, typename REWARD_FUNCTION>
    constexpr typename ParametersBase<T, TI, ACTION_DIM, REWARD_FUNCTION>::MDP::Initialization all_positions = {
            0.5, // guidance
            0.3, // position
            0,   // orientation
            0,   // linear velocity
            0,  // angular velocity
            true,// relative rpm
            0,  // min rpm
            0,  // max rpm
    };

    // -------------------------------------------------------------------
    // CTBR-specific initializations.
    // The `relative_rpm / min_rpm / max_rpm` fields are re-interpreted for
    // CTBR as:  initial_command = uniform([min_rpm, max_rpm]) normalized
    // where [0, 0] means all CTBR commands start at zero (hover-like).
    // -------------------------------------------------------------------

    // All-around CTBR initialization: large position/angle perturbations,
    // initial CTBR commands all zero (angular rates = 0, thrust = mid-range).
    template<typename T, typename TI, TI ACTION_DIM, typename REWARD_FUNCTION>
    constexpr typename ParametersBase<T, TI, ACTION_DIM, REWARD_FUNCTION>::MDP::Initialization ctbr_all_around = {
            0.1, // guidance
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_POSITION,   // position
            90.0/180.0 * 3.14,   // orientation (same as orientation_biggest_angle)
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_LINEAR_VELOCITY,   // linear velocity
            RL_TOOLS_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_INIT_ANGULAR_VELOCITY,   // angular velocity
            true, // relative_rpm (interpreted as: init CTBR commands in relative range)
            0,    // min_cmd (CTBR: zero angular rate, midpoint thrust)
            0,    // max_cmd (same: zero initial CTBR commands)
    };
}

#endif // LEARNING_TO_FLY_IN_SECONDS_SIMULATOR_PARAMETERS_INIT_DEFAULT_H
