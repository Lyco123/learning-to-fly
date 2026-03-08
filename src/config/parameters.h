#ifndef RL_TOOLS_SRC_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_H
#define RL_TOOLS_SRC_RL_ENVIRONMENTS_MULTIROTOR_PARAMETERS_H

#include <learning_to_fly/simulator/parameters/reward_functions/abs_exp.h>
#include <learning_to_fly/simulator/parameters/reward_functions/squared.h>
#include <learning_to_fly/simulator/parameters/reward_functions/absolute.h>
#include <learning_to_fly/simulator/parameters/reward_functions/default.h>
#include <learning_to_fly/simulator/parameters/dynamics/crazy_flie.h>
#include <learning_to_fly/simulator/parameters/init/default.h>
#include <learning_to_fly/simulator/parameters/termination/default.h>
#include <learning_to_fly/simulator/parameters/ctbr_controller/default.h>

#include <rl_tools/utils/generic/typing.h>

namespace parameters{
    namespace rlt = rl_tools;
    struct DefaultAblationSpec{
        static constexpr bool DISTURBANCE = true;
        static constexpr bool OBSERVATION_NOISE = true;
        static constexpr bool ASYMMETRIC_ACTOR_CRITIC = true;
        static constexpr bool ROTOR_DELAY = true;
        static constexpr bool ACTION_HISTORY = true;
        static constexpr bool ENABLE_CURRICULUM = true;
        static constexpr bool USE_INITIAL_REWARD_FUNCTION = true;
        static constexpr bool INIT_NORMAL = true;
    };
    namespace builder {
        namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER::rl_tools;
        using namespace rlt::rl::environments::multirotor;
        template<typename T, typename TI, typename T_ABLATION_SPEC>
        struct environment {
            using ABLATION_SPEC = T_ABLATION_SPEC;

            // CTBR reward functions (used for the CTBR actor/critic interface)
            static constexpr auto initial_reward_function = rl_tools::rl::environments::multirotor::parameters::reward_functions::reward_ctbr_hover<T>;
            static constexpr auto target_reward_function  = rl_tools::rl::environments::multirotor::parameters::reward_functions::reward_ctbr_curriculum_target<T>;
            static constexpr auto reward_function = ABLATION_SPEC::USE_INITIAL_REWARD_FUNCTION ? initial_reward_function : target_reward_function;

            using REWARD_FUNCTION_CONST = typename rl_tools::utils::typing::remove_cv_t<decltype(reward_function)>;
            using REWARD_FUNCTION = typename rl_tools::utils::typing::remove_cv<REWARD_FUNCTION_CONST>::type;

            // CTBR wraps Disturbances which wraps ParametersBase
            using PARAMETERS_TYPE = rl_tools::rl::environments::multirotor::ParametersCTBR<T, TI,
                                        rl_tools::rl::environments::multirotor::ParametersDisturbances<T, TI,
                                            rl_tools::rl::environments::multirotor::ParametersBase<T, TI, 4, REWARD_FUNCTION>>>;

            static_assert(ABLATION_SPEC::INIT_NORMAL);
            static constexpr auto init_params = rl_tools::rl::environments::multirotor::parameters::init::ctbr_all_around<T, TI, 4, REWARD_FUNCTION>;

            static constexpr PARAMETERS_TYPE parameters = {
                    // ParametersBase::Dynamics (innermost base class first)
                    rl_tools::rl::environments::multirotor::parameters::dynamics::crazy_flie<T, TI, REWARD_FUNCTION>,
                    // ParametersBase::Integration
                    {0.01}, // dt
                    // ParametersBase::MDP
                    {
                            init_params,
                            reward_function,
                            {   // Observation noise
                                    0.001 * ABLATION_SPEC::OBSERVATION_NOISE, // position
                                    0.001 * ABLATION_SPEC::OBSERVATION_NOISE, // orientation
                                    0.002 * ABLATION_SPEC::OBSERVATION_NOISE, // linear_velocity
                                    0.002 * ABLATION_SPEC::OBSERVATION_NOISE, // angular_velocity
                            },
                            {   // Action noise (unused for CTBR)
                                    0,
                            },
                            rl_tools::rl::environments::multirotor::parameters::termination::fast_learning<T, TI, 4, REWARD_FUNCTION>
                    },
                    // ParametersDisturbances::Disturbances
                    {
                        typename PARAMETERS_TYPE::Disturbances::UnivariateGaussian{0, 0.027 * 9.81 / 20 * ABLATION_SPEC::DISTURBANCE}, // random_force
                        typename PARAMETERS_TYPE::Disturbances::UnivariateGaussian{0, 0.027 * 9.81 / 10000 * ABLATION_SPEC::DISTURBANCE} // random_torque
                    },
                    // ParametersCTBR::CTBR (Crazyflie 2.x defaults)
                    {
                        (T)0.001,  // kp_xy
                        (T)0.0005, // kp_z
                        (T)10.0,   // max_body_rate_xy [rad/s]
                        (T)5.0,    // max_body_rate_z  [rad/s]
                        (T)22.0,   // max_thrust_acc   [m/s^2]
                    },
            };

            using PARAMETERS = typename rl_tools::utils::typing::remove_cv_t<decltype(parameters)>;

            // CTBR environment static parameters
            struct ENVIRONMENT_STATIC_PARAMETERS{
                // Keep 1 history step to store the current CTBR command for the
                // privileged critic observation.
                static constexpr TI ACTION_HISTORY_LENGTH = 1;
                static constexpr bool CTBR = true;

                // State: rotor-delay model + 1 action history step + disturbances
                using STATE_TYPE = rlt::utils::typing::conditional_t<ABLATION_SPEC::ROTOR_DELAY,
                    StateRotorsHistory<T, TI, ACTION_HISTORY_LENGTH,
                        rlt::utils::typing::conditional_t<ABLATION_SPEC::DISTURBANCE,
                            StateRandomForce<T, TI, StateBase<T, TI>>,
                            StateBase<T, TI>>>,
                    rlt::utils::typing::conditional_t<ABLATION_SPEC::DISTURBANCE,
                        StateRandomForce<T, TI, StateBase<T, TI>>,
                        StateBase<T, TI>>>;

                // Actor observation: position (3) + rotation matrix (9) + linear velocity (3) = 15
                // No angular velocity, no action history for the actor.
                using OBSERVATION_TYPE =
                    observation::Position<observation::PositionSpecification<T, TI,
                        observation::OrientationRotationMatrix<observation::OrientationRotationMatrixSpecification<T, TI,
                            observation::LinearVelocity<observation::LinearVelocitySpecification<T, TI>>>>>>;

                // Privileged critic observation:
                //   actor obs (15) + CTBR command (4) + disturbance force+torque (6) = 25
                using OBSERVATION_TYPE_PRIVILEGED = rlt::utils::typing::conditional_t<ABLATION_SPEC::ASYMMETRIC_ACTOR_CRITIC,
                    observation::Position<observation::PositionSpecificationPrivileged<T, TI,
                        observation::OrientationRotationMatrix<observation::OrientationRotationMatrixSpecificationPrivileged<T, TI,
                            observation::LinearVelocity<observation::LinearVelocitySpecificationPrivileged<T, TI,
                                rlt::utils::typing::conditional_t<ABLATION_SPEC::DISTURBANCE,
                                    observation::RandomForce<observation::RandomForceSpecification<T, TI,
                                        rlt::utils::typing::conditional_t<ABLATION_SPEC::ROTOR_DELAY,
                                            observation::CTBRCommand<observation::CTBRCommandSpecification<T, TI>>,
                                            observation::LastComponent<TI>>>>,
                                    rlt::utils::typing::conditional_t<ABLATION_SPEC::ROTOR_DELAY,
                                        observation::CTBRCommand<observation::CTBRCommandSpecification<T, TI>>,
                                        observation::LastComponent<TI>>
                                >
                            >>
                        >>
                    >>,
                    observation::NONE<TI>
                >;
                static constexpr bool PRIVILEGED_OBSERVATION_NOISE = false;
            };

            using ENVIRONMENT_SPEC = rlt::rl::environments::multirotor::Specification<T, TI, PARAMETERS, ENVIRONMENT_STATIC_PARAMETERS>;
            using ENVIRONMENT = rlt::rl::environments::Multirotor<ENVIRONMENT_SPEC>;
        };
    }
    template<typename T, typename TI, typename ABLATION_SPEC>
    using environment = builder::environment<T, TI, ABLATION_SPEC>;
}

#endif
