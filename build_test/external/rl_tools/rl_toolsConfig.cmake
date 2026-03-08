add_library(rl_tools::rl_tools SHARED IMPORTED)

set_target_properties(rl_tools::rl_tools PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "/home/runner/work/learning-to-fly/learning-to-fly/external/rl_tools/include"
)
