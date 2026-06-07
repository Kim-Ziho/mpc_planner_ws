#include <mpc_planner_types/module_data.h>

namespace MPCPlanner
{
        void ModuleData::reset()
        {
                path.reset();
                path_width_left.reset();
                path_width_right.reset();
                path_velocity.reset();
                step_map.reset();
                current_path_segment = -1;
        }
}