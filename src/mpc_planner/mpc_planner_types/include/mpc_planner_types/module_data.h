#ifndef MODULE_DATA_H
#define MODULE_DATA_H

#include <mpc_planner_types/data_types.h>

#include <vector>
#include <memory>

namespace RosTools
{
    class Spline2D;
}

namespace tk
{
    class spline;
}

namespace MPCPlannerStepMap
{
    class StepMap;
}

namespace MPCPlanner
{
    struct ModuleData
    {
        std::vector<StaticObstacle> static_obstacles;

        // These are shared between different modules
        std::shared_ptr<RosTools::Spline2D> path{nullptr};
        std::shared_ptr<tk::spline> path_width_left{nullptr};
        std::shared_ptr<tk::spline> path_width_right{nullptr};
        std::shared_ptr<tk::spline> path_velocity{nullptr};

        // 3D (x, y, t) occupancy grid, built once per tick (e.g. by GuidanceReference) and shared
        // with constraint modules (e.g. StepDecompConstraints) to avoid rebuilding it.
        std::shared_ptr<MPCPlannerStepMap::StepMap> step_map{nullptr};

        int current_path_segment{-1};

        void reset();
    };
}

#endif // MODULE_DATA_H
