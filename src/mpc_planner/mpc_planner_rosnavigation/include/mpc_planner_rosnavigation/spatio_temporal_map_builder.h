#ifndef MPC_PLANNER_ROSNAVIGATION_SPATIO_TEMPORAL_MAP_BUILDER_H
#define MPC_PLANNER_ROSNAVIGATION_SPATIO_TEMPORAL_MAP_BUILDER_H

#include <mpc_planner_types/realtime_data.h>

#include <costmap_2d/costmap_2d.h>

#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>

#include <memory>
#include <vector>

namespace MPCPlanner
{
    struct DynamicObstacle;
}

namespace local_planner
{
    struct SpatioTemporalMapParams
    {
        double length_ratio{1.0};
        double width_ratio{1.0};
        double resolution_ratio_x{1.0 / 3.0};
        double resolution_ratio_y{1.0 / 3.0};
        double heading_offset_ratio{0.5};
        double time_step{0.2};
        double static_cost_threshold{-1.0};
        std::string frame_id{"base_link"};
        std::string marker_namespace{"spatio_temporal_map"};
        bool enabled{true};
    };

    class SpatioTemporalMapBuilder
    {
    public:
        SpatioTemporalMapBuilder();

        void setParams(const SpatioTemporalMapParams &params);
        void setCostmap(const costmap_2d::Costmap2D *costmap);

        void update(const Eigen::Vector2d &robot_position,
                    double robot_yaw,
                    const std::vector<MPCPlanner::DynamicObstacle> &dynamic_obstacles,
                    double robot_radius,
                    int horizon_steps);

        std::shared_ptr<const MPCPlanner::SpatioTemporalMapSnapshot> getSnapshot() const;

        visualization_msgs::MarkerArray buildVisualization() const;

    private:
        void refreshDimensions(int horizon_steps);
        void clearOccupancy();
        void integrateStaticLayer();
        void integrateDynamicLayer(const std::vector<MPCPlanner::DynamicObstacle> &dynamic_obstacles,
                                   double robot_radius,
                                   int horizon_steps);
        void ensureCoordinateCaches();
        void setOccupied(int index_x, int index_y, int layer_index, float probability);

        const costmap_2d::Costmap2D *costmap_{nullptr};
        SpatioTemporalMapParams params_;

        std::shared_ptr<MPCPlanner::SpatioTemporalMapSnapshot> snapshot_;
        std::vector<std::size_t> active_indices_;

        Eigen::Vector2d robot_position_;
        double robot_yaw_{0.0};

        double min_local_x_{0.0};
        double min_local_y_{0.0};

        std::vector<double> cached_center_x_;
        std::vector<double> cached_center_y_;

        bool dimensions_dirty_{true};
    };
}

#endif // MPC_PLANNER_ROSNAVIGATION_SPATIO_TEMPORAL_MAP_BUILDER_H
