#ifndef MPC_PLANNER_STEPMAP_STEP_MAP_BUILDER_H
#define MPC_PLANNER_STEPMAP_STEP_MAP_BUILDER_H

#include <mpc_planner_stepmap/step_map.h>

#include <mpc_planner_types/data_types.h>

#include <costmap_2d/costmap_2d.h>

#include <ros/ros.h>

#include <memory>
#include <vector>

namespace MPCPlannerStepMap
{
  struct StepMapParameters
  {
    double resolution_ratio{2.0};
    double size_scale{1.0};
    double forward_offset_ratio{0.25};
    double max_alpha{0.3};
    double z_scale{0.5};
    int gaussian_samples{1000};
    double gaussian_sample_value{0.2};
    double occupancy_threshold{0.4};
    bool visualize_use_threshold{false};
    bool publish{true};
    std::string topic{"guidance_planner/step_map"};
    std::string frame_id{"map"};
  };

  class StepMapVisualizer;

  class StepMapBuilder
  {
  public:
    explicit StepMapBuilder(const ros::NodeHandle &parent_nh);

    std::shared_ptr<StepMap> update(const costmap_2d::Costmap2D *costmap,
                                    const Eigen::Vector2d &robot_position,
                                    double heading,
                                    const std::vector<MPCPlanner::DynamicObstacle> &dynamic_obstacles,
                                    const std::vector<MPCPlanner::Disc> &robot_discs,
                                    int horizon_steps,
                                    double time_scale);

    const std::shared_ptr<StepMap> &map() const { return map_; }

  private:
    void readParameters(const ros::NodeHandle &parent_nh);
    double robotRadius(const std::vector<MPCPlanner::Disc> &robot_discs) const;

    void updatePose(const Eigen::Vector2d &robot_position, double heading);
    void copyStaticLayer(const costmap_2d::Costmap2D &costmap);
    void copyDynamicObstacles(const std::vector<MPCPlanner::DynamicObstacle> &dynamic_obstacles,
                              double robot_radius, int horizon_steps);

  private:
    StepMapParameters params_;
    ros::NodeHandle nh_;
    std::shared_ptr<StepMap> map_;
    std::shared_ptr<StepMapVisualizer> visualizer_;

    double resolution_{0.1};
    double forward_offset_{0.0};
    double inverse_ratio_{0.5};
  };
} // namespace MPCPlannerStepMap

#endif // MPC_PLANNER_STEPMAP_STEP_MAP_BUILDER_H
