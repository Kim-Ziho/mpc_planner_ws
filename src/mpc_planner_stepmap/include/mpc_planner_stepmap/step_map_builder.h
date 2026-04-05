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
    bool publish{true};
    std::string topic{"guidance_planner/step_map"};
    std::string frame_id{"map"};
    // "gaussian_independent": per-step covariance sampling (default)
    // "gaussian_trajectory":  velocity-noise trajectory sampling
    std::string dynamic_method{"gaussian_independent"};
    // gaussian_independent 에서 예측 불확실성을 시간에 따라 누적 전파할지 여부
    bool propagate_uncertainty{false};
    // stage 간 추가 z 오프셋 [m] (stage idx * stage_z_offset 만큼 z에 더해짐)
    double stage_z_offset{0.0};
    // 0 = 전체 시각화, N>0 → start/terminal 포함 N개 스테이지만 시각화
    int vis_stages{0};
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
    double time_scale_{0.2};
  };
} // namespace MPCPlannerStepMap

#endif // MPC_PLANNER_STEPMAP_STEP_MAP_BUILDER_H
