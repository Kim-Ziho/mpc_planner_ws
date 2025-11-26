#include <mpc_planner_stepmap/step_map_builder.h>
#include <mpc_planner_stepmap/step_map_visualizer.h>

#include <costmap_2d/cost_values.h>

#include <ros/console.h>

#include <algorithm>
#include <cmath>
#include <random>

namespace MPCPlannerStepMap
{
  namespace
  {
    constexpr double kMinResolution = 1e-4;

    Eigen::Matrix2d rotationMatrix(double heading)
    {
      const double c = std::cos(heading);
      const double s = std::sin(heading);
      Eigen::Matrix2d rot;
      rot << c, -s,
          s, c;
      return rot;
    }
  } // namespace

  StepMapBuilder::StepMapBuilder(const ros::NodeHandle &parent_nh)
  {
    readParameters(parent_nh);

    map_ = std::make_shared<StepMap>();

    if (params_.publish)
    {
      visualizer_ = std::make_shared<StepMapVisualizer>(nh_, params_);
    }
  }

  std::shared_ptr<StepMap> StepMapBuilder::update(const costmap_2d::Costmap2D *costmap,
                                                  const Eigen::Vector2d &robot_position,
                                                  double heading,
                                                  const std::vector<MPCPlanner::DynamicObstacle> &dynamic_obstacles,
                                                  const std::vector<MPCPlanner::Disc> &robot_discs,
                                                  int horizon_steps,
                                                  double time_scale)
  {
    if (costmap == nullptr)
    {
      ROS_WARN_THROTTLE(1.0, "[STEP Map] Costmap pointer is null.");
      return map_;
    }

    if (horizon_steps <= 0)
    {
      ROS_WARN_THROTTLE(1.0, "[STEP Map] Horizon steps must be positive.");
      return map_;
    }

    double costmap_resolution = costmap->getResolution();                                  // 0.1m/cell
    resolution_ = std::max(costmap_resolution * params_.resolution_ratio, kMinResolution); // 0.2m/cell
    inverse_ratio_ = costmap_resolution / resolution_;

    double costmap_length = static_cast<double>(costmap->getSizeInCellsX()) * costmap_resolution;
    double costmap_width = static_cast<double>(costmap->getSizeInCellsY()) * costmap_resolution;

    double target_length = std::max(resolution_, costmap_length * params_.size_scale);
    double target_width = std::max(resolution_, costmap_width * params_.size_scale);

    int cells_x = static_cast<int>(std::ceil(target_length / resolution_));
    int cells_y = static_cast<int>(std::ceil(target_width / resolution_));

    map_->configure(cells_x, cells_y, horizon_steps, resolution_, time_scale);
    map_->setOccupancyThreshold(params_.occupancy_threshold);

    forward_offset_ = params_.forward_offset_ratio * static_cast<double>(cells_x) * resolution_;

    updatePose(robot_position, heading);
    map_->clear();

    copyStaticLayer(*costmap);
    double robot_radius = robotRadius(robot_discs);
    copyDynamicObstacles(dynamic_obstacles, robot_radius, horizon_steps);

    if (visualizer_)
    {
      visualizer_->publish(*map_);
    }

    return map_;
  }

  void StepMapBuilder::readParameters(const ros::NodeHandle &parent_nh)
  {
    ros::NodeHandle global_nh("/guidance_planner/step_map");
    global_nh.param("resolution_ratio", params_.resolution_ratio, params_.resolution_ratio);
    global_nh.param("size_scale", params_.size_scale, params_.size_scale);
    global_nh.param("forward_offset_ratio", params_.forward_offset_ratio, params_.forward_offset_ratio);
    global_nh.param("max_alpha", params_.max_alpha, params_.max_alpha);
    global_nh.param("z_scale", params_.z_scale, params_.z_scale);
    global_nh.param("gaussian_samples", params_.gaussian_samples, params_.gaussian_samples);
    global_nh.param("gaussian_sample_value", params_.gaussian_sample_value, params_.gaussian_sample_value);
    global_nh.param("occupancy_threshold", params_.occupancy_threshold, params_.occupancy_threshold);
    global_nh.param("visualize_use_threshold", params_.visualize_use_threshold, params_.visualize_use_threshold);
    global_nh.param("publish", params_.publish, params_.publish);
    global_nh.param("topic", params_.topic, params_.topic);
    global_nh.param("frame_id", params_.frame_id, params_.frame_id);

    nh_ = ros::NodeHandle(parent_nh, "step_map");
    nh_.param("resolution_ratio", params_.resolution_ratio, params_.resolution_ratio);
    nh_.param("size_scale", params_.size_scale, params_.size_scale);
    nh_.param("forward_offset_ratio", params_.forward_offset_ratio, params_.forward_offset_ratio);
    nh_.param("max_alpha", params_.max_alpha, params_.max_alpha);
    nh_.param("z_scale", params_.z_scale, params_.z_scale);
    nh_.param("gaussian_samples", params_.gaussian_samples, params_.gaussian_samples);
    nh_.param("gaussian_sample_value", params_.gaussian_sample_value, params_.gaussian_sample_value);
    nh_.param("occupancy_threshold", params_.occupancy_threshold, params_.occupancy_threshold);
    nh_.param("visualize_use_threshold", params_.visualize_use_threshold, params_.visualize_use_threshold);
    nh_.param("publish", params_.publish, params_.publish);
    nh_.param("topic", params_.topic, params_.topic);
    nh_.param("frame_id", params_.frame_id, params_.frame_id);
  }

  double StepMapBuilder::robotRadius(const std::vector<MPCPlanner::Disc> &robot_discs) const
  {
    double radius = 0.0;
    for (const auto &disc : robot_discs)
    {
      radius = std::max(radius, disc.radius);
    }
    return radius;
  }

  void StepMapBuilder::updatePose(const Eigen::Vector2d &robot_position, double heading)
  {
    Eigen::Matrix2d rot = rotationMatrix(heading);
    Eigen::Vector2d forward(forward_offset_, 0.0);
    Eigen::Vector2d center = robot_position + rot * forward;
    map_->setPose(center, heading);
  }

  void StepMapBuilder::copyStaticLayer(const costmap_2d::Costmap2D &costmap)
  {
    const unsigned int size_x = costmap.getSizeInCellsX();
    const unsigned int size_y = costmap.getSizeInCellsY();

    for (unsigned int ix = 0; ix < size_x; ++ix)
    {
      for (unsigned int iy = 0; iy < size_y; ++iy)
      {
        unsigned char cost = costmap.getCost(ix, iy);
        if (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
          continue;

        double world_x, world_y;
        costmap.mapToWorld(ix, iy, world_x, world_y);
        map_->markStaticWorld(Eigen::Vector2d(world_x, world_y));
      }
    }
  }

  void StepMapBuilder::copyDynamicObstacles(const std::vector<MPCPlanner::DynamicObstacle> &dynamic_obstacles,
                                            double robot_radius, int horizon_steps)
  {
    static thread_local std::mt19937 rng(std::random_device{}());

    auto sampleGaussianStep = [&](const Eigen::Vector2d &mean, double sigma_x, double sigma_y, int step_index) {
      if (params_.gaussian_samples <= 0 || params_.gaussian_sample_value <= 0.0)
        return;

      const double std_x = std::max(sigma_x, 1e-6);
      const double std_y = std::max(sigma_y, 1e-6);
      std::normal_distribution<double> dist_x(mean.x(), std_x);
      std::normal_distribution<double> dist_y(mean.y(), std_y);

      for (int i = 0; i < params_.gaussian_samples; ++i)
      {
        Eigen::Vector2d sample(dist_x(rng), dist_y(rng));
        map_->addCostWorld(sample, step_index, params_.gaussian_sample_value);
      }
    };

    for (const auto &obstacle : dynamic_obstacles)
    {
      bool has_gaussian_prediction = obstacle.prediction.type == MPCPlanner::PredictionType::GAUSSIAN &&
                                     !obstacle.prediction.modes.empty() && !obstacle.prediction.modes.front().empty();

      if (has_gaussian_prediction)
      {
        const auto &mode = obstacle.prediction.modes.front();
        int limit = std::min(static_cast<int>(mode.size()), horizon_steps);

        for (int step = 0; step < limit; ++step)
        {
          const auto &prediction = mode[step];
          sampleGaussianStep(prediction.position, prediction.major_radius, prediction.minor_radius, step);
        }

        if (limit < horizon_steps)
        {
          const auto &final_prediction = mode.back();
          for (int step = limit; step < horizon_steps; ++step)
          {
            sampleGaussianStep(final_prediction.position, final_prediction.major_radius, final_prediction.minor_radius, step);
          }
        }
        continue;
      }

      // Fallback for non-Gaussian predictions: use deterministic discs as before.
      double combined_radius = obstacle.radius + robot_radius;
      if (combined_radius <= 0.0)
        continue;

      Eigen::Vector2d current_position = obstacle.position;
      map_->markDynamicCircleWorld(current_position, 0, combined_radius);

      if (obstacle.prediction.modes.empty())
        continue;

      const auto &mode = obstacle.prediction.modes.front();
      if (mode.empty())
        continue;

      int limit = std::min(static_cast<int>(mode.size()), horizon_steps - 1);
      for (int step = 1; step <= limit; ++step)
      {
        int mode_index = std::min(static_cast<int>(mode.size()) - 1, step - 1);
        Eigen::Vector2d prediction = mode[mode_index].position;
        map_->markDynamicCircleWorld(prediction, step, combined_radius);
      }

      if (limit < horizon_steps - 1 && !mode.empty())
      {
        Eigen::Vector2d final_prediction = mode.back().position;
        for (int step = limit + 1; step < horizon_steps; ++step)
        {
          map_->markDynamicCircleWorld(final_prediction, step, combined_radius);
        }
      }
    }
  }
} // namespace MPCPlannerStepMap
