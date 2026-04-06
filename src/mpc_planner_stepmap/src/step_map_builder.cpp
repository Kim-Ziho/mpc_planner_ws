#include <mpc_planner_stepmap/step_map_builder.h>
#include <mpc_planner_stepmap/step_map_visualizer.h>

#include <costmap_2d/cost_values.h>

#include <ros/console.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <random>

namespace MPCPlannerStepMap
{
  namespace
  {
    constexpr double kMinResolution = 1e-4;

    // 양방향 sliding window max (monotone deque)
    // output[i] = max(input[j]) for j in [i-r, i+r]
    void bidirectionalSlidingMax1D(const double *input, double *output, int n, int r)
    {
      std::deque<int> dq;

      for (int i = 0; i < n + r; ++i)
      {
        if (i < n)
        {
          while (!dq.empty() && input[dq.back()] <= input[i])
            dq.pop_back();
          dq.push_back(i);
        }

        int out_idx = i - r;
        if (out_idx >= 0)
        {
          while (!dq.empty() && dq.front() < out_idx - r)
            dq.pop_front();

          output[out_idx] = dq.empty() ? 0.0 : input[dq.front()];
        }
      }
    }

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

    time_scale_ = time_scale;
    map_->configure(cells_x, cells_y, horizon_steps, resolution_, time_scale);
    map_->setOccupancyThreshold(params_.occupancy_threshold);

    forward_offset_ = params_.forward_offset_ratio * static_cast<double>(cells_x) * resolution_;

    updatePose(robot_position, heading);
    map_->clear();

    double robot_radius = robotRadius(robot_discs);

    if (params_.inflate_dynamic)
    {
      // inflation 모드: 동적 → inflate → 정적 (이중 inflation 방지)
      copyDynamicObstacles(dynamic_obstacles, robot_radius, horizon_steps);
      int r_cells = static_cast<int>(std::ceil(robot_radius / resolution_));
      if (r_cells > 0)
        inflateDynamicLayers(r_cells);
      copyStaticLayer(*costmap);
    }
    else
    {
      copyStaticLayer(*costmap);
      copyDynamicObstacles(dynamic_obstacles, robot_radius, horizon_steps);
    }

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
    global_nh.param("publish", params_.publish, params_.publish);
    global_nh.param("topic", params_.topic, params_.topic);
    global_nh.param("frame_id", params_.frame_id, params_.frame_id);
    global_nh.param("dynamic_method", params_.dynamic_method, params_.dynamic_method);
    global_nh.param("propagate_uncertainty", params_.propagate_uncertainty, params_.propagate_uncertainty);
    global_nh.param("stage_z_offset", params_.stage_z_offset, params_.stage_z_offset);
    global_nh.param("vis_stages", params_.vis_stages, params_.vis_stages);
    global_nh.param("inflate_dynamic", params_.inflate_dynamic, params_.inflate_dynamic);

    nh_ = ros::NodeHandle(parent_nh, "step_map");
    nh_.param("resolution_ratio", params_.resolution_ratio, params_.resolution_ratio);
    nh_.param("size_scale", params_.size_scale, params_.size_scale);
    nh_.param("forward_offset_ratio", params_.forward_offset_ratio, params_.forward_offset_ratio);
    nh_.param("max_alpha", params_.max_alpha, params_.max_alpha);
    nh_.param("z_scale", params_.z_scale, params_.z_scale);
    nh_.param("gaussian_samples", params_.gaussian_samples, params_.gaussian_samples);
    nh_.param("gaussian_sample_value", params_.gaussian_sample_value, params_.gaussian_sample_value);
    nh_.param("occupancy_threshold", params_.occupancy_threshold, params_.occupancy_threshold);
    nh_.param("publish", params_.publish, params_.publish);
    nh_.param("topic", params_.topic, params_.topic);
    nh_.param("frame_id", params_.frame_id, params_.frame_id);
    nh_.param("dynamic_method", params_.dynamic_method, params_.dynamic_method);
    nh_.param("propagate_uncertainty", params_.propagate_uncertainty, params_.propagate_uncertainty);
    nh_.param("stage_z_offset", params_.stage_z_offset, params_.stage_z_offset);
    nh_.param("vis_stages", params_.vis_stages, params_.vis_stages);
    nh_.param("inflate_dynamic", params_.inflate_dynamic, params_.inflate_dynamic);
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
    static thread_local std::normal_distribution<double> dist(0.0, 1.0);

    auto sampleGaussianStep = [&](const Eigen::Vector2d &mean, double sigma_major, double sigma_minor, double heading, int step_index) {
      if (params_.gaussian_samples <= 0 || params_.gaussian_sample_value <= 0.0)
        return;

      const double std_major = std::max(sigma_major, 1e-6);
      const double std_minor = std::max(sigma_minor, 1e-6);
      Eigen::Matrix2d rot = rotationMatrix(heading);

      for (int i = 0; i < params_.gaussian_samples; ++i)
      {
        Eigen::Vector2d local_sample(dist(rng) * std_major, dist(rng) * std_minor);
        Eigen::Vector2d sample = mean + rot * local_sample;
        map_->addCostWorld(sample, step_index, params_.gaussian_sample_value);
      }
    };

    for (const auto &obstacle : dynamic_obstacles)
    {
      bool has_gaussian_prediction = obstacle.prediction.type == MPCPlanner::PredictionType::GAUSSIAN &&
                                     !obstacle.prediction.modes.empty() && !obstacle.prediction.modes.front().empty() &&
                                     obstacle.prediction.modes.front().front().major_radius > 0.0;

      if (has_gaussian_prediction)
      {
        const auto &mode = obstacle.prediction.modes.front();
        int limit = std::min(static_cast<int>(mode.size()), horizon_steps);

        if (params_.dynamic_method == "gaussian_trajectory")
        {
          // Trajectory sampling: at each step k, draw a new velocity noise and accumulate it.
          // This integrates the noise over time (random walk), creating temporal correlation
          // analogous to ScenarioModule::IntegrateAndTranslateToMeanAndVariance.
          if (params_.gaussian_samples > 0 && params_.gaussian_sample_value > 0.0)
          {
            const double sigma = std::max(mode.front().major_radius, 1e-6);

            for (int i = 0; i < params_.gaussian_samples; ++i)
            {
              Eigen::Vector2d accumulated(0.0, 0.0);

              for (int step = 0; step < limit; ++step)
              {
                // Draw new noise at each step and accumulate (integrate)
                accumulated += Eigen::Vector2d(dist(rng) * sigma, dist(rng) * sigma);
                map_->addCostWorld(mode[step].position + accumulated * time_scale_, step, params_.gaussian_sample_value);
              }

              if (limit < horizon_steps)
              {
                const Eigen::Vector2d final_pos = mode.back().position + accumulated * time_scale_;
                for (int step = limit; step < horizon_steps; ++step)
                  map_->addCostWorld(final_pos, step, params_.gaussian_sample_value);
              }
            }
          }
        }
        else // "gaussian_independent" (default): per-step independent sampling
        {
          // propagate_uncertainty: 각 스텝의 반경을 시간에 따라 누적 전파
          // r_k = sqrt(r_{k-1}^2 + (r_input * dt)^2)
          double prop_maj = 0.0, prop_min = 0.0;

          for (int step = 0; step < limit; ++step)
          {
            const auto &prediction = mode[step];
            double maj = prediction.major_radius;
            double min = prediction.minor_radius;
            if (params_.propagate_uncertainty)
            {
              prop_maj = std::sqrt(prop_maj * prop_maj + (maj * time_scale_) * (maj * time_scale_));
              prop_min = std::sqrt(prop_min * prop_min + (min * time_scale_) * (min * time_scale_));
              maj = prop_maj;
              min = prop_min;
            }
            sampleGaussianStep(prediction.position, maj, min, prediction.angle, step);
          }

          if (limit < horizon_steps)
          {
            const auto &final_prediction = mode.back();
            double maj = final_prediction.major_radius;
            double min = final_prediction.minor_radius;
            if (params_.propagate_uncertainty)
            {
              maj = prop_maj;
              min = prop_min;
            }
            for (int step = limit; step < horizon_steps; ++step)
              sampleGaussianStep(final_prediction.position, maj, min, final_prediction.angle, step);
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
      {
        ROS_WARN_STREAM("[StepMapBuilder] Obstacle " << obstacle.index << " has no prediction modes, skipping.");
        continue;
      }

      const auto &mode = obstacle.prediction.modes.front();
      if (mode.empty())
      {
        ROS_WARN_STREAM("[StepMapBuilder] Obstacle " << obstacle.index << " has an empty prediction mode, skipping.");
        continue;
      }

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

  void StepMapBuilder::inflateDynamicLayers(int r_cells)
  {
    const int cx = map_->cellsX();
    const int cy = map_->cellsY();
    const int ct = map_->cellsT();

    // 임시 버퍼: 1개 시간층 크기
    std::vector<double> temp(static_cast<size_t>(cx) * static_cast<size_t>(cy), 0.0);
    // 1D 입출력 버퍼 (행/열 단위)
    const int max_dim = std::max(cx, cy);
    std::vector<double> row_in(max_dim), row_out(max_dim);

    for (int gt = 0; gt < ct; ++gt)
    {
      // Pass 1: 수평 (행 방향) sliding window max
      for (int gy = 0; gy < cy; ++gy)
      {
        for (int gx = 0; gx < cx; ++gx)
          row_in[gx] = map_->cellCost(gx, gy, gt);

        bidirectionalSlidingMax1D(row_in.data(), row_out.data(), cx, r_cells);

        for (int gx = 0; gx < cx; ++gx)
          temp[static_cast<size_t>(gy) * cx + gx] = row_out[gx];
      }

      // Pass 2: 수직 (열 방향) sliding window max
      for (int gx = 0; gx < cx; ++gx)
      {
        for (int gy = 0; gy < cy; ++gy)
          row_in[gy] = temp[static_cast<size_t>(gy) * cx + gx];

        bidirectionalSlidingMax1D(row_in.data(), row_out.data(), cy, r_cells);

        for (int gy = 0; gy < cy; ++gy)
          map_->setCostCell(gx, gy, gt, row_out[gy]);
      }
    }
  }
} // namespace MPCPlannerStepMap
