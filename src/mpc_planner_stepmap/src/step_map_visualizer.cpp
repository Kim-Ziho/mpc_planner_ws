#include <mpc_planner_stepmap/step_map_visualizer.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include <algorithm>
namespace MPCPlannerStepMap
{
  StepMapVisualizer::StepMapVisualizer(ros::NodeHandle &nh, const StepMapParameters &params)
      : params_(params)
  {
    publisher_ = nh.advertise<visualization_msgs::Marker>(params_.topic, 1);

    marker_.ns = "step_map";
    marker_.type = visualization_msgs::Marker::CUBE_LIST;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.orientation.w = 1.0;
    marker_.color.a = 1.0; // fallback when per-point colors are not set
  }

  void StepMapVisualizer::publish(const StepMap &map)
  {
    if (publisher_.getNumSubscribers() == 0)
      return;

    prepareMarker(map);
    if (marker_.points.empty())
      return;

    marker_.header.frame_id = params_.frame_id;
    marker_.header.stamp = ros::Time::now();
    publisher_.publish(marker_);
  }

  void StepMapVisualizer::prepareMarker(const StepMap &map)
  {
    points_buffer_.clear();
    colors_buffer_.clear();

    // vis_stages에 따라 어떤 시간층을 시각화할지 결정 — O(vis_stages) 전처리
    int cells_t = map.cellsT();
    std::vector<bool> vis_mask(cells_t, false);
    int n = params_.vis_stages;
    if (n <= 0 || n >= cells_t)
    {
      std::fill(vis_mask.begin(), vis_mask.end(), true);
      n = cells_t;
    }
    else if (n == 1)
    {
      vis_mask[0] = true;
    }
    else
    {
      for (int i = 0; i < n; ++i)
      {
        int idx = static_cast<int>(std::round(i * (cells_t - 1.0) / (n - 1)));
        vis_mask[idx] = true;
      }
    }

    size_t expected_points = static_cast<size_t>(map.cellsX()) * static_cast<size_t>(map.cellsY()) * static_cast<size_t>(n);
    points_buffer_.reserve(expected_points);
    colors_buffer_.reserve(expected_points);

    marker_.scale.x = map.resolution();
    marker_.scale.y = map.resolution();
    marker_.scale.z = params_.z_scale;

    for (int gx = 0; gx < map.cellsX(); ++gx)
    {
      for (int gy = 0; gy < map.cellsY(); ++gy)
      {
        for (int gt = 0; gt < cells_t; ++gt)
        {
          if (!vis_mask[gt])
            continue;
          double cost = map.cellCost(gx, gy, gt);
          // visualize_occupied_only: 충돌 판정과 동일하게 occupancy_threshold 이상만 표시
          // (false일 때는 기존 동작 유지 — cost > 0 인 모든 셀 표시)
          if (params_.visualize_occupied_only)
          {
            if (cost < params_.occupancy_threshold)
              continue;
          }
          else if (cost <= 0.0)
          {
            continue;
          }

          Eigen::Vector2d world_point = map.worldFromCell(gx, gy);

          geometry_msgs::Point marker_point;
          marker_point.x = world_point.x();
          marker_point.y = world_point.y();
          marker_point.z = map.layerHeight(gt) + static_cast<double>(gt) * params_.stage_z_offset;

          std_msgs::ColorRGBA color;
          if (cost >= params_.occupancy_threshold)
          {
            // occupancy_threshold 이상으로 점유된 셀 → 진한 보라색으로 강조
            color.r = 0.29f;
            color.g = 0.0f;
            color.b = 0.51f;
          }
          else
          {
            // gamma correction: 낮은 cost 값들을 더 넓은 색 범위에 펼쳐서 구별 용이하게 함
            // gamma < 1이면 낮은 값 → 노랑/주황으로 이동, 1.0이면 기존 선형 매핑
            double display_cost = std::pow(std::clamp(cost, 0.0, 1.0), params_.color_gamma);
            double hue = (1.0 - display_cost) * 120.0;
            double f = hue / 60.0;
            if (hue < 60.0) { // red → yellow
              color.r = 1.0f;
              color.g = static_cast<float>(f);
              color.b = 0.0f;
            } else { // yellow → green
              color.r = static_cast<float>(2.0 - f);
              color.g = 1.0f;
              color.b = 0.0f;
            }
          }
          color.a = params_.max_alpha;

          points_buffer_.push_back(marker_point);
          colors_buffer_.push_back(color);
        }
      }
    }

    marker_.points = points_buffer_;
    marker_.colors = colors_buffer_;
    marker_.lifetime = ros::Duration(0.0);
  }
} // namespace MPCPlannerStepMap
