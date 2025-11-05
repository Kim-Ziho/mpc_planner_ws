#include <mpc_planner_stepmap/step_map_visualizer.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

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
  }

  void StepMapVisualizer::publish(const StepMap &map)
  {
    if (publisher_.getNumSubscribers() == 0)
      return;

    prepareMarker(map);
    marker_.header.frame_id = params_.frame_id;
    marker_.header.stamp = ros::Time::now();
    publisher_.publish(marker_);
  }

  void StepMapVisualizer::prepareMarker(const StepMap &map)
  {
    points_buffer_.clear();
    colors_buffer_.clear();

    size_t expected_points = static_cast<size_t>(map.cellsX()) * static_cast<size_t>(map.cellsY()) * static_cast<size_t>(map.cellsT());
    points_buffer_.reserve(expected_points);
    colors_buffer_.reserve(expected_points);

    marker_.scale.x = map.resolution();
    marker_.scale.y = map.resolution();
    marker_.scale.z = params_.z_scale;

    for (int gx = 0; gx < map.cellsX(); ++gx)
    {
      for (int gy = 0; gy < map.cellsY(); ++gy)
      {
        for (int gt = 0; gt < map.cellsT(); ++gt)
        {
          if (!map.cellOccupied(gx, gy, gt))
            continue;

          Eigen::Vector2d world_point = map.worldFromCell(gx, gy);

          geometry_msgs::Point marker_point;
          marker_point.x = world_point.x();
          marker_point.y = world_point.y();
          marker_point.z = map.layerHeight(gt);

          std_msgs::ColorRGBA color;
          color.r = 189.0 / 255.0;
          color.g = 147.0 / 255.0;
          color.b = 249.0 / 255.0;
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
