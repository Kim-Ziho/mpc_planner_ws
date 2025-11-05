#ifndef MPC_PLANNER_STEPMAP_STEP_MAP_VISUALIZER_H
#define MPC_PLANNER_STEPMAP_STEP_MAP_VISUALIZER_H

#include <mpc_planner_stepmap/step_map.h>
#include <mpc_planner_stepmap/step_map_builder.h>

#include <visualization_msgs/Marker.h>

#include <ros/ros.h>

#include <vector>

namespace MPCPlannerStepMap
{
  class StepMapVisualizer
  {
  public:
    StepMapVisualizer(ros::NodeHandle &nh, const StepMapParameters &params);

    void publish(const StepMap &map);

  private:
    void prepareMarker(const StepMap &map);

  private:
    ros::Publisher publisher_;
    StepMapParameters params_;

    visualization_msgs::Marker marker_;
    std::vector<geometry_msgs::Point> points_buffer_;
    std::vector<std_msgs::ColorRGBA> colors_buffer_;
  };
} // namespace MPCPlannerStepMap

#endif // MPC_PLANNER_STEPMAP_STEP_MAP_VISUALIZER_H
