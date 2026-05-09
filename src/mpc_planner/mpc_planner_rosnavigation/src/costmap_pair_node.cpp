// Standalone bringup of a nav2_costmap_2d::Costmap2DROS local_costmap
// instance, using the same NodeOptions trick that JackalPlanner::initializeCostmap
// uses. The stock "nav2_costmap_2d" executable hardcodes the node name and
// ignores __node remap, which makes running an instance under an external
// lifecycle_manager unworkable.
//
// Used by jackal_world_test.launch.py for component-level testing of jackal
// motion under cmd_vel without the full MPC stack. The global_costmap there
// is owned by nav2_planner's planner_server.

#include <chrono>
#include <csignal>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/node_thread.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
std::shared_ptr<nav2_costmap_2d::Costmap2DROS>
bringUpCostmap(const std::string & node_name, const std::string & yaml_path)
{
  // Costmap2DROS publishes to relative topics ("costmap", "costmap_raw", etc.)
  // and our node lives at namespace "/", so without remaps both instances
  // would collide on /costmap. Remap each publisher onto a node-name prefix
  // so RViz can distinguish /local_costmap/costmap from /global_costmap/costmap.
  rclcpp::NodeOptions opts;
  opts.arguments({
    "--ros-args",
    "-r", "__node:=" + node_name,
    "-r", "costmap:=" + node_name + "/costmap",
    "-r", "costmap_raw:=" + node_name + "/costmap_raw",
    "-r", "costmap_updates:=" + node_name + "/costmap_updates",
    "-r", "published_footprint:=" + node_name + "/published_footprint",
    "--params-file", yaml_path,
  });

  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(opts);
  costmap->configure();
  costmap->activate();
  return costmap;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const std::string pkg_share =
    ament_index_cpp::get_package_share_directory("mpc_planner_rosnavigation");

  auto local_costmap = bringUpCostmap("local_costmap", pkg_share + "/config/local_costmap.yaml");
  auto local_thread = std::make_unique<nav2_util::NodeThread>(local_costmap);

  auto idle = std::make_shared<rclcpp::Node>("costmap_pair_node");
  RCLCPP_INFO(idle->get_logger(),
              "local_costmap is active. Spinning idle node.");

  rclcpp::spin(idle);

  local_costmap->deactivate();
  local_costmap->cleanup();

  rclcpp::shutdown();
  return 0;
}
