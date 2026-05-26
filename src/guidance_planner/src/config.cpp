#include <guidance_planner/config.h>

#include <ros_tools/logging.h>

#ifndef MPC_PLANNER_ROS
#include <ros_tools/ros2_wrappers.h>
#endif

namespace GuidancePlanner
{
  // Need to be initialized outside of a member function
  bool Config::debug_output_ = false;
  bool Config::debug_visuals_ = false;
  double Config::DT = 0.05;
  double Config::CONTROL_DT = 0.0;
  int Config::N = 20;

  bool Config::use_non_passing_ = false;
  bool Config::use_dubins_path_ = false;
  double Config::reference_velocity_ = 2.0; // Is updated based on rqt_reconfigure
  double Config::turning_radius_ = 0.5;     // Is updated based on rqt_reconfigure

  Config::Config()
  {
#ifdef MPC_PLANNER_ROS
    ros::NodeHandle node;
#else
    LOG_INFO("Get static node pointer");
    rclcpp::Node *node = GET_STATIC_NODE_POINTER();
#endif

    retrieveParameter(node, "guidance_planner/debug/output", Config::debug_output_);
    retrieveParameter(node, "guidance_planner/debug/visuals", Config::debug_visuals_);

    // High-level settings
    retrieveParameter(node, "guidance_planner/T", T_);
    retrieveParameter(node, "guidance_planner/N", Config::N);
    Config::DT = T_ / (double)Config::N;

    retrieveParameter(node, "clock_frequency", Config::CONTROL_DT, 10.); // NOTE: from LMPCC
    Config::CONTROL_DT = 1. / Config::CONTROL_DT;                        // dt = 1 / Hz

    retrieveParameter(node, "guidance_planner/seed", seed_);
    retrieveParameter(node, "guidance_planner/sampling/n_samples", n_samples_);
    retrieveParameter(node, "guidance_planner/sampling/timeout", timeout_);
    retrieveParameter(node, "guidance_planner/sampling/margin", sample_margin_, 0.);

    retrieveParameter(node, "guidance_planner/homotopy/n_paths", n_paths_);
    retrieveParameter(node, "guidance_planner/homotopy/track_selected_homology_only", track_selected_homology_only_);
    retrieveParameter(node, "guidance_planner/homotopy/comparison_function", topology_comparison_function_, std::string("Homology"));
    retrieveParameter(node, "guidance_planner/homotopy/winding/pass_threshold", winding_pass_threshold_, 0.25);
    retrieveParameter(node, "guidance_planner/homotopy/winding/use_non_passing", Config::use_non_passing_, false);
    retrieveParameter(node, "guidance_planner/homotopy/use_learning", use_learning, false);

    retrieveParameter(node, "guidance_planner/predictions_are_constant_velocity", assume_constant_velocity_);

    retrieveParameter(node, "guidance_planner/dynamics/connections", connection_type_, std::string("Straight"));
    Config::use_dubins_path_ = connection_type_ == "Dubins";
    retrieveParameter(node, "guidance_planner/dynamics/turning_radius", Config::turning_radius_, 0.5);

    retrieveParameter(node, "guidance_planner/goals/longitudinal", longitudinal_goals_);
    retrieveParameter(node, "guidance_planner/goals/vertical", vertical_goals_);

    retrieveParameter(node, "guidance_planner/max_velocity", max_velocity_);
    retrieveParameter(node, "guidance_planner/max_acceleration", max_acceleration_);

    retrieveParameter(node, "guidance_planner/connection_filters/forward", enable_forward_filter_);
    retrieveParameter(node, "guidance_planner/connection_filters/velocity", enable_velocity_filter_);
    retrieveParameter(node, "guidance_planner/connection_filters/acceleration", enable_acceleration_filter_);

    retrieveParameter(node, "guidance_planner/spline_optimization/enable", optimize_splines_);
    retrieveParameter(node, "guidance_planner/spline_optimization/geometric", geometric_weight_);
    retrieveParameter(node, "guidance_planner/spline_optimization/smoothness", smoothness_weight_);
    retrieveParameter(node, "guidance_planner/spline_optimization/collision", collision_weight_);
    retrieveParameter(node, "guidance_planner/spline_optimization/velocity_tracking", velocity_tracking_);

    // Parameters that determine the heuristic spline weighting
    retrieveParameter(node, "guidance_planner/selection_weights/length", selection_weight_length_);
    retrieveParameter(node, "guidance_planner/selection_weights/velocity", selection_weight_velocity_);
    retrieveParameter(node, "guidance_planner/selection_weights/acceleration", selection_weight_acceleration_);

    retrieveParameter(node, "guidance_planner/selection_weights/consistency", selection_weight_consistency_);

    retrieveParameter(node, "guidance_planner/spline_optimization/num_points", num_points_);
    if (num_points_ == -1)
      num_points_ = N;

    retrieveParameter(node, "guidance_planner/visuals/transparency", visuals_transparency_);
    retrieveParameter(node, "guidance_planner/visuals/visualize_all_samples", visualize_all_samples_);
    retrieveParameter(node, "guidance_planner/visuals/visualize_homology", visualize_homology_);
    retrieveParameter(node, "guidance_planner/visuals/show_indices", show_trajectory_indices_);

    retrieveParameter(node, "guidance_planner/enable/dynamically_propagate_nodes", dynamically_propagate_nodes_);
    retrieveParameter(node, "guidance_planner/enable/project_from_obstacles", project_from_obstacles_);

    retrieveParameter(node, "guidance_planner/test_node/continuous_replanning", debug_continuous_replanning_);

    retrieveParameter(node, "guidance_planner/algorithm", algorithm_, std::string("PRM"));
    retrieveParameter(node, "guidance_planner/astar/num_headings", astar_num_headings_, 16);
    retrieveParameter(node, "guidance_planner/astar/w_max",   astar_w_max_,   0.8);
    retrieveParameter(node, "guidance_planner/astar/w_time",  astar_w_time_,  1.0);
    retrieveParameter(node, "guidance_planner/astar/w_occ",   astar_w_occ_,   5.0);
    retrieveParameter(node, "guidance_planner/astar/w_accel", astar_w_accel_, 0.2);
    retrieveParameter(node, "guidance_planner/astar/w_yaw",   astar_w_yaw_,   0.5);

    retrieveParameter(node, "guidance_planner/hybrid_astar/num_heading_bins", hastar_num_heading_bins_, 16);
    retrieveParameter(node, "guidance_planner/hybrid_astar/speed_bins",       hastar_speed_bins_,       2);
    retrieveParameter(node, "guidance_planner/hybrid_astar/n_v_samples",      hastar_n_v_samples_,      2);
    retrieveParameter(node, "guidance_planner/hybrid_astar/n_w_samples",      hastar_n_w_samples_,      5);
    retrieveParameter(node, "guidance_planner/hybrid_astar/n_substeps",       hastar_n_substeps_,       3);
    retrieveParameter(node, "guidance_planner/hybrid_astar/w_max",            hastar_w_max_,            1.5);
    retrieveParameter(node, "guidance_planner/hybrid_astar/a_max",            hastar_a_max_,            8.0);
    retrieveParameter(node, "guidance_planner/hybrid_astar/goal_tol_xy",      hastar_goal_tol_xy_,      0.5);
    retrieveParameter(node, "guidance_planner/hybrid_astar/w_time",           hastar_w_time_,           1.0);
    retrieveParameter(node, "guidance_planner/hybrid_astar/w_occ",            hastar_w_occ_,            5.0);
    retrieveParameter(node, "guidance_planner/hybrid_astar/w_accel",          hastar_w_accel_,          0.2);
    retrieveParameter(node, "guidance_planner/hybrid_astar/w_yaw",            hastar_w_yaw_,            0.5);
    retrieveParameter(node, "guidance_planner/hybrid_astar/w_yaw_rate",       hastar_w_yaw_rate_,       0.1);
    retrieveParameter(node, "guidance_planner/hybrid_astar/time_budget_ms",   hastar_time_budget_ms_,   45.0);

    retrieveParameter(node, "guidance_planner/st_rrt/max_iter",        strrt_max_iter_,        3000);
    retrieveParameter(node, "guidance_planner/st_rrt/steer_dt_min",    strrt_steer_dt_min_,    0.2);
    retrieveParameter(node, "guidance_planner/st_rrt/steer_dt_max",    strrt_steer_dt_max_,    0.8);
    retrieveParameter(node, "guidance_planner/st_rrt/neighbor_radius", strrt_neighbor_radius_, 2.0);
    retrieveParameter(node, "guidance_planner/st_rrt/match_tol",       strrt_match_tol_,       0.4);
    retrieveParameter(node, "guidance_planner/st_rrt/goal_radius",     strrt_goal_radius_,     0.5);
    retrieveParameter(node, "guidance_planner/st_rrt/goal_bias",       strrt_goal_bias_,       0.10);
    retrieveParameter(node, "guidance_planner/st_rrt/w_time",          strrt_w_time_,          1.0);
    retrieveParameter(node, "guidance_planner/st_rrt/w_ctrl",          strrt_w_ctrl_,          0.05);
    retrieveParameter(node, "guidance_planner/st_rrt/check_dt",        strrt_check_dt_,        0.05);
    retrieveParameter(node, "guidance_planner/st_rrt/path_lat_half_width", strrt_path_lat_half_width_, 3.0);

    // Risk-Aware ST-RRT*
    retrieveParameter(node, "guidance_planner/ra_strrt/max_iter",            ra_strrt_max_iter_,            2000);
    retrieveParameter(node, "guidance_planner/ra_strrt/k_rrtstar",           ra_strrt_k_rrtstar_,           10);
    retrieveParameter(node, "guidance_planner/ra_strrt/max_step_cells",      ra_strrt_max_step_cells_,      3);
    retrieveParameter(node, "guidance_planner/ra_strrt/initial_goal_count",  ra_strrt_initial_goal_count_,  10);
    retrieveParameter(node, "guidance_planner/ra_strrt/max_goal_count",      ra_strrt_max_goal_count_,      50);
    retrieveParameter(node, "guidance_planner/ra_strrt/max_nodes",           ra_strrt_max_nodes_,           4000);
    retrieveParameter(node, "guidance_planner/ra_strrt/cold_start_min_remaining_nodes",
                                                                              ra_strrt_cold_start_min_remaining_nodes_, 50);

    retrieveParameter(node, "guidance_planner/ra_strrt/v_max",               ra_strrt_v_max_,               3.0);
    retrieveParameter(node, "guidance_planner/ra_strrt/v_preferred",         ra_strrt_v_preferred_,         2.0);
    retrieveParameter(node, "guidance_planner/ra_strrt/w_max",               ra_strrt_w_max_,               2.0);

    retrieveParameter(node, "guidance_planner/ra_strrt/risk/tau_hard",       ra_strrt_tau_hard_,            0.7);
    retrieveParameter(node, "guidance_planner/ra_strrt/risk/tau_soft",       ra_strrt_tau_soft_,            0.2);

    retrieveParameter(node, "guidance_planner/ra_strrt/cost/w_time",         ra_strrt_w_time_,              1.0);
    retrieveParameter(node, "guidance_planner/ra_strrt/cost/w_risk",         ra_strrt_w_risk_,              2.0);
    retrieveParameter(node, "guidance_planner/ra_strrt/cost/w_progress",     ra_strrt_w_progress_,          0.5);
    retrieveParameter(node, "guidance_planner/ra_strrt/cost/w_curvature",    ra_strrt_w_curvature_,         0.3);
    retrieveParameter(node, "guidance_planner/ra_strrt/cost/w_goal_progress",ra_strrt_w_goal_progress_,     1.0);
    retrieveParameter(node, "guidance_planner/ra_strrt/cost/lambda_nn",      ra_strrt_lambda_nn_,           0.5);

    retrieveParameter(node, "guidance_planner/ra_strrt/goal/tube_width",     ra_strrt_tube_width_,          1.0);
    retrieveParameter(node, "guidance_planner/ra_strrt/goal/s_min_offset",   ra_strrt_s_min_offset_,        0.5);
    retrieveParameter(node, "guidance_planner/ra_strrt/goal/t_min",          ra_strrt_t_min_,               1.0);
    retrieveParameter(node, "guidance_planner/ra_strrt/goal/t_max",          ra_strrt_t_max_,               4.0);

    retrieveParameter(node, "guidance_planner/ra_strrt/time_budget_ms",      ra_strrt_time_budget_ms_,      90.0);
    retrieveParameter(node, "guidance_planner/ra_strrt/enable_warm_start",   ra_strrt_enable_warm_start_,   true);
  }
}
