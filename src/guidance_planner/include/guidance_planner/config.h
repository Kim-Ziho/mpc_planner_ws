/**
 * @file homotopy_config.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Loads parameters for PRM
 * @version 0.1
 * @date 2022-07-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef HOMOTOPY_CONFIGURATION_H
#define HOMOTOPY_CONFIGURATION_H

#include <ros_tools/base_configuration.h>

#include <string>

namespace GuidancePlanner
{
  class Config : public RosTools::BaseConfiguration
  {

  public:
    Config();

    Config(const Config &other) = delete;

    /************ CONFIGURATION VARIABLES **************/

    // Debug
    static bool debug_output_;
    static bool debug_visuals_;

    // Homotopy (Key variables)
    double T_;
    static int N;
    static double DT;
    static double CONTROL_DT;
    static double turning_radius_;

    // Other statics
    static double reference_velocity_;

    // PRM Settings
    int seed_;
    double obstacle_radius_extension_;
    int n_samples_;
    double timeout_;
    bool assume_constant_velocity_;
    bool track_selected_homology_only_;
    int n_paths_;
    int path_after_samples_;
    double prefer_goal_over_smoothness_;
    double max_velocity_, max_acceleration_;

    double winding_pass_threshold_;

    int longitudinal_goals_, vertical_goals_;

    bool use_learning;
    static bool use_non_passing_;
    static bool use_dubins_path_;

    // Topology
    std::string topology_comparison_function_;
    std::string connection_type_;

    // Connection Filters
    bool enable_forward_filter_;
    bool enable_velocity_filter_;
    bool enable_acceleration_filter_;

    // Sampling parameters
    double sample_margin_;

    // Weights (deprecated, only here so that cubicspline3d still compiles)
    double geometric_weight_, smoothness_weight_, collision_weight_, velocity_tracking_;

    bool optimize_splines_;
    double selection_weight_length_, selection_weight_velocity_, selection_weight_acceleration_;

    // Spline selection weights
    double selection_weight_consistency_;

    double visuals_transparency_;
    bool show_trajectory_indices_;

    // Spline settings
    int num_points_;

    // Toggles
    bool visualize_all_samples_, visualize_homology_;
    bool dynamically_propagate_nodes_;
    bool project_from_obstacles_;
    bool debug_continuous_replanning_;

    // Algorithm selector
    std::string algorithm_;   // "PRM" | "AStar"

    // A* planner parameters
    int    astar_num_headings_;
    double astar_w_max_;
    double astar_w_time_, astar_w_occ_, astar_w_accel_, astar_w_yaw_;

    // Hybrid A* planner parameters
    int    hastar_num_heading_bins_;
    int    hastar_speed_bins_;
    int    hastar_n_v_samples_;
    int    hastar_n_w_samples_;
    int    hastar_n_substeps_;
    double hastar_w_max_;
    double hastar_a_max_;
    double hastar_goal_tol_xy_;
    double hastar_w_time_, hastar_w_occ_, hastar_w_accel_, hastar_w_yaw_, hastar_w_yaw_rate_;
    double hastar_time_budget_ms_;

    // ST-RRT* planner parameters
    int    strrt_max_iter_;
    double strrt_steer_dt_min_;
    double strrt_steer_dt_max_;
    double strrt_neighbor_radius_;
    double strrt_match_tol_;
    double strrt_goal_radius_;
    double strrt_goal_bias_;
    double strrt_w_time_;
    double strrt_w_ctrl_;
    double strrt_check_dt_;
    double strrt_path_lat_half_width_;

    // Risk-Aware ST-RRT* planner parameters
    int    ra_strrt_max_iter_;
    int    ra_strrt_k_rrtstar_;
    int    ra_strrt_max_step_cells_;
    int    ra_strrt_initial_goal_count_;
    int    ra_strrt_max_goal_count_;
    int    ra_strrt_max_nodes_;
    int    ra_strrt_cold_start_min_remaining_nodes_;
    double ra_strrt_v_max_, ra_strrt_v_preferred_, ra_strrt_w_max_;
    double ra_strrt_tau_hard_, ra_strrt_tau_soft_;
    double ra_strrt_w_time_, ra_strrt_w_risk_, ra_strrt_w_progress_, ra_strrt_w_curvature_;
    double ra_strrt_w_goal_progress_, ra_strrt_lambda_nn_;
    double ra_strrt_tube_width_, ra_strrt_s_min_offset_;
    double ra_strrt_t_min_, ra_strrt_t_max_;
    double ra_strrt_time_budget_ms_;
    bool   ra_strrt_enable_warm_start_;
  };
}

#endif
