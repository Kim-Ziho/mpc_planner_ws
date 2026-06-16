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
    bool velocity_aware_sampling_; // Restrict PRM samples to the space-time set reachable from the start at max_velocity_

    double winding_pass_threshold_;

    int longitudinal_goals_, vertical_goals_;

    bool use_learning;
    static bool use_non_passing_;
    static bool use_dubins_path_;

    // Topology
    std::string topology_comparison_function_;
    std::string connection_type_;
    int uvd_samples_; // Longitudinal samples for UVD comparison. <= 0 => derive from StepMap resolution

    // StepMap-only mode: ignore dynamic/static obstacle data and rely solely on the StepMap
    bool step_map_only_;

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
    bool   strrt_greedy_goal_connect_;
    // corridor-guided 샘플링 (PRM best path 주변 시공간 튜브)
    int    strrt_prm_period_;            // PRM corridor 갱신 주기 [frames] (20Hz/4 ≈ 5Hz)
    double strrt_corridor_w_base_;       // 튜브 기본 반폭 [m]
    double strrt_corridor_p_explore_;    // 전역 exploration 샘플 비율
    double strrt_corridor_dt_win_minus_; // corridor 시각 창 과거측 [s]
    double strrt_corridor_dt_win_plus_;  // corridor 시각 창 미래측 [s]
    double strrt_corridor_w_risk_;       // risk 적응형 튜브 폭 증가 게인 [m] (W=w_base+w_risk·p)
    double strrt_corridor_w_max_;        // 튜브 반폭 상한 [m]
    // risk-aware edge cost (soft risk 적분)
    double strrt_risk_w_risk_;           // risk 적분 비용 가중치
    double strrt_risk_tau_soft_;         // p < tau_soft 는 risk 적분에서 무시 (가우시안 꼬리)
    // risk 비례 감속 (steer 속도 상한)
    double strrt_risk_v_min_ratio_;      // v_min = v_min_ratio·v_max (로봇 freeze 방지 바닥)
    double strrt_risk_beta_;             // g(p)=(1-p)^β 감속 곡률 (클수록 급감속)

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
