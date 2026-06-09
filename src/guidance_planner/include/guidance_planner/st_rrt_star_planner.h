#pragma once

#include <guidance_planner/config.h>
#include <guidance_planner/types/paths.h>
#include <guidance_planner/types/node.h>
#include <mpc_planner_stepmap/step_map.h>

#include <Eigen/Dense>
#include <list>
#include <memory>
#include <optional>
#include <random>
#include <vector>

namespace RosTools { class Spline2D; }

namespace GuidancePlanner
{

class STRRTStarPlanner
{
public:
  void Init(Config *config);
  void SetStepMap(std::shared_ptr<MPCPlannerStepMap::StepMap> step_map);
  void Reset();

  /** @brief StepMap 기반 단일 최적 경로 탐색 (ST-RRT*)
   *  @return 성공 시 GeometricPath, 실패 시 std::nullopt */
  std::optional<GeometricPath> Plan(const Eigen::Vector2d &start_xy,
                                    double start_theta,
                                    double start_speed,
                                    const Eigen::Vector2d &goal_xy,
                                    const std::shared_ptr<RosTools::Spline2D> &reference_path = nullptr,
                                    double spline_start = 0.0);

private:
  struct RRTNode
  {
    double x, y, theta, t;
    double v, w;
    double cost;
    int    parent;
    std::vector<int> children;
  };

  struct SteerResult
  {
    double x, y, theta, t, v, w;
  };

  struct Sample
  {
    double x, y, t;
  };

  std::optional<SteerResult> steer(const RRTNode &from,
                                   double x_to, double y_to, double t_to) const;

  bool edgeCollisionFree(const RRTNode &from, double v, double w, double dt) const;

  struct SampleContext
  {
    double t_upper;
    Eigen::Vector2d goal_xy;
    double t_min_goal;
    double x_min, x_max, y_min, y_max;
    std::shared_ptr<RosTools::Spline2D> reference_path;
    double cur_s;
    double max_s;
  };

  std::optional<Sample> sampleState(const SampleContext &ctx) const;

  double timeAwareDist(const RRTNode &a, double x, double y, double t) const;

  double edgeCost(double dt, double v, double w) const;

  GeometricPath reconstructPath(const std::vector<RRTNode> &nodes, int goal_idx);

  static void unicycleStep(double &x, double &y, double &theta,
                           double v, double w, double dt);

  Config *config_{nullptr};
  std::shared_ptr<MPCPlannerStepMap::StepMap> step_map_;
  std::list<Node> path_nodes_;

  int    max_iter_;
  double steer_dt_min_, steer_dt_max_;
  double neighbor_radius_;
  double match_tol_;
  double goal_radius_;
  double goal_bias_;
  double w_time_, w_ctrl_;
  double v_max_, w_max_;
  double check_dt_;
  double path_lat_half_width_;
  bool   greedy_goal_connect_;

  // sample_accept_rate 누적 통계 (Plan() 호출마다 갱신)
  double accept_rate_sum_{0.0};
  int    plan_call_count_{0};
  int    plan_fail_count_{0};  // best_idx < 0 (No path found) 누적 횟수

  mutable std::mt19937 rng_;
};

}  // namespace GuidancePlanner
