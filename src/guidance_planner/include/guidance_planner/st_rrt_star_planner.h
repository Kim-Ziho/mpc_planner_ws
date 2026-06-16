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

namespace GuidancePlanner
{

struct PathCorridor;

class STRRTStarPlanner
{
public:
  void Init(Config *config);
  void SetStepMap(std::shared_ptr<MPCPlannerStepMap::StepMap> step_map);
  void Reset();

  /** @brief StepMap 기반 단일 최적 경로 탐색 (ST-RRT*)
   *  @param corridor PRM best path 시공간 corridor (nullptr 이면 corridor 없이 탐색)
   *  @return 성공 시 GeometricPath, 실패 시 std::nullopt */
  std::optional<GeometricPath> Plan(const Eigen::Vector2d &start_xy,
                                    double start_theta,
                                    double start_speed,
                                    const Eigen::Vector2d &goal_xy,
                                    const PathCorridor *corridor = nullptr);

  /** @brief (v,w) 호 시퀀스를 닫힌 식으로 조밀 평가한 reference 궤적.
   *  cubic 이 sparse 노드를 피팅해 왜곡하는 대신, arc 위 샘플을 그대로 담아
   *  MPC reference(GetPath: 호길이 s, GetTrajectory: 시간 k)로 쓴다. */
  struct ArcTrajectory
  {
    std::vector<double> xs, ys; // 위치 샘플
    std::vector<double> ks;     // 시간-index k (= t/DT)
    std::vector<double> ss;     // 누적 호길이 [m]
    bool valid{false};
  };

  /** @brief 마지막 Plan() 의 arc reference 궤적 (reconstructPath 에서 채움) */
  const ArcTrajectory &GetLastArcTrajectory() const { return last_arc_traj_; }

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

  /** @brief 엣지를 스윕하며 hard 충돌 검사 + soft risk 적분을 동시에 수행.
   *  @param risk_integral [out] ∫φ(p)·dl  (충돌 시 값 미정의)
   *  @return 충돌 없으면 true */
  bool edgeEvaluate(const RRTNode &from, double v, double w, double dt,
                    double &risk_integral) const;

  /** @brief 점유확률 p → risk 밀도 φ. p<tau_soft 는 0, 그 외 -log(max(ε,1-p)). */
  double riskDensity(double p) const;

  struct SampleContext
  {
    double t_upper;
    Eigen::Vector2d goal_xy;
    double t_min_goal;
    double x_min, x_max, y_min, y_max;
    const PathCorridor *corridor{nullptr};
  };

  std::optional<Sample> sampleState(const SampleContext &ctx) const;

  double timeAwareDist(const RRTNode &a, double x, double y, double t) const;

  double edgeCost(double dt, double v, double w, double risk_integral) const;

  GeometricPath reconstructPath(const std::vector<RRTNode> &nodes, int goal_idx);

  /** @brief chain(root→goal)의 각 (v,w) 호를 조밀 평가해 last_arc_traj_ 채움 */
  void buildArcTrajectory(const std::vector<RRTNode> &nodes,
                          const std::vector<int> &chain);

  static void unicycleStep(double &x, double &y, double &theta,
                           double v, double w, double dt);

  Config *config_{nullptr};
  std::shared_ptr<MPCPlannerStepMap::StepMap> step_map_;
  std::list<Node> path_nodes_;
  ArcTrajectory last_arc_traj_;

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
  // corridor-guided 샘플링 파라미터
  double corridor_w_base_;
  double corridor_p_explore_;
  double corridor_dt_win_minus_;
  double corridor_dt_win_plus_;
  double corridor_w_risk_;   // risk 적응형 튜브 폭 게인
  double corridor_w_max_;    // 튜브 폭 상한
  // risk-aware edge cost
  double risk_w_risk_;       // risk 적분 비용 가중치
  double risk_tau_soft_;     // soft 무시 임계
  // risk 비례 감속 (steer 속도 상한)
  double risk_v_min_ratio_;
  double risk_beta_;

  // sample_accept_rate 누적 통계 (Plan() 호출마다 갱신)
  double accept_rate_sum_{0.0};
  int    plan_call_count_{0};
  int    plan_fail_count_{0};  // best_idx < 0 (No path found) 누적 횟수

  mutable std::mt19937 rng_;
};

}  // namespace GuidancePlanner
