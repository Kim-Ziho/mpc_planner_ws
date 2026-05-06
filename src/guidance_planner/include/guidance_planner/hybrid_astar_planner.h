#pragma once

#include <guidance_planner/config.h>
#include <guidance_planner/types/paths.h>
#include <guidance_planner/types/node.h>
#include <mpc_planner_stepmap/step_map.h>

#include <Eigen/Dense>
#include <list>
#include <memory>
#include <optional>
#include <queue>
#include <unordered_map>
#include <vector>

namespace GuidancePlanner
{

class HybridAStarPlanner
{
public:
  void Init(Config *config);
  void SetStepMap(std::shared_ptr<MPCPlannerStepMap::StepMap> step_map);
  void Reset();

  /** @brief StepMap 기반 단일 Hybrid A* 경로 탐색
   *  @return 성공 시 GeometricPath, 실패 시 std::nullopt */
  std::optional<GeometricPath> Plan(const Eigen::Vector2d &start_xy,
                                    double start_theta,
                                    double start_speed,
                                    const Eigen::Vector2d &goal_xy);

private:
  // ---- 내부 타입 ----

  struct ClosedKey
  {
    int i, j, k, h_bin, v_bin;
  };

  struct ClosedKeyHash
  {
    size_t operator()(const ClosedKey &s) const noexcept
    {
      size_t seed = 0;
      auto hc = [&](int v)
      { seed ^= std::hash<int>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2); };
      hc(s.i);
      hc(s.j);
      hc(s.k);
      hc(s.h_bin);
      hc(s.v_bin);
      return seed;
    }
  };

  struct ClosedKeyEq
  {
    bool operator()(const ClosedKey &a, const ClosedKey &b) const noexcept
    {
      return a.i == b.i && a.j == b.j && a.k == b.k &&
             a.h_bin == b.h_bin && a.v_bin == b.v_bin;
    }
  };

  struct SearchNode
  {
    double f, g;
    int counter;
    // 연속 상태
    double x, y, theta, v;
    int k;
    // 부모 참조 (경로 재구성용)
    std::shared_ptr<SearchNode> parent;

    bool operator>(const SearchNode &o) const noexcept
    {
      return f != o.f ? f > o.f : counter > o.counter;
    }
  };

  struct SearchNodePtrCmp
  {
    bool operator()(const std::shared_ptr<SearchNode> &a,
                    const std::shared_ptr<SearchNode> &b) const noexcept
    {
      return *a > *b;
    }
  };

  // ---- 헬퍼 ----
  ClosedKey makeKey(double x, double y, double theta, double v, int k) const;

  /** @brief 유니사이클 Euler 적분: (x,y,theta) → sub-step 위치 목록 */
  std::vector<Eigen::Vector2d> integrate(double x, double y, double theta,
                                          double v_cmd, double w_cmd) const;

  /** @brief sub-step 위치 목록에 대해 StepMap 충돌 검사 + 점유 비용 누계 */
  bool checkSwept(const std::vector<Eigen::Vector2d> &pts, int nk,
                  double &occ_total) const;

  double heuristic(double x, double y, double gx, double gy) const;

  GeometricPath reconstructPath(std::shared_ptr<SearchNode> goal_node);

  void pushIfBetter(
      std::priority_queue<std::shared_ptr<SearchNode>,
                          std::vector<std::shared_ptr<SearchNode>>,
                          SearchNodePtrCmp> &pq,
      std::unordered_map<ClosedKey, double, ClosedKeyHash, ClosedKeyEq> &best_g,
      std::shared_ptr<SearchNode> parent,
      double v_cmd, double w_cmd,
      double gx, double gy, int nk, int &counter);

  // ---- 데이터 ----
  Config *config_{nullptr};
  std::shared_ptr<MPCPlannerStepMap::StepMap> step_map_;
  std::list<Node> nodes_;  // StraightConnection의 Node* 포인터 안정성을 위한 list

  // 파라미터 (Init 시 config에서 로드)
  int    num_heading_bins_;
  int    speed_bins_;
  int    n_v_samples_;
  int    n_w_samples_;
  int    n_substeps_;
  double w_max_;
  double a_max_;
  double goal_tol_xy_;
  double w_time_, w_occ_, w_accel_, w_yaw_, w_yaw_rate_;
};

}  // namespace GuidancePlanner
