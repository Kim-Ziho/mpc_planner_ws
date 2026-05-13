#pragma once

#include <guidance_planner/config.h>
#include <guidance_planner/types/paths.h>
#include <guidance_planner/types/node.h>
#include <mpc_planner_stepmap/step_map.h>

#include <Eigen/Dense>
#include <array>
#include <functional>
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
      size_t seed = static_cast<size_t>(s.i);
      seed = seed * 1315423911u + static_cast<size_t>(s.j);
      seed = seed * 1315423911u + static_cast<size_t>(s.k);
      seed = seed * 1315423911u + static_cast<size_t>(s.h_bin);
      seed = seed * 1315423911u + static_cast<size_t>(s.v_bin);
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

  // 인덱스 기반 노드 풀 (shared_ptr 제거)
  struct SearchNode
  {
    double f, g;
    double x, y, theta, v;
    int    k;
    int    parent_idx;   // -1 = no parent
    int    counter;
    ClosedKey key;       // makeKey 중복 호출 방지를 위해 캐시
  };

  // priority_queue 엔트리 (포인터 대신 인덱스만 보관)
  struct PQEntry
  {
    double f;
    int    counter;
    int    idx;
    bool operator>(const PQEntry &o) const noexcept
    {
      return f != o.f ? f > o.f : counter > o.counter;
    }
  };

  using BestG = std::unordered_map<ClosedKey, double, ClosedKeyHash, ClosedKeyEq>;
  using OpenPQ = std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>>;

  // ---- 헬퍼 ----
  static constexpr int kMaxSubsteps = 16;

  ClosedKey makeKey(double x, double y, double theta, double v, int k) const;
  void cellFromWorld(double x, double y, int &ii, int &jj) const;

  /** @brief 유니사이클 Euler 적분: (x,y,theta) → sub-step 위치 목록 (스택 배열) */
  int integrate(double x, double y, double theta,
                double v_cmd, double w_cmd,
                std::array<Eigen::Vector2d, kMaxSubsteps> &pts_out) const;

  /** @brief sub-step 위치 목록에 대해 StepMap 충돌 검사 + 점유 비용 누계 */
  bool checkSwept(const std::array<Eigen::Vector2d, kMaxSubsteps> &pts,
                  int n_pts, int nk, double &occ_total) const;

  double heuristic(double x, double y, double gx, double gy) const;

  /** @brief t=0 정적 슬라이스 위 2D Dijkstra 휴리스틱 사전 계산 */
  void buildHeuristicMap(double gx, double gy);

  GeometricPath reconstructPath(int goal_idx);

  void pushIfBetter(OpenPQ &pq, BestG &best_g,
                    int parent_idx,
                    double v_cmd, double w_cmd,
                    double gx, double gy, int nk, int &counter);

  // ---- 데이터 ----
  Config *config_{nullptr};
  std::shared_ptr<MPCPlannerStepMap::StepMap> step_map_;
  std::list<Node> nodes_;  // StraightConnection의 Node* 포인터 안정성을 위한 list

  // 노드 풀: Plan() 동안만 살아 있음. Init에서 capacity 사전 할당
  std::vector<SearchNode> pool_;

  // 2D Dijkstra 휴리스틱 맵 (cells_x_ * cells_y_), 도달 불가 = +inf
  std::vector<double> heur_grid_;
  int                 heur_cells_x_{0};
  int                 heur_cells_y_{0};

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
  double time_budget_ms_;
};

}  // namespace GuidancePlanner
