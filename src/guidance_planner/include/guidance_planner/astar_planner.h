#pragma once

#include <guidance_planner/config.h>
#include <guidance_planner/types/paths.h>
#include <guidance_planner/types/node.h>
#include <mpc_planner_stepmap/step_map.h>

#include <Eigen/Dense>
#include <list>
#include <optional>
#include <queue>
#include <unordered_map>
#include <vector>

namespace GuidancePlanner
{

class AStarPlanner
{
public:
  void Init(Config *config);
  void SetStepMap(std::shared_ptr<MPCPlannerStepMap::StepMap> step_map);

  /** @brief StepMap 기반 단일 최적 경로 탐색
   *  @return 성공 시 GeometricPath, 실패 시 std::nullopt */
  std::optional<GeometricPath> Plan(const Eigen::Vector2d &start_xy,
                                    double start_heading,
                                    double start_speed,
                                    const Eigen::Vector2d &goal_xy);

  void Reset();

private:
  struct State
  {
    int gx, gy, gt, h;
  };

  struct StateHash
  {
    size_t operator()(const State &s) const noexcept
    {
      size_t seed = 0;
      auto hc = [&](int v)
      { seed ^= std::hash<int>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2); };
      hc(s.gx);
      hc(s.gy);
      hc(s.gt);
      hc(s.h);
      return seed;
    }
  };

  struct StateEq
  {
    bool operator()(const State &a, const State &b) const noexcept
    {
      return a.gx == b.gx && a.gy == b.gy && a.gt == b.gt && a.h == b.h;
    }
  };

  struct PQItem
  {
    double f, g;
    int counter;
    State state;
    double v_prev;
    State parent;
    bool has_parent{false};

    bool operator>(const PQItem &o) const noexcept
    {
      return f != o.f ? f > o.f : counter > o.counter;
    }
  };

  std::pair<int, int> cellFromWorld(const Eigen::Vector2d &world) const;
  bool inBounds(int gx, int gy, int gt) const;
  std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1) const;
  double heuristic(int gx, int gy, int gi, int gj) const;
  bool isBlocked(const std::vector<std::pair<int, int>> &cells, int gt) const;
  double sweptCost(const std::vector<std::pair<int, int>> &cells, int gt) const;
  GeometricPath reconstructPath(
      const std::unordered_map<State, PQItem, StateHash, StateEq> &closed,
      const State &goal_state);

  Config *config_{nullptr};
  std::shared_ptr<MPCPlannerStepMap::StepMap> step_map_;
  std::list<Node> nodes_;  // StraightConnection의 Node* 포인터 안정성을 위한 list

  int num_headings_{16};
  double w_max_{0.8};
  double w_time_{1.0}, w_occ_{5.0}, w_accel_{0.2}, w_yaw_{0.5};
  std::vector<double> headings_;
  int max_cells_{0};
  int max_dh_bins_{1};
};

}  // namespace GuidancePlanner
