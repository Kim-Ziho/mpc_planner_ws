#pragma once

#include <guidance_planner/config.h>
#include <guidance_planner/types/paths.h>
#include <guidance_planner/types/node.h>
#include <mpc_planner_stepmap/step_map.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <list>
#include <optional>
#include <queue>
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
  // Priority-queue entry. State and parent are encoded as flat indices,
  // so the entry is a small POD that copies cheaply inside the heap.
  struct PQItem
  {
    double f;
    double g;
    int counter;
    int state_idx;

    bool operator>(const PQItem &o) const noexcept
    {
      return f != o.f ? f > o.f : counter > o.counter;
    }
  };

  std::pair<int, int> cellFromWorld(const Eigen::Vector2d &world) const;

  // Flat state index: ((gt * cellsY + gy) * cellsX + gx) * num_headings + h.
  inline int stateIdx(int gx, int gy, int gt, int h) const
  {
    return ((gt * cells_y_ + gy) * cells_x_ + gx) * num_headings_ + h;
  }

  inline void unpack(int idx, int &gx, int &gy, int &gt, int &h) const
  {
    h = idx % num_headings_;
    int rest = idx / num_headings_;
    gx = rest % cells_x_;
    rest /= cells_x_;
    gy = rest % cells_y_;
    gt = rest / cells_y_;
  }

  inline double heuristic(int gx, int gy) const
  {
    return std::hypot(gx - goal_gx_, gy - goal_gy_) * res_inv_v_max_;
  }

  // Walks Bresenham line from (x0,y0) to (x1,y1) on time layer gt. Returns
  // the swept occupancy sum, or -1.0 if any cell is blocked. All cells are
  // assumed in-bounds (caller guarantees via inBounds check on endpoints).
  inline double sweptCheck(int x0, int y0, int x1, int y1, int gt) const
  {
    const std::size_t plane = static_cast<std::size_t>(cells_x_) * cells_y_;
    const double *occ = occ_data_ + static_cast<std::size_t>(gt) * plane;
    const double thr = occ_threshold_;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int x = x0, y = y0;
    double cost = 0.0;
    while (true)
    {
      const double v = occ[static_cast<std::size_t>(y) * cells_x_ + x];
      if (v >= thr) return -1.0;
      cost += v;
      if (x == x1 && y == y1) break;
      int e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x += sx; }
      if (e2 <  dx) { err += dx; y += sy; }
    }
    return cost;
  }

  void rebuildOffsets();
  GeometricPath reconstructPath(int goal_idx);

  Config *config_{nullptr};
  std::shared_ptr<MPCPlannerStepMap::StepMap> step_map_;
  std::list<Node> nodes_;  // StraightConnection은 Node*를 참조 — list로 포인터 안정성 확보

  // Algorithm parameters
  int num_headings_{16};
  double w_max_{0.8};
  double w_time_{1.0}, w_occ_{5.0}, w_accel_{0.2}, w_yaw_{0.5};

  // Cached step-map dimensions / direct buffer
  int cells_x_{0}, cells_y_{0}, cells_t_{0};
  int n_states_{0};
  double res_{0.0};
  double res_inv_v_max_{0.0};
  const double *occ_data_{nullptr};
  double occ_threshold_{0.0};

  // Movement table
  int max_cells_{1};
  int max_dh_bins_{1};
  std::vector<int> offset_dx_;        // size = num_headings * max_cells
  std::vector<int> offset_dy_;
  std::vector<double> v_step_table_;  // size = max_cells (precomputed v per n_cells)
  std::vector<double> yaw_cost_table_; // size = 2*max_dh_bins+1, indexed by (dh + max_dh_bins)
  std::vector<int> dh_offsets_;        // dh values from -max_dh_bins to +max_dh_bins
  double base_time_cost_{0.0};         // w_time * DT

  // Search state buffers (reused across Plan() calls)
  std::vector<double> g_arr_;
  std::vector<int> parent_arr_;
  std::vector<double> v_prev_arr_;
  std::vector<int> dirty_;  // indices touched in last Plan(), reset at start of next call

  // Goal cell (cached for heuristic)
  int goal_gx_{0}, goal_gy_{0};
};

}  // namespace GuidancePlanner
