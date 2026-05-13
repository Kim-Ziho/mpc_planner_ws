#include <guidance_planner/astar_planner.h>
#include <guidance_planner/types/connection.h>
#include <guidance_planner/utils.h>

#include <ros_tools/logging.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace GuidancePlanner
{

void AStarPlanner::Init(Config *config)
{
  config_ = config;

  num_headings_ = config->astar_num_headings_;
  w_max_        = config->astar_w_max_;
  w_time_       = config->astar_w_time_;
  w_occ_        = config->astar_w_occ_;
  w_accel_      = config->astar_w_accel_;
  w_yaw_        = config->astar_w_yaw_;

  base_time_cost_ = w_time_ * Config::DT;

  const double bin_size = 2.0 * M_PI / num_headings_;
  max_dh_bins_ = std::max(1, static_cast<int>(std::floor(w_max_ * Config::DT / bin_size)));

  // dh offsets and yaw cost table can be set already (independent of step map)
  dh_offsets_.clear();
  yaw_cost_table_.assign(2 * max_dh_bins_ + 1, 0.0);
  for (int dh = -max_dh_bins_; dh <= max_dh_bins_; ++dh)
  {
    dh_offsets_.push_back(dh);
    yaw_cost_table_[dh + max_dh_bins_] = w_yaw_ * std::abs(dh) * bin_size;
  }
}

void AStarPlanner::SetStepMap(std::shared_ptr<MPCPlannerStepMap::StepMap> step_map)
{
  step_map_ = step_map;
  if (!step_map_ || !step_map_->valid() || !config_) return;

  cells_x_       = step_map_->cellsX();
  cells_y_       = step_map_->cellsY();
  cells_t_       = step_map_->cellsT();
  res_           = step_map_->resolution();
  occ_data_      = step_map_->occupancyData();
  occ_threshold_ = step_map_->occupancyThreshold();
  n_states_      = cells_x_ * cells_y_ * cells_t_ * num_headings_;
  res_inv_v_max_ = res_ / config_->max_velocity_;

  const double bin_size = 2.0 * M_PI / num_headings_;
  max_dh_bins_ = std::max(1, static_cast<int>(std::floor(w_max_ * Config::DT / bin_size)));
  max_cells_   = std::max(1, static_cast<int>(std::floor(
                     config_->max_velocity_ * Config::DT / res_ + 1e-9)));

  rebuildOffsets();

  // Allocate flat search buffers once. Reset() refills them per call.
  g_arr_.assign(n_states_, std::numeric_limits<double>::infinity());
  parent_arr_.assign(n_states_, -1);
  v_prev_arr_.assign(n_states_, 0.0);
  dirty_.clear();  // any stale indices from a previous (different-sized) grid are gone
}

void AStarPlanner::rebuildOffsets()
{
  offset_dx_.assign(num_headings_ * max_cells_, 0);
  offset_dy_.assign(num_headings_ * max_cells_, 0);
  v_step_table_.assign(max_cells_, 0.0);

  for (int h = 0; h < num_headings_; ++h)
  {
    const double theta = 2.0 * M_PI * h / num_headings_;
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    for (int nc = 1; nc <= max_cells_; ++nc)
    {
      offset_dx_[h * max_cells_ + (nc - 1)] = static_cast<int>(std::round(c * nc));
      offset_dy_[h * max_cells_ + (nc - 1)] = static_cast<int>(std::round(s * nc));
    }
  }
  for (int nc = 1; nc <= max_cells_; ++nc)
    v_step_table_[nc - 1] = static_cast<double>(nc) * res_ / Config::DT;
}

void AStarPlanner::Reset()
{
  nodes_.clear();
}

// ── 좌표 변환 ──────────────────────────────────────────────────────────────

std::pair<int, int> AStarPlanner::cellFromWorld(const Eigen::Vector2d &world) const
{
  Eigen::Vector2d local = step_map_->localFromWorld(world);
  int gx = static_cast<int>(std::round((local.x() + step_map_->halfLength()) / res_));
  int gy = static_cast<int>(std::round((local.y() + step_map_->halfWidth())  / res_));
  return {gx, gy};
}

// ── 탐색 ───────────────────────────────────────────────────────────────────

std::optional<GeometricPath> AStarPlanner::Plan(const Eigen::Vector2d &start_xy,
                                                 double start_heading,
                                                 double start_speed,
                                                 const Eigen::Vector2d &goal_xy)
{
  if (!step_map_ || !step_map_->valid() || n_states_ == 0)
  {
    LOG_WARN("AStarPlanner::Plan — StepMap is not valid");
    return std::nullopt;
  }

  // StepMap pose / dimensions can change between calls. Refresh cached pointers
  // and grow buffers if the grid grew. (Same dimensions → no realloc.)
  if (cells_x_ != step_map_->cellsX() || cells_y_ != step_map_->cellsY() ||
      cells_t_ != step_map_->cellsT())
  {
    SetStepMap(step_map_);
  }
  occ_data_      = step_map_->occupancyData();
  occ_threshold_ = step_map_->occupancyThreshold();

  auto [si, sj] = cellFromWorld(start_xy);
  auto [gi, gj] = cellFromWorld(goal_xy);

  if (si < 0 || si >= cells_x_ || sj < 0 || sj >= cells_y_)
  {
    LOG_WARN("AStarPlanner::Plan — Start position is out of StepMap bounds");
    return std::nullopt;
  }
  gi = std::clamp(gi, 0, cells_x_ - 1);
  gj = std::clamp(gj, 0, cells_y_ - 1);
  goal_gx_ = gi;
  goal_gy_ = gj;

  int sh = static_cast<int>(std::round(start_heading / (2.0 * M_PI) * num_headings_)) % num_headings_;
  if (sh < 0) sh += num_headings_;

  // Reset only the entries touched by the previous call. The dirty list keeps
  // per-call reset cost proportional to the explored set, not n_states_ (~1M).
  for (int idx : dirty_)
  {
    g_arr_[idx] = std::numeric_limits<double>::infinity();
    parent_arr_[idx] = -1;
    v_prev_arr_[idx] = 0.0;
  }
  dirty_.clear();

  using PQ = std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>>;
  PQ open_pq;

  int counter = 0;
  const int start_idx = stateIdx(si, sj, 0, sh);
  g_arr_[start_idx]      = 0.0;
  parent_arr_[start_idx] = -1;  // root marker
  v_prev_arr_[start_idx] = start_speed;
  dirty_.push_back(start_idx);

  open_pq.push(PQItem{heuristic(si, sj), 0.0, counter++, start_idx});

  int goal_idx = -1;
  bool found = false;

  while (!open_pq.empty())
  {
    PQItem cur = open_pq.top();
    open_pq.pop();

    // Lazy deletion: a fresher (lower-g) push superseded this entry.
    if (cur.g > g_arr_[cur.state_idx] + 1e-9) continue;

    int ci, cj, ck, ch;
    unpack(cur.state_idx, ci, cj, ck, ch);

    if (ci == goal_gx_ && cj == goal_gy_)
    {
      goal_idx = cur.state_idx;
      found = true;
      break;
    }

    if (ck >= cells_t_ - 1) continue;

    const int nk = ck + 1;
    const double v_prev = v_prev_arr_[cur.state_idx];
    const double cur_g  = cur.g;

    for (int dh : dh_offsets_)
    {
      const int nh = ((ch + dh) % num_headings_ + num_headings_) % num_headings_;
      const double yaw_cost = yaw_cost_table_[dh + max_dh_bins_];

      const int *off_dx = offset_dx_.data() + nh * max_cells_;
      const int *off_dy = offset_dy_.data() + nh * max_cells_;

      for (int nc_idx = 0; nc_idx < max_cells_; ++nc_idx)
      {
        const int dx = off_dx[nc_idx];
        const int dy = off_dy[nc_idx];
        if (dx == 0 && dy == 0) continue;  // no movement (rare)

        const int ni = ci + dx;
        const int nj = cj + dy;
        if (static_cast<unsigned>(ni) >= static_cast<unsigned>(cells_x_)) continue;
        if (static_cast<unsigned>(nj) >= static_cast<unsigned>(cells_y_)) continue;

        // Fused Bresenham + occupancy check + swept cost (single pass, no alloc)
        const double occ_sum = sweptCheck(ci, cj, ni, nj, nk);
        if (occ_sum < 0.0) continue;  // blocked

        const double v_step = v_step_table_[nc_idx];
        const double dv     = std::abs(v_step - v_prev);

        const double step_cost = base_time_cost_
                               + w_occ_   * occ_sum
                               + w_accel_ * dv
                               + yaw_cost;
        const double ng = cur_g + step_cost;

        const int nidx = stateIdx(ni, nj, nk, nh);
        if (ng >= g_arr_[nidx] - 1e-9) continue;

        if (g_arr_[nidx] == std::numeric_limits<double>::infinity())
          dirty_.push_back(nidx);

        g_arr_[nidx]      = ng;
        parent_arr_[nidx] = cur.state_idx;
        v_prev_arr_[nidx] = v_step;

        open_pq.push(PQItem{ng + heuristic(ni, nj), ng, counter++, nidx});
      }
    }
  }

  if (!found)
  {
    LOG_WARN("AStarPlanner: No path found to goal");
    return std::nullopt;
  }

  return reconstructPath(goal_idx);
}

// ── 경로 재구성 → GeometricPath ────────────────────────────────────────────

GeometricPath AStarPlanner::reconstructPath(int goal_idx)
{
  std::vector<int> idx_chain;
  for (int idx = goal_idx; idx != -1; idx = parent_arr_[idx])
    idx_chain.push_back(idx);
  std::reverse(idx_chain.begin(), idx_chain.end());

  nodes_.clear();
  for (std::size_t i = 0; i < idx_chain.size(); ++i)
  {
    int gx, gy, gt, h;
    unpack(idx_chain[i], gx, gy, gt, h);
    const Eigen::Vector2d world = step_map_->worldFromCell(gx, gy);

    // Goal node는 PRM 관례와 동일하게 k=Config::N으로 강제
    // CubicSpline3D::ConvertToTrajectory()는 path(1.)의 시간이 정확히 Config::N임을 전제함
    const double k_time = (i == idx_chain.size() - 1)
        ? static_cast<double>(Config::N)
        : static_cast<double>(gt);
    const SpaceTimePoint pt(world.x(), world.y(), k_time);

    NodeType type;
    int id;
    if (i == 0)
    {
      type = NodeType::GUARD;
      id   = -1;
    }
    else if (i == idx_chain.size() - 1)
    {
      type = NodeType::GOAL;
      id   = -2;
    }
    else
    {
      type = NodeType::CONNECTOR;
      id   = static_cast<int>(i);
    }
    nodes_.emplace_back(id, pt, type);
  }

  std::vector<Node *> ptrs;
  ptrs.reserve(nodes_.size());
  for (auto &n : nodes_)
    ptrs.push_back(&n);

  return GeometricPath(ptrs);
}

}  // namespace GuidancePlanner
