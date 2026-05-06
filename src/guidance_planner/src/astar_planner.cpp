#include <guidance_planner/astar_planner.h>
#include <guidance_planner/types/connection.h>
#include <guidance_planner/utils.h>

#include <ros_tools/logging.h>

#include <algorithm>
#include <cmath>

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

  headings_.resize(num_headings_);
  for (int i = 0; i < num_headings_; ++i)
    headings_[i] = 2.0 * M_PI * i / num_headings_;

  const double bin_size = 2.0 * M_PI / num_headings_;
  max_dh_bins_ = std::max(1, (int)std::floor(w_max_ * Config::DT / bin_size));
}

void AStarPlanner::SetStepMap(std::shared_ptr<MPCPlannerStepMap::StepMap> step_map)
{
  step_map_ = step_map;
  if (step_map_ && step_map_->valid() && config_)
  {
    const double bin_size = 2.0 * M_PI / num_headings_;
    max_dh_bins_ = std::max(1, (int)std::floor(w_max_ * Config::DT / bin_size));
    max_cells_   = std::max(1, (int)std::floor(
                       config_->max_velocity_ * Config::DT / step_map_->resolution() + 1e-9));
  }
}

void AStarPlanner::Reset()
{
  nodes_.clear();
}

// ── 좌표 변환 ──────────────────────────────────────────────────────────────

std::pair<int, int> AStarPlanner::cellFromWorld(const Eigen::Vector2d &world) const
{
  // StepMap: local(gx,gy) = (-halfLength + gx*res, -halfWidth + gy*res)
  // worldFromCell: rot_world_from_local * local + center_world
  // 역변환: local = localFromWorld(world)
  Eigen::Vector2d local = step_map_->localFromWorld(world);
  int gx = (int)std::round((local.x() + step_map_->halfLength()) / step_map_->resolution());
  int gy = (int)std::round((local.y() + step_map_->halfWidth())  / step_map_->resolution());
  return {gx, gy};
}

bool AStarPlanner::inBounds(int gx, int gy, int gt) const
{
  return gx >= 0 && gx < step_map_->cellsX() &&
         gy >= 0 && gy < step_map_->cellsY() &&
         gt >= 0 && gt < step_map_->cellsT();
}

// ── Bresenham 선분 ─────────────────────────────────────────────────────────

std::vector<std::pair<int, int>> AStarPlanner::bresenhamLine(
    int x0, int y0, int x1, int y1) const
{
  std::vector<std::pair<int, int>> cells;
  int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  int x = x0, y = y0;
  while (true)
  {
    cells.emplace_back(x, y);
    if (x == x1 && y == y1) break;
    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x += sx; }
    if (e2 <  dx) { err += dx; y += sy; }
  }
  return cells;
}

// ── 비용 함수 ──────────────────────────────────────────────────────────────

double AStarPlanner::heuristic(int gx, int gy, int gi, int gj) const
{
  double dist = std::hypot(gx - gi, gy - gj) * step_map_->resolution();
  return dist / config_->max_velocity_;
}

bool AStarPlanner::isBlocked(const std::vector<std::pair<int, int>> &cells, int gt) const
{
  for (auto &[x, y] : cells)
  {
    if (!inBounds(x, y, gt)) return true;
    if (step_map_->cellOccupied(x, y, gt)) return true;
  }
  return false;
}

double AStarPlanner::sweptCost(const std::vector<std::pair<int, int>> &cells, int gt) const
{
  double total = 0.0;
  for (auto &[x, y] : cells)
    if (inBounds(x, y, gt))
      total += step_map_->cellCost(x, y, gt);
  return total;
}

// ── 탐색 ───────────────────────────────────────────────────────────────────

std::optional<GeometricPath> AStarPlanner::Plan(const Eigen::Vector2d &start_xy,
                                                 double start_heading,
                                                 double start_speed,
                                                 const Eigen::Vector2d &goal_xy)
{
  if (!step_map_ || !step_map_->valid())
  {
    LOG_WARN("AStarPlanner::Plan — StepMap is not valid");
    return std::nullopt;
  }

  auto [si, sj] = cellFromWorld(start_xy);
  auto [gi, gj] = cellFromWorld(goal_xy);

  if (!inBounds(si, sj, 0))
  {
    LOG_WARN("AStarPlanner::Plan — Start position is out of StepMap bounds");
    return std::nullopt;
  }
  gi = std::clamp(gi, 0, step_map_->cellsX() - 1);
  gj = std::clamp(gj, 0, step_map_->cellsY() - 1);

  int sh = (int)std::round(start_heading / (2.0 * M_PI) * num_headings_) % num_headings_;
  if (sh < 0) sh += num_headings_;

  using PQ = std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>>;
  PQ open_pq;
  std::unordered_map<State, double,   StateHash, StateEq> best_g;
  std::unordered_map<State, PQItem,   StateHash, StateEq> closed;

  int counter = 0;
  const State start_state{si, sj, 0, sh};
  open_pq.push(PQItem{heuristic(si, sj, gi, gj), 0.0, counter++,
                      start_state, start_speed, {}, false});
  best_g[start_state] = 0.0;

  // 허용 헤딩 변화 오프셋 사전 계산
  std::vector<int> dh_offsets;
  for (int dh = -max_dh_bins_; dh <= max_dh_bins_; ++dh)
    dh_offsets.push_back(dh);

  State goal_state{-1, -1, -1, -1};
  bool found = false;

  while (!open_pq.empty())
  {
    PQItem cur = open_pq.top();
    open_pq.pop();

    // 이미 더 좋은 경로로 처리된 경우 skip
    {
      auto it = best_g.find(cur.state);
      if (it != best_g.end() && cur.g > it->second + 1e-9) continue;
    }
    if (closed.count(cur.state)) continue;
    closed[cur.state] = cur;

    const auto [ci, cj, ck, ch] = cur.state;

    // 목표 도달 판정
    if (ci == gi && cj == gj)
    {
      goal_state = cur.state;
      found = true;
      break;
    }

    if (ck >= step_map_->cellsT() - 1) continue;

    for (int dh : dh_offsets)
    {
      const int nh = ((ch + dh) % num_headings_ + num_headings_) % num_headings_;
      const double ntheta = headings_[nh];

      for (int n_cells = 1; n_cells <= max_cells_; ++n_cells)
      {
        const int ni = ci + (int)std::round(std::cos(ntheta) * n_cells);
        const int nj = cj + (int)std::round(std::sin(ntheta) * n_cells);
        const int nk = ck + 1;

        if (!inBounds(ni, nj, nk)) continue;

        const auto swept = bresenhamLine(ci, cj, ni, nj);
        if (isBlocked(swept, nk)) continue;

        const double v_step  = n_cells * step_map_->resolution() / Config::DT;
        const double dv      = std::abs(v_step - cur.v_prev);
        const double dtheta  = std::abs(dh) * (2.0 * M_PI / num_headings_);
        const double occ     = sweptCost(swept, nk);

        const double step_cost = w_time_ * Config::DT
                               + w_occ_  * occ
                               + w_accel_ * dv
                               + w_yaw_   * dtheta;
        const double ng = cur.g + step_cost;

        const State nstate{ni, nj, nk, nh};
        auto bg_it = best_g.find(nstate);
        if (bg_it != best_g.end() && ng >= bg_it->second - 1e-9) continue;
        best_g[nstate] = ng;

        open_pq.push(PQItem{ng + heuristic(ni, nj, gi, gj), ng, counter++,
                            nstate, v_step, cur.state, true});
      }
    }
  }

  if (!found)
  {
    LOG_WARN("AStarPlanner: No path found to goal");
    return std::nullopt;
  }

  return reconstructPath(closed, goal_state);
}

// ── 경로 재구성 → GeometricPath ────────────────────────────────────────────

GeometricPath AStarPlanner::reconstructPath(
    const std::unordered_map<State, PQItem, StateHash, StateEq> &closed,
    const State &goal_state)
{
  // goal → start 역추적
  std::vector<State> states;
  State cur = goal_state;
  while (true)
  {
    states.push_back(cur);
    auto it = closed.find(cur);
    if (it == closed.end() || !it->second.has_parent) break;
    cur = it->second.parent;
  }
  std::reverse(states.begin(), states.end());

  // State → Node (std::list 에 저장하여 포인터 안정성 확보)
  nodes_.clear();
  for (size_t i = 0; i < states.size(); ++i)
  {
    const auto &s = states[i];
    const Eigen::Vector2d world = step_map_->worldFromCell(s.gx, s.gy);
    // Goal node는 PRM 관례와 동일하게 k=Config::N으로 강제
    // CubicSpline3D::ConvertToTrajectory()는 path(1.)의 시간이 정확히 Config::N임을 전제함
    const double k_time = (i == states.size() - 1)
        ? static_cast<double>(Config::N)
        : static_cast<double>(s.gt);
    const SpaceTimePoint pt(world.x(), world.y(), k_time);

    NodeType type;
    int id;
    if (i == 0)
    {
      type = NodeType::GUARD;
      id   = -1;
    }
    else if (i == states.size() - 1)
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

  // Node 포인터 벡터 → GeometricPath (StraightConnection 자동 생성)
  std::vector<Node *> ptrs;
  ptrs.reserve(nodes_.size());
  for (auto &n : nodes_)
    ptrs.push_back(&n);

  return GeometricPath(ptrs);
}

}  // namespace GuidancePlanner
