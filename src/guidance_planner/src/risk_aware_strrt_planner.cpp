#include <guidance_planner/risk_aware_strrt_planner.h>

#include <guidance_planner/types/connection.h>
#include <guidance_planner/types/space_time_point.h>

#include <ros_tools/logging.h>
#include <ros_tools/spline.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace GuidancePlanner
{

namespace
{
constexpr double kEps = 1e-9;
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

// ── lifecycle ────────────────────────────────────────────────────────────────

void RiskAwareSTRRTPlanner::Init(Config *config)
{
  config_ = config;

  max_iter_            = config->ra_strrt_max_iter_;
  v_max_               = config->ra_strrt_v_max_;
  v_preferred_         = config->ra_strrt_v_preferred_;
  w_max_               = config->ra_strrt_w_max_;
  tau_hard_            = config->ra_strrt_tau_hard_;
  tau_soft_            = config->ra_strrt_tau_soft_;
  w_t_                 = config->ra_strrt_w_time_;
  w_r_                 = config->ra_strrt_w_risk_;
  w_p_                 = config->ra_strrt_w_progress_;
  w_c_                 = config->ra_strrt_w_curvature_;
  w_goal_progress_     = config->ra_strrt_w_goal_progress_;
  lambda_nn_           = config->ra_strrt_lambda_nn_;
  max_step_cells_      = config->ra_strrt_max_step_cells_;
  k_rrtstar_           = config->ra_strrt_k_rrtstar_;
  initial_goal_count_  = config->ra_strrt_initial_goal_count_;
  max_goal_count_      = config->ra_strrt_max_goal_count_;
  max_nodes_           = config->ra_strrt_max_nodes_;
  time_budget_ms_      = config->ra_strrt_time_budget_ms_;
  tube_width_          = config->ra_strrt_tube_width_;
  s_min_offset_        = config->ra_strrt_s_min_offset_;
  t_min_               = config->ra_strrt_t_min_;
  t_max_               = config->ra_strrt_t_max_;
  enable_warm_start_   = config->ra_strrt_enable_warm_start_;
  cold_start_min_remaining_nodes_ = config->ra_strrt_cold_start_min_remaining_nodes_;

  rng_.seed(static_cast<uint32_t>(config->seed_ >= 0 ? config->seed_ : std::random_device{}()));

  has_prev_tree_ = false;
}

void RiskAwareSTRRTPlanner::SetStepMap(std::shared_ptr<MPCPlannerStepMap::StepMap> step_map)
{
  step_map_ = step_map;
}

void RiskAwareSTRRTPlanner::Reset()
{
  nodes_.clear();
  start_roots_.clear();
  goal_roots_.clear();
  forward_mask_.clear();
  backward_mask_.clear();
  valid_indices_.clear();
  path_nodes_.clear();
  has_prev_tree_ = false;
  best_link_ = {-1, -1};
}

// ── coordinate helpers ───────────────────────────────────────────────────────

bool RiskAwareSTRRTPlanner::worldToCell(const Eigen::Vector2d &p, double t,
                                         int &gx, int &gy, int &gt) const
{
  if (!step_map_)
    return false;
  Eigen::Vector2d local = step_map_->localFromWorld(p);
  const double res = step_map_->resolution();
  gx = static_cast<int>(std::floor((local.x() + step_map_->halfLength()) / res));
  gy = static_cast<int>(std::floor((local.y() + step_map_->halfWidth())  / res));
  gt = static_cast<int>(std::round(t / step_map_->timeScale()));
  return gx >= 0 && gx < step_map_->cellsX() &&
         gy >= 0 && gy < step_map_->cellsY() &&
         gt >= 0 && gt < step_map_->cellsT();
}

void RiskAwareSTRRTPlanner::cellToWorld(int gx, int gy, int gt,
                                         double &x, double &y, double &t) const
{
  Eigen::Vector2d p = step_map_->worldFromCell(gx, gy);
  x = p.x();
  y = p.y();
  t = static_cast<double>(gt) * step_map_->timeScale();
}

size_t RiskAwareSTRRTPlanner::flatIndex(int gx, int gy, int gt) const
{
  return static_cast<size_t>(gt) * cells_xy_ +
         static_cast<size_t>(gy) * static_cast<size_t>(cells_x_) +
         static_cast<size_t>(gx);
}

double RiskAwareSTRRTPlanner::riskPhi(double p) const
{
  if (p < tau_soft_)
    return 0.0;
  const double one_minus = std::max(1e-6, 1.0 - p);
  return -std::log(one_minus);
}

// ── DDA ──────────────────────────────────────────────────────────────────────

template <class F>
void RiskAwareSTRRTPlanner::dda3D(const RRTNode &a, const RRTNode &b, F &&visit) const
{
  int ix = a.ix, iy = a.iy, it = a.it;
  const int ix_end = b.ix, iy_end = b.iy, it_end = b.it;

  if (!visit(ix, iy, it))
    return;
  if (ix == ix_end && iy == iy_end && it == it_end)
    return;

  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double dt = b.t - a.t;

  const int step_x = (dx > 0) ? 1 : (dx < 0 ? -1 : 0);
  const int step_y = (dy > 0) ? 1 : (dy < 0 ? -1 : 0);
  const int step_t = (dt > 0) ? 1 : 0;  // 시간은 단조 증가

  const double res    = step_map_->resolution();
  const double tscale = step_map_->timeScale();
  const double inv_dx = (std::abs(dx) > kEps) ? 1.0 / std::abs(dx) : kInf;
  const double inv_dy = (std::abs(dy) > kEps) ? 1.0 / std::abs(dy) : kInf;
  const double inv_dt = (std::abs(dt) > kEps) ? 1.0 / std::abs(dt) : kInf;

  const double t_delta_x = (step_x != 0) ? res    * inv_dx : kInf;
  const double t_delta_y = (step_y != 0) ? res    * inv_dy : kInf;
  const double t_delta_t = (step_t != 0) ? tscale * inv_dt : kInf;

  // 셀 경계까지 남은 거리 → t_max
  Eigen::Vector2d local_a = step_map_->localFromWorld(Eigen::Vector2d(a.x, a.y));
  const double frac_x = (local_a.x() + step_map_->halfLength()) / res - static_cast<double>(ix);
  const double frac_y = (local_a.y() + step_map_->halfWidth())  / res - static_cast<double>(iy);
  const double frac_t = a.t / tscale - static_cast<double>(it);

  double t_max_x = (step_x > 0) ? (1.0 - frac_x) * t_delta_x
                  : (step_x < 0) ? (frac_x      ) * t_delta_x
                  : kInf;
  double t_max_y = (step_y > 0) ? (1.0 - frac_y) * t_delta_y
                  : (step_y < 0) ? (frac_y      ) * t_delta_y
                  : kInf;
  double t_max_t = (step_t > 0) ? (1.0 - frac_t) * t_delta_t : kInf;

  // 안전 가드
  const int max_steps = (std::abs(ix_end - ix) + std::abs(iy_end - iy) + std::abs(it_end - it)) + 4;
  int steps = 0;

  while (steps++ < max_steps)
  {
    if (t_max_x < t_max_y && t_max_x < t_max_t)
    {
      ix += step_x;
      t_max_x += t_delta_x;
    }
    else if (t_max_y < t_max_t)
    {
      iy += step_y;
      t_max_y += t_delta_y;
    }
    else
    {
      it += step_t;
      t_max_t += t_delta_t;
    }

    if (!visit(ix, iy, it))
      return;
    if (ix == ix_end && iy == iy_end && it == it_end)
      return;
  }
}

// ── cost / distance ──────────────────────────────────────────────────────────

double RiskAwareSTRRTPlanner::timeAwareDist(const RRTNode &a, const RRTNode &b) const
{
  const double dt = b.t - a.t;
  if (dt <= 1e-6)
    return kInf;
  const double d = std::hypot(b.x - a.x, b.y - a.y);
  if (d > v_max_ * dt + 1e-6)
    return kInf;
  return lambda_nn_ * d + (1.0 - lambda_nn_) * dt;
}

double RiskAwareSTRRTPlanner::riskCostAlongEdge(const RRTNode &a, const RRTNode &b,
                                                  bool &collision) const
{
  collision = false;
  double acc = 0.0;
  const double dl = step_map_->resolution();
  dda3D(a, b, [&](int ix, int iy, int it) -> bool
  {
    if (ix < 0 || ix >= cells_x_ || iy < 0 || iy >= cells_y_ ||
        it < 0 || it >= cells_t_)
    {
      collision = true;
      return false;
    }
    const double p = step_map_->cellCost(ix, iy, it);
    if (p >= tau_hard_)
    {
      collision = true;
      return false;
    }
    acc += riskPhi(p) * dl;
    return true;
  });
  return acc;
}

double RiskAwareSTRRTPlanner::edgeCost(const RRTNode &a, const RRTNode &b) const
{
  const double dt = b.t - a.t;
  bool collision = false;
  const double c_risk = w_r_ * riskCostAlongEdge(a, b, collision);
  // 곡률 비용 — parent의 heading이 a.heading임. b.heading은 a→b edge 방향.
  double c_curv = 0.0;
  if (a.parent >= 0 && w_c_ > 0.0)
  {
    const double dpsi = std::atan2(std::sin(b.heading - a.heading),
                                    std::cos(b.heading - a.heading));
    c_curv = w_c_ * std::abs(dpsi) / std::max(1e-3, w_max_);
  }
  return w_t_ * dt + c_risk + c_curv;
}

bool RiskAwareSTRRTPlanner::curvatureFeasible(const RRTNode &parent,
                                                const RRTNode &child) const
{
  if (parent.parent < 0)
    return true;
  const double dt = child.t - parent.t;
  if (dt < kEps)
    return false;
  const double new_heading = std::atan2(child.y - parent.y, child.x - parent.x);
  const double dpsi = std::atan2(std::sin(new_heading - parent.heading),
                                  std::cos(new_heading - parent.heading));
  return std::abs(dpsi) <= w_max_ * dt + 1e-3;
}

// ── edge check ───────────────────────────────────────────────────────────────

RiskAwareSTRRTPlanner::EdgeCheck
RiskAwareSTRRTPlanner::edgeCheck(const RRTNode &from, const RRTNode &to) const
{
  EdgeCheck ec;
  if (to.t <= from.t + 1e-6)
    return ec;
  const double dt = to.t - from.t;
  const double d  = std::hypot(to.x - from.x, to.y - from.y);
  if (d > v_max_ * dt + 1e-6)
    return ec;

  bool collision = false;
  const double c_risk = w_r_ * riskCostAlongEdge(from, to, collision);
  if (collision)
    return ec;

  double c_curv = 0.0;
  if (from.parent >= 0 && w_c_ > 0.0)
  {
    const double new_heading = std::atan2(to.y - from.y, to.x - from.x);
    const double dpsi = std::atan2(std::sin(new_heading - from.heading),
                                    std::cos(new_heading - from.heading));
    if (std::abs(dpsi) > w_max_ * dt + 1e-3)
      return ec;
    c_curv = w_c_ * std::abs(dpsi) / std::max(1e-3, w_max_);
  }

  ec.ok = true;
  ec.cost = w_t_ * dt + c_risk + c_curv;
  return ec;
}

// ── steer ────────────────────────────────────────────────────────────────────

std::optional<RiskAwareSTRRTPlanner::RRTNode>
RiskAwareSTRRTPlanner::steer(const RRTNode &near, const RRTNode &target) const
{
  if (target.t <= near.t + 1e-6)
    return std::nullopt;

  const double dx = target.x - near.x;
  const double dy = target.y - near.y;
  const double dist = std::hypot(dx, dy);
  if (dist < kEps)
    return std::nullopt;

  const double res = step_map_->resolution();
  const double max_dist = static_cast<double>(max_step_cells_) * res;
  double k = (dist <= max_dist) ? 1.0 : max_dist / dist;

  RRTNode s;
  double x_w = near.x + k * dx;
  double y_w = near.y + k * dy;
  double t_w = near.t + k * (target.t - near.t);

  int gx, gy, gt;
  if (!worldToCell(Eigen::Vector2d(x_w, y_w), t_w, gx, gy, gt))
    return std::nullopt;

  // 격자에 snap
  cellToWorld(gx, gy, gt, s.x, s.y, s.t);
  s.ix = gx;
  s.iy = gy;
  s.it = gt;
  if (s.t <= near.t + 1e-6)
    return std::nullopt;

  s.heading = std::atan2(s.y - near.y, s.x - near.x);
  return s;
}

// ── conditional sampling ─────────────────────────────────────────────────────

std::optional<RiskAwareSTRRTPlanner::RRTNode>
RiskAwareSTRRTPlanner::sampleConditionally(double /*t_upper*/) const
{
  if (valid_indices_.empty())
    return std::nullopt;

  std::uniform_int_distribution<size_t> dist(0, valid_indices_.size() - 1);
  const int flat = valid_indices_[dist(rng_)];

  const int it = flat / static_cast<int>(cells_xy_);
  const int rem = flat - it * static_cast<int>(cells_xy_);
  const int iy = rem / cells_x_;
  const int ix = rem - iy * cells_x_;

  RRTNode n;
  n.ix = ix;
  n.iy = iy;
  n.it = it;
  cellToWorld(ix, iy, it, n.x, n.y, n.t);
  return n;
}

void RiskAwareSTRRTPlanner::buildForwardReachableMask(const RRTNode &start)
{
  forward_mask_.assign(cells_total_, 0);
  const double res    = step_map_->resolution();
  const double tscale = step_map_->timeScale();
  for (int it = start.it; it < cells_t_; ++it)
  {
    const double dt = (it - start.it) * tscale;
    if (dt < 0)
      continue;
    const int reach = static_cast<int>(std::ceil(v_max_ * dt / res)) + 1;
    const int x0 = std::max(0, start.ix - reach);
    const int x1 = std::min(cells_x_ - 1, start.ix + reach);
    const int y0 = std::max(0, start.iy - reach);
    const int y1 = std::min(cells_y_ - 1, start.iy + reach);
    for (int iy = y0; iy <= y1; ++iy)
    {
      for (int ix = x0; ix <= x1; ++ix)
      {
        const int dxi = ix - start.ix;
        const int dyi = iy - start.iy;
        if (dxi * dxi + dyi * dyi <= reach * reach)
          forward_mask_[flatIndex(ix, iy, it)] = 1;
      }
    }
  }
}

void RiskAwareSTRRTPlanner::buildBackwardReachableMask(const std::vector<int> &goal_roots)
{
  backward_mask_.assign(cells_total_, 0);
  const double res    = step_map_->resolution();
  const double tscale = step_map_->timeScale();
  for (int gidx : goal_roots)
  {
    if (gidx < 0 || gidx >= static_cast<int>(nodes_.size()))
      continue;
    const RRTNode &g = nodes_[gidx];
    for (int it = 0; it <= g.it; ++it)
    {
      const double dt = (g.it - it) * tscale;
      const int reach = static_cast<int>(std::ceil(v_max_ * dt / res)) + 1;
      const int x0 = std::max(0, g.ix - reach);
      const int x1 = std::min(cells_x_ - 1, g.ix + reach);
      const int y0 = std::max(0, g.iy - reach);
      const int y1 = std::min(cells_y_ - 1, g.iy + reach);
      for (int iy = y0; iy <= y1; ++iy)
      {
        for (int ix = x0; ix <= x1; ++ix)
        {
          const int dxi = ix - g.ix;
          const int dyi = iy - g.iy;
          if (dxi * dxi + dyi * dyi <= reach * reach)
            backward_mask_[flatIndex(ix, iy, it)] = 1;
        }
      }
    }
  }
}

void RiskAwareSTRRTPlanner::rebuildValidIndexList()
{
  valid_indices_.clear();
  valid_indices_.reserve(cells_total_ / 4);
  for (int it = 0; it < cells_t_; ++it)
  {
    for (int iy = 0; iy < cells_y_; ++iy)
    {
      for (int ix = 0; ix < cells_x_; ++ix)
      {
        const size_t flat = flatIndex(ix, iy, it);
        if (!forward_mask_[flat] || !backward_mask_[flat])
          continue;
        if (step_map_->cellCost(ix, iy, it) >= tau_hard_)
          continue;
        valid_indices_.push_back(static_cast<int>(flat));
      }
    }
  }
}

// ── neighbors ────────────────────────────────────────────────────────────────

std::vector<int> RiskAwareSTRRTPlanner::collectNeighbors(const RRTNode &x_new,
                                                          int k, TreeType tree) const
{
  // O(N) brute force, k-NN by timeAwareDist. nodes_.size()는 max_nodes_ 한정.
  std::vector<std::pair<double, int>> scored;
  scored.reserve(nodes_.size());
  for (int i = 0; i < static_cast<int>(nodes_.size()); ++i)
  {
    const RRTNode &n = nodes_[i];
    if (n.removed || n.tree != tree)
      continue;
    double d;
    if (tree == TreeType::START)
      d = timeAwareDist(n, x_new);   // start tree: n.t < x_new.t
    else
      d = timeAwareDist(x_new, n);   // goal tree: x_new.t < n.t
    if (std::isinf(d))
      continue;
    scored.emplace_back(d, i);
  }

  if (static_cast<int>(scored.size()) > k)
  {
    std::nth_element(scored.begin(), scored.begin() + k, scored.end(),
                     [](const auto &a, const auto &b) { return a.first < b.first; });
    scored.resize(k);
  }
  std::vector<int> out;
  out.reserve(scored.size());
  for (auto &p : scored)
    out.push_back(p.second);
  return out;
}

int RiskAwareSTRRTPlanner::nearest(const RRTNode &target, TreeType tree) const
{
  int best = -1;
  double best_d = kInf;
  for (int i = 0; i < static_cast<int>(nodes_.size()); ++i)
  {
    const RRTNode &n = nodes_[i];
    if (n.removed || n.tree != tree)
      continue;
    double d;
    if (tree == TreeType::START)
      d = timeAwareDist(n, target);
    else
      d = timeAwareDist(target, n);
    if (d < best_d)
    {
      best_d = d;
      best = i;
    }
  }
  return best;
}

// ── extend / connect / rewire ────────────────────────────────────────────────

int RiskAwareSTRRTPlanner::extend(TreeType tree_type)
{
  if (static_cast<int>(nodes_.size()) >= max_nodes_)
    return -1;

  auto x_rand_opt = sampleConditionally(0.0);
  if (!x_rand_opt)
    return -1;
  RRTNode x_rand = *x_rand_opt;

  const int i_near = nearest(x_rand, tree_type);
  if (i_near < 0)
    return -1;

  std::optional<RRTNode> x_new_opt;
  if (tree_type == TreeType::START)
  {
    x_new_opt = steer(nodes_[i_near], x_rand);
  }
  else
  {
    // goal tree: 시간이 거꾸로. steer 는 from.t < to.t 가정.
    // 따라서 (x_rand → nodes_[i_near]) 방향으로 steer 한 뒤 거꾸로 사용.
    x_new_opt = steer(x_rand, nodes_[i_near]);
    if (!x_new_opt)
      return -1;
    // x_new가 goal tree에 합류: 부모는 i_near, 다만 x_new.t < i_near.t 여야 함
    // steer가 만든 x_new는 x_rand 근처(시간 작음). 후속 처리에서 edgeCheck(x_new, i_near).
  }
  if (!x_new_opt)
    return -1;
  RRTNode x_new = *x_new_opt;

  // hard occupancy 빠른 컷
  if (x_new.ix < 0 || x_new.ix >= cells_x_ ||
      x_new.iy < 0 || x_new.iy >= cells_y_ ||
      x_new.it < 0 || x_new.it >= cells_t_)
    return -1;
  if (step_map_->cellCost(x_new.ix, x_new.iy, x_new.it) >= tau_hard_)
    return -1;

  // 1차 충돌 + 비용 (i_near 와 x_new)
  EdgeCheck ec_near = (tree_type == TreeType::START)
                        ? edgeCheck(nodes_[i_near], x_new)
                        : edgeCheck(x_new, nodes_[i_near]);
  if (!ec_near.ok)
    return -1;

  // RRT* best-parent
  std::vector<int> neighbors = collectNeighbors(x_new, k_rrtstar_, tree_type);

  int best_parent = i_near;
  double best_cost = nodes_[i_near].path_cost + ec_near.cost;

  for (int n : neighbors)
  {
    if (n == i_near)
      continue;
    if (nodes_[n].removed || nodes_[n].tree != tree_type)
      continue;

    EdgeCheck e;
    if (tree_type == TreeType::START)
    {
      if (nodes_[n].t >= x_new.t)
        continue;
      e = edgeCheck(nodes_[n], x_new);
    }
    else
    {
      if (nodes_[n].t <= x_new.t)
        continue;
      e = edgeCheck(x_new, nodes_[n]);
    }
    if (!e.ok)
      continue;

    const double c = nodes_[n].path_cost + e.cost;
    if (c < best_cost)
    {
      best_cost = c;
      best_parent = n;
    }
  }

  x_new.parent = best_parent;
  x_new.tree   = tree_type;
  x_new.root   = nodes_[best_parent].root;
  x_new.path_cost = best_cost;

  nodes_.push_back(x_new);
  const int idx = static_cast<int>(nodes_.size()) - 1;
  nodes_[best_parent].children.push_back(idx);

  if (tree_type == TreeType::GOAL)
    rewireGoalForest(idx, neighbors);

  return idx;
}

void RiskAwareSTRRTPlanner::rewireGoalForest(int idx_new, const std::vector<int> &neighbors)
{
  RRTNode &x_new = nodes_[idx_new];
  for (int nb : neighbors)
  {
    if (nb == x_new.parent || nb == idx_new)
      continue;
    if (nodes_[nb].removed || nodes_[nb].tree != TreeType::GOAL)
      continue;
    // goal tree에선 부모의 시간이 더 크다 (역방향). x_new를 nb의 새 부모로 만들려면 x_new.t > nb.t
    if (x_new.t <= nodes_[nb].t)
      continue;
    EdgeCheck e = edgeCheck(nodes_[nb], x_new);
    if (!e.ok)
      continue;
    const double new_cost = x_new.path_cost + e.cost;
    if (new_cost < nodes_[nb].path_cost)
    {
      const double delta = new_cost - nodes_[nb].path_cost;
      detachFromParent(nb);
      nodes_[nb].parent = idx_new;
      const int new_root = x_new.root;
      x_new.children.push_back(nb);
      nodes_[nb].path_cost = new_cost;
      nodes_[nb].root = new_root;
      propagateCostAndRoot(nb, delta, new_root);
    }
  }
}

void RiskAwareSTRRTPlanner::propagateCostAndRoot(int idx, double delta, int new_root)
{
  // BFS through descendants
  std::vector<int> stack;
  for (int c : nodes_[idx].children)
    stack.push_back(c);
  while (!stack.empty())
  {
    const int cur = stack.back();
    stack.pop_back();
    nodes_[cur].path_cost += delta;
    nodes_[cur].root = new_root;
    for (int c : nodes_[cur].children)
      stack.push_back(c);
  }
}

void RiskAwareSTRRTPlanner::detachFromParent(int idx)
{
  const int p = nodes_[idx].parent;
  if (p < 0)
    return;
  auto &ch = nodes_[p].children;
  ch.erase(std::remove(ch.begin(), ch.end(), idx), ch.end());
}

void RiskAwareSTRRTPlanner::tryConnect(int idx_new)
{
  if (idx_new < 0)
    return;
  const RRTNode &x_new = nodes_[idx_new];
  const TreeType other = (x_new.tree == TreeType::START) ? TreeType::GOAL : TreeType::START;

  // 후보: 반대 트리에서 시간 단조 만족하는 노드들
  for (int i = 0; i < static_cast<int>(nodes_.size()); ++i)
  {
    const RRTNode &y = nodes_[i];
    if (y.removed || y.tree != other)
      continue;

    // 시간 순서: start tree → goal tree 면 (x_new.t < y.t)
    const RRTNode *a = nullptr;
    const RRTNode *b = nullptr;
    if (x_new.tree == TreeType::START)
    {
      if (y.t <= x_new.t)
        continue;
      a = &x_new;
      b = &y;
    }
    else
    {
      if (x_new.t <= y.t)
        continue;
      a = &y;
      b = &x_new;
    }
    EdgeCheck e = edgeCheck(*a, *b);
    if (!e.ok)
      continue;

    // goal root의 s_progress 보너스
    int goal_root_idx = (other == TreeType::GOAL) ? y.root : x_new.root;
    double s_progress = 0.0;
    if (goal_root_idx >= 0 && goal_root_idx < static_cast<int>(nodes_.size()))
    {
      // metadata: 부족하므로 0으로 처리 (Phase 4에서 reference tube 활성화 시 reuse)
      // 향후 GoalSample::s_progress를 root에 별도 저장하면 사용.
      s_progress = 0.0;
    }

    const double sol_cost = x_new.path_cost + e.cost + y.path_cost
                            - w_goal_progress_ * (s_progress / std::max(1e-3, v_preferred_));

    if (sol_cost < best_solution_cost_)
    {
      best_solution_cost_ = sol_cost;
      if (x_new.tree == TreeType::START)
        best_link_ = {idx_new, i};
      else
        best_link_ = {i, idx_new};
    }
  }
}

// ── goal sampling ────────────────────────────────────────────────────────────

RiskAwareSTRRTPlanner::GoalSample
RiskAwareSTRRTPlanner::sampleGoalFromTube(const Eigen::Vector2d &start_xy,
                                            const std::shared_ptr<RosTools::Spline2D> &ref,
                                            double spline_start) const
{
  GoalSample g;
  if (!ref)
    return g;

  const double t_horizon = cells_t_ * step_map_->timeScale();
  double s0 = spline_start;
  // Spline2D::getMatchedClosestS 가 있다면 사용 (없으면 spline_start 사용)
  const double s_min = s0 + s_min_offset_;
  const double s_max_h = s0 + v_max_ * t_horizon;
  const double s_max_l = static_cast<double>(ref->numSegments()) * 1.0;  // 안전 상한
  const double s_max = std::min(s_max_h, s_max_l);
  if (s_max <= s_min)
    return g;

  // Beta(2,1) ≈ sqrt(U)  (점점 멀리)
  std::uniform_real_distribution<double> uni01(0.0, 1.0);
  const double u = std::sqrt(uni01(rng_));
  const double s = s_min + u * (s_max - s_min);

  Eigen::Vector2d p_ref = ref->getPoint(s);
  Eigen::Vector2d tang  = ref->getVelocity(s);
  if (tang.norm() < kEps)
    return g;
  tang.normalize();
  Eigen::Vector2d normal(-tang.y(), tang.x());

  std::uniform_real_distribution<double> off_dist(-tube_width_, tube_width_);
  const double offset = off_dist(rng_);
  const Eigen::Vector2d goal_xy = p_ref + offset * normal;

  const double d = (goal_xy - start_xy).norm();
  const double t_min_feasible = std::max(t_min_, d / std::max(1e-3, v_max_));
  if (t_min_feasible >= t_horizon)
    return g;
  std::uniform_real_distribution<double> td(t_min_feasible, t_horizon);
  const double t_goal = td(rng_);

  int gx, gy, gt;
  if (!worldToCell(goal_xy, t_goal, gx, gy, gt))
    return g;
  if (step_map_->cellCost(gx, gy, gt) >= tau_hard_)
    return g;

  g.ix = gx; g.iy = gy; g.it = gt;
  cellToWorld(gx, gy, gt, g.x, g.y, g.t);
  g.s_progress = s - s0;
  g.valid = true;
  return g;
}

RiskAwareSTRRTPlanner::GoalSample
RiskAwareSTRRTPlanner::sampleGoalFromGoalsList(const Eigen::Vector2d &start_xy,
                                                 const std::vector<Goal> &goals) const
{
  GoalSample g;
  if (goals.empty())
    return g;

  std::uniform_int_distribution<size_t> pick(0, goals.size() - 1);
  const Goal &gl = goals[pick(rng_)];
  const Eigen::Vector2d goal_xy(gl.pos(0), gl.pos(1));

  const double t_horizon = cells_t_ * step_map_->timeScale();
  const double d = (goal_xy - start_xy).norm();
  const double t_min_feasible = std::max(t_min_, d / std::max(1e-3, v_max_));
  if (t_min_feasible >= t_horizon)
    return g;
  std::uniform_real_distribution<double> td(t_min_feasible, t_horizon);
  const double t_goal = td(rng_);

  int gx, gy, gt;
  if (!worldToCell(goal_xy, t_goal, gx, gy, gt))
    return g;
  if (step_map_->cellCost(gx, gy, gt) >= tau_hard_)
    return g;

  g.ix = gx; g.iy = gy; g.it = gt;
  cellToWorld(gx, gy, gt, g.x, g.y, g.t);
  g.s_progress = -gl.cost;  // 낮은 cost → 큰 progress
  g.valid = true;
  return g;
}

std::vector<RiskAwareSTRRTPlanner::GoalSample>
RiskAwareSTRRTPlanner::initialGoalSeeds(const Eigen::Vector2d &start_xy,
                                          const std::vector<Goal> &goals,
                                          const std::shared_ptr<RosTools::Spline2D> &ref,
                                          double spline_start)
{
  std::vector<GoalSample> seeds;
  seeds.reserve(initial_goal_count_);
  const int max_tries = initial_goal_count_ * 5;
  for (int t = 0; t < max_tries && static_cast<int>(seeds.size()) < initial_goal_count_; ++t)
  {
    GoalSample g = ref ? sampleGoalFromTube(start_xy, ref, spline_start)
                       : sampleGoalFromGoalsList(start_xy, goals);
    if (g.valid)
      seeds.push_back(g);
  }
  // Fallback: 비어 있으면 goals 중 비용 최소 1개 강제 삽입
  if (seeds.empty() && !goals.empty())
  {
    auto best = std::min_element(goals.begin(), goals.end(),
        [](const Goal &a, const Goal &b) { return a.cost < b.cost; });
    GoalSample g;
    const Eigen::Vector2d gp(best->pos(0), best->pos(1));
    const double t_horizon = cells_t_ * step_map_->timeScale();
    const double d = (gp - start_xy).norm();
    const double t_goal = std::min(t_horizon - step_map_->timeScale(),
                                    std::max(t_min_, d / std::max(1e-3, v_max_)));
    int gx, gy, gt;
    if (worldToCell(gp, t_goal, gx, gy, gt))
    {
      g.ix = gx; g.iy = gy; g.it = gt;
      cellToWorld(gx, gy, gt, g.x, g.y, g.t);
      g.s_progress = -best->cost;
      g.valid = true;
      seeds.push_back(g);
    }
  }
  return seeds;
}

void RiskAwareSTRRTPlanner::addGoalRoot(const GoalSample &g)
{
  RRTNode r;
  r.ix = g.ix; r.iy = g.iy; r.it = g.it;
  r.x = g.x; r.y = g.y; r.t = g.t;
  r.heading = 0.0;
  r.path_cost = 0.0;
  r.parent = -1;
  r.tree = TreeType::GOAL;
  r.root = static_cast<int>(nodes_.size());
  nodes_.push_back(r);
  goal_roots_.push_back(r.root);
}

// ── cold / warm start ────────────────────────────────────────────────────────

void RiskAwareSTRRTPlanner::coldStart(const RRTNode &start_root,
                                        const std::vector<GoalSample> &goal_seeds)
{
  nodes_.clear();
  start_roots_.clear();
  goal_roots_.clear();
  best_link_ = {-1, -1};

  RRTNode s = start_root;
  s.parent = -1;
  s.tree = TreeType::START;
  s.path_cost = 0.0;
  s.root = 0;
  nodes_.push_back(s);
  start_roots_.push_back(0);

  for (const auto &g : goal_seeds)
  {
    if (!g.valid)
      continue;
    addGoalRoot(g);
  }
}

bool RiskAwareSTRRTPlanner::canWarmStart(const Eigen::Vector2d &start_xy) const
{
  if (!has_prev_tree_)
    return false;
  if (static_cast<int>(nodes_.size()) < cold_start_min_remaining_nodes_)
    return false;
  if ((start_xy - last_start_xy_).norm() > 5.0)  // 큰 도약 → cold
    return false;
  return true;
}

void RiskAwareSTRRTPlanner::warmStartShift(double elapsed_seconds,
                                             const RRTNode &new_start_root)
{
  for (auto &n : nodes_)
  {
    if (n.removed)
      continue;
    n.t -= elapsed_seconds;
    if (n.t < 0.0 || n.t > cells_t_ * step_map_->timeScale())
    {
      n.removed = true;
      continue;
    }
    int gx, gy, gt;
    if (!worldToCell(Eigen::Vector2d(n.x, n.y), n.t, gx, gy, gt))
    {
      n.removed = true;
      continue;
    }
    n.ix = gx; n.iy = gy; n.it = gt;
    if (step_map_->cellCost(gx, gy, gt) >= tau_hard_)
    {
      n.removed = true;
    }
  }
  // cascade remove children of removed parents
  for (auto &n : nodes_)
  {
    if (n.removed)
      continue;
    int p = n.parent;
    while (p >= 0)
    {
      if (nodes_[p].removed)
      {
        n.removed = true;
        break;
      }
      p = nodes_[p].parent;
    }
  }
  compactNodes();

  // start root 갱신: 새 root를 노드로 추가 + start_roots_ 갱신
  RRTNode s = new_start_root;
  s.parent = -1;
  s.tree = TreeType::START;
  s.path_cost = 0.0;
  s.root = static_cast<int>(nodes_.size());
  nodes_.push_back(s);
  start_roots_.clear();
  start_roots_.push_back(s.root);
}

void RiskAwareSTRRTPlanner::compactNodes()
{
  std::vector<int> remap(nodes_.size(), -1);
  std::vector<RRTNode> kept;
  kept.reserve(nodes_.size());
  for (int i = 0; i < static_cast<int>(nodes_.size()); ++i)
  {
    if (!nodes_[i].removed)
    {
      remap[i] = static_cast<int>(kept.size());
      kept.push_back(nodes_[i]);
    }
  }
  // remap parent/root/children
  for (auto &n : kept)
  {
    if (n.parent >= 0)
      n.parent = remap[n.parent];
    if (n.root >= 0)
      n.root = remap[n.root];
    if (n.root < 0)
      n.root = 0;  // 안전 가드
    std::vector<int> new_ch;
    new_ch.reserve(n.children.size());
    for (int c : n.children)
    {
      if (c >= 0 && remap[c] >= 0)
        new_ch.push_back(remap[c]);
    }
    n.children = std::move(new_ch);
  }
  // remap goal_roots_
  std::vector<int> new_roots;
  new_roots.reserve(goal_roots_.size());
  for (int g : goal_roots_)
  {
    if (g >= 0 && remap[g] >= 0)
      new_roots.push_back(remap[g]);
  }
  goal_roots_ = std::move(new_roots);

  nodes_ = std::move(kept);
}

// ── reconstruct ──────────────────────────────────────────────────────────────

GeometricPath RiskAwareSTRRTPlanner::reconstructPath(int start_leaf, int goal_leaf)
{
  // start_leaf: start tree, parent chain → start_root
  // goal_leaf : goal tree,  parent chain → goal_root
  std::vector<int> start_chain;
  for (int i = start_leaf; i >= 0; i = nodes_[i].parent)
  {
    start_chain.push_back(i);
    if (nodes_[i].parent < 0)
      break;
  }
  std::reverse(start_chain.begin(), start_chain.end());

  std::vector<int> goal_chain;
  for (int i = goal_leaf; i >= 0; i = nodes_[i].parent)
  {
    goal_chain.push_back(i);
    if (nodes_[i].parent < 0)
      break;
  }
  // goal_chain은 leaf→root, 결과 경로에는 start_leaf 다음에 goal_leaf, ..., goal_root 순으로 붙음
  // (goal root는 leaf 끝, 작은 시간 leaf가 goal_leaf)

  path_nodes_.clear();
  std::vector<int> full;
  full.reserve(start_chain.size() + goal_chain.size());
  for (int i : start_chain)
    full.push_back(i);
  for (int i : goal_chain)
    full.push_back(i);

  // 시간 단조성 확인 (서버 방어용)
  for (size_t k = 0; k < full.size(); ++k)
  {
    const RRTNode &rn = nodes_[full[k]];
    double k_time = (k == full.size() - 1)
                      ? static_cast<double>(Config::N)
                      : rn.t / Config::DT;
    SpaceTimePoint pt(rn.x, rn.y, k_time);
    NodeType type;
    int id;
    if (k == 0)
    {
      type = NodeType::GUARD;
      id = -1;
    }
    else if (k == full.size() - 1)
    {
      type = NodeType::GOAL;
      id = -2;
    }
    else
    {
      type = NodeType::CONNECTOR;
      id = static_cast<int>(k);
    }
    path_nodes_.emplace_back(id, pt, type);
  }

  std::vector<Node *> ptrs;
  ptrs.reserve(path_nodes_.size());
  for (auto &n : path_nodes_)
    ptrs.push_back(&n);

  return GeometricPath(ptrs);
}

// ── Plan() ───────────────────────────────────────────────────────────────────

std::optional<GeometricPath>
RiskAwareSTRRTPlanner::Plan(const Eigen::Vector2d &start_xy,
                             double start_theta,
                             double /*start_speed*/,
                             const std::vector<Goal> &goals,
                             const std::shared_ptr<RosTools::Spline2D> &reference_path,
                             double spline_start)
{
  if (!step_map_ || !step_map_->valid())
  {
    LOG_WARN("RiskAwareSTRRT: StepMap is not valid");
    return std::nullopt;
  }
  if (goals.empty() && !reference_path)
  {
    LOG_WARN("RiskAwareSTRRT: no goals and no reference path");
    return std::nullopt;
  }

  cells_x_ = step_map_->cellsX();
  cells_y_ = step_map_->cellsY();
  cells_t_ = step_map_->cellsT();
  cells_xy_ = static_cast<size_t>(cells_x_) * static_cast<size_t>(cells_y_);
  cells_total_ = cells_xy_ * static_cast<size_t>(cells_t_);

  // build start root
  RRTNode start_root;
  start_root.x = start_xy.x();
  start_root.y = start_xy.y();
  start_root.t = 0.0;
  start_root.heading = start_theta;
  start_root.parent = -1;
  start_root.tree = TreeType::START;
  start_root.path_cost = 0.0;
  if (!worldToCell(start_xy, 0.0, start_root.ix, start_root.iy, start_root.it))
  {
    LOG_WARN("RiskAwareSTRRT: start position outside StepMap grid");
    return std::nullopt;
  }

  auto t_begin = std::chrono::steady_clock::now();
  auto budget  = std::chrono::milliseconds(static_cast<int>(time_budget_ms_));

  const double now_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  const double elapsed = now_sec - last_plan_time_;

  if (enable_warm_start_ && canWarmStart(start_xy))
  {
    warmStartShift(elapsed, start_root);
  }
  else
  {
    auto seeds = initialGoalSeeds(start_xy, goals, reference_path, spline_start);
    if (seeds.empty())
    {
      LOG_WARN("RiskAwareSTRRT: failed to seed goals");
      return std::nullopt;
    }
    coldStart(start_root, seeds);
  }

  // Masks
  buildForwardReachableMask(nodes_[start_roots_.front()]);
  buildBackwardReachableMask(goal_roots_);
  rebuildValidIndexList();

  best_solution_cost_ = kInf;
  best_link_ = {-1, -1};

  bool a_is_start = true;
  for (int iter = 0; iter < max_iter_; ++iter)
  {
    if (std::chrono::steady_clock::now() - t_begin > budget)
      break;

    // 추가 goal 샘플 (주기적)
    if (iter > 0 && iter % 50 == 0 &&
        static_cast<int>(goal_roots_.size()) < max_goal_count_)
    {
      GoalSample g = reference_path
                       ? sampleGoalFromTube(start_xy, reference_path, spline_start)
                       : sampleGoalFromGoalsList(start_xy, goals);
      if (g.valid)
      {
        addGoalRoot(g);
        buildBackwardReachableMask(goal_roots_);
        rebuildValidIndexList();
      }
    }

    const int new_idx = extend(a_is_start ? TreeType::START : TreeType::GOAL);
    if (new_idx >= 0)
      tryConnect(new_idx);

    a_is_start = !a_is_start;
  }

  has_prev_tree_  = true;
  last_plan_time_ = now_sec;
  last_start_xy_  = start_xy;

  if (best_link_.first < 0 || best_link_.second < 0)
  {
    LOG_WARN("RiskAwareSTRRT: no solution within time budget");
    return std::nullopt;
  }

  return reconstructPath(best_link_.first, best_link_.second);
}

}  // namespace GuidancePlanner
