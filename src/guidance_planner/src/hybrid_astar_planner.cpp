#include <guidance_planner/hybrid_astar_planner.h>
#include <guidance_planner/types/connection.h>

#include <ros_tools/logging.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace GuidancePlanner
{

void HybridAStarPlanner::Init(Config *config)
{
  config_ = config;

  num_heading_bins_ = config->hastar_num_heading_bins_;
  speed_bins_       = config->hastar_speed_bins_;
  n_v_samples_      = config->hastar_n_v_samples_;
  n_w_samples_      = config->hastar_n_w_samples_;
  n_substeps_       = config->hastar_n_substeps_;
  w_max_            = config->hastar_w_max_;
  a_max_            = config->hastar_a_max_;
  goal_tol_xy_      = config->hastar_goal_tol_xy_;
  w_time_           = config->hastar_w_time_;
  w_occ_            = config->hastar_w_occ_;
  w_accel_          = config->hastar_w_accel_;
  w_yaw_            = config->hastar_w_yaw_;
  w_yaw_rate_       = config->hastar_w_yaw_rate_;
}

void HybridAStarPlanner::SetStepMap(std::shared_ptr<MPCPlannerStepMap::StepMap> step_map)
{
  step_map_ = step_map;
}

void HybridAStarPlanner::Reset()
{
  nodes_.clear();
}

// ── 헬퍼 ──────────────────────────────────────────────────────────────────────

HybridAStarPlanner::ClosedKey HybridAStarPlanner::makeKey(
    double x, double y, double theta, double v, int k) const
{
  Eigen::Vector2d local = step_map_->localFromWorld(Eigen::Vector2d(x, y));
  int i = (int)std::round((local.x() + step_map_->halfLength()) / step_map_->resolution());
  int j = (int)std::round((local.y() + step_map_->halfWidth())  / step_map_->resolution());

  // theta를 [0, 2π) 로 정규화
  double theta_norm = std::fmod(theta, 2.0 * M_PI);
  if (theta_norm < 0.0) theta_norm += 2.0 * M_PI;
  int h_bin = (int)std::floor(theta_norm / (2.0 * M_PI) * num_heading_bins_) % num_heading_bins_;

  int v_bin = std::clamp(
      (int)std::floor(v / config_->max_velocity_ * speed_bins_),
      0, speed_bins_ - 1);

  return {i, j, k, h_bin, v_bin};
}

std::vector<Eigen::Vector2d> HybridAStarPlanner::integrate(
    double x, double y, double theta, double v_cmd, double w_cmd) const
{
  const double h = Config::DT / n_substeps_;
  std::vector<Eigen::Vector2d> pts;
  pts.reserve(n_substeps_);

  double cx = x, cy = y, ctheta = theta;
  for (int s = 0; s < n_substeps_; ++s)
  {
    cx     += v_cmd * std::cos(ctheta) * h;
    cy     += v_cmd * std::sin(ctheta) * h;
    ctheta += w_cmd * h;
    pts.emplace_back(cx, cy);
  }
  return pts;
}

bool HybridAStarPlanner::checkSwept(
    const std::vector<Eigen::Vector2d> &pts, int nk, double &occ_total) const
{
  occ_total = 0.0;
  std::set<std::pair<int, int>> visited;

  for (const auto &pt : pts)
  {
    Eigen::Vector2d local = step_map_->localFromWorld(pt);
    int ii = (int)std::round((local.x() + step_map_->halfLength()) / step_map_->resolution());
    int jj = (int)std::round((local.y() + step_map_->halfWidth())  / step_map_->resolution());

    if (ii < 0 || ii >= step_map_->cellsX() ||
        jj < 0 || jj >= step_map_->cellsY() ||
        nk < 0 || nk >= step_map_->cellsT())
      return true;

    if (step_map_->cellOccupied(ii, jj, nk)) return true;

    if (visited.insert({ii, jj}).second)
      occ_total += step_map_->cellCost(ii, jj, nk);
  }
  return false;
}

double HybridAStarPlanner::heuristic(double x, double y, double gx, double gy) const
{
  return std::hypot(x - gx, y - gy) / config_->max_velocity_;
}

// ── 프리미티브 push 헬퍼 ──────────────────────────────────────────────────────

void HybridAStarPlanner::pushIfBetter(
    std::priority_queue<std::shared_ptr<SearchNode>,
                        std::vector<std::shared_ptr<SearchNode>>,
                        SearchNodePtrCmp> &pq,
    std::unordered_map<ClosedKey, double, ClosedKeyHash, ClosedKeyEq> &best_g,
    std::shared_ptr<SearchNode> parent,
    double v_cmd, double w_cmd,
    double gx, double gy, int nk, int &counter)
{
  auto pts = integrate(parent->x, parent->y, parent->theta, v_cmd, w_cmd);
  double occ_total;
  if (checkSwept(pts, nk, occ_total)) return;

  const auto &end_pt   = pts.back();
  const double dtheta  = std::abs(w_cmd * Config::DT);
  const double step_cost = w_time_     * Config::DT
                         + w_occ_      * occ_total
                         + w_accel_    * std::abs(v_cmd - parent->v)
                         + w_yaw_      * dtheta
                         + w_yaw_rate_ * std::abs(w_cmd) * Config::DT;
  const double ng = parent->g + step_cost;

  const double new_theta = parent->theta + w_cmd * Config::DT;
  auto nkey = makeKey(end_pt.x(), end_pt.y(), new_theta, v_cmd, nk);

  auto it = best_g.find(nkey);
  if (it != best_g.end() && ng >= it->second - 1e-9) return;
  best_g[nkey] = ng;

  auto next      = std::make_shared<SearchNode>();
  next->x        = end_pt.x();
  next->y        = end_pt.y();
  next->theta    = new_theta;
  next->v        = v_cmd;
  next->k        = nk;
  next->g        = ng;
  next->f        = ng + heuristic(end_pt.x(), end_pt.y(), gx, gy);
  next->counter  = counter++;
  next->parent   = parent;
  pq.push(next);
}

// ── 탐색 ──────────────────────────────────────────────────────────────────────

std::optional<GeometricPath> HybridAStarPlanner::Plan(
    const Eigen::Vector2d &start_xy,
    double start_theta,
    double start_speed,
    const Eigen::Vector2d &goal_xy)
{
  if (!step_map_ || !step_map_->valid())
  {
    LOG_WARN("HybridAStarPlanner::Plan — StepMap is not valid");
    return std::nullopt;
  }

  const double gx  = goal_xy.x(), gy = goal_xy.y();
  const int    N_T = step_map_->cellsT();

  using PQ = std::priority_queue<std::shared_ptr<SearchNode>,
                                  std::vector<std::shared_ptr<SearchNode>>,
                                  SearchNodePtrCmp>;
  PQ open_pq;
  std::unordered_map<ClosedKey, double, ClosedKeyHash, ClosedKeyEq> best_g;

  int counter = 0;

  auto start_node    = std::make_shared<SearchNode>();
  start_node->x      = start_xy.x();
  start_node->y      = start_xy.y();
  start_node->theta  = start_theta;
  start_node->v      = start_speed;
  start_node->k      = 0;
  start_node->g      = 0.0;
  start_node->f      = heuristic(start_xy.x(), start_xy.y(), gx, gy);
  start_node->counter = counter++;
  start_node->parent  = nullptr;

  auto start_key = makeKey(start_node->x, start_node->y,
                           start_node->theta, start_node->v, 0);
  best_g[start_key] = 0.0;
  open_pq.push(start_node);

  std::shared_ptr<SearchNode> goal_node       = nullptr;
  std::shared_ptr<SearchNode> best_terminal   = nullptr;
  double                       best_term_dist = std::numeric_limits<double>::infinity();

  while (!open_pq.empty())
  {
    auto cur = open_pq.top();
    open_pq.pop();

    // stale 항목 skip
    auto cur_key = makeKey(cur->x, cur->y, cur->theta, cur->v, cur->k);
    {
      auto it = best_g.find(cur_key);
      if (it != best_g.end() && cur->g > it->second + 1e-9) continue;
    }

    // 목표 도달 판정: 마지막 시간 스텝 + 위치 근접
    if (cur->k == N_T - 1)
    {
      double dist = std::hypot(cur->x - gx, cur->y - gy);
      if (dist <= goal_tol_xy_)
      {
        goal_node = cur;
        break;
      }
      if (dist < best_term_dist)
      {
        best_term_dist = dist;
        best_terminal  = cur;
      }
      continue;
    }

    const int nk = cur->k + 1;
    const double dist_to_goal = std::hypot(cur->x - gx, cur->y - gy);

    // 가속도 제한에 의한 속도 범위
    const double v_lo = std::max(0.0,                     cur->v - a_max_ * Config::DT);
    const double v_hi = std::min(config_->max_velocity_,  cur->v + a_max_ * Config::DT);

    // 그리드 모션 프리미티브
    for (int vi = 0; vi < n_v_samples_; ++vi)
    {
      const double v_cmd = (n_v_samples_ > 1)
                         ? v_lo + (v_hi - v_lo) * vi / (n_v_samples_ - 1)
                         : (v_lo + v_hi) * 0.5;

      for (int wi = 0; wi < n_w_samples_; ++wi)
      {
        const double w_cmd = (n_w_samples_ > 1)
                           ? -w_max_ + 2.0 * w_max_ * wi / (n_w_samples_ - 1)
                           : 0.0;

        pushIfBetter(open_pq, best_g, cur, v_cmd, w_cmd, gx, gy, nk, counter);
      }
    }

    // 목표 근방이면 명시적 정지(hover) 프리미티브 추가
    if (dist_to_goal <= goal_tol_xy_)
      pushIfBetter(open_pq, best_g, cur, 0.0, 0.0, gx, gy, nk, counter);
  }

  if (!goal_node)
  {
    if (best_terminal)
    {
      LOG_WARN("HybridAStarPlanner: Exact goal not reached, falling back to best terminal node"
               << " (dist=" << best_term_dist << "m)");
      goal_node = best_terminal;
    }
    else
    {
      LOG_WARN("HybridAStarPlanner: No path found");
      return std::nullopt;
    }
  }

  return reconstructPath(goal_node);
}

// ── 경로 재구성 → GeometricPath ───────────────────────────────────────────────

GeometricPath HybridAStarPlanner::reconstructPath(std::shared_ptr<SearchNode> goal_node)
{
  std::vector<std::shared_ptr<SearchNode>> chain;
  for (auto cur = goal_node; cur; cur = cur->parent)
    chain.push_back(cur);
  std::reverse(chain.begin(), chain.end());

  nodes_.clear();
  for (size_t i = 0; i < chain.size(); ++i)
  {
    const auto &sn = chain[i];
    // goal 노드의 k는 CubicSpline3D 관례에 맞게 Config::N 으로 강제
    const double k_time = (i == chain.size() - 1)
        ? static_cast<double>(Config::N)
        : static_cast<double>(sn->k);
    const SpaceTimePoint pt(sn->x, sn->y, k_time);

    NodeType type;
    int id;
    if (i == 0)
    {
      type = NodeType::GUARD;
      id   = -1;
    }
    else if (i == chain.size() - 1)
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
