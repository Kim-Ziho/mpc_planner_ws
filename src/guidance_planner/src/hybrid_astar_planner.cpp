#include <guidance_planner/hybrid_astar_planner.h>
#include <guidance_planner/types/connection.h>

#include <ros_tools/logging.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace GuidancePlanner
{

namespace
{
constexpr size_t kNodePoolReserve = 64 * 1024;  // ~64k nodes pre-allocation
constexpr size_t kBestGReserve    = 16 * 1024;
constexpr int    kBudgetCheckMask = 0xFF;       // every 256 pops
}  // namespace

void HybridAStarPlanner::Init(Config *config)
{
  config_ = config;

  num_heading_bins_ = config->hastar_num_heading_bins_;
  speed_bins_       = config->hastar_speed_bins_;
  n_v_samples_      = config->hastar_n_v_samples_;
  n_w_samples_      = config->hastar_n_w_samples_;
  n_substeps_       = std::min(config->hastar_n_substeps_, kMaxSubsteps);
  w_max_            = config->hastar_w_max_;
  a_max_            = config->hastar_a_max_;
  goal_tol_xy_      = config->hastar_goal_tol_xy_;
  w_time_           = config->hastar_w_time_;
  w_occ_            = config->hastar_w_occ_;
  w_accel_          = config->hastar_w_accel_;
  w_yaw_            = config->hastar_w_yaw_;
  w_yaw_rate_       = config->hastar_w_yaw_rate_;
  time_budget_ms_   = config->hastar_time_budget_ms_;

  pool_.reserve(kNodePoolReserve);
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

void HybridAStarPlanner::cellFromWorld(double x, double y, int &ii, int &jj) const
{
  Eigen::Vector2d local = step_map_->localFromWorld(Eigen::Vector2d(x, y));
  ii = static_cast<int>(std::round((local.x() + step_map_->halfLength()) / step_map_->resolution()));
  jj = static_cast<int>(std::round((local.y() + step_map_->halfWidth())  / step_map_->resolution()));
}

HybridAStarPlanner::ClosedKey HybridAStarPlanner::makeKey(
    double x, double y, double theta, double v, int k) const
{
  int i, j;
  cellFromWorld(x, y, i, j);

  // theta를 [0, 2π) 로 정규화
  double theta_norm = std::fmod(theta, 2.0 * M_PI);
  if (theta_norm < 0.0) theta_norm += 2.0 * M_PI;
  int h_bin = static_cast<int>(std::floor(theta_norm / (2.0 * M_PI) * num_heading_bins_)) % num_heading_bins_;

  int v_bin = std::clamp(
      static_cast<int>(std::floor(v / config_->max_velocity_ * speed_bins_)),
      0, speed_bins_ - 1);

  return {i, j, k, h_bin, v_bin};
}

int HybridAStarPlanner::integrate(
    double x, double y, double theta, double v_cmd, double w_cmd,
    std::array<Eigen::Vector2d, kMaxSubsteps> &pts_out) const
{
  const double h = Config::DT / n_substeps_;

  double cx = x, cy = y, ctheta = theta;
  for (int s = 0; s < n_substeps_; ++s)
  {
    cx     += v_cmd * std::cos(ctheta) * h;
    cy     += v_cmd * std::sin(ctheta) * h;
    ctheta += w_cmd * h;
    pts_out[s] = Eigen::Vector2d(cx, cy);
  }
  return n_substeps_;
}

bool HybridAStarPlanner::checkSwept(
    const std::array<Eigen::Vector2d, kMaxSubsteps> &pts, int n_pts,
    int nk, double &occ_total) const
{
  occ_total = 0.0;

  // n_pts ≤ kMaxSubsteps(=16)이므로 선형 탐색이 std::set보다 빠르다
  std::array<std::pair<int, int>, kMaxSubsteps> visited;
  int n_visited = 0;

  const int CX = step_map_->cellsX();
  const int CY = step_map_->cellsY();
  const int CT = step_map_->cellsT();
  if (nk < 0 || nk >= CT) return true;

  for (int p = 0; p < n_pts; ++p)
  {
    int ii, jj;
    cellFromWorld(pts[p].x(), pts[p].y(), ii, jj);

    if (ii < 0 || ii >= CX || jj < 0 || jj >= CY) return true;
    if (step_map_->cellOccupied(ii, jj, nk)) return true;

    // 중복 셀 비용 집계 방지: 선형 탐색 (n ≤ 16)
    bool seen = false;
    for (int q = 0; q < n_visited; ++q)
    {
      if (visited[q].first == ii && visited[q].second == jj) { seen = true; break; }
    }
    if (!seen)
    {
      visited[n_visited++] = {ii, jj};
      occ_total += step_map_->cellCost(ii, jj, nk);
    }
  }
  return false;
}

double HybridAStarPlanner::heuristic(double x, double y, double gx, double gy) const
{
  // 2D Dijkstra 맵이 준비된 경우: 정적 장애물을 고려한 admissible 휴리스틱
  if (!heur_grid_.empty())
  {
    int ii, jj;
    cellFromWorld(x, y, ii, jj);
    if (ii >= 0 && ii < heur_cells_x_ && jj >= 0 && jj < heur_cells_y_)
    {
      double d = heur_grid_[static_cast<size_t>(jj) * heur_cells_x_ + ii];
      if (std::isfinite(d))
        return d / config_->max_velocity_;
    }
  }
  // fallback: 순수 유클리드
  return std::hypot(x - gx, y - gy) / config_->max_velocity_;
}

// ── 2D Dijkstra 휴리스틱 사전 계산 ───────────────────────────────────────────

void HybridAStarPlanner::buildHeuristicMap(double gx, double gy)
{
  const int CX = step_map_->cellsX();
  const int CY = step_map_->cellsY();
  heur_cells_x_ = CX;
  heur_cells_y_ = CY;
  heur_grid_.assign(static_cast<size_t>(CX) * CY, std::numeric_limits<double>::infinity());

  int gi, gj;
  cellFromWorld(gx, gy, gi, gj);
  if (gi < 0 || gi >= CX || gj < 0 || gj >= CY) return;

  using QEntry = std::pair<double, int>;  // (dist, flat_idx)
  std::priority_queue<QEntry, std::vector<QEntry>, std::greater<QEntry>> pq;
  const int gidx = gj * CX + gi;
  heur_grid_[gidx] = 0.0;
  pq.emplace(0.0, gidx);

  const double res    = step_map_->resolution();
  const double d_card = res;
  const double d_diag = res * std::sqrt(2.0);

  static const int dx[8]    = { 1, -1,  0,  0,  1,  1, -1, -1};
  static const int dy[8]    = { 0,  0,  1, -1,  1, -1,  1, -1};
  const double      dcost[8] = {d_card, d_card, d_card, d_card,
                                 d_diag, d_diag, d_diag, d_diag};

  while (!pq.empty())
  {
    const auto [d, idx] = pq.top();
    pq.pop();
    if (d > heur_grid_[idx] + 1e-9) continue;
    const int ci = idx % CX;
    const int cj = idx / CX;
    for (int n = 0; n < 8; ++n)
    {
      const int ni = ci + dx[n];
      const int nj = cj + dy[n];
      if (ni < 0 || ni >= CX || nj < 0 || nj >= CY) continue;
      // t=0 정적 슬라이스 기준 (동적 장애물 무시 → admissible)
      if (step_map_->cellOccupied(ni, nj, 0)) continue;
      const double nd = d + dcost[n];
      const int nidx  = nj * CX + ni;
      if (nd < heur_grid_[nidx])
      {
        heur_grid_[nidx] = nd;
        pq.emplace(nd, nidx);
      }
    }
  }
}

// ── 프리미티브 push 헬퍼 ──────────────────────────────────────────────────────

void HybridAStarPlanner::pushIfBetter(
    OpenPQ &pq, BestG &best_g,
    int parent_idx,
    double v_cmd, double w_cmd,
    double gx, double gy, int nk, int &counter)
{
  // 부모 상태를 로컬 변수로 복사 — 이후 pool_.push_back이 reference를 무효화할 수 있다
  const double px     = pool_[parent_idx].x;
  const double py     = pool_[parent_idx].y;
  const double ptheta = pool_[parent_idx].theta;
  const double pv     = pool_[parent_idx].v;
  const double pg     = pool_[parent_idx].g;

  std::array<Eigen::Vector2d, kMaxSubsteps> pts;
  const int n_pts = integrate(px, py, ptheta, v_cmd, w_cmd, pts);

  double occ_total;
  if (checkSwept(pts, n_pts, nk, occ_total)) return;

  const Eigen::Vector2d &end_pt = pts[n_pts - 1];
  const double dtheta    = std::abs(w_cmd * Config::DT);
  const double step_cost = w_time_     * Config::DT
                         + w_occ_      * occ_total
                         + w_accel_    * std::abs(v_cmd - pv)
                         + w_yaw_      * dtheta
                         + w_yaw_rate_ * std::abs(w_cmd) * Config::DT;
  const double ng        = pg + step_cost;
  const double new_theta = ptheta + w_cmd * Config::DT;

  const ClosedKey nkey = makeKey(end_pt.x(), end_pt.y(), new_theta, v_cmd, nk);

  auto it = best_g.find(nkey);
  if (it != best_g.end() && ng >= it->second - 1e-9) return;
  best_g[nkey] = ng;

  pool_.push_back({});
  SearchNode &nn = pool_.back();
  nn.x          = end_pt.x();
  nn.y          = end_pt.y();
  nn.theta      = new_theta;
  nn.v          = v_cmd;
  nn.k          = nk;
  nn.g          = ng;
  nn.f          = ng + heuristic(end_pt.x(), end_pt.y(), gx, gy);
  nn.counter    = counter++;
  nn.parent_idx = parent_idx;
  nn.key        = nkey;
  pq.push({nn.f, nn.counter, static_cast<int>(pool_.size()) - 1});
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

  const auto   t_start   = std::chrono::steady_clock::now();
  const double budget_ms = time_budget_ms_;

  const double gx  = goal_xy.x(), gy = goal_xy.y();
  const int    N_T = step_map_->cellsT();

  // 1) 2D Dijkstra 휴리스틱 사전 계산 (수 ms)
  buildHeuristicMap(gx, gy);

  // 2) 탐색 자료구조 초기화
  pool_.clear();
  OpenPQ open_pq;
  BestG  best_g;
  best_g.reserve(kBestGReserve);

  int counter = 0;

  pool_.push_back({});
  SearchNode &start_node = pool_.back();
  start_node.x          = start_xy.x();
  start_node.y          = start_xy.y();
  start_node.theta      = start_theta;
  start_node.v          = start_speed;
  start_node.k          = 0;
  start_node.g          = 0.0;
  start_node.f          = heuristic(start_xy.x(), start_xy.y(), gx, gy);
  start_node.counter    = counter++;
  start_node.parent_idx = -1;
  start_node.key        = makeKey(start_xy.x(), start_xy.y(), start_theta, start_speed, 0);

  best_g[start_node.key] = 0.0;
  open_pq.push({start_node.f, start_node.counter, 0});

  int    goal_idx          = -1;
  int    best_terminal_idx = -1;
  double best_term_dist    = std::numeric_limits<double>::infinity();

  bool budget_exceeded = false;
  int  pop_count       = 0;

  while (!open_pq.empty())
  {
    // 3) 시간 예산 조기 종료 (chrono 호출 비용 회피 위해 256회마다 검사)
    if ((++pop_count & kBudgetCheckMask) == 0)
    {
      const double elapsed = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - t_start).count();
      if (elapsed > budget_ms) { budget_exceeded = true; break; }
    }

    const PQEntry top = open_pq.top();
    open_pq.pop();

    const int cur_idx = top.idx;

    // 4) stale 체크: 캐시된 key 사용 → makeKey 재호출 없음
    {
      const SearchNode &cur_ref = pool_[cur_idx];
      auto it = best_g.find(cur_ref.key);
      if (it != best_g.end() && cur_ref.g > it->second + 1e-9) continue;
    }

    // 부모 상태 복사 — 이후 pushIfBetter가 pool_ 재할당을 유발할 수 있음
    const double cx = pool_[cur_idx].x;
    const double cy = pool_[cur_idx].y;
    const int    ck = pool_[cur_idx].k;
    const double cv = pool_[cur_idx].v;

    // 5) 목표 도달 판정
    if (ck == N_T - 1)
    {
      const double dist = std::hypot(cx - gx, cy - gy);
      if (dist <= goal_tol_xy_) { goal_idx = cur_idx; break; }
      if (dist < best_term_dist) { best_term_dist = dist; best_terminal_idx = cur_idx; }
      continue;
    }

    const int    nk           = ck + 1;
    const double v_lo         = std::max(0.0,                    cv - a_max_ * Config::DT);
    const double v_hi         = std::min(config_->max_velocity_, cv + a_max_ * Config::DT);
    const double dist_to_goal = std::hypot(cx - gx, cy - gy);

    // 6) 그리드 모션 프리미티브
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
        pushIfBetter(open_pq, best_g, cur_idx, v_cmd, w_cmd, gx, gy, nk, counter);
      }
    }

    // 7) 목표 근방이면 hover 프리미티브 추가
    if (dist_to_goal <= goal_tol_xy_)
      pushIfBetter(open_pq, best_g, cur_idx, 0.0, 0.0, gx, gy, nk, counter);
  }

  if (budget_exceeded)
    LOG_WARN("HybridAStarPlanner: time budget (" << budget_ms << " ms) exceeded, using best terminal");

  if (goal_idx < 0)
  {
    if (best_terminal_idx >= 0)
    {
      if (!budget_exceeded)
        LOG_WARN("HybridAStarPlanner: Exact goal not reached, falling back to best terminal node"
                 << " (dist=" << best_term_dist << "m)");
      goal_idx = best_terminal_idx;
    }
    else
    {
      LOG_WARN("HybridAStarPlanner: No path found");
      return std::nullopt;
    }
  }

  return reconstructPath(goal_idx);
}

// ── 경로 재구성 → GeometricPath ───────────────────────────────────────────────

GeometricPath HybridAStarPlanner::reconstructPath(int goal_idx)
{
  std::vector<int> chain;
  for (int idx = goal_idx; idx >= 0; idx = pool_[idx].parent_idx)
    chain.push_back(idx);
  std::reverse(chain.begin(), chain.end());

  nodes_.clear();
  for (size_t i = 0; i < chain.size(); ++i)
  {
    const SearchNode &sn = pool_[chain[i]];
    // goal 노드의 k는 CubicSpline3D 관례에 맞게 Config::N 으로 강제
    const double k_time = (i == chain.size() - 1)
        ? static_cast<double>(Config::N)
        : static_cast<double>(sn.k);
    const SpaceTimePoint pt(sn.x, sn.y, k_time);

    NodeType type;
    int      id;
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
