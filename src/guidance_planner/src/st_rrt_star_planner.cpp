#include <guidance_planner/st_rrt_star_planner.h>
#include <guidance_planner/types/connection.h>
#include <guidance_planner/types/space_time_point.h>

#include <ros_tools/logging.h>
#include <ros_tools/spline.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace GuidancePlanner
{

void STRRTStarPlanner::Init(Config *config)
{
  config_       = config;
  max_iter_     = config->strrt_max_iter_;
  steer_dt_min_ = config->strrt_steer_dt_min_;
  steer_dt_max_ = config->strrt_steer_dt_max_;
  neighbor_radius_ = config->strrt_neighbor_radius_;
  match_tol_    = config->strrt_match_tol_;
  goal_radius_  = config->strrt_goal_radius_;
  goal_bias_    = config->strrt_goal_bias_;
  w_time_       = config->strrt_w_time_;
  w_ctrl_       = config->strrt_w_ctrl_;
  check_dt_     = config->strrt_check_dt_;
  v_max_        = config->max_velocity_;
  w_max_        = config->hastar_w_max_;
  path_lat_half_width_ = config->strrt_path_lat_half_width_;

  rng_.seed(static_cast<uint32_t>(config->seed_ >= 0 ? config->seed_ : std::random_device{}()));
}

void STRRTStarPlanner::SetStepMap(std::shared_ptr<MPCPlannerStepMap::StepMap> step_map)
{
  step_map_ = step_map;
}

void STRRTStarPlanner::Reset()
{
  path_nodes_.clear();
}

// ── 유니사이클 적분 ────────────────────────────────────────────────────────────

void STRRTStarPlanner::unicycleStep(double &x, double &y, double &theta,
                                    double v, double w, double dt)
{
  if (std::abs(w) < 1e-6)
  {
    x += v * std::cos(theta) * dt;
    y += v * std::sin(theta) * dt;
  }
  else
  {
    double th_new = theta + w * dt;
    x += (v / w) * (std::sin(th_new) - std::sin(theta));
    y -= (v / w) * (std::cos(th_new) - std::cos(theta));
    theta = th_new;
    return;
  }
  theta += w * dt;
}

// ── Steer ──────────────────────────────────────────────────────────────────────

std::optional<STRRTStarPlanner::SteerResult>
STRRTStarPlanner::steer(const RRTNode &from,
                         double x_to, double y_to, double t_to) const
{
  double dt = t_to - from.t;
  if (dt < steer_dt_min_)
    return std::nullopt;
  if (dt > steer_dt_max_)
    dt = steer_dt_max_;

  double dx = x_to - from.x;
  double dy = y_to - from.y;
  double d  = std::hypot(dx, dy);
  if (d < 1e-6)
    return std::nullopt;

  double psi  = std::atan2(dy, dx);
  double dpsi = std::atan2(std::sin(psi - from.theta), std::cos(psi - from.theta));

  double w = std::max(-w_max_, std::min(w_max_, dpsi / dt));
  double v = std::max(0.0,     std::min(v_max_, d    / dt));

  double x_new = from.x;
  double y_new = from.y;
  double th    = from.theta;
  unicycleStep(x_new, y_new, th, v, w, dt);

  SteerResult r;
  r.x     = x_new;
  r.y     = y_new;
  r.theta = th;
  r.t     = from.t + dt;
  r.v     = v;
  r.w     = w;
  return r;
}

// ── Edge 충돌 검사 ────────────────────────────────────────────────────────────

bool STRRTStarPlanner::edgeCollisionFree(const RRTNode &from,
                                          double v, double w, double dt) const
{
  if (!step_map_ || !step_map_->valid())
    return true;

  int n_steps = std::max(2, static_cast<int>(dt / check_dt_));
  for (int k = 0; k <= n_steps; ++k)
  {
    double tau = static_cast<double>(k) / n_steps * dt;
    double x = from.x, y = from.y, th = from.theta;
    unicycleStep(x, y, th, v, w, tau);

    double t_abs = from.t + tau;
    int layer = static_cast<int>(std::round(t_abs / Config::DT));
    layer = std::max(0, std::min(layer, step_map_->cellsT() - 1));

    if (step_map_->isOccupiedWorld(Eigen::Vector2d(x, y), layer))
      return false;
  }
  return true;
}

// ── 샘플링 ────────────────────────────────────────────────────────────────────

std::optional<STRRTStarPlanner::Sample>
STRRTStarPlanner::sampleState(const SampleContext &ctx) const
{
  std::uniform_real_distribution<double> uni01(0.0, 1.0);

  // ── (1) goal-bias 모드: goal 근방, 단 AABB 안으로 clamp ────────────────────
  if (uni01(rng_) < goal_bias_)
  {
    if (ctx.t_min_goal >= ctx.t_upper)
      return std::nullopt;
    std::uniform_real_distribution<double> gt_dist(ctx.t_min_goal, ctx.t_upper);
    std::uniform_real_distribution<double> xy_dist(-0.3, 0.3);
    double x = std::min(ctx.x_max, std::max(ctx.x_min, ctx.goal_xy.x() + xy_dist(rng_)));
    double y = std::min(ctx.y_max, std::max(ctx.y_min, ctx.goal_xy.y() + xy_dist(rng_)));
    return Sample{x, y, gt_dist(rng_)};
  }

  // ── (2) reference path 가 없으면 AABB 전체 균일 샘플 (기존 동작) ───────────
  if (!ctx.reference_path || ctx.max_s <= ctx.cur_s)
  {
    std::uniform_real_distribution<double> xd(ctx.x_min, ctx.x_max);
    std::uniform_real_distribution<double> yd(ctx.y_min, ctx.y_max);
    double t_lower = steer_dt_min_;
    if (t_lower >= ctx.t_upper)
      return std::nullopt;
    std::uniform_real_distribution<double> td(t_lower, ctx.t_upper);
    return Sample{xd(rng_), yd(rng_), td(rng_)};
  }

  // ── (3) AABB ∩ along-reference-path 샘플 ────────────────────────────────
  //   s ∈ [cur_s, max_s], lat ∈ [-W, W], position = ref(s) + lat·n(s)
  //   AABB 밖이면 reject 후 재시도. t_lower = (s - cur_s) / v_max
  std::uniform_real_distribution<double> sd(ctx.cur_s, ctx.max_s);
  std::uniform_real_distribution<double> ld(-path_lat_half_width_, path_lat_half_width_);

  for (int retry = 0; retry < 16; ++retry)
  {
    const double s = sd(rng_);
    const double lat = ld(rng_);

    Eigen::Vector2d p = ctx.reference_path->getPoint(s);
    Eigen::Vector2d n = ctx.reference_path->getOrthogonal(s);
    Eigen::Vector2d xy = p + lat * n;

    if (xy.x() < ctx.x_min || xy.x() > ctx.x_max ||
        xy.y() < ctx.y_min || xy.y() > ctx.y_max)
      continue;  // AABB 밖 — 교집합 조건 위반

    // 도달 가능 최소 시각: start → 해당 s 까지 ref 곡선거리(≈ s - cur_s) / v_max
    // 횡방향 오프셋의 추가 거리는 무시(작은 lat 가정). 보수적으로 약간의 여유는
    // steer_dt_min_ 로 보장.
    const double s_offset = s - ctx.cur_s;
    const double t_lower  = std::max(steer_dt_min_, s_offset / std::max(1e-3, v_max_));
    if (t_lower >= ctx.t_upper)
      continue;

    std::uniform_real_distribution<double> td(t_lower, ctx.t_upper);
    return Sample{xy.x(), xy.y(), td(rng_)};
  }
  return std::nullopt;
}

// ── 거리 메트릭 ────────────────────────────────────────────────────────────────

double STRRTStarPlanner::timeAwareDist(const RRTNode &a,
                                        double x, double y, double t) const
{
  double dt = t - a.t;
  if (dt <= 1e-6)
    return std::numeric_limits<double>::infinity();
  double d = std::hypot(x - a.x, y - a.y);
  if (d > v_max_ * dt + 1e-6)
    return std::numeric_limits<double>::infinity();
  return dt + d / v_max_;
}

// ── 비용 ──────────────────────────────────────────────────────────────────────

double STRRTStarPlanner::edgeCost(double dt, double v, double w) const
{
  return w_time_ * dt + w_ctrl_ * (v * v + 5.0 * w * w) * dt;
}

// ── 경로 재구성 ────────────────────────────────────────────────────────────────

GeometricPath STRRTStarPlanner::reconstructPath(const std::vector<RRTNode> &nodes,
                                                 int goal_idx)
{
  // goal → root 역추적
  std::vector<int> chain;
  for (int i = goal_idx; i >= 0; i = nodes[i].parent)
  {
    chain.push_back(i);
    if (nodes[i].parent < 0) break;
  }
  std::reverse(chain.begin(), chain.end());

  path_nodes_.clear();
  for (size_t i = 0; i < chain.size(); ++i)
  {
    const RRTNode &rn = nodes[chain[i]];
    // goal 노드: CubicSpline3D 관례에 맞춰 k = Config::N 으로 강제
    double k_time = (i == chain.size() - 1)
        ? static_cast<double>(Config::N)
        : rn.t / Config::DT;

    const SpaceTimePoint pt(rn.x, rn.y, k_time);

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
    path_nodes_.emplace_back(id, pt, type);
  }

  std::vector<Node *> ptrs;
  ptrs.reserve(path_nodes_.size());
  for (auto &n : path_nodes_)
    ptrs.push_back(&n);

  return GeometricPath(ptrs);
}

// ── Plan() 메인 루프 ───────────────────────────────────────────────────────────

std::optional<GeometricPath>
STRRTStarPlanner::Plan(const Eigen::Vector2d &start_xy,
                        double start_theta,
                        double start_speed,
                        const Eigen::Vector2d &goal_xy,
                        const std::shared_ptr<RosTools::Spline2D> &reference_path,
                        double spline_start)
{
  if (!step_map_ || !step_map_->valid())
  {
    LOG_WARN("STRRTStarPlanner::Plan — StepMap is not valid");
    return std::nullopt;
  }

  const double t_horizon = Config::N * Config::DT;
  const double d_sg      = std::hypot(goal_xy.x() - start_xy.x(),
                                      goal_xy.y() - start_xy.y());
  const double t_min_goal = d_sg / v_max_;

  // ── StepMap AABB 미리 계산 (sample 마다 재계산 방지) ──────────────────────
  SampleContext ctx;
  ctx.goal_xy    = goal_xy;
  ctx.t_min_goal = t_min_goal;
  {
    Eigen::Vector2d c00 = step_map_->worldFromCell(0, 0);
    Eigen::Vector2d cNN = step_map_->worldFromCell(step_map_->cellsX() - 1,
                                                    step_map_->cellsY() - 1);
    ctx.x_min = std::min(c00.x(), cNN.x());
    ctx.x_max = std::max(c00.x(), cNN.x());
    ctx.y_min = std::min(c00.y(), cNN.y());
    ctx.y_max = std::max(c00.y(), cNN.y());
  }

  // ── reference path 기반 s 범위 산정 ───────────────────────────────────────
  ctx.reference_path = reference_path;
  ctx.cur_s = spline_start;
  if (reference_path)
  {
    const double s_reach = spline_start + v_max_ * t_horizon;
    const double s_lim   = reference_path->parameterLength();
    ctx.max_s = std::min(s_reach, s_lim);
  }
  else
  {
    ctx.max_s = spline_start;  // disables path-band branch in sampleState
  }

  if (t_min_goal >= t_horizon)
  {
    LOG_WARN("STRRTStarPlanner::Plan — Goal unreachable within time horizon "
             "(dist=" << d_sg << "m, t_min=" << t_min_goal << "s, horizon=" << t_horizon << "s)");
    return std::nullopt;
  }

  // ── 초기화 ────────────────────────────────────────────────────────────────
  std::vector<RRTNode> nodes;
  nodes.reserve(static_cast<size_t>(max_iter_) + 1);

  RRTNode root;
  root.x     = start_xy.x();
  root.y     = start_xy.y();
  root.theta = start_theta;
  root.t     = 0.0;
  root.v     = start_speed;
  root.w     = 0.0;
  root.cost  = 0.0;
  root.parent = -1;
  nodes.push_back(root);

  int    best_idx  = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  double t_upper   = t_horizon;

  // ── 메인 루프 ─────────────────────────────────────────────────────────────
  for (int iter = 0; iter < max_iter_; ++iter)
  {
    // 1) 샘플
    ctx.t_upper = t_upper;
    auto smp = sampleState(ctx);
    if (!smp)
      continue;

    // 2) nearest
    int i_near = -1;
    double best_d = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i)
    {
      double d = timeAwareDist(nodes[i], smp->x, smp->y, smp->t);
      if (d < best_d)
      {
        best_d = d;
        i_near = i;
      }
    }
    if (i_near < 0 || std::isinf(best_d))
      continue;

    // 3) steer
    auto st = steer(nodes[i_near], smp->x, smp->y, smp->t);
    if (!st || st->t > t_upper || st->t > t_horizon)
      continue;

    double dt_e = st->t - nodes[i_near].t;

    // 4) 충돌 검사
    if (!edgeCollisionFree(nodes[i_near], st->v, st->w, dt_e))
      continue;

    // 5) choose-parent: NEIGHBOR_RADIUS 내 후보 중 최저 cost
    double ec_near = edgeCost(dt_e, st->v, st->w);
    int    best_par      = i_near;
    double best_par_cost = nodes[i_near].cost + ec_near;
    double bv = st->v, bw = st->w;

    for (int j = 0; j < static_cast<int>(nodes.size()); ++j)
    {
      const RRTNode &nj = nodes[j];
      if (nj.t >= st->t) continue;

      double dtj = st->t - nj.t;
      double dj  = std::hypot(st->x - nj.x, st->y - nj.y);
      if (dj > v_max_ * dtj || dj > neighbor_radius_)
        continue;

      auto cand = steer(nj, st->x, st->y, st->t);
      if (!cand) continue;
      if (std::hypot(cand->x - st->x, cand->y - st->y) > match_tol_) continue;
      if (!edgeCollisionFree(nj, cand->v, cand->w, cand->t - nj.t)) continue;

      double c = nj.cost + edgeCost(cand->t - nj.t, cand->v, cand->w);
      if (c < best_par_cost)
      {
        best_par_cost = c;
        best_par      = j;
        bv = cand->v;
        bw = cand->w;
      }
    }

    // 6) 노드 추가
    RRTNode new_node;
    new_node.x      = st->x;
    new_node.y      = st->y;
    new_node.theta  = st->theta;
    new_node.t      = st->t;
    new_node.v      = bv;
    new_node.w      = bw;
    new_node.cost   = best_par_cost;
    new_node.parent = best_par;
    nodes.push_back(new_node);

    int i_new = static_cast<int>(nodes.size()) - 1;
    nodes[best_par].children.push_back(i_new);

    // 7) rewire: 미래 노드 중 비용 개선 가능한 것
    for (int j = 0; j < i_new; ++j)
    {
      RRTNode &nj = nodes[j];
      if (nj.t <= new_node.t) continue;

      double dtj = nj.t - new_node.t;
      double dj  = std::hypot(nj.x - new_node.x, nj.y - new_node.y);
      if (dj > v_max_ * dtj || dj > neighbor_radius_)
        continue;

      auto cand = steer(new_node, nj.x, nj.y, nj.t);
      if (!cand) continue;
      if (std::hypot(cand->x - nj.x, cand->y - nj.y) > match_tol_) continue;
      if (!edgeCollisionFree(new_node, cand->v, cand->w, cand->t - new_node.t)) continue;

      double c = new_node.cost + edgeCost(cand->t - new_node.t, cand->v, cand->w);
      if (c < nj.cost)
      {
        // 기존 부모에서 자식 링크 제거
        int old_par = nj.parent;
        if (old_par >= 0)
        {
          auto &ch = nodes[old_par].children;
          ch.erase(std::remove(ch.begin(), ch.end(), j), ch.end());
        }
        nj.parent = i_new;
        nj.cost   = c;
        nj.v      = cand->v;
        nj.w      = cand->w;
        nodes[i_new].children.push_back(j);
      }
    }

    // 8) goal check
    double dist_goal = std::hypot(new_node.x - goal_xy.x(), new_node.y - goal_xy.y());
    if (dist_goal < goal_radius_ && new_node.cost < best_cost)
    {
      best_cost = new_node.cost;
      best_idx  = i_new;
      t_upper   = std::min(t_upper, new_node.t);
    }
  }

  if (best_idx < 0)
  {
    LOG_WARN("STRRTStarPlanner: No path found after " << max_iter_ << " iterations");
    return std::nullopt;
  }

  return reconstructPath(nodes, best_idx);
}

}  // namespace GuidancePlanner
