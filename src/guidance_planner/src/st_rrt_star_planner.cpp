#include <guidance_planner/st_rrt_star_planner.h>
#include <guidance_planner/path_corridor.h>
#include <guidance_planner/space_time_kdtree.h>
#include <guidance_planner/types/connection.h>
#include <guidance_planner/types/space_time_point.h>

#include <ros_tools/logging.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace GuidancePlanner
{

  void STRRTStarPlanner::Init(Config *config)
  {
    config_ = config;
    max_iter_ = config->strrt_max_iter_;
    steer_dt_min_ = config->strrt_steer_dt_min_;
    steer_dt_max_ = config->strrt_steer_dt_max_;
    neighbor_radius_ = config->strrt_neighbor_radius_;
    match_tol_ = config->strrt_match_tol_;
    goal_radius_ = config->strrt_goal_radius_;
    goal_bias_ = config->strrt_goal_bias_;
    w_time_ = config->strrt_w_time_;
    w_ctrl_ = config->strrt_w_ctrl_;
    check_dt_ = config->strrt_check_dt_;
    v_max_ = config->max_velocity_;
    w_max_ = config->hastar_w_max_;
    path_lat_half_width_ = config->strrt_path_lat_half_width_;
    greedy_goal_connect_ = config->strrt_greedy_goal_connect_;
    corridor_w_base_ = config->strrt_corridor_w_base_;
    corridor_p_explore_ = config->strrt_corridor_p_explore_;
    corridor_dt_win_minus_ = config->strrt_corridor_dt_win_minus_;
    corridor_dt_win_plus_ = config->strrt_corridor_dt_win_plus_;

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
    double d = std::hypot(dx, dy);
    if (d < 1e-6)
      return std::nullopt;

    double psi = std::atan2(dy, dx);
    double dpsi = std::atan2(std::sin(psi - from.theta), std::cos(psi - from.theta));

    double w = std::max(-w_max_, std::min(w_max_, dpsi / dt));
    double v = std::max(0.0, std::min(v_max_, d / dt));

    double x_new = from.x;
    double y_new = from.y;
    double th = from.theta;
    unicycleStep(x_new, y_new, th, v, w, dt);

    SteerResult r;
    r.x = x_new;
    r.y = y_new;
    r.theta = th;
    r.t = from.t + dt;
    r.v = v;
    r.w = w;
    return r;
  }

  // ── Edge 충돌 검사 ────────────────────────────────────────────────────────────

  bool STRRTStarPlanner::edgeCollisionFree(const RRTNode &from,
                                           double v, double w, double dt) const
  {
    if (!step_map_ || !step_map_->valid())
      return true;

    int n_steps = std::max(2, static_cast<int>(dt / check_dt_));
    // k=0(시작점 `from`) 검사는 건너뛴다. `from` 은 항상 이미 검증된 노드(부모
    // 엣지의 종단점) 또는 루트(로봇 현재 위치)이므로 중복 검사다. 특히 StepMap 은
    // forward_offset 때문에 로봇이 격자 뒤쪽 경계에 위치하는데, 부동소수 오차로
    // 루트가 격자 밖으로 떨어지면 StepMap 이 격자 밖을 '점유'로 간주하여 k=0 에서
    // 루트의 모든 엣지가 거부되고 계획이 실패한다(sample_accept_rate=0). k=1 부터
    // 검사하면 이 하드 실패가 사라지고, 루트를 옮기지 않으므로 뒤/옆으로 가는
    // 엣지는 여전히 k>=1 스윕 점에서 거부되어 기존 탐색 특성이 보존된다.
    for (int k = 1; k <= n_steps; ++k)
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
    const double r = uni01(rng_);

    // ── (A) goal-bias 모드: goal 근방, 단 AABB 안으로 clamp ────────────────────
    if (r < goal_bias_)
    {
      if (ctx.t_min_goal >= ctx.t_upper)
        return std::nullopt;
      std::uniform_real_distribution<double> gt_dist(ctx.t_min_goal, ctx.t_upper);
      std::uniform_real_distribution<double> xy_dist(-0.3, 0.3);
      double x = std::min(ctx.x_max, std::max(ctx.x_min, ctx.goal_xy.x() + xy_dist(rng_)));
      double y = std::min(ctx.y_max, std::max(ctx.y_min, ctx.goal_xy.y() + xy_dist(rng_)));
      return Sample{x, y, gt_dist(rng_)};
    }

    // ── (B) 전역 exploration / corridor 폴백: AABB 전체 균일 샘플 ───────────────
    //   corridor 가 없거나(폴백) exploration 비율에 당첨되면 균일 샘플로 완전성·
    //   risk-escape 를 보존한다.
    const bool use_corridor = (ctx.corridor != nullptr) && ctx.corridor->valid();
    if (!use_corridor || r < goal_bias_ + corridor_p_explore_)
    {
      std::uniform_real_distribution<double> xd(ctx.x_min, ctx.x_max);
      std::uniform_real_distribution<double> yd(ctx.y_min, ctx.y_max);
      double t_lower = steer_dt_min_;
      if (t_lower >= ctx.t_upper)
        return std::nullopt;
      std::uniform_real_distribution<double> td(t_lower, ctx.t_upper);
      return Sample{xd(rng_), yd(rng_), td(rng_)};
    }

    // ── (C) corridor 시공간 튜브 샘플 ──────────────────────────────────────────
    //   u ∈ [0, L], position = corridor(u) + lat·n(u),  lat ∈ [-W, W]
    //   시각은 corridor 자신의 시각 t_p(u) 근방 창에서 추출 (시공간 일관 샘플).
    //   AABB 밖 또는 시각 창이 t_upper 를 넘으면 reject 후 재시도.
    const PathCorridor &cor = *ctx.corridor;
    std::uniform_real_distribution<double> sd(0.0, cor.length());
    std::uniform_real_distribution<double> ld(-corridor_w_base_, corridor_w_base_);

    for (int retry = 0; retry < 16; ++retry)
    {
      const double u = sd(rng_);
      const Eigen::Vector2d p = cor.point(u);
      const Eigen::Vector2d n = cor.normal(u);
      const Eigen::Vector2d xy = p + ld(rng_) * n;

      if (xy.x() < ctx.x_min || xy.x() > ctx.x_max ||
          xy.y() < ctx.y_min || xy.y() > ctx.y_max)
        continue; // AABB 밖

      const double tp = cor.time(u);
      // corridor 시각 근방 창. 미래측을 넓게(감속 여유) — risk 감속 시 같은 지점에
      // corridor 보다 늦게 도달하는 것을 허용한다.
      const double t_lo = std::max(steer_dt_min_, tp - corridor_dt_win_minus_);
      const double t_hi = std::min(ctx.t_upper, tp + corridor_dt_win_plus_);
      if (t_lo >= t_hi)
        continue;

      std::uniform_real_distribution<double> td(t_lo, t_hi);
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
      if (nodes[i].parent < 0)
        break;
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
        id = -1;
      }
      else if (i == chain.size() - 1)
      {
        type = NodeType::GOAL;
        id = -2;
      }
      else
      {
        type = NodeType::CONNECTOR;
        id = static_cast<int>(i);
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
                         const PathCorridor *corridor)
  {
    if (!step_map_ || !step_map_->valid())
    {
      LOG_WARN("STRRTStarPlanner::Plan — StepMap is not valid");
      return std::nullopt;
    }

    const double t_horizon = Config::N * Config::DT;
    const double d_sg = std::hypot(goal_xy.x() - start_xy.x(),
                                   goal_xy.y() - start_xy.y());
    const double t_min_goal = d_sg / v_max_;

    // ── StepMap AABB 미리 계산 (sample 마다 재계산 방지) ──────────────────────
    SampleContext ctx;
    ctx.goal_xy = goal_xy;
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

    // ── corridor 주입 (시공간 튜브 샘플링 가이드) ─────────────────────────────
    ctx.corridor = (corridor && corridor->valid()) ? corridor : nullptr;

    if (t_min_goal >= t_horizon)
    {
      LOG_WARN("STRRTStarPlanner::Plan — Goal unreachable within time horizon "
               "(dist="
               << d_sg << "m, t_min=" << t_min_goal << "s, horizon=" << t_horizon << "s)");
      return std::nullopt;
    }

    // ── 초기화 ────────────────────────────────────────────────────────────────
    // greedy goal-connect 가 max_iter_ 를 넘는 추가 노드를 생성할 수 있으므로
    // reallocation 빈도를 줄이도록 capacity 를 넉넉히 잡는다.
    const size_t node_cap = static_cast<size_t>(max_iter_) * 2 + 1;

    std::vector<RRTNode> nodes;
    nodes.reserve(node_cap);

    RRTNode root;
    root.x = start_xy.x();
    root.y = start_xy.y();
    root.theta = start_theta;
    root.t = 0.0;
    root.v = start_speed;
    root.w = 0.0;
    root.cost = 0.0;
    root.parent = -1;
    nodes.push_back(root);

    // ── k-d 트리: nearest / radius 질의 가속 (nodes 와 동기화 유지) ────────────
    SpaceTimeKDTree kd(v_max_);
    kd.reserve(node_cap);
    kd.insert(root.x, root.y, root.t, 0);
    std::vector<int> nbr; // radius 질의 결과 재사용 버퍼

    int best_idx = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    double t_upper = t_horizon;

    // ── 메인 루프 ─────────────────────────────────────────────────────────────
    for (int iter = 0; iter < max_iter_; ++iter)
    {
      // 1) 샘플
      ctx.t_upper = t_upper;
      auto smp = sampleState(ctx);
      if (!smp)
        continue;

      // 2) nearest (k-d 트리, time-aware 메트릭)
      double best_d = std::numeric_limits<double>::infinity();
      int i_near = kd.nearestTimeAware(smp->x, smp->y, smp->t, &best_d);
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
      int best_par = i_near;
      double best_par_cost = nodes[i_near].cost + ec_near;
      double bv = st->v, bw = st->w;

      kd.radiusXY(st->x, st->y, neighbor_radius_, nbr);
      for (int j : nbr)
      {
        const RRTNode &nj = nodes[j];
        if (nj.t >= st->t)
          continue;

        double dtj = st->t - nj.t;
        double dj = std::hypot(st->x - nj.x, st->y - nj.y);
        if (dj > v_max_ * dtj || dj > neighbor_radius_)
          continue;

        auto cand = steer(nj, st->x, st->y, st->t);
        if (!cand)
          continue;
        if (std::hypot(cand->x - st->x, cand->y - st->y) > match_tol_)
          continue;
        if (!edgeCollisionFree(nj, cand->v, cand->w, cand->t - nj.t))
          continue;

        double c = nj.cost + edgeCost(cand->t - nj.t, cand->v, cand->w);
        if (c < best_par_cost)
        {
          best_par_cost = c;
          best_par = j;
          bv = cand->v;
          bw = cand->w;
        }
      }

      // 6) 노드 추가
      RRTNode new_node;
      new_node.x = st->x;
      new_node.y = st->y;
      new_node.theta = st->theta;
      new_node.t = st->t;
      new_node.v = bv;
      new_node.w = bw;
      new_node.cost = best_par_cost;
      new_node.parent = best_par;
      nodes.push_back(new_node);

      int i_new = static_cast<int>(nodes.size()) - 1;
      nodes[best_par].children.push_back(i_new);
      kd.insert(new_node.x, new_node.y, new_node.t, i_new);

      // 7) rewire: 미래 노드 중 비용 개선 가능한 것
      kd.radiusXY(new_node.x, new_node.y, neighbor_radius_, nbr);
      for (int j : nbr)
      {
        if (j == i_new)
          continue; // 자기 자신 제외
        RRTNode &nj = nodes[j];
        if (nj.t <= new_node.t)
          continue;

        double dtj = nj.t - new_node.t;
        double dj = std::hypot(nj.x - new_node.x, nj.y - new_node.y);
        if (dj > v_max_ * dtj || dj > neighbor_radius_)
          continue;

        auto cand = steer(new_node, nj.x, nj.y, nj.t);
        if (!cand)
          continue;
        if (std::hypot(cand->x - nj.x, cand->y - nj.y) > match_tol_)
          continue;
        if (!edgeCollisionFree(new_node, cand->v, cand->w, cand->t - new_node.t))
          continue;

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
          nj.cost = c;
          nj.v = cand->v;
          nj.w = cand->w;
          nodes[i_new].children.push_back(j);
        }
      }

      // 8) greedy goal-connect: RRT-Connect 식으로 goal 방향 반복 extend
      if (greedy_goal_connect_)
      {
        constexpr int kMaxConnectSteps = 8; // t_horizon/steer_dt_min 자연 상한의 안전 캡
        int cur = i_new;
        for (int step = 0; step < kMaxConnectSteps; ++step)
        {
          const double cx = nodes[cur].x, cy = nodes[cur].y, ct = nodes[cur].t;
          const double cc = nodes[cur].cost;
          const double d_ng = std::hypot(goal_xy.x() - cx, goal_xy.y() - cy);

          if (d_ng < goal_radius_) // 이미 goal 도달
          {
            if (cc < best_cost)
            {
              best_cost = cc;
              best_idx = cur;
              t_upper = std::min(t_upper, ct);
            }
            break;
          }

          const double dt_step =
              std::max(steer_dt_min_, std::min(steer_dt_max_, d_ng / v_max_));
          const double t_next = ct + dt_step;
          if (t_next > t_upper || t_next > t_horizon)
            break; // 시간 초과 → 폴백

          auto sg = steer(nodes[cur], goal_xy.x(), goal_xy.y(), t_next);
          if (!sg)
            break;
          const double dt_e2 = sg->t - ct;
          if (!edgeCollisionFree(nodes[cur], sg->v, sg->w, dt_e2))
            break; // 충돌 → 폴백

          RRTNode gn;
          gn.x = sg->x;
          gn.y = sg->y;
          gn.theta = sg->theta;
          gn.t = sg->t;
          gn.v = sg->v;
          gn.w = sg->w;
          gn.cost = cc + edgeCost(dt_e2, sg->v, sg->w);
          gn.parent = cur;
          nodes.push_back(gn); // 이후 cur 참조 무효 — 인덱스로만 접근
          const int i_gn = static_cast<int>(nodes.size()) - 1;
          nodes[cur].children.push_back(i_gn);
          kd.insert(gn.x, gn.y, gn.t, i_gn);
          cur = i_gn; // 다음 step 에서 goal 도달 여부 재검사
        }
      }
      else
      {
        // (기존 동작) 새 노드가 우연히 goal_radius 안에 떨어졌는지만 검사
        double dist_goal = std::hypot(new_node.x - goal_xy.x(), new_node.y - goal_xy.y());
        if (dist_goal < goal_radius_ && new_node.cost < best_cost)
        {
          best_cost = new_node.cost;
          best_idx = i_new;
          t_upper = std::min(t_upper, new_node.t);
        }
      }
    }

    const double sample_accept_rate =
        max_iter_ > 0 ? static_cast<double>(nodes.size() - 1) / max_iter_ : 0.0;
    accept_rate_sum_ += sample_accept_rate;
    ++plan_call_count_;
    const double avg_accept_rate = accept_rate_sum_ / plan_call_count_;
    LOG_INFO("STRRT: final nodes.size() = " << nodes.size()
                                            << ", iterations = " << max_iter_
                                            << ", sample_accept_rate = " << sample_accept_rate
                                            << ", avg_accept_rate = " << avg_accept_rate
                                            << " (over " << plan_call_count_ << " plans)");

    if (best_idx < 0)
      ++plan_fail_count_;
    const double fail_rate = static_cast<double>(plan_fail_count_) / plan_call_count_;
    LOG_INFO("STRRT: plan failures = " << plan_fail_count_ << "/" << plan_call_count_
                                       << " (fail_rate = " << fail_rate << ")");

    if (best_idx < 0)
    {
      LOG_WARN("STRRTStarPlanner: No path found after " << max_iter_ << " iterations");
      return std::nullopt;
    }

    return reconstructPath(nodes, best_idx);
  }

} // namespace GuidancePlanner
