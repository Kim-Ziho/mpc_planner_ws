#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace GuidancePlanner
{

// ── Space-Time k-d tree ─────────────────────────────────────────────────────
// 노드 (x, y, t) 를 3D k-d 트리(축 순서 x→y→t)에 점진적으로 삽입한다. RRT 샘플이
// 무작위라 비교 기반 삽입만으로도 평균 O(log n) 깊이의 균형 트리가 만들어진다.
// 노드 좌표는 한 번 삽입되면 불변(rewire 는 parent/cost 만 바꿈)이라 트리가 항상
// 유효하다. 두 가지 질의를 제공한다.
//   - nearestTimeAware : timeAwareDist 메트릭 최소 노드 (brute force 와 동일 결과)
//   - radiusXY         : 평면거리 ≤ r 인 노드 모두 (choose-parent/rewire 후보)
//
// time-aware reachability 의 정의(STRRTStarPlanner::timeAwareDist 와 동일):
//   feasible ⇔ dt = qt - a.t > 1e-6  그리고  d ≤ v_max·dt + 1e-6
//   비용     = dt + d / v_max
// nearestTimeAware 의 박스 하한은 위 cone 제약을 무시(하한만 약화)하므로 feasible
// 노드를 잘못 가지치기하지 않는다 → brute force 와 정확히 동일한 결과를 보장.
class SpaceTimeKDTree
{
public:
  explicit SpaceTimeKDTree(double v_max) : inv_v_max_(1.0 / std::max(1e-9, v_max)) {}

  void reserve(std::size_t n) { e_.reserve(n); }

  std::size_t size() const { return e_.size(); }

  void insert(double x, double y, double t, int node_idx)
  {
    const int new_id = static_cast<int>(e_.size());
    e_.push_back(Entry{x, y, t, node_idx, -1, -1});
    if (root_ < 0)
    {
      root_ = new_id;
      return;
    }

    int cur = root_, depth = 0;
    while (true)
    {
      const int axis = depth % 3;
      int &child = (axisVal(e_[new_id], axis) < axisVal(e_[cur], axis)) ? e_[cur].left
                                                                        : e_[cur].right;
      if (child < 0)
      {
        child = new_id;
        return;
      }
      cur = child;
      ++depth;
    }
  }

  // timeAwareDist(a) = (qt - a.t) + d/v_max,  단 qt-a.t > 1e-6 이고 d ≤ v_max·dt 일 때만.
  // STRRTStarPlanner::timeAwareDist 와 동일한 메트릭. 최소 노드 index 반환(없으면 -1).
  int nearestTimeAware(double qx, double qy, double qt, double *out_dist) const
  {
    double best = INF;
    int best_idx = -1;
    if (root_ >= 0)
    {
      double lo[3] = {-INF, -INF, -INF};
      double hi[3] = {INF, INF, INF};
      nearestRec(root_, 0, qx, qy, qt, lo, hi, best, best_idx);
    }
    if (out_dist)
      *out_dist = best;
    return best_idx;
  }

  // 평면거리 hypot(qx-x, qy-y) ≤ radius 인 노드 index 를 out 에 수집.
  void radiusXY(double qx, double qy, double radius, std::vector<int> &out) const
  {
    out.clear();
    if (root_ >= 0)
      radiusRec(root_, 0, qx, qy, radius * radius, out);
  }

private:
  struct Entry
  {
    double x, y, t;
    int node_idx;
    int left, right;
  };

  static constexpr double INF = std::numeric_limits<double>::infinity();

  static double axisVal(const Entry &e, int axis)
  {
    return axis == 0 ? e.x : (axis == 1 ? e.y : e.t);
  }

  double metric(const Entry &e, double qx, double qy, double qt) const
  {
    const double dt = qt - e.t;
    if (dt <= 1e-6)
      return INF;
    const double d = std::hypot(qx - e.x, qy - e.y);
    if (d > dt / inv_v_max_ + 1e-6)  // d > v_max·dt + 1e-6
      return INF;
    return dt + d * inv_v_max_;
  }

  // 박스 [lo,hi] 내 임의의 가능(feasible) 점에 대한 timeAwareDist 하한.
  double nearestLB(const double *lo, const double *hi, double qx, double qy, double qt) const
  {
    if (lo[2] >= qt - 1e-6)  // 박스 전체가 (약)미래 → 도달 불가
      return INF;
    double dx = 0.0, dy = 0.0;
    if (qx < lo[0]) dx = lo[0] - qx; else if (qx > hi[0]) dx = qx - hi[0];
    if (qy < lo[1]) dy = lo[1] - qy; else if (qy > hi[1]) dy = qy - hi[1];
    return std::max(0.0, qt - hi[2]) + std::hypot(dx, dy) * inv_v_max_;
  }

  void nearestRec(int cur, int depth, double qx, double qy, double qt,
                  double *lo, double *hi, double &best, int &best_idx) const
  {
    if (cur < 0 || nearestLB(lo, hi, qx, qy, qt) >= best)
      return;

    const Entry &e = e_[cur];
    const double m = metric(e, qx, qy, qt);
    if (m < best)
    {
      best = m;
      best_idx = e.node_idx;
    }

    const int axis = depth % 3;
    const double split = axisVal(e, axis);
    const double qv = axis == 0 ? qx : (axis == 1 ? qy : qt);
    const bool go_left = qv < split;
    const int near_child = go_left ? e.left : e.right;
    const int far_child  = go_left ? e.right : e.left;

    // 가까운 쪽: 해당 축의 박스 경계를 split 으로 좁혀 재귀
    {
      double &bound = go_left ? hi[axis] : lo[axis];
      const double save = bound;
      bound = split;
      nearestRec(near_child, depth + 1, qx, qy, qt, lo, hi, best, best_idx);
      bound = save;
    }
    // 먼 쪽
    {
      double &bound = go_left ? lo[axis] : hi[axis];
      const double save = bound;
      bound = split;
      nearestRec(far_child, depth + 1, qx, qy, qt, lo, hi, best, best_idx);
      bound = save;
    }
  }

  void radiusRec(int cur, int depth, double qx, double qy, double r2, std::vector<int> &out) const
  {
    if (cur < 0)
      return;
    const Entry &e = e_[cur];

    const double ddx = qx - e.x, ddy = qy - e.y;
    if (ddx * ddx + ddy * ddy <= r2)
      out.push_back(e.node_idx);

    const int axis = depth % 3;
    if (axis == 2)  // 시간축: 평면 가지치기 불가 → 양쪽 모두 방문
    {
      radiusRec(e.left, depth + 1, qx, qy, r2, out);
      radiusRec(e.right, depth + 1, qx, qy, r2, out);
      return;
    }

    const double diff = (axis == 0 ? qx : qy) - axisVal(e, axis);
    const int near_child = diff <= 0.0 ? e.left : e.right;
    const int far_child  = diff <= 0.0 ? e.right : e.left;
    radiusRec(near_child, depth + 1, qx, qy, r2, out);
    if (diff * diff <= r2)  // |diff| ≤ radius 일 때만 먼 쪽 탐색
      radiusRec(far_child, depth + 1, qx, qy, r2, out);
  }

  double inv_v_max_;
  int root_ = -1;
  std::vector<Entry> e_;
};

}  // namespace GuidancePlanner
