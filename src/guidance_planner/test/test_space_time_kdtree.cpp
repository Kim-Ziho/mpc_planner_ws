// Differential test for SpaceTimeKDTree (st_rrt_star_planner.cpp 가속 구조).
//
// 목적: k-d 트리가 brute-force 와 "정확히 동일한" time-aware reachability 결과를
// 내는지 검증한다. 세 가지 질의를 production 코드와 같은 조건으로 비교한다.
//   1) nearestTimeAware  ↔  brute-force min timeAwareDist (cone + future 필터)
//   2) choose-parent 후보 집합 (past + cone + radius)
//   3) rewire 후보 집합        (future + cone + radius)
//
// 빌드/실행:
//   g++ -std=c++17 -O2 -I <pkg>/include test_space_time_kdtree.cpp -o /tmp/kdtest && /tmp/kdtest

#include <guidance_planner/space_time_kdtree.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>

using GuidancePlanner::SpaceTimeKDTree;

namespace
{

struct Pt { double x, y, t; };

constexpr double INF = std::numeric_limits<double>::infinity();

// production STRRTStarPlanner::timeAwareDist 와 동일한 메트릭/feasibility.
double timeAwareDist(const Pt &a, double qx, double qy, double qt, double v_max)
{
  const double dt = qt - a.t;
  if (dt <= 1e-6)
    return INF;
  const double d = std::hypot(qx - a.x, qy - a.y);
  if (d > v_max * dt + 1e-6)
    return INF;
  return dt + d / v_max;
}

// nearest: brute-force 최소 timeAwareDist
int bruteNearest(const std::vector<Pt> &pts, double qx, double qy, double qt,
                 double v_max, double &best)
{
  best = INF;
  int idx = -1;
  for (int i = 0; i < (int)pts.size(); ++i)
  {
    const double d = timeAwareDist(pts[i], qx, qy, qt, v_max);
    if (d < best) { best = d; idx = i; }
  }
  return idx;
}

// choose-parent 후보: nj.t < qt (past) ∧ dj ≤ v_max·dt ∧ dj ≤ radius
std::vector<int> bruteChooseParent(const std::vector<Pt> &pts, double qx, double qy,
                                   double qt, double v_max, double radius)
{
  std::vector<int> out;
  for (int j = 0; j < (int)pts.size(); ++j)
  {
    if (pts[j].t >= qt) continue;
    const double dtj = qt - pts[j].t;
    const double dj  = std::hypot(qx - pts[j].x, qy - pts[j].y);
    if (dj > v_max * dtj || dj > radius) continue;
    out.push_back(j);
  }
  std::sort(out.begin(), out.end());
  return out;
}

// rewire 후보: nj.t > qt (future) ∧ dj ≤ v_max·dt ∧ dj ≤ radius
std::vector<int> bruteRewire(const std::vector<Pt> &pts, double qx, double qy,
                             double qt, double v_max, double radius)
{
  std::vector<int> out;
  for (int j = 0; j < (int)pts.size(); ++j)
  {
    if (pts[j].t <= qt) continue;
    const double dtj = pts[j].t - qt;
    const double dj  = std::hypot(qx - pts[j].x, qy - pts[j].y);
    if (dj > v_max * dtj || dj > radius) continue;
    out.push_back(j);
  }
  std::sort(out.begin(), out.end());
  return out;
}

// 트리 radiusXY 결과에 production 필터를 적용해 choose-parent/rewire 후보를 만든다.
std::vector<int> treeFilter(const SpaceTimeKDTree &kd, const std::vector<Pt> &pts,
                            double qx, double qy, double qt, double v_max,
                            double radius, bool future)
{
  std::vector<int> raw;
  kd.radiusXY(qx, qy, radius, raw);
  std::vector<int> out;
  for (int j : raw)
  {
    const Pt &p = pts[j];
    if (future) { if (p.t <= qt) continue; }
    else        { if (p.t >= qt) continue; }
    const double dtj = future ? (p.t - qt) : (qt - p.t);
    const double dj  = std::hypot(qx - p.x, qy - p.y);
    if (dj > v_max * dtj || dj > radius) continue;
    out.push_back(j);
  }
  std::sort(out.begin(), out.end());
  return out;
}

int g_fail = 0;
long g_checks = 0;

void check(bool ok, const char *what)
{
  ++g_checks;
  if (!ok) { ++g_fail; std::printf("  [FAIL] %s\n", what); }
}

// 한 데이터셋 + 한 질의에 대해 세 질의 모두 brute-force 와 일치하는지 검사
void compareAll(const SpaceTimeKDTree &kd, const std::vector<Pt> &pts,
                double qx, double qy, double qt, double v_max, double radius)
{
  // 1) nearest
  double bd, td;
  int bi = bruteNearest(pts, qx, qy, qt, v_max, bd);
  int ti = kd.nearestTimeAware(qx, qy, qt, &td);

  if (bi < 0)
  {
    check(ti < 0 && std::isinf(td), "nearest: both report no-feasible");
  }
  else
  {
    // 거리 값은 동일 산술이므로 사실상 비트-동일해야 한다(타이 제외).
    check(std::isfinite(td) && std::fabs(td - bd) <= 1e-12, "nearest: min distance matches");
    // 인덱스가 다르면 동률(같은 거리)이어야 한다.
    if (ti != bi && ti >= 0)
    {
      const double di = timeAwareDist(pts[ti], qx, qy, qt, v_max);
      check(std::fabs(di - bd) <= 1e-9, "nearest: differing index is a true tie");
    }
  }

  // 2) choose-parent 집합
  check(treeFilter(kd, pts, qx, qy, qt, v_max, radius, /*future=*/false) ==
            bruteChooseParent(pts, qx, qy, qt, v_max, radius),
        "choose-parent candidate set matches");

  // 3) rewire 집합
  check(treeFilter(kd, pts, qx, qy, qt, v_max, radius, /*future=*/true) ==
            bruteRewire(pts, qx, qy, qt, v_max, radius),
        "rewire candidate set matches");
}

}  // namespace

int main()
{
  std::mt19937 rng(12345);

  // ── 1) 무작위 대규모 차등 테스트 ───────────────────────────────────────────
  {
    std::uniform_real_distribution<double> xy(-10.0, 10.0);
    std::uniform_real_distribution<double> tt(0.0, 8.0);
    std::uniform_int_distribution<int>    nd(1, 400);

    for (int trial = 0; trial < 300; ++trial)
    {
      const double v_max = std::uniform_real_distribution<double>(0.5, 3.0)(rng);
      const double radius = std::uniform_real_distribution<double>(0.5, 12.0)(rng);
      const int N = nd(rng);

      std::vector<Pt> pts;
      pts.reserve(N);
      SpaceTimeKDTree kd(v_max);
      kd.reserve(N);
      for (int i = 0; i < N; ++i)
      {
        Pt p{xy(rng), xy(rng), tt(rng)};
        pts.push_back(p);
        kd.insert(p.x, p.y, p.t, i);
      }
      // 데이터셋당 여러 무작위 질의
      for (int q = 0; q < 20; ++q)
        compareAll(kd, pts, xy(rng), xy(rng), tt(rng), v_max, radius);
    }
  }

  // ── 2) reachability 경계 케이스 (cone) ─────────────────────────────────────
  {
    const double v_max = 2.0;
    std::vector<Pt> pts;
    SpaceTimeKDTree kd(v_max);
    // 공간적으로는 가깝지만 시간적으로 너무 가까워 도달 불가(cone 위반)한 노드
    pts = {
        {0.0, 0.0, 0.0},   // 0: 멀리 과거, 도달 가능
        {1.0, 0.0, 1.99},  // 1: query(t=2)에서 dt=0.01, v_max*dt=0.02 < d=1 → 도달 불가
        {0.1, 0.0, 1.0},   // 2: dt=1, d=0.1 ≤ 2 → 도달 가능
        {5.0, 5.0, 0.5},   // 3: dt=1.5, d≈7.07 > 3 → 도달 불가
    };
    for (int i = 0; i < (int)pts.size(); ++i)
      kd.insert(pts[i].x, pts[i].y, pts[i].t, i);

    // query (0,0,2): 도달 가능한 것 중 최소는 노드 2 여야 한다.
    double bd, td;
    int bi = bruteNearest(pts, 0.0, 0.0, 2.0, v_max, bd);
    int ti = kd.nearestTimeAware(0.0, 0.0, 2.0, &td);
    check(bi == 2, "cone case: brute nearest is the reachable node #2");
    check(ti == bi && std::fabs(td - bd) <= 1e-12, "cone case: tree nearest matches");
    compareAll(kd, pts, 0.0, 0.0, 2.0, v_max, /*radius=*/3.0);
    // 큰 반경이라도 cone 위반 노드(1,3)는 choose-parent 후보에서 제외돼야 한다.
    auto cp = treeFilter(kd, pts, 0.0, 0.0, 2.0, v_max, 100.0, false);
    check(std::find(cp.begin(), cp.end(), 1) == cp.end(), "cone case: node #1 excluded (unreachable)");
    check(std::find(cp.begin(), cp.end(), 3) == cp.end(), "cone case: node #3 excluded (unreachable)");
  }

  // ── 3) 미래/과거 방향성 + t=0 질의(과거 없음) ──────────────────────────────
  {
    const double v_max = 1.5;
    std::vector<Pt> pts = {{0, 0, 0.0}, {0.5, 0, 1.0}, {1.0, 0, 2.0}, {-0.5, 0, 1.0}};
    SpaceTimeKDTree kd(v_max);
    for (int i = 0; i < (int)pts.size(); ++i)
      kd.insert(pts[i].x, pts[i].y, pts[i].t, i);

    // query t=0: 과거 노드 없음 → nearest 없음, choose-parent 비어야 함
    double td;
    int ti = kd.nearestTimeAware(0.0, 0.0, 0.0, &td);
    check(ti < 0 && std::isinf(td), "t=0 query: no feasible nearest");
    check(treeFilter(kd, pts, 0.0, 0.0, 0.0, v_max, 100.0, false).empty(),
          "t=0 query: choose-parent set empty");
    // 동일 좌표/다른 시간, 여러 질의 비교
    for (double qt : {0.5, 1.0, 1.5, 2.0, 3.0})
      for (double qx : {-1.0, 0.0, 0.7})
        compareAll(kd, pts, qx, 0.0, qt, v_max, 5.0);
  }

  // ── 4) 단일 노드 / 빈 반경 ─────────────────────────────────────────────────
  {
    SpaceTimeKDTree kd(2.0);
    kd.insert(0, 0, 0.0, 0);
    std::vector<Pt> pts = {{0, 0, 0.0}};
    compareAll(kd, pts, 0.0, 0.0, 1.0, 2.0, 0.0);   // radius 0
    compareAll(kd, pts, 3.0, 0.0, 1.0, 2.0, 10.0);  // 도달 불가(d=3 > 2)
  }

  std::printf("\n%ld checks, %d failures\n", g_checks, g_fail);
  if (g_fail == 0)
    std::printf("ALL PASS: k-d tree time-aware reachability matches brute force.\n");
  return g_fail == 0 ? 0 : 1;
}
