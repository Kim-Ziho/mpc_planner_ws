#pragma once

#include <guidance_planner/config.h>
#include <guidance_planner/types/paths.h>
#include <guidance_planner/types/node.h>
#include <guidance_planner/types/types.h>
#include <mpc_planner_stepmap/step_map.h>

#include <Eigen/Dense>
#include <cstdint>
#include <list>
#include <memory>
#include <optional>
#include <random>
#include <utility>
#include <vector>

namespace RosTools { class Spline2D; }

namespace GuidancePlanner
{

class RiskAwareSTRRTPlanner
{
public:
  void Init(Config *config);
  void SetStepMap(std::shared_ptr<MPCPlannerStepMap::StepMap> step_map);
  void Reset();

  /** @brief StepMap 기반 위험 가중 단일 최적 경로 탐색
   *  @return 성공 시 GeometricPath, 실패 시 std::nullopt */
  std::optional<GeometricPath> Plan(const Eigen::Vector2d &start_xy,
                                    double start_theta,
                                    double start_speed,
                                    const std::vector<Goal> &goals,
                                    const std::shared_ptr<RosTools::Spline2D> &reference_path = nullptr,
                                    double spline_start = 0.0);

private:
  enum class TreeType : uint8_t { START = 0, GOAL = 1 };

  struct RRTNode
  {
    int ix{0}, iy{0}, it{0};
    double x{0.0}, y{0.0}, t{0.0};
    double heading{0.0};
    double path_cost{0.0};
    int    parent{-1};
    int    root{-1};
    TreeType tree{TreeType::START};
    bool   removed{false};
    std::vector<int> children;
  };

  struct GoalSample
  {
    int ix{0}, iy{0}, it{0};
    double x{0.0}, y{0.0}, t{0.0};
    double s_progress{0.0};
    bool   valid{false};
  };

  struct EdgeCheck
  {
    bool ok{false};
    double cost{0.0};
  };

  // ── coordinate helpers ─────────────────────────────────────────────────────
  bool worldToCell(const Eigen::Vector2d &p, double t,
                   int &gx, int &gy, int &gt) const;
  void cellToWorld(int gx, int gy, int gt,
                   double &x, double &y, double &t) const;
  size_t flatIndex(int gx, int gy, int gt) const;

  // ── core stages ────────────────────────────────────────────────────────────
  std::optional<RRTNode> sampleConditionally(double t_upper) const;
  int                    nearest(const RRTNode &target, TreeType tree) const;
  std::optional<RRTNode> steer(const RRTNode &near, const RRTNode &target) const;
  EdgeCheck              edgeCheck(const RRTNode &from, const RRTNode &to) const;
  bool                   curvatureFeasible(const RRTNode &parent, const RRTNode &child) const;

  std::vector<int> collectNeighbors(const RRTNode &x_new, int k, TreeType tree) const;
  int  extend(TreeType tree_type);
  void rewireGoalForest(int idx_new, const std::vector<int> &neighbors);
  void tryConnect(int idx_new);

  // ── goal sampling ──────────────────────────────────────────────────────────
  GoalSample sampleGoalFromTube(const Eigen::Vector2d &start_xy,
                                const std::shared_ptr<RosTools::Spline2D> &ref,
                                double spline_start) const;
  GoalSample sampleGoalFromGoalsList(const Eigen::Vector2d &start_xy,
                                     const std::vector<Goal> &goals) const;
  std::vector<GoalSample> initialGoalSeeds(const Eigen::Vector2d &start_xy,
                                           const std::vector<Goal> &goals,
                                           const std::shared_ptr<RosTools::Spline2D> &ref,
                                           double spline_start);
  void addGoalRoot(const GoalSample &g);

  // ── masks ──────────────────────────────────────────────────────────────────
  void buildForwardReachableMask(const RRTNode &start);
  void buildBackwardReachableMask(const std::vector<int> &goal_roots);
  void rebuildValidIndexList();

  // ── lifecycle ──────────────────────────────────────────────────────────────
  void coldStart(const RRTNode &start_root, const std::vector<GoalSample> &goal_seeds);
  void warmStartShift(double elapsed_seconds, const RRTNode &new_start_root);
  bool canWarmStart(const Eigen::Vector2d &start_xy) const;
  void compactNodes();

  // ── reconstruction ─────────────────────────────────────────────────────────
  GeometricPath reconstructPath(int start_leaf, int goal_leaf);

  // ── DDA (templated; defined inline) ────────────────────────────────────────
  template <class F>
  void dda3D(const RRTNode &a, const RRTNode &b, F &&visit) const;

  // ── helpers ────────────────────────────────────────────────────────────────
  double timeAwareDist(const RRTNode &a, const RRTNode &b) const;
  double edgeCost(const RRTNode &a, const RRTNode &b) const;
  double riskCostAlongEdge(const RRTNode &a, const RRTNode &b, bool &collision) const;
  void   propagateCostAndRoot(int idx, double delta, int new_root);
  void   detachFromParent(int idx);
  inline double riskPhi(double p) const;

  // ── data ───────────────────────────────────────────────────────────────────
  Config *config_{nullptr};
  std::shared_ptr<MPCPlannerStepMap::StepMap> step_map_;
  std::list<Node> path_nodes_;

  std::vector<RRTNode> nodes_;
  std::vector<int>     start_roots_;
  std::vector<int>     goal_roots_;

  std::vector<uint8_t> forward_mask_;
  std::vector<uint8_t> backward_mask_;
  std::vector<int>     valid_indices_;

  int cells_x_{0}, cells_y_{0}, cells_t_{0};
  size_t cells_xy_{0}, cells_total_{0};

  double best_solution_cost_{0.0};
  std::pair<int, int> best_link_{-1, -1};

  // warm start state
  bool   has_prev_tree_{false};
  double last_plan_time_{0.0};
  Eigen::Vector2d last_start_xy_{Eigen::Vector2d::Zero()};

  // cached parameters
  int    max_iter_{0};
  double v_max_{0.0}, v_preferred_{0.0}, w_max_{0.0};
  double tau_hard_{0.0}, tau_soft_{0.0};
  double w_t_{0.0}, w_r_{0.0}, w_p_{0.0}, w_c_{0.0}, w_goal_progress_{0.0}, lambda_nn_{0.0};
  int    max_step_cells_{0};
  int    k_rrtstar_{0};
  int    initial_goal_count_{0};
  int    max_goal_count_{0};
  int    max_nodes_{0};
  double time_budget_ms_{0.0};
  double tube_width_{0.0};
  double s_min_offset_{0.0};
  double t_min_{0.0}, t_max_{0.0};
  bool   enable_warm_start_{true};
  int    cold_start_min_remaining_nodes_{50};

  mutable std::mt19937 rng_;
};

}  // namespace GuidancePlanner
