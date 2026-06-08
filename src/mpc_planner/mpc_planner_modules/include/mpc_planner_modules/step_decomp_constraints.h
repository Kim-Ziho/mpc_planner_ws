/**
 * @file step_decomp_constraints.h
 * @brief Unified static + dynamic collision-avoidance via per-stage convex decomposition of the StepMap.
 *
 * Merges the roles of the dynamic-obstacle LinearizedConstraints "tube" and the static-obstacle
 * DecompConstraints into a single convex constraint set. For each horizon stage k it:
 *   1. takes the occupied cells (occupancy >= threshold) of the StepMap time layer gt=k, which fuse
 *      static (costmap) and dynamic (predicted) obstacles into one space-time grid,
 *   2. seeds a SeedDecomp at the ego-predicted position at stage k (= the injected guidance trajectory),
 *   3. inflates it against those occupied cells to obtain a convex polytope (halfspaces a1*x+a2*y<=b).
 *
 * The StepMap is reused from module_data.step_map (built by GuidanceReference) when available, and
 * rebuilt locally otherwise.
 */

#ifndef __STEP_DECOMP_CONSTRAINTS_H_
#define __STEP_DECOMP_CONSTRAINTS_H_

#include <mpc_planner_modules/controller_module.h>

#include <mpc_planner_stepmap/step_map_builder.h>
#include <mpc_planner_stepmap/step_map.h>

#include <mpc_planner_modules/heading_seed_decomp.h>

#include <decomp_util/seed_decomp.h>
#include <decomp_util/decomp_geometry/geometric_utils.h>

namespace MPCPlanner
{
  class StepDecompConstraints : public ControllerModule
  {
  public:
    StepDecompConstraints(std::shared_ptr<Solver> solver);

  public:
    void update(State &state, const RealTimeData &data, ModuleData &module_data) override;
    void setParameters(const RealTimeData &data, const ModuleData &module_data, int k) override;

    bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

    void visualize(const RealTimeData &data, const ModuleData &module_data) override;

  private:
    std::vector<std::vector<Eigen::ArrayXd>> _a1, _a2, _b; // Constraints [disc x step]
    vec_E<Polyhedron<2>> _polyhedrons;                     // Per-stage corridors (for visualization)

    vec_Vec2f _occ_pos; // Reusable buffer of occupied cell centers (per stage)

    // Fallback builder, only constructed/used when module_data does not provide a StepMap.
    std::shared_ptr<MPCPlannerStepMap::StepMapBuilder> step_map_builder_;

    double _dummy_a1{1.}, _dummy_a2{0.}, _dummy_b;

    int _n_discs{1};
    int _max_constraints;
    double _range_long;   // Forward (longitudinal) bbox half-extent, aligned with the seed heading
    double _range_lat;    // Lateral bbox half-extent
    double _window_range; // Cell-collection window radius (covers the rotated bbox): hypot(long, lat)
    double _robot_radius;

    std::shared_ptr<MPCPlannerStepMap::StepMap> getStepMap(State &state, const RealTimeData &data,
                                                           ModuleData &module_data);

    void collectOccupiedCells(const MPCPlannerStepMap::StepMap &map, int gt,
                              const Eigen::Vector2d &seed, vec_Vec2f &out) const;

    // Heading at stage k for bbox alignment: finite-difference of the ego prediction (direction of
    // travel) with fallback to the predicted psi state, then the current robot psi.
    double seedHeading(int k, double fallback_psi) const;

    void fillDummies(int k);
  };
} // namespace MPCPlanner
#endif // __STEP_DECOMP_CONSTRAINTS_H_
