#include "mpc_planner_modules/step_decomp_constraints.h"

#include <mpc_planner_solver/mpc_planner_parameters.h>

#include <mpc_planner_util/parameters.h>

#include <ros_tools/profiling.h>
#include <ros_tools/visuals.h>
#include <ros_tools/logging.h>

#include <ros/ros.h>

#include <algorithm>
#include <cmath>

namespace MPCPlanner
{
  StepDecompConstraints::StepDecompConstraints(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::CONSTRAINT, solver, "step_decomp_constraints")
  {
    LOG_INITIALIZE("StepDecomp Constraints");

    _n_discs = 1; // Single robot-center disc (constraints evaluated at the disc position)
    _max_constraints = CONFIG["step_decomp"]["max_constraints"].as<int>();
    _robot_radius = CONFIG["robot_radius"].as<double>();

    // Heading-aligned bbox: forward (longitudinal) and lateral half-extents. Both default to the
    // symmetric "range" when the asymmetric keys are absent (backward compatible).
    const double range = CONFIG["step_decomp"]["range"].as<double>();
    _range_long = CONFIG["step_decomp"]["range_long"] ? CONFIG["step_decomp"]["range_long"].as<double>() : range;
    _range_lat = CONFIG["step_decomp"]["range_lat"] ? CONFIG["step_decomp"]["range_lat"].as<double>() : range;
    _window_range = std::hypot(_range_long, _range_lat); // covers the rotated bbox corners

    _occ_pos.reserve(2000);

    _a1.resize(_n_discs);
    _a2.resize(_n_discs);
    _b.resize(_n_discs);
    for (int d = 0; d < _n_discs; d++)
    {
      _a1[d].resize(CONFIG["N"].as<int>());
      _a2[d].resize(CONFIG["N"].as<int>());
      _b[d].resize(CONFIG["N"].as<int>());
      for (int k = 0; k < CONFIG["N"].as<int>(); k++)
      {
        _a1[d][k] = Eigen::ArrayXd(_max_constraints);
        _a2[d][k] = Eigen::ArrayXd(_max_constraints);
        _b[d][k] = Eigen::ArrayXd(_max_constraints);
      }
    }

    LOG_INITIALIZED();
  }

  std::shared_ptr<MPCPlannerStepMap::StepMap> StepDecompConstraints::getStepMap(
      State &state, const RealTimeData &data, ModuleData &module_data)
  {
    // Prefer the StepMap built upstream (e.g. by GuidanceReference) to avoid rebuilding it.
    if (module_data.step_map && module_data.step_map->valid())
      return module_data.step_map;

    // Fallback: build our own from the costmap and dynamic obstacle predictions.
    if (data.costmap == nullptr)
      return nullptr;

    if (step_map_builder_ == nullptr)
    {
      ros::NodeHandle private_nh("~");
      step_map_builder_ = std::make_shared<MPCPlannerStepMap::StepMapBuilder>(private_nh);
    }

    auto map = step_map_builder_->update(data.costmap,
                                         state.getPos(),
                                         state.get("psi"),
                                         data.dynamic_obstacles,
                                         data.robot_area,
                                         _solver->N,
                                         _solver->dt);
    return (map && map->valid()) ? map : nullptr;
  }

  void StepDecompConstraints::collectOccupiedCells(const MPCPlannerStepMap::StepMap &map, int gt,
                                                   const Eigen::Vector2d &seed, vec_Vec2f &out) const
  {
    out.clear();

    const double res = map.resolution();
    if (res <= 0.0)
      return;

    // Only iterate cells within a (range + robot_radius) window around the seed (in local frame).
    const Eigen::Vector2d local = map.localFromWorld(seed);
    const double cx = (local.x() + map.halfLength()) / res;
    const double cy = (local.y() + map.halfWidth()) / res;
    const int window = static_cast<int>(std::ceil((_window_range + _robot_radius) / res)) + 1;

    const int gx0 = std::max(0, static_cast<int>(std::floor(cx)) - window);
    const int gx1 = std::min(map.cellsX() - 1, static_cast<int>(std::floor(cx)) + window);
    const int gy0 = std::max(0, static_cast<int>(std::floor(cy)) - window);
    const int gy1 = std::min(map.cellsY() - 1, static_cast<int>(std::floor(cy)) + window);

    const double threshold = map.occupancyThreshold();
    for (int gx = gx0; gx <= gx1; gx++)
    {
      for (int gy = gy0; gy <= gy1; gy++)
      {
        if (map.cellCost(gx, gy, gt) >= threshold)
        {
          const Eigen::Vector2d w = map.worldFromCell(gx, gy);
          out.emplace_back(w.x(), w.y());
        }
      }
    }
  }

  double StepDecompConstraints::seedHeading(int k, double fallback_psi) const
  {
    // Prefer the direction of travel from the ego prediction (the guidance trajectory): a forward
    // difference, else a backward difference. Fall back to the predicted psi state, then to the
    // robot's current psi when the trajectory is degenerate or the warmstart is empty.
    const double xk = _solver->getEgoPrediction(k, "x");
    const double yk = _solver->getEgoPrediction(k, "y");

    if (k + 1 < _solver->N)
    {
      const double dx = _solver->getEgoPrediction(k + 1, "x") - xk;
      const double dy = _solver->getEgoPrediction(k + 1, "y") - yk;
      if (std::hypot(dx, dy) > 1e-3)
        return std::atan2(dy, dx);
    }
    if (k - 1 >= 0)
    {
      const double dx = xk - _solver->getEgoPrediction(k - 1, "x");
      const double dy = yk - _solver->getEgoPrediction(k - 1, "y");
      if (std::hypot(dx, dy) > 1e-3)
        return std::atan2(dy, dx);
    }

    const double psi = _solver->getEgoPrediction(k, "psi");
    if (psi == psi) // not nan
      return psi;
    return fallback_psi;
  }

  void StepDecompConstraints::fillDummies(int k)
  {
    for (int i = 0; i < _max_constraints; i++)
    {
      _a1[0][k](i) = _dummy_a1;
      _a2[0][k](i) = _dummy_a2;
      _b[0][k](i) = _dummy_b;
    }
  }

  void StepDecompConstraints::update(State &state, const RealTimeData &data, ModuleData &module_data)
  {
    PROFILE_SCOPE("StepDecompConstraints::Update");
    LOG_MARK("StepDecompConstraints::update");

    _dummy_b = state.get("x") + 100.;

    _polyhedrons.clear();
    _polyhedrons.resize(_solver->N);

    auto map = getStepMap(state, data, module_data);

    // k = 0 is the current state: dummy (always-satisfied) constraints.
    fillDummies(0);

    if (map == nullptr)
    {
      // No collision model available: leave the remaining stages unconstrained (dummies).
      for (int k = 1; k < _solver->N; k++)
        fillDummies(k);
      return;
    }

    const double robot_psi = state.get("psi");

    int max_decomp_constraints = 0;

    for (int k = 1; k < _solver->N; k++)
    {
      // Seed = ego-predicted position at stage k (= the injected guidance trajectory).
      const Eigen::Vector2d seed_pos(_solver->getEgoPrediction(k, "x"), _solver->getEgoPrediction(k, "y"));
      const Vec2f seed(seed_pos.x(), seed_pos.y());

      // Occupied cells of the matching time layer (static + dynamic fused).
      const int gt = std::min(k, map->cellsT() - 1);
      collectOccupiedCells(*map, gt, seed_pos, _occ_pos);

      // Inflate a convex region around the seed avoiding the occupied cells. The virtual bbox is
      // aligned with the direction of travel (heading), giving a corridor long forward / narrow
      // laterally instead of an axis-parallel square.
      const double yaw = seedHeading(k, robot_psi);
      HeadingSeedDecomp2D seed_decomp(seed, yaw);
      seed_decomp.set_local_bbox(Vec2f(_range_long, _range_lat));
      seed_decomp.set_obs(_occ_pos);
      seed_decomp.dilate(_robot_radius);

      const Polyhedron<2> poly = seed_decomp.get_polyhedron();
      _polyhedrons[k] = poly;

      const LinearConstraint<2> lc(seed, poly.hyperplanes());
      max_decomp_constraints = std::max(max_decomp_constraints, (int)lc.A_.rows());

      int i = 0;
      for (; i < std::min((int)lc.A_.rows(), _max_constraints); i++)
      {
        if (lc.A_.row(i).norm() < 1e-3 || lc.A_(i, 0) != lc.A_(i, 0)) // zero or nan
          break;

        _a1[0][k](i) = lc.A_.row(i)[0];
        _a2[0][k](i) = lc.A_.row(i)[1];
        _b[0][k](i) = lc.b_(i);
      }
      for (; i < _max_constraints; i++)
      {
        _a1[0][k](i) = _dummy_a1;
        _a2[0][k](i) = _dummy_a2;
        _b[0][k](i) = _dummy_b;
      }
    }

    if (max_decomp_constraints > _max_constraints)
      LOG_WARN("Maximum number of StepDecomp constraints exceeds specification: "
               << max_decomp_constraints << " > " << _max_constraints);

    LOG_MARK("StepDecompConstraints::update done");
  }

  void StepDecompConstraints::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
  {
    (void)module_data;

    if (k == 0) // Dummies
    {
      for (int d = 0; d < _n_discs; d++)
      {
        setSolverParameterEgoDiscOffset(k, _solver->_params, data.robot_area[d].offset, d);

        int constraint_counter = 0;
        for (int i = 0; i < _max_constraints; i++)
        {
          setSolverParameterStepDecompA1(k, _solver->_params, _dummy_a1, constraint_counter);
          setSolverParameterStepDecompA2(k, _solver->_params, _dummy_a2, constraint_counter);
          setSolverParameterStepDecompB(k, _solver->_params, _dummy_b, constraint_counter);
          constraint_counter++;
        }
      }
      return;
    }

    int constraint_counter = 0;
    for (int d = 0; d < _n_discs; d++)
    {
      setSolverParameterEgoDiscOffset(k, _solver->_params, data.robot_area[d].offset, d);

      for (int i = 0; i < _max_constraints; i++)
      {
        setSolverParameterStepDecompA1(k, _solver->_params, _a1[d][k](i), constraint_counter);
        setSolverParameterStepDecompA2(k, _solver->_params, _a2[d][k](i), constraint_counter);
        setSolverParameterStepDecompB(k, _solver->_params, _b[d][k](i), constraint_counter);
        constraint_counter++;
      }
    }
  }

  bool StepDecompConstraints::isDataReady(const RealTimeData &data, std::string &missing_data)
  {
    // A StepMap can be supplied via module_data (checked at runtime); otherwise we build one from the
    // costmap, so the costmap is the hard requirement here.
    if (data.costmap == nullptr)
    {
      missing_data += "Costmap ";
      return false;
    }
    return true;
  }

  void StepDecompConstraints::visualize(const RealTimeData &data, const ModuleData &module_data)
  {
    (void)data;
    (void)module_data;
    PROFILE_FUNCTION();

    if (_polyhedrons.empty())
      return;

    auto &publisher = VISUALS.getPublisher("free_space");
    auto &polyline = publisher.getNewLine();
    polyline.setScale(0.1, 0.1);

    const int draw_every = CONFIG["visualization"]["draw_every"].as<int>();
    for (int k = 1; k < _solver->N; k += std::max(1, draw_every))
    {
      if (k >= (int)_polyhedrons.size())
        break;

      const auto &poly = _polyhedrons[k];
      polyline.setColorInt(k, _solver->N);

      const auto vertices = cal_vertices(poly);
      if (vertices.size() < 2)
        continue;

      for (size_t i = 1; i < vertices.size(); i++)
      {
        polyline.addLine(Eigen::Vector3d(vertices[i - 1](0), vertices[i - 1](1), 0),
                         Eigen::Vector3d(vertices[i](0), vertices[i](1), 0));
      }
      polyline.addLine(Eigen::Vector3d(vertices.back()(0), vertices.back()(1), 0),
                       Eigen::Vector3d(vertices[0](0), vertices[0](1), 0));
    }

    publisher.publish();
  }

} // namespace MPCPlanner
