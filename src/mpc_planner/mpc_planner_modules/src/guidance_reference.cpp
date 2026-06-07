#include "mpc_planner_modules/guidance_reference.h"

#include <mpc_planner_util/parameters.h>

#include <guidance_planner/global_guidance.h>
#include <guidance_planner/config.h>

#include <ros_tools/visuals.h>
#include <ros_tools/profiling.h>
#include <ros_tools/data_saver.h>
#include <ros_tools/math.h>

#include <ros/ros.h>
#include <algorithm>

namespace MPCPlanner
{
    GuidanceReference::GuidanceReference(std::shared_ptr<Solver> solver)
        : ControllerModule(ModuleType::CONSTRAINT, solver, "guidance_reference")
    {
        LOG_INITIALIZE("Guidance Reference (G-MPCC)");

        global_guidance_ = std::make_shared<GuidancePlanner::GlobalGuidance>();
        GuidancePlanner::Config::debug_visuals_ = CONFIG["debug_visuals"].as<bool>();
        global_guidance_->SetPlanningFrequency(CONFIG["control_frequency"].as<double>());

        if (CONFIG["step_map"] && CONFIG["step_map"]["enable"])
            _enable_step_map = CONFIG["step_map"]["enable"].as<bool>();

        if (CONFIG["guidance_reference"] && CONFIG["guidance_reference"]["use_tube"])
            _use_tube = CONFIG["guidance_reference"]["use_tube"].as<bool>();

        // A single topology-mode linear-constraint tube around the injected guidance trajectory.
        // use_full_radius = true: this is the only dynamic collision-avoidance constraint here,
        // so it must use the real obstacle radius (T-MPC uses 1e-3 because Ellipsoid does the avoidance).
        // Skipped when use_tube=false: a downstream module then handles avoidance from the shared StepMap.
        if (_use_tube)
        {
            guidance_constraints_ = std::make_unique<LinearizedConstraints>(solver);
            guidance_constraints_->setTopologyConstraints(true);
        }

        if (_enable_step_map)
        {
            ros::NodeHandle private_nh("~");
            step_map_builder_ = std::make_shared<MPCPlannerStepMap::StepMapBuilder>(private_nh);
        }

        LOG_INITIALIZED();
    }

    void GuidanceReference::update(State &state, const RealTimeData &data, ModuleData &module_data)
    {
        LOG_MARK("Guidance Reference: Update");
        _guidance_valid = false;

        if (_reference_spline == nullptr)
        {
            LOG_MARK("Reference path not yet available");
            return;
        }

        // 1) StepMap (collision model for the PRM search)
        if (_enable_step_map && step_map_builder_ != nullptr)
        {
            step_map_ = step_map_builder_->update(data.costmap,
                                                  state.getPos(),
                                                  state.get("psi"),
                                                  data.dynamic_obstacles,
                                                  data.robot_area,
                                                  global_guidance_->GetConfig()->N,
                                                  GuidancePlanner::Config::DT);
            if (step_map_ && step_map_->valid())
                global_guidance_->SetStepMap(step_map_);
            else
                global_guidance_->SetStepMap(nullptr);
        }
        else
        {
            step_map_.reset();
            global_guidance_->SetStepMap(nullptr);
        }

        // Share the StepMap with downstream constraint modules (e.g. StepDecompConstraints) so they
        // do not have to rebuild it. Null if the map is disabled or invalid.
        module_data.step_map = (step_map_ && step_map_->valid()) ? step_map_ : nullptr;

        // 2) Start / reference velocity (constant velocity reference in v1)
        global_guidance_->SetStart(state.getPos(), state.get("psi"), state.get("v"));
        global_guidance_->SetReferenceVelocity(CONFIG["weights"]["reference_velocity"].as<double>());

        if (!CONFIG["enable_output"].as<bool>())
        {
            LOG_INFO_THROTTLE(15000, "Not propagating nodes (output is disabled)");
            global_guidance_->DoNotPropagateNodes();
        }

        // 3) Goals along the global reference path
        setGoals(state, module_data);

        // 4) Run the Visibility-PRM search
        LOG_MARK("Running Guidance Search");
        global_guidance_->Update();

        // 5) Select the best guidance trajectory and inject it
        if (global_guidance_->Succeeded())
        {
            // GetGuidanceTrajectory(0) is the best by quality (guidance maintains consistency internally)
            auto &best = global_guidance_->GetGuidanceTrajectory(0);

            // Inject as the MPCC reference (arc-length path). Contouring (next module) adopts module_data.path.
            module_data.path = std::make_shared<RosTools::Spline2D>(best.spline.GetPath());

            // Inject into the main solver ego-prediction so the tube linearizes around this trajectory
            initializeSolverWithGuidance(0);

            // Inform the guidance which class we follow (hysteresis for next cycle's ordering)
            global_guidance_->OverrideSelectedTrajectory(best.topology_class, false);

            _guidance_valid = true;
        }

        // 6) Build the linear collision-avoidance tube around the (injected) ego-prediction.
        //    If guidance failed, this linearizes around the warmstart and still avoids obstacles.
        if (_use_tube)
            guidance_constraints_->update(state, data, module_data);
    }

    void GuidanceReference::setGoals(State &state, const ModuleData &module_data)
    {
        (void)module_data;
        LOG_MARK("Setting guidance planner goals");

        double robot_radius = CONFIG["robot_radius"].as<double>();
        double half_width = CONFIG["road"]["width"].as<double>() / 2. - robot_radius - 0.1;

        // The robot's progress along the GLOBAL reference path (state "spline" tracks the guidance
        // path, so we recompute the projection here).
        int segment = 0;
        double start_s = 0.;
        _reference_spline->findClosestPoint(state.getPos(), segment, start_s);

        global_guidance_->LoadReferencePath(std::max(0., start_s), _reference_spline,
                                            half_width, half_width);
    }

    void GuidanceReference::initializeSolverWithGuidance(int trajectory_id)
    {
        RosTools::Spline2D &trajectory_spline =
            global_guidance_->GetGuidanceTrajectory(trajectory_id).spline.GetTrajectory();

        for (int k = 1; k < _solver->N; k++) // k = 0 is the current state
        {
            Eigen::Vector2d cur_position = trajectory_spline.getPoint((double)(k)*_solver->dt);
            _solver->setEgoPrediction(k, "x", cur_position(0));
            _solver->setEgoPrediction(k, "y", cur_position(1));

            Eigen::Vector2d cur_velocity = trajectory_spline.getVelocity((double)(k)*_solver->dt);
            _solver->setEgoPrediction(k, "psi", std::atan2(cur_velocity(1), cur_velocity(0)));
            _solver->setEgoPrediction(k, "v", cur_velocity.norm());
        }
    }

    void GuidanceReference::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
    {
        if (_use_tube)
            guidance_constraints_->setParameters(data, module_data, k);
    }

    bool GuidanceReference::isDataReady(const RealTimeData &data, std::string &missing_data)
    {
        bool ready = true;
        if (data.reference_path.x.empty())
        {
            missing_data += "Reference Path ";
            ready = false;
        }
        if (_use_tube)
            ready = ready && guidance_constraints_->isDataReady(data, missing_data);
        return ready;
    }

    void GuidanceReference::onDataReceived(RealTimeData &data, std::string &&data_name)
    {
        if (data_name == "reference_path")
        {
            LOG_MARK("Guidance Reference: Received Reference Path");
            if (data.reference_path.s.empty())
                _reference_spline = std::make_shared<RosTools::Spline2D>(data.reference_path.x, data.reference_path.y);
            else
                _reference_spline = std::make_shared<RosTools::Spline2D>(data.reference_path.x, data.reference_path.y, data.reference_path.s);
        }

        if (data_name == "dynamic obstacles")
        {
            LOG_MARK("Guidance Reference: Received dynamic obstacles");
            if (_use_tube)
                guidance_constraints_->onDataReceived(data, std::forward<std::string>(data_name));

            std::vector<GuidancePlanner::Obstacle> obstacles;
            for (auto &obstacle : data.dynamic_obstacles)
            {
                std::vector<Eigen::Vector2d> positions;
                positions.push_back(obstacle.position); /** @note k = 0 is the current position */

                for (size_t k = 0; k < obstacle.prediction.modes[0].size(); k++)
                    positions.push_back(obstacle.prediction.modes[0][k].position);

                obstacles.emplace_back(obstacle.index, positions, obstacle.radius + data.robot_area[0].radius);
            }
            global_guidance_->LoadObstacles(obstacles, {});
        }
    }

    void GuidanceReference::visualize(const RealTimeData &data, const ModuleData &module_data)
    {
        PROFILE_SCOPE("GuidanceReference::Visualize");
        LOG_MARK("Guidance Reference: Visualize");

        if (global_guidance_->Succeeded())
            global_guidance_->Visualize(CONFIG["t-mpc"]["highlight_selected"].as<bool>(), -1);

        if (_use_tube)
            guidance_constraints_->visualize(data, module_data);
    }

    void GuidanceReference::reset()
    {
        global_guidance_->Reset();
    }

    void GuidanceReference::saveData(RosTools::DataSaver &data_saver)
    {
        data_saver.AddData("runtime_guidance", global_guidance_->GetLastRuntime());
        data_saver.AddData("gmpcc_guidance_valid", _guidance_valid ? 1. : 0.);
        global_guidance_->saveData(data_saver);
    }
} // namespace MPCPlanner
