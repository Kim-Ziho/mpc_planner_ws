/**
 * @file guidance_reference.h
 * @brief Guidance-MPCC (G-MPCC) controller module.
 *
 * Selects the single best guidance trajectory from guidance_planner and:
 *   1. injects it as the MPCC reference path (via module_data.path, adopted by Contouring),
 *   2. loads it into the main solver ego-prediction so the owned topology-mode
 *      LinearizedConstraints builds a linear collision-avoidance "tube" around it.
 *
 * This is the single-solver counterpart of GuidanceConstraints (T-MPC++): no parallel optimization,
 * so this module does NOT override optimize() — the main solver solves once.
 */

#ifndef __GUIDANCE_REFERENCE_H__
#define __GUIDANCE_REFERENCE_H__

#include <mpc_planner_modules/linearized_constraints.h>

#include <mpc_planner_modules/controller_module.h>
#include <mpc_planner_solver/solver_interface.h>

#include <mpc_planner_stepmap/step_map_builder.h>

#include <ros_tools/spline.h>

#include <memory>

namespace GuidancePlanner
{
    class GlobalGuidance;
}

namespace MPCPlanner
{
    class GuidanceReference : public ControllerModule
    {
    public:
        GuidanceReference(std::shared_ptr<Solver> solver);

    public:
        void update(State &state, const RealTimeData &data, ModuleData &module_data) override;
        void setParameters(const RealTimeData &data, const ModuleData &module_data, int k) override;

        bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

        void visualize(const RealTimeData &data, const ModuleData &module_data) override;

        /** @brief Load obstacles / reference path into the guidance module */
        void onDataReceived(RealTimeData &data, std::string &&data_name) override;

        void reset() override;
        void saveData(RosTools::DataSaver &data_saver) override;

    private: // Private functions
        void setGoals(State &state, const ModuleData &module_data);

        /** @brief Load the guidance trajectory into the main solver ego-prediction (x, y, psi, v) */
        void initializeSolverWithGuidance(int trajectory_id);

    private: // Member variables
        std::shared_ptr<GuidancePlanner::GlobalGuidance> global_guidance_;

        // Topology-mode linear constraints around the injected guidance trajectory (full obstacle radius)
        std::unique_ptr<LinearizedConstraints> guidance_constraints_;

        // The global reference path, kept here for goal seeding (independent of module_data.path)
        std::shared_ptr<RosTools::Spline2D> _reference_spline;

        std::shared_ptr<MPCPlannerStepMap::StepMapBuilder> step_map_builder_;
        std::shared_ptr<MPCPlannerStepMap::StepMap> step_map_;

        bool _enable_step_map{false};
        bool _guidance_valid{false};

        // When false, no linearized "tube" is built here. Dynamic collision avoidance is then
        // expected to be handled by a downstream module (e.g. StepDecompConstraints) that consumes
        // the shared StepMap. Controlled by CONFIG["guidance_reference"]["use_tube"].
        bool _use_tube{true};
    };
} // namespace MPCPlanner
#endif // __GUIDANCE_REFERENCE_H__
