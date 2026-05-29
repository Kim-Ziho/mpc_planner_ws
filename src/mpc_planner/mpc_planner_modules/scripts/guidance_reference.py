"""
Guidance-MPCC (G-MPCC)

Selects a single best guidance trajectory (from the homotopy-distinct set produced by
guidance_planner) and uses it as the MPCC reference (the Contouring spline is swapped to it
at runtime). The same trajectory is loaded into the main solver ego-prediction so that a single
LinearizedConstraints "tube" linearizes the dynamic collision-avoidance constraints around it,
which both keeps the solution in the chosen homotopy class and avoids the obstacles.

This is the single-solver counterpart of GuidanceConstraints (T-MPC++): no parallel optimization.
The C++ side (guidance_reference.h/.cpp) owns the GlobalGuidance instance and a topology-mode
LinearizedConstraints; the parameters declared here are filled by that owned constraint.
"""

import numpy as np

import sys
import os

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))

from control_modules import ConstraintModule


class GuidanceReferenceModule(ConstraintModule):

    def __init__(self, settings):
        super().__init__()

        self.module_name = "GuidanceReference"  # c++ name of the module
        self.import_name = "guidance_reference.h"

        self.dependencies.append("guidance_planner")

        # The C++ module owns a topology-mode LinearizedConstraints; pull in its source for the build
        self.sources.append("linearized_constraints.h")

        # A single linear constraint per obstacle (robot-centered), filled by the owned
        # LinearizedConstraints in C++. Slack is used for feasibility robustness.
        self.constraints.append(
            GuidanceLinearConstraints(
                max_obstacles=settings["max_obstacles"],
                other_halfspaces=settings["linearized_constraints"]["add_halfspaces"],
                use_slack=True,
            )
        )

        self.description = (
            "G-MPCC: tracks the best guidance trajectory as the MPCC reference and constrains "
            "the MPC to its homotopy class with a linearized collision-avoidance tube"
        )


# Constraints of the form a1*x + a2*y - (b + slack) <= 0, evaluated at the robot center.
# Mirrors guidance_constraints.py::LinearConstraints (single disc / robot center), matching the
# C++ LinearizedConstraints topology mode (_use_guidance = true). Parameter bundle names match the
# setSolverParameterLinConstraint{A1,A2,B} setters used by the C++ side.
class GuidanceLinearConstraints:

    def __init__(self, max_obstacles, other_halfspaces=0, use_slack=False):
        self.max_obstacles = max_obstacles
        self.nh = self.max_obstacles + other_halfspaces
        self.use_slack = use_slack

    def define_parameters(self, params):
        for index in range(self.nh):
            params.add(self.constraint_name(index) + "_a1", bundle_name="lin_constraint_a1")
            params.add(self.constraint_name(index) + "_a2", bundle_name="lin_constraint_a2")
            params.add(self.constraint_name(index) + "_b", bundle_name="lin_constraint_b")

    def constraint_name(self, index):
        return f"lin_constraint_{index}"

    def get_lower_bound(self):
        return [-np.inf for _ in range(self.nh)]

    def get_upper_bound(self):
        return [0.0 for _ in range(self.nh)]

    def get_constraints(self, model, params, settings, stage_idx):
        constraints = []

        pos_x = model.get("x")
        pos_y = model.get("y")

        try:
            slack = model.get("slack") if self.use_slack else 0.0
        except Exception:
            slack = 0.0

        for index in range(self.nh):
            a1 = params.get(self.constraint_name(index) + "_a1")
            a2 = params.get(self.constraint_name(index) + "_a2")
            b = params.get(self.constraint_name(index) + "_b")

            constraints.append(a1 * pos_x + a2 * pos_y - (b + slack))

        return constraints
