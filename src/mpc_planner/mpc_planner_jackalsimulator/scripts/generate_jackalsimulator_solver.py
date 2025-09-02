#!/usr/bin/python3

import os
import sys
import numpy as np

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))
sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc_planner_modules", "scripts"))

# SET YOUR FORCES PATH HERE (can also be in PYTHONPATH)
forces_path = os.path.join(os.path.expanduser("~"), "forces_pro_client")
sys.path.append(forces_path)

from util.files import load_settings, get_current_package
from control_modules import ModuleManager
from generate_solver import generate_solver

# Import modules here from mpc_planner_modules
from mpc_base import MPCBaseModule
from contouring import ContouringModule
from curvature_aware_contouring import CurvatureAwareContouringModule
from goal_module import GoalModule
from path_reference_velocity import PathReferenceVelocityModule

from ellipsoid_constraints import EllipsoidConstraintModule
from gaussian_constraints import GaussianConstraintModule
from guidance_constraints import GuidanceConstraintModule
from linearized_constraints import LinearizedConstraintModule
from scenario_constraints import ScenarioConstraintModule

# Import solver models that you want to use
from solver_model import ContouringSecondOrderUnicycleModel, ContouringSecondOrderUnicycleModelWithSlack
from solver_model import ContouringSecondOrderUnicycleModelCurvatureAware


def configuration_no_obstacles(settings):
    modules = ModuleManager()
    model = ContouringSecondOrderUnicycleModel()

    # You can manually set state/input bounds
    # lower_bound = [-2.0, -0.8, -2000.0, -2000.0, -np.pi * 2, -1.0, -1.0]
    # upper_bound = [2.0, 0.8, 2000.0, 2000.0, np.pi * 2, 3.0, 10000.0]
    # model.set_bounds(lower_bound, upper_bound)

    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration") # w_a * ||a||_2^2
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity") # w_w * ||w||_2^2

    if not settings["contouring"]["dynamic_velocity_reference"]:
        base_module.weigh_variable(var_name="v",    
                                weight_names=["velocity", "reference_velocity"], 
                                cost_function=lambda x, w: w[0] * (x-w[1])**2) # w_v * ||v - v_ref||_2^2

    modules.add_module(ContouringModule(settings)) # Contouring costs
    if settings["contouring"]["dynamic_velocity_reference"]:
        modules.add_module(PathReferenceVelocityModule(settings)) # Possibly adaptive v_ref

    return model, modules


def configuration_basic(settings):
    model, modules = configuration_no_obstacles(settings)

    modules.add_module(EllipsoidConstraintModule(settings))

    return model, modules


def configuration_safe_horizon(settings):
    modules = ModuleManager()
    model = ContouringSecondOrderUnicycleModelWithSlack()

    # Module that allows for penalization of variables
    base_module = modules.add_module(MPCBaseModule(settings))

    # Penalize ||a||_2^2 and ||w||_2^2
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")
    base_module.weigh_variable(var_name="slack", weight_names="slack", rqt_max_value=10000.0)

    if not settings["contouring"]["dynamic_velocity_reference"]:
        base_module.weigh_variable(var_name="v",    
                                weight_names=["velocity", "reference_velocity"], 
                                cost_function=lambda x, w: w[0] * (x-w[1])**2) # w_v * ||v - v_ref||_2^2

    modules.add_module(ContouringModule(settings)) # Contouring costs
    if settings["contouring"]["dynamic_velocity_reference"]:
        modules.add_module(PathReferenceVelocityModule(settings)) # Possibly adaptive v_ref

    modules.add_module(ScenarioConstraintModule(settings))
    return model, modules


def configuration_tmpc(settings):
    model, modules = configuration_no_obstacles(settings)

    modules.add_module(GuidanceConstraintModule(
        settings, 
        constraint_submodule=EllipsoidConstraintModule # This configures the obstacle avoidance used in each planner
    ))
    # modules.add_module(GuidanceConstraintModule(settings, constraint_submodule=GaussianConstraintModule))

    return model, modules


def configuration_lmpcc(settings):
    modules = ModuleManager()
    model = ContouringSecondOrderUnicycleModel()

    # Penalize ||a||_2^2 and ||w||_2^2
    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")

    # modules.add_module(ContouringModule(settings))
    modules.add_module(GoalModule(settings))

    modules.add_module(PathReferenceVelocityModule(settings))

    modules.add_module(EllipsoidConstraintModule(settings))

    return model, modules


settings = load_settings()

# model, modules = configuration_basic(settings)
# model, modules = configuration_no_obstacles(settings)

# NOTE: LMPCC - basic MPC with deterministic obstacle avoidance
# model, modules = configuration_lmpcc(settings)

# NOTE: T-MPC - Parallelized MPC optimizing trajectories with several distinct passing behaviors.
model, modules = configuration_tmpc(settings)

# NOTE: SH-MPC - MPC incorporating non Gaussian uncertainty in obstacle motion. 
# More configuration parameters in `scenario_module/config/params.yaml`
# model, modules = configuration_safe_horizon(settings)

generate_solver(modules, model, settings)
exit(0)
