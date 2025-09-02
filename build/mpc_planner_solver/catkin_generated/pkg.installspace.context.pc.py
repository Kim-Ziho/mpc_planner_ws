# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/workspace/acados/lib/../include;/workspace/acados/lib/../include/blasfeo/include;/workspace/acados/lib/../include/hpipm/include".split(';') if "${prefix}/include;/workspace/acados/lib/../include;/workspace/acados/lib/../include/blasfeo/include;/workspace/acados/lib/../include/hpipm/include" != "" else []
PROJECT_CATKIN_DEPENDS = "mpc_planner_util".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmpc_planner_solver;/workspace/src/mpc_planner/mpc_planner_solver/acados/Solver/libacados_ocp_solver_Solver.so;/workspace/acados/lib/libacados.so;/workspace/acados/lib/libblasfeo.so;/workspace/acados/lib/libhpipm.so".split(';') if "-lmpc_planner_solver;/workspace/src/mpc_planner/mpc_planner_solver/acados/Solver/libacados_ocp_solver_Solver.so;/workspace/acados/lib/libacados.so;/workspace/acados/lib/libblasfeo.so;/workspace/acados/lib/libhpipm.so" != "" else []
PROJECT_NAME = "mpc_planner_solver"
PROJECT_SPACE_DIR = "/workspace/install"
PROJECT_VERSION = "0.0.0"
