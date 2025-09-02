#include <mpc_planner_generated.h>

#include <ros_tools/logging.h>

#include <stdexcept>

namespace MPCPlanner{

double getForcesOutput(const Solver_output& output, const int k, const int var_index){
		if(k == 0)
			{
				if(var_index >= 2)					LOG_WARN("getForcesOutput for k = 0 returns the initial state.");
			return output.x01[var_index];
		}
		if(k == 1)
			return output.x02[var_index];
		if(k == 2)
			return output.x03[var_index];
		if(k == 3)
			return output.x04[var_index];
		if(k == 4)
			return output.x05[var_index];
		if(k == 5)
			return output.x06[var_index];
		if(k == 6)
			return output.x07[var_index];
		if(k == 7)
			return output.x08[var_index];
		if(k == 8)
			return output.x09[var_index];
		if(k == 9)
			return output.x10[var_index];
		if(k == 10)
			return output.x11[var_index];
		if(k == 11)
			return output.x12[var_index];
		if(k == 12)
			return output.x13[var_index];
		if(k == 13)
			return output.x14[var_index];
		if(k == 14)
			return output.x15[var_index];
		if(k == 15)
			return output.x16[var_index];
		if(k == 16)
			return output.x17[var_index];
		if(k == 17)
			return output.x18[var_index];
		if(k == 18)
			return output.x19[var_index];
		if(k == 19)
			return output.x20[var_index];
throw std::runtime_error("Invalid k value for getForcesOutput");
}

void loadForcesWarmstart(Solver_params& params, const Solver_output& output){
		for (int i = 0; i < 2; i++){
			params.z_init_00[i] = params.x0[i];
		}
		for (int i = 0; i < 8; i++){
			params.z_init_01[i] = params.x0[8*1 + i];
			params.z_init_02[i] = params.x0[8*2 + i];
			params.z_init_03[i] = params.x0[8*3 + i];
			params.z_init_04[i] = params.x0[8*4 + i];
			params.z_init_05[i] = params.x0[8*5 + i];
			params.z_init_06[i] = params.x0[8*6 + i];
			params.z_init_07[i] = params.x0[8*7 + i];
			params.z_init_08[i] = params.x0[8*8 + i];
			params.z_init_09[i] = params.x0[8*9 + i];
			params.z_init_10[i] = params.x0[8*10 + i];
			params.z_init_11[i] = params.x0[8*11 + i];
			params.z_init_12[i] = params.x0[8*12 + i];
			params.z_init_13[i] = params.x0[8*13 + i];
			params.z_init_14[i] = params.x0[8*14 + i];
			params.z_init_15[i] = params.x0[8*15 + i];
			params.z_init_16[i] = params.x0[8*16 + i];
			params.z_init_17[i] = params.x0[8*17 + i];
			params.z_init_18[i] = params.x0[8*18 + i];
			params.z_init_19[i] = params.x0[8*19 + i];
		}
	}
	void setForcesReinitialize(Solver_params& params, const bool value){
	}
}
