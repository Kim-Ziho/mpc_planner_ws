#include <mpc_planner_solver/mpc_planner_parameters.h>

#include <mpc_planner_solver/solver_interface.h>
namespace MPCPlanner{

void setSolverParameterAcceleration(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 239 + 0] = value;
}
void setSolverParameterAngularVelocity(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 239 + 1] = value;
}
void setSolverParameterSlack(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 239 + 2] = value;
}
void setSolverParameterVelocity(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 239 + 3] = value;
}
void setSolverParameterReferenceVelocity(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 239 + 4] = value;
}
void setSolverParameterContour(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 239 + 5] = value;
}
void setSolverParameterLag(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 239 + 6] = value;
}
void setSolverParameterTerminalAngle(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 239 + 7] = value;
}
void setSolverParameterTerminalContouring(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 239 + 8] = value;
}
void setSolverParameterSplineXA(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 9] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 18] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 27] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 36] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 45] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 54] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 63] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 72] = value;
}
void setSolverParameterSplineXB(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 10] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 19] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 28] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 37] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 46] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 55] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 64] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 73] = value;
}
void setSolverParameterSplineXC(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 11] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 20] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 29] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 38] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 47] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 56] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 65] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 74] = value;
}
void setSolverParameterSplineXD(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 12] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 21] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 30] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 39] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 48] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 57] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 66] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 75] = value;
}
void setSolverParameterSplineYA(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 13] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 22] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 31] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 40] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 49] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 58] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 67] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 76] = value;
}
void setSolverParameterSplineYB(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 14] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 23] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 32] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 41] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 50] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 59] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 68] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 77] = value;
}
void setSolverParameterSplineYC(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 15] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 24] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 33] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 42] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 51] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 60] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 69] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 78] = value;
}
void setSolverParameterSplineYD(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 16] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 25] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 34] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 43] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 52] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 61] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 70] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 79] = value;
}
void setSolverParameterSplineStart(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 17] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 26] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 35] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 44] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 53] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 62] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 71] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 80] = value;
}
void setSolverParameterLinConstraintA1(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 81] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 84] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 87] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 90] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 93] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 96] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 99] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 102] = value;
	else if(index == 8)
		params.all_parameters[k * 239 + 105] = value;
	else if(index == 9)
		params.all_parameters[k * 239 + 108] = value;
	else if(index == 10)
		params.all_parameters[k * 239 + 111] = value;
	else if(index == 11)
		params.all_parameters[k * 239 + 114] = value;
}
void setSolverParameterLinConstraintA2(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 82] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 85] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 88] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 91] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 94] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 97] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 100] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 103] = value;
	else if(index == 8)
		params.all_parameters[k * 239 + 106] = value;
	else if(index == 9)
		params.all_parameters[k * 239 + 109] = value;
	else if(index == 10)
		params.all_parameters[k * 239 + 112] = value;
	else if(index == 11)
		params.all_parameters[k * 239 + 115] = value;
}
void setSolverParameterLinConstraintB(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 83] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 86] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 89] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 92] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 95] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 98] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 101] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 104] = value;
	else if(index == 8)
		params.all_parameters[k * 239 + 107] = value;
	else if(index == 9)
		params.all_parameters[k * 239 + 110] = value;
	else if(index == 10)
		params.all_parameters[k * 239 + 113] = value;
	else if(index == 11)
		params.all_parameters[k * 239 + 116] = value;
}
void setSolverParameterEgoDiscRadius(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 239 + 117] = value;
}
void setSolverParameterEgoDiscOffset(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 239 + 118] = value;
}
void setSolverParameterEllipsoidObstX(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 119] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 126] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 133] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 140] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 147] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 154] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 161] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 168] = value;
	else if(index == 8)
		params.all_parameters[k * 239 + 175] = value;
	else if(index == 9)
		params.all_parameters[k * 239 + 182] = value;
	else if(index == 10)
		params.all_parameters[k * 239 + 189] = value;
	else if(index == 11)
		params.all_parameters[k * 239 + 196] = value;
}
void setSolverParameterEllipsoidObstY(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 120] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 127] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 134] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 141] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 148] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 155] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 162] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 169] = value;
	else if(index == 8)
		params.all_parameters[k * 239 + 176] = value;
	else if(index == 9)
		params.all_parameters[k * 239 + 183] = value;
	else if(index == 10)
		params.all_parameters[k * 239 + 190] = value;
	else if(index == 11)
		params.all_parameters[k * 239 + 197] = value;
}
void setSolverParameterEllipsoidObstPsi(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 121] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 128] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 135] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 142] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 149] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 156] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 163] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 170] = value;
	else if(index == 8)
		params.all_parameters[k * 239 + 177] = value;
	else if(index == 9)
		params.all_parameters[k * 239 + 184] = value;
	else if(index == 10)
		params.all_parameters[k * 239 + 191] = value;
	else if(index == 11)
		params.all_parameters[k * 239 + 198] = value;
}
void setSolverParameterEllipsoidObstMajor(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 122] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 129] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 136] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 143] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 150] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 157] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 164] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 171] = value;
	else if(index == 8)
		params.all_parameters[k * 239 + 178] = value;
	else if(index == 9)
		params.all_parameters[k * 239 + 185] = value;
	else if(index == 10)
		params.all_parameters[k * 239 + 192] = value;
	else if(index == 11)
		params.all_parameters[k * 239 + 199] = value;
}
void setSolverParameterEllipsoidObstMinor(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 123] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 130] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 137] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 144] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 151] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 158] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 165] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 172] = value;
	else if(index == 8)
		params.all_parameters[k * 239 + 179] = value;
	else if(index == 9)
		params.all_parameters[k * 239 + 186] = value;
	else if(index == 10)
		params.all_parameters[k * 239 + 193] = value;
	else if(index == 11)
		params.all_parameters[k * 239 + 200] = value;
}
void setSolverParameterEllipsoidObstChi(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 124] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 131] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 138] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 145] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 152] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 159] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 166] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 173] = value;
	else if(index == 8)
		params.all_parameters[k * 239 + 180] = value;
	else if(index == 9)
		params.all_parameters[k * 239 + 187] = value;
	else if(index == 10)
		params.all_parameters[k * 239 + 194] = value;
	else if(index == 11)
		params.all_parameters[k * 239 + 201] = value;
}
void setSolverParameterEllipsoidObstR(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 125] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 132] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 139] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 146] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 153] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 160] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 167] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 174] = value;
	else if(index == 8)
		params.all_parameters[k * 239 + 181] = value;
	else if(index == 9)
		params.all_parameters[k * 239 + 188] = value;
	else if(index == 10)
		params.all_parameters[k * 239 + 195] = value;
	else if(index == 11)
		params.all_parameters[k * 239 + 202] = value;
}
void setSolverParameterDecompA1(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 203] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 206] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 209] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 212] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 215] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 218] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 221] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 224] = value;
	else if(index == 8)
		params.all_parameters[k * 239 + 227] = value;
	else if(index == 9)
		params.all_parameters[k * 239 + 230] = value;
	else if(index == 10)
		params.all_parameters[k * 239 + 233] = value;
	else if(index == 11)
		params.all_parameters[k * 239 + 236] = value;
}
void setSolverParameterDecompA2(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 204] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 207] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 210] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 213] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 216] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 219] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 222] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 225] = value;
	else if(index == 8)
		params.all_parameters[k * 239 + 228] = value;
	else if(index == 9)
		params.all_parameters[k * 239 + 231] = value;
	else if(index == 10)
		params.all_parameters[k * 239 + 234] = value;
	else if(index == 11)
		params.all_parameters[k * 239 + 237] = value;
}
void setSolverParameterDecompB(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 239 + 205] = value;
	else if(index == 1)
		params.all_parameters[k * 239 + 208] = value;
	else if(index == 2)
		params.all_parameters[k * 239 + 211] = value;
	else if(index == 3)
		params.all_parameters[k * 239 + 214] = value;
	else if(index == 4)
		params.all_parameters[k * 239 + 217] = value;
	else if(index == 5)
		params.all_parameters[k * 239 + 220] = value;
	else if(index == 6)
		params.all_parameters[k * 239 + 223] = value;
	else if(index == 7)
		params.all_parameters[k * 239 + 226] = value;
	else if(index == 8)
		params.all_parameters[k * 239 + 229] = value;
	else if(index == 9)
		params.all_parameters[k * 239 + 232] = value;
	else if(index == 10)
		params.all_parameters[k * 239 + 235] = value;
	else if(index == 11)
		params.all_parameters[k * 239 + 238] = value;
}
}
