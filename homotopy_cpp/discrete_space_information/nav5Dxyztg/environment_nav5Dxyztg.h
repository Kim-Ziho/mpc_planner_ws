/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __ENVIRONMENT_NAV5DXYZTG_H_
#define __ENVIRONMENT_NAV5DXYZTG_H_


//eight-connected grid
#define NAV5DXYZTG_DXYWIDTH 8

/*
//definition of Timet orientations
//0 - is aligned with X-axis in the positive direction (1,0 in polar coordinates)
//Timet increases as we go counterclockwise
//number of Timet values 
#define NAV5DXYZTG_TimetDIRS 8 
*/

//number of actions per x,y,Timet state
#define NAV5DXYZTG_ACTIONWIDTH 5 //decrease, increase, same angle while moving plus decrease, increase angle while standing.

// ------------------------

#define NAV5DXYZTG_COSTMULT 1000 // Needs to be 1000 if using 2Dgridsearch for precomputing huristics //1000000 //10000
#define COST_NORM_ORDER 1.0

#define LVAL_EQUAL_THRESH (1.0)
#define LVAL_INTEGRATION_STEP (1e-2)

#define RANDF (1.0 - 2.0*((float)rand())/RAND_MAX) // Samples from U((-1,1))
#define RANDU (((float)rand())/RAND_MAX) // Samples from U((0,1))

// Define metrics on the XYT space (actually a semi-norm)
// #define TRANSITIONCOST_XYT(DX, DY, DT) ((int) NAV5DXYZTG_COSTMULT*sqrt((double)((DX)*(DX)+(DY)*(DY))))
// #define CELLCOST_XYT(DX, DY, DT) ((int) NAV5DXYZTG_COSTMULT*sqrt((double)((DX)*(DX)+(DY)*(DY))))
int TRANSITIONCOST_XYZT(int DX, int DY, int DZ, int DT);
int CELLCOST_XYZT(int DX, int DY, int DZ, int DT, int ConstraintDist);

// ------------------------

typedef struct{
	
	float VIOLATION_COST_POWER;
	int COLLISION_CHECK_RADIUS;
	int HEURISTIC_TYPE;
	int PRECOMPUTE_HEURISTIC;
	bool IS_ITERATION_SYMMETRIC;
	
	bool DO_JOINTSTATESPACE_LOGGING;
	int JOINTSTATESPACE_LOGGING_METHOD;
	int JOINTSTATESPACE_LOGGING_RAD;
	
	int ITERATION_TYPE;
	int MAX_ITERATION_COUNT;
	int MIN_ITERATION_COUNT;
	int MAX_SUPERITER_COUNT;
	int CONVERGENCE_CYCLE_COUNT;
	
	int PENALTY_WEIGHT_INCREMENT_METHOD;
	int PENALTY_BIN_SEARCH_MAXSTEPS;
	float PENALTY_BIN_SEARCH_THRESH;
	//float PENALTY_INCREMENT_MULT_FAC;
	//vector<float> PenaltyWeightParams;
	
	int STATIC_OBSTACLE_INFLATION_RADIUS;
	
	//bool CHECK_HOMOTOPY_CLASS;
	//int HOMOTOPY_CLASS_METHOD;
	//vector<EnvNAV5DXYZTG2Dpt_t> ObstacleCenters;
	
} GlobalParams_EnvNAV5DXYZTG;

// ------------------------

typedef struct{
	double x;
	double y;
	double z;
} EnvNAV5DXYZTG2Dpt_t;

typedef struct{
	int x;
	int y;
	int z;
} EnvNAV5DXYZTG2DptInt_t;

typedef struct{
	double x;
	double y;
	double z;
	double Timet;
} EnvNAV5DXYZTG3Dpt_t;

typedef struct{
	double x;
	double y;
	double z;
	double Timet;
	double g;
} EnvNAV5DXYZTG4Dpt_t;

// ------------------------

typedef struct{
	int X;
	int Y;
	int Z;
	int Timet;
//	int g;
} EnvNAV5DXYZTG_pos_t;

typedef struct{
	int X;
	int Y;
	int Z;
	int Timet;
	bool isStrong;
} EnvNAV5DXYZTG_pos_key;

EnvNAV5DXYZTG_pos_key GetKeyPt(EnvNAV5DXYZTG_pos_t pt, bool isstrong);

typedef struct{
	vector<EnvNAV5DXYZTG_pos_t> pos_t;
	vector<EnvNAV5DXYZTG_pos_key> KeyPts;
	complex<double> Lval;
} EnvNAV5DXYZTG_pos_trajectory;

typedef struct{
	int D;	// Discretized units - Already multiplied by NAV5DXYZTG_COSTMULT
	int Timet;
} EnvNAV5DXYZTG_dist_t;

typedef struct{
	vector<EnvNAV5DXYZTG_dist_t> dist_t;
} EnvNAV5DXYZTG_dist_trajectory;


int interpVal(int x1, int y1, int x2, int y2, int xs);

vector<int> FindAndInterpInTrajectory(EnvNAV5DXYZTG_dist_trajectory* distTraj, int timet, EnvNAV5DXYZTG_dist_t* InterpedDist);
void PutInTrajectory(EnvNAV5DXYZTG_dist_trajectory* distTraj, EnvNAV5DXYZTG_dist_t distToPut);
vector<int> FindAndInterpInTrajectory(EnvNAV5DXYZTG_pos_trajectory* posTraj, int timet, EnvNAV5DXYZTG_pos_t* InterpedPos);
void PutInTrajectory(EnvNAV5DXYZTG_pos_trajectory* posTraj, EnvNAV5DXYZTG_pos_t posToPut);

//bool CompareTrajectories(EnvNAV5DXYZTG_pos_trajectory* traj1, EnvNAV5DXYZTG_pos_trajectory* traj2, int thresh);

// ------------------------

class obstacleMap3D
{
public:

	bool* data;
	int size_x, size_y, size_z;
	obstacleMap3D() { };
	obstacleMap3D(int xSize, int ySize, int zSize);
	obstacleMap3D(int xSize, int ySize, int zSize, bool initVal);
	//~obstacleMap2D() {delete[] data;};  // MEM_CLEAR ***
	void init(int xSize, int ySize, int zSize, bool initVal);
	bool get(int xx, int yy, int zz);
	void put(int xx, int yy, int zz, bool flag);
	
};

/*
class obstacleMap3D
{
public:

	bool* data;
	int size_x, size_y, size_tt;
	obstacleMap3D() { };
	obstacleMap3D(int xSize, int ySize, int tSize);
	obstacleMap3D(int xSize, int ySize, int tSize, bool initVal);
	~obstacleMap3D() {delete[] data;};  // MEM_CLEAR
	void init(int xSize, int ySize, int tSize, bool initVal);
	bool get(int xx, int yy, int tt);
	void put(int xx, int yy, int tt, bool flag);
	void ConstructFromTrajectories(vector<EnvNAV5DXYZTG_pos_trajectory> otherBots_trajectories, int collisionCheckRad);
	
};
*/

// ------------------------
/*
class PenaltyTracker_t
{
public:

	char PenaltyChangeMethod[1024];
	vector<float> PenaltyChangeParams;
	float Weight;
	//float p;
	
	PenaltyTracker_t() { };
	PenaltyTracker_t(char* method, char* paramsString);
	
	void next(void);
	void reset(void);
};
*/
// ------

typedef struct
{
	char dX;
	char dY;
	char dZ;
	char dTimet;
	char dG;
	unsigned int cost; // Only the edge costs 
} EnvNAV5DXYZTGAction_t;

//----

typedef struct
{
	EnvNAV5DXYZTG_dist_trajectory DistTraj;
	int RobotIndex;
	vector<float> PenaltyParams;
} AConstraintOfARobot_t;


/*
typedef struct
{
	int StartX_c;
	int StartY_c;
	int StartTimet_c;
	int StartG_c;

	int EndX_c;
	int EndY_c;
	int EndTimet_c;
	int EndG_c;
	
	vector<EnvNAV5DXYZTG2DptInt_t> Tasks;
	
	vector<EnvNAV5DXYZTGAction_t> ActionsV;
	
	vector<AConstraintOfARobot_t> constraints;
	
} EnvNAV5DXYZTG_AParticularRobot_t;
*/

class EnvNAV5DXYZTG_AParticularRobot_t
{
public:

	int StartX_c;
	int StartY_c;
	int StartZ_c;
	int StartTimet_c;
	int StartG_c;

	int EndX_c;
	int EndY_c;
	int EndZ_c;
	int EndTimet_c;
	int EndG_c;
	
	vector<EnvNAV5DXYZTG2DptInt_t> Tasks;
	
	vector<EnvNAV5DXYZTGAction_t> ActionsV;
	
	vector<AConstraintOfARobot_t> constraints;
	
	vector< complex<double> > BlockedHomotopyClass_Const_LVals;
};


typedef struct
{
	int Robot1;
	int Robot2;
	//PenaltyTracker_t Penalty;
	vector<float> PenaltyWeightParams;
	EnvNAV5DXYZTG_dist_trajectory constraint;
	
} EnvNAV5DXYZTG_AParticularConstraint_t;


// ------------------------

typedef struct
{
	vector<int> RobotStateX;
	vector<int> RobotStateY;
	vector<int> RobotStateZ;
	int Timet;
	vector<int> ViolatingRobotIndices;
} JointStatespacePoint_t;

/*
typedef struct
{
	vector<int> x;
	vector<int> y;
} PointsCollection_t;

typedef struct
{
	vector<PointsCollection_t> robot;
} HomotopyClassBlocker_t;
*/

/*
typedef struct
{
	vector<int> ObsCount; // e.g. {2,-3,1,4,-5}
}CanonicalSequence_t;
*/

//CanonicalSequence_t DetermineCanonicalSequence(EnvNAV5DXYZTG_pos_trajectory posTraj, vector<EnvNAV5DXYZTG2Dpt_t> ObstacleCenters);

// ------------------------

class HomotopyClassList_allRobots
{
public:
	vector< vector< complex<double> > > HomotopyClassList; // HomotopyClassList[RobotIndex][BlockedIndex]
};

class HomotopyClass_allRobots
{
public:
	vector< complex<double> > HomotopyClass; // HomotopyClass[RobotIndex]
};


class CentralizedInfo_t
{
public:

	CentralizedInfo_t() { };
	CentralizedInfo_t(int RobCount)
		{
			BlockedHomotopyClasses.HomotopyClassList.resize(RobCount);
			InitialConstrainHomotopyClass.HomotopyClassList.resize(RobCount);
		};
	
	// -----
	
	vector<JointStatespacePoint_t> JointStatespaceObstacles;
	vector<JointStatespacePoint_t> JointStatespaceObstacles_temp;
	
	bool isPointInJointStatespaceObstacles(JointStatespacePoint_t pt, int flag, int radi, bool debug=false);
	int AddObstaclePointsToJointStatespace(vector<EnvNAV5DXYZTG_pos_trajectory> posTrajs, 
												vector<EnvNAV5DXYZTG_AParticularConstraint_t> theConstraints, 
												int TimetStart, int TimetEnd, bool isMethod2=false);
	void ConcatenateJointStatespaceLists(void);
	
	// -----
	
	vector<float> PenaltyWeights;
	//vector<float> AveragePenaltyWeights_Robots;
	vector< vector<float> > PenaltyWeightVectors; // PenaltyWeightMatrix[RobotIndex][ConstraintIndex]
		
	// -----
	
	HomotopyClassList_allRobots BlockedHomotopyClasses;
	HomotopyClassList_allRobots InitialConstrainHomotopyClass;
	
	//vector<HomotopyClass_allRobots> NoSolutionHomotopy_hist;
	//vector<HomotopyClassList_allRobots> BlockedHomotopy_hist;
	
	/*
	vector<HomotopyClassBlocker_t> BlockedHomotopyClasses;
	
	bool isPointInBlockedHomotopyClasses(JointStatespacePoint_t pt, int flag=1);
	int AddObstaclePointsToBlockedHomotopyClasses(vector<EnvNAV5DXYZTG_pos_trajectory> posTrajs, 
												vector<EnvNAV5DXYZTG_AParticularConstraint_t> theConstraints, 
												int TimetStart, int TimetEnd, int radi);
	*/
	
	// -----
	
	// BlockedHomotopy_traj [RobotNo] [HomotopyClassNo]
	//vector< vector<EnvNAV5DXYZTG_pos_trajectory> > BlockedHomotopy_traj;
	
	// CurrentHomotopy_traj [RobotNo]
	//vector<EnvNAV5DXYZTG_pos_trajectory> CurrentHomotopy_traj;
};

// -----------------------------------

class LValDiffMap_t
{
public:

	vector< complex<double> >* LValDiffs;
	int size_x, size_y;
	vector< complex<double> > CriticalPoints;
	vector<EnvNAV5DXYZTGAction_t> ActionsV;
	double IntegrationStepSize;
	
	LValDiffMap_t() { };
	void SetCriticalPoints(vector<int> Xs, vector<int> Ys);
	void ComputeLValDiffs(int xSize, int ySize, vector<EnvNAV5DXYZTGAction_t> ActionsVIn, double IntegrationStepSizeIn=LVAL_INTEGRATION_STEP);
	complex<double> IntegrateLValDiff(int xv, int yv, int av);
	//~LValDiffMap_t() {delete[] data;};  // MEM_CLEAR ***
	//void init(int xSize, int ySize, bool initVal);
};

class RobotHomotopyInfo_t
{
public:

	bool ForcedActive;
	vector< complex<double> > BlockedHomotopyClass_LVals;
	vector< complex<double> > ConstrainHomotopyClass_LVals;
	
	LValDiffMap_t* LValDiffMap;
	
	complex<double> getLValDiff(int xs, int ys, int ActionInd);
	bool isActive() { return (ForcedActive || !(BlockedHomotopyClass_LVals.size()==0 && ConstrainHomotopyClass_LVals.size()==0)); };
};

// -----------------------------------


//configuration parameters
typedef struct ENV_NAV5DXYZTG_CONFIG
{
	int EnvWidth_c;
	int EnvHeight_c;
	int EnvDepth_c;
	int EnvMaxTime_c;
	//int EnvMaxG_c;

	int StartX_c;
	int StartY_c;
	int StartZ_c;
	int StartTimet_c;
	int StartG_c;

	int EndX_c;
	int EndY_c;
	int EndZ_c;
	int EndTimet_c;
	int EndG_c;
	
	vector<EnvNAV5DXYZTG2DptInt_t> Tasks;

	double cellsize_m;
	double timestepsize_m;
	
	GlobalParams_EnvNAV5DXYZTG GlobalParams;
	
	int BotIdentifier;
	int MaxBotIdentifier;

	// Trajectories in continuous space-time
	vector<int> otherBots_identifiers;
	vector<EnvNAV5DXYZTG_pos_trajectory> otherBots_trajectories;
	vector<EnvNAV5DXYZTG_dist_trajectory> distConstraint_trajectories;
	vector<float> penaltyWeights;

	// Centrally shared information
	CentralizedInfo_t* CentralizedInfo;

	//int dXY[NAV5DXYZTG_DXYWIDTH][2];

	vector<EnvNAV5DXYZTGAction_t> ActionsV; //array of standard actions
	
	obstacleMap3D StaticObstacleMap;
	//obstacleMap3D DynamicObstacleMap; //Include dynamic obstacles
	
	//bool StayInCurrentHomotopy;
	//EnvNAV5DXYZTG_pos_trajectory CurrentHomotopy_traj;
	RobotHomotopyInfo_t RobotHomotopyInfo;

} EnvNAV5DXYZTGConfig_t;

// ------------------------------------------


typedef struct
{
	int EnvWidth_c;
	int EnvHeight_c;
	int EnvDepth_c;
	int EnvMaxTime_c;
	
	double cellsize_m;
	double timestepsize_m;
	
	GlobalParams_EnvNAV5DXYZTG GlobalParams;
	
	int RobotCount;
	
	vector<EnvNAV5DXYZTG_AParticularRobot_t> TheRobots;
	vector<EnvNAV5DXYZTG_AParticularConstraint_t> TheConstraints;
	
	obstacleMap3D StaticObstacleMap;
	
	LValDiffMap_t* LValDiffMap; // Works only when all robots use 8-connected grids
	
} ConfigFileInfo;

// -----------------------------------



typedef struct 
{
	int stateID;
	int X;
	int Y;
	int Z;
	int Timet;
	int G;
	complex<double> Lval;
} EnvNAV5DXYZTGHashEntry_t;


typedef struct
{
	vector<int> RemTaskIndices;
	vector<int> MinTaskTravelHeuSum;
	int RemTasksCount;
} GComputeHistory_t;

// A modification on SBPL2DGridSearch for pre-computing heuristics
class SBPL2DGridSearchWithTasks
{
public:

	int width_x, height_y, cellsize_m;
	//int** PreComputedHeu;
	int TasksCount;
	vector<EnvNAV5DXYZTG2DptInt_t> Tasks;
	vector<SBPL2DGridSearch*> TasksPreSearch;
	SBPL2DGridSearch* OriginPreSearch;
	
	//int lastComputedG;
	//vector<int> RemTaskIndices;
	//vector<int> MinTaskTravelHeuSum;
	//int RemTasksCount;
	vector<int> ComputedGs;
	vector<GComputeHistory_t> StoredCompute;
	

	SBPL2DGridSearchWithTasks(int in_width_x, int in_height_y, float in_cellsize_m);
	~SBPL2DGridSearchWithTasks();
	
	bool PreCompute(unsigned char** Grid2D, unsigned char obsthresh, int ExpandOrigin_x, int ExpandOrigin_y, vector<EnvNAV5DXYZTG2DptInt_t> Tasks, int ExpandStop_x, int ExpandStop_y, SBPL_2DGRIDSEARCH_TERM_CONDITION termination_condition);
	
	int getPreComputedHeu(int x, int y, int g);
};


//variables that dynamically change (e.g., array of states, ...)A hypergiant (luminosity class 0) is a star with a tremendous mass and luminosity, showing signs of a very high rate of mass loss.
class EnvironmentNAV5DXYZTG_t
{
public:

	int startstateid;
	int goalstateid;
	
	EnvNAV5DXYZTGHashEntry_t* CopyOfGoalState;

	//hash table of size x_size*y_size. Maps from coords to stateId	
	int HashTableSize;
	vector<EnvNAV5DXYZTGHashEntry_t*>* Coord2StateIDHashTable;

	//vector that maps from stateID to coords	
	vector<EnvNAV5DXYZTGHashEntry_t*> StateID2CoordTable;

	//any additional variables
	SBPL2DGridSearch* grid2DsearchFwd;
	SBPL2DGridSearch* grid2DsearchBak;
	SBPL2DGridSearchWithTasks* grid2DsearchFwdTasks;
	
	EnvironmentNAV5DXYZTG_t() {CopyOfGoalState=NULL; grid2DsearchFwd=NULL; grid2DsearchBak=NULL; grid2DsearchFwdTasks=NULL;};

};



class EnvironmentNAV5DXYZTG : public DiscreteSpaceInformation
{

public:

	EnvNAV5DXYZTGConfig_t EnvNAV5DXYZTGCfg;
	bool DebugPrint;

	bool InitializeEnv(const char* sEnvFile); // This function is of no use! - Just because it is inherited from DiscreteSpaceInformation
	bool InitializeMDPCfg(MDPConfig *MDPCfg);
	
	int  HeuristicFunction(int X1, int Y1, int Z1, int X2, int Y2, int Z2);
	int  GetFromToHeuristic(int FromStateID, int ToStateID);
	int  GetGoalHeuristic(int stateID);
	int  GetStartHeuristic(int stateID);
	
	void SetAllActionsandAllOutcomes(CMDPSTATE* state);
	void SetAllPreds(CMDPSTATE* state);
	void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
	void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);

	int	 SizeofCreatedEnv();
	void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);
	void GetTrajectoryFromSolutionStateIDs(vector<int> solution_stateIDs_V, EnvNAV5DXYZTG_pos_trajectory* posTraj);
	void PostProcessTrajectory(EnvNAV5DXYZTG_pos_trajectory* posTraj);
	void PrintEnv_Config(FILE* fOut);

    //TODO - add perimeter, goal with tols
    // bool InitializeEnv(int width, int height,
    //                   const unsigned char* mapdata,
    //                   double startx, double starty, double startTimet,
    //                   double goalx, double goaly, double goalTimet,
	//				   double goaltol_x, double goaltol_y, double goaltol_Timet,
	//				   vector<sbpl_2Dpt_t> perimeterptsV,
	//				   double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs);

	// bool InitializeEnv(const char* sEnvFile, vector<EnvNAV5DXYZTG_pos_trajectory> otherBots_trajectories,
	//				vector<EnvNAV5DXYZTG_dist_trajectory> distConstraint_trajectories,
	//				vector<float> penaltyWeights);

	/*bool InitializeEnv(vector<int> otherBots_identifiers,
					vector<EnvNAV5DXYZTG_pos_trajectory> otherBots_trajectories,
					vector<EnvNAV5DXYZTG_dist_trajectory> distConstraint_trajectories,
					vector<float> penaltyWeights);*/

	bool InitializeEnv(ConfigFileInfo* CfgInfo, CentralizedInfo_t* CentralizedInfo, int robotIndex,
										vector<int> otherBots_identifiers,
										vector<EnvNAV5DXYZTG_pos_trajectory> otherBots_trajectories,
										vector<EnvNAV5DXYZTG_dist_trajectory> distConstraint_trajectories);
										
	bool SetPenaltyWeights(vector<float> penaltyWeights);

    int SetStart(double x, double y, double z, double Timet, int g, complex<double> LVal = complex<double>(0.0,0.0));
    int SetGoal(double x, double y, double z, double Timet, int g);
    // bool UpdateCost(int x, int y, int new_status);
	// void GetPredsofChangedEdges(vector<nav2dcell_t>* changedcellsV, vector<int> *preds_of_changededgesIDV);


	void GetCoordFromState(int stateID, int& x, int& y, int& z, int& Timet, int& g) const;

	//int GetStateFromCoord(int x, int y, int Timet, int g);
	int GetStateFromCoord(int x, int y, int z, int Timet, int g, complex<double> Lval);

	// bool IsObstacle(int x, int y);
	void GetEnvParms(int *size_x, int *size_y, int *size_z, double* startx, double* starty, double* startz, double*startTimet, 
							double* goalx, double* goaly, double* goalz, double* goalTimet, double* cellsize_m);

	const EnvNAV5DXYZTGConfig_t* GetEnvNavConfig();


    ~EnvironmentNAV5DXYZTG();
/*    {
    	if (this)
    	{
			if (EnvNAV5DXYZTG.grid2DsearchFwd)
				delete EnvNAV5DXYZTG.grid2DsearchFwd;
			if (EnvNAV5DXYZTG.grid2DsearchBak)
				delete EnvNAV5DXYZTG.grid2DsearchBak;
			if (EnvNAV5DXYZTG.grid2DsearchFwdTasks)
				delete EnvNAV5DXYZTG.grid2DsearchFwdTasks;
			if (EnvNAV5DXYZTG.Coord2StateIDHashTable)
				delete EnvNAV5DXYZTG.Coord2StateIDHashTable;
	    }
    };  // MEM_CLEAR
   */

    void PrintTimeStat(FILE* fOut);
    
   	vector<int> ComputeCellConstraintViolationCost(int X, int Y, int Z, int T, bool CheckJointStatespaceObstacle=true);
	
 //private:

	//member data
	// EnvNAV5DXYZTGConfig_t EnvNAV5DXYZTGCfg;
	EnvironmentNAV5DXYZTG_t EnvNAV5DXYZTG;

	
	void ReadConfiguration(FILE* fCfg);

	void InitializeEnvConfig();

	unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Z, unsigned int Timet, unsigned int G);

	void PrintHashTableHist();
	bool CheckQuant(FILE* fOut);

	void SetConfiguration_constraints(vector<int> otherBots_identifiers,
					vector<EnvNAV5DXYZTG_pos_trajectory> otherBots_trajectories,
					vector<EnvNAV5DXYZTG_dist_trajectory> distConstraint_trajectories,
					vector<float> penaltyWeights);

	void SetConfiguration_all(int width, int height, int depth, int maxTime,
					int startx, int starty, int startz, int startTimet, int startg,
					int goalx, int goaly, int goalz, int goalTimet, int goalg,
					double cellsize_m, double timestepsize_m,
					vector<EnvNAV5DXYZTG_pos_trajectory> otherBots_trajectories,
					vector<EnvNAV5DXYZTG_dist_trajectory> distConstraint_trajectories,
					vector<float> penaltyWeights);
	
	bool InitGeneral();



	EnvNAV5DXYZTGHashEntry_t* GetHashEntry(int X, int Y, int Z, int Timet, int G, complex<double> Lval, bool isGoal=false);

	EnvNAV5DXYZTGHashEntry_t* CreateNewHashEntry(int X, int Y, int Z, int Timet, int G, complex<double> Lval, bool isGoal=false);


	void CreateStartandGoalStates();

	void InitializeEnvironment();

	void ComputeHeuristicValues();

	bool IsValidCell(int X, int Y, int Z, int T, bool CheckStaticObstacle = true, bool CheckDynamicObstacle = true);

	bool IsWithinMapCell(int X, int Y, int Z, int T);

	// void CalculateFootprintForPose(EnvNAV5DXYZTG3Dpt_t pose, vector<sbpl_2Dcell_t>* footprint);

	int GetActionCost(int SourceX, int SourceY, int SourceZ, int SourceTimet, int SourceG, EnvNAV5DXYZTGAction_t* action);
	
	// int GetActionCost(int SourceX, int SourceY, int SourceTimet, int TargetX, int TargetY, int TargetTimet);


};

// ------------------------------------------


void ReadConfigurationFile(const char* sEnvFile, ConfigFileInfo* theInfo);
EnvNAV5DXYZTG_dist_trajectory* GenerateDistanceConstraint(EnvNAV5DXYZTG_pos_trajectory RefPosTraj, int dist);

bool IsNearTrajectories(EnvNAV5DXYZTG_pos_trajectory traj1, EnvNAV5DXYZTG_pos_trajectory traj2, int MismatchThresh);

vector<EnvNAV5DXYZTG_pos_t> GenerateStraightSegment(EnvNAV5DXYZTG_pos_t startPos, EnvNAV5DXYZTG_pos_t endPos, vector<EnvNAV5DXYZTGAction_t> ActionsV); // This works well only for 8-connected grid - TODO: Generelize this


// ----

void CentralizedInfo_t_InitiatePenaltyWeights(CentralizedInfo_t* theCentralInfo, ConfigFileInfo theInfo);
void CentralizedInfo_t_UpdatePenaltyWeights(CentralizedInfo_t* theCentralInfo, ConfigFileInfo theInfo, int ActiveRobot, EnvNAV5DXYZTG_pos_trajectory* traj, EnvironmentNAV5DXYZTG* env, bool isSummetricWeights=true);

//vector<int> ComputeTrajectoryCost(EnvNAV5DXYZTG_pos_trajectory* traj, EnvironmentNAV5DXYZTG* env, int* Cp, int* Cc);
void ComputeTrajectoryCost(EnvNAV5DXYZTG_pos_trajectory* traj, EnvironmentNAV5DXYZTG* env, int* Cp, vector<int>* Cc, vector< vector<int> >* ConstraintViolationIndices);
vector<EnvNAV5DXYZTG_pos_trajectory> FindVariationsOfATrajectory(EnvNAV5DXYZTG_pos_trajectory* traj, EnvironmentNAV5DXYZTG* env, vector<int> varIndices);
//?float FindPenaltyWeightIncrement(float oldPenaltyWeight, EnvNAV5DXYZTG_pos_trajectory* traj, EnvironmentNAV5DXYZTG* env);
//float SuggestNextPenaltyWeight(float oldPenaltyWeight, EnvNAV5DXYZTG_pos_trajectory* traj, EnvironmentNAV5DXYZTG* env);
vector<float> SuggestNextPenaltyWeight(EnvNAV5DXYZTG_pos_trajectory* traj, EnvironmentNAV5DXYZTG* env);

void EvaluateTrajectoryChange(EnvNAV5DXYZTG_pos_trajectory* newTraj, EnvNAV5DXYZTG_pos_trajectory* oldTraj, EnvironmentNAV5DXYZTG* env, vector<int>* ConstraintNegociationFlag, float thresh);
bool AreTrajectoriesSame(EnvNAV5DXYZTG_pos_trajectory* newTraj, EnvNAV5DXYZTG_pos_trajectory* oldTraj);

vector< vector<int> > IndexPermute(int count);
float sampled_atof(char* fmtStr);


void PostProcessTrajectory_Joint(vector<EnvironmentNAV5DXYZTG*> robEnvs, vector<EnvNAV5DXYZTG_pos_trajectory*> posTrajs);

// --------------------

class OutputFile
{
public:
	FILE* fOut;
	
	OutputFile(const char* filename);
	
	void WriteTrajectories(vector<EnvNAV5DXYZTG_pos_trajectory> pos_trajectories, int IterNo, int ActiveRobot, vector<float> PenaltyWeight, vector< vector<int> > PenaltyWeightRobotsThisIteration = vector< vector<int> >());
	void WriteStaticObstacles(obstacleMap3D obs);
	void WriteConstraints(vector<EnvNAV5DXYZTG_AParticularConstraint_t> TheConstraints);
	void WriteConfigFileParameters(ConfigFileInfo* theConfigFileInfo);
	void Close(void);
	
};

// =================================

// void ComputeTrajectoryCost(EnvNAV5DXYZTG_pos_trajectory* traj, EnvironmentNAV5DXYZTG* env, int* Cp, int* Cc, vector<int>* NonzeroViolationIndices);


// **********************************
// **********************************
// Utilities

//int MYprintf ( const char * format, ... );

class IndexTracker
{
public:
	int tracker;
	int tracker_max;
	vector<int> index;
	vector<int> dim;
	int dim_count;
	
	IndexTracker(vector<int> DimSizes);
	void reset(void);
	void next(void);
};

#endif
