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
#ifndef __ENVIRONMENT_NAV3DXYT_H_
#define __ENVIRONMENT_NAV3DXYT_H_


//eight-connected grid
#define NAV3DXYT_DXYWIDTH 8

/*
//definition of Timet orientations
//0 - is aligned with X-axis in the positive direction (1,0 in polar coordinates)
//Timet increases as we go counterclockwise
//number of Timet values 
#define NAV3DXYT_TimetDIRS 8 
*/

//number of actions per x,y,Timet state
#define NAV3DXYT_ACTIONWIDTH 5 //decrease, increase, same angle while moving plus decrease, increase angle while standing.

// ------------------------

#define NAV3DXYT_COSTMULT 1000 //1000000 //10000
#define COST_NORM_ORDER 1.0

// Define metrics on the XYT space (actually a semi-norm)
// #define TRANSITIONCOST_XYT(DX, DY, DT) ((int) NAV3DXYT_COSTMULT*sqrt((double)((DX)*(DX)+(DY)*(DY))))
// #define CELLCOST_XYT(DX, DY, DT) ((int) NAV3DXYT_COSTMULT*sqrt((double)((DX)*(DX)+(DY)*(DY))))
int TRANSITIONCOST_XYT(int DX, int DY, int DT);
int CELLCOST_XYT(int DX, int DY, int DT, int ConstraintDist);

// ------------------------

typedef struct{
	bool Stopology; // true if S1 topology. False if R1 topology.
	
} StateVariable_t;

typedef vector<double> State_t;
typedef vector<int> DiscState_t;

// ------------------------

typedef struct{
	double x;
	double y;
} EnvNAV3DXYT2Dpt_t;

typedef struct{
	double x;
	double y;
	double Timet;
} EnvNAV3DXYT3Dpt_t;

// ------------------------

typedef struct{
	int X;
	int Y;
	int Timet;
} EnvNAV3DXYT_pos_t;

typedef struct{
	vector<EnvNAV3DXYT_pos_t> pos_t;
} EnvNAV3DXYT_pos_trajectory;

typedef struct{
	int D;	// Discretized units - Already multiplied by NAV3DXYT_COSTMULT
	int Timet;
} EnvNAV3DXYT_dist_t;

typedef struct{
	vector<EnvNAV3DXYT_dist_t> dist_t;
} EnvNAV3DXYT_dist_trajectory;


int interpVal(int x1, int y1, int x2, int y2, int xs);

vector<int> FindAndInterpInTrajectory(EnvNAV3DXYT_dist_trajectory* distTraj, int timet, EnvNAV3DXYT_dist_t* InterpedDist);
void PutInTrajectory(EnvNAV3DXYT_dist_trajectory* distTraj, EnvNAV3DXYT_dist_t distToPut);
vector<int> FindAndInterpInTrajectory(EnvNAV3DXYT_pos_trajectory* posTraj, int timet, EnvNAV3DXYT_pos_t* InterpedPos);
void PutInTrajectory(EnvNAV3DXYT_pos_trajectory* posTraj, EnvNAV3DXYT_pos_t posToPut);

//bool CompareTrajectories(EnvNAV3DXYT_pos_trajectory* traj1, EnvNAV3DXYT_pos_trajectory* traj2, int thresh);

// ------------------------

class obstacleMap2D
{
public:

	bool* data;
	int size_x, size_y;
	obstacleMap2D() { };
	obstacleMap2D(int xSize, int ySize);
	obstacleMap2D(int xSize, int ySize, bool initVal);
	void init(int xSize, int ySize, bool initVal);
	bool get(int xx, int yy);
	void put(int xx, int yy, bool flag);
	
};

class obstacleMap3D
{
public:

	bool* data;
	int size_x, size_y, size_tt;
	obstacleMap3D() { };
	obstacleMap3D(int xSize, int ySize, int tSize);
	obstacleMap3D(int xSize, int ySize, int tSize, bool initVal);
	void init(int xSize, int ySize, int tSize, bool initVal);
	bool get(int xx, int yy, int tt);
	void put(int xx, int yy, int tt, bool flag);
	void ConstructFromTrajectories(vector<EnvNAV3DXYT_pos_trajectory> otherBots_trajectories, int collisionCheckRad);
	
};

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
	char dTimet;
	unsigned int cost; // Only the edge costs 
} EnvNAV3DXYTAction_t;

//----

typedef struct
{
	EnvNAV3DXYT_dist_trajectory DistTraj;
	int RobotIndex;
	vector<float> PenaltyParams;
} AConstraintOfARobot_t;


typedef struct
{
	int StartX_c;
	int StartY_c;
	int StartTimet_c;

	int EndX_c;
	int EndY_c;
	int EndTimet_c;
	
	vector<EnvNAV3DXYTAction_t> ActionsV;
	
	vector<AConstraintOfARobot_t> constraints;
	
} EnvNAV3DXYT_AParticularRobot_t;


typedef struct
{
	int Robot1;
	int Robot2;
	//PenaltyTracker_t Penalty;
	vector<float> PenaltyWeightParams;
	EnvNAV3DXYT_dist_trajectory constraint;
	
} EnvNAV3DXYT_AParticularConstraint_t;


// ------------------------

typedef struct
{
	vector<int> RobotStateX;
	vector<int> RobotStateY;
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

//CanonicalSequence_t DetermineCanonicalSequence(EnvNAV3DXYT_pos_trajectory posTraj, vector<EnvNAV3DXYT2Dpt_t> ObstacleCenters);

// ------------------------

class CentralizedInfo_t
{
public:

	CentralizedInfo_t() { };
	
	// -----
	
	vector<JointStatespacePoint_t> JointStatespaceObstacles;
	vector<JointStatespacePoint_t> JointStatespaceObstacles_temp;
	
	bool isPointInJointStatespaceObstacles(JointStatespacePoint_t pt, int flag, int radi, bool debug=false);
	int AddObstaclePointsToJointStatespace(vector<EnvNAV3DXYT_pos_trajectory> posTrajs, 
												vector<EnvNAV3DXYT_AParticularConstraint_t> theConstraints, 
												int TimetStart, int TimetEnd, bool isMethod2=false);
	void ConcatenateJointStatespaceLists(void);
	
	// -----
	
	vector<float> PenaltyWeights;
	//vector<float> AveragePenaltyWeights_Robots;
	vector< vector<float> > PenaltyWeightVectors; // PenaltyWeightMatrix[RobotIndex][ConstraintIndex]
		
	// -----
	/*
	vector<HomotopyClassBlocker_t> BlockedHomotopyClasses;
	
	bool isPointInBlockedHomotopyClasses(JointStatespacePoint_t pt, int flag=1);
	int AddObstaclePointsToBlockedHomotopyClasses(vector<EnvNAV3DXYT_pos_trajectory> posTrajs, 
												vector<EnvNAV3DXYT_AParticularConstraint_t> theConstraints, 
												int TimetStart, int TimetEnd, int radi);
	*/
	
	// -----
	
};

// -----------------------------------

//configuration parameters
typedef struct ENV_NAV3DXYT_CONFIG
{
	int EnvWidth_c;
	int EnvHeight_c;
	int EnvMaxTime_c;

	int StartX_c;
	int StartY_c;
	int StartTimet_c;

	int EndX_c;
	int EndY_c;
	int EndTimet_c;

	double cellsize_m;
	double timestepsize_m;
	
	GlobalParams_EnvNAV3DXYT GlobalParams;
	
	int BotIdentifier;

	// Trajectories in continuous space-time
	vector<int> otherBots_identifiers;
	vector<EnvNAV3DXYT_pos_trajectory> otherBots_trajectories;
	vector<EnvNAV3DXYT_dist_trajectory> distConstraint_trajectories;
	vector<float> penaltyWeights;

	// Centrally shared information
	CentralizedInfo_t* CentralizedInfo;

	//int dXY[NAV3DXYT_DXYWIDTH][2];

	vector<EnvNAV3DXYTAction_t> ActionsV; //array of standard actions
	
	obstacleMap2D StaticObstacleMap;
	obstacleMap3D DynamicObstacleMap;

} EnvNAV3DXYTConfig_t;

// ------------------------------------------


typedef struct
{
	int EnvWidth_c;
	int EnvHeight_c;
	int EnvMaxTime_c;
	
	double cellsize_m;
	double timestepsize_m;
	
	GlobalParams_EnvNAV3DXYT GlobalParams;
	
	int RobotCount;
	
	vector<EnvNAV3DXYT_AParticularRobot_t> TheRobots;
	vector<EnvNAV3DXYT_AParticularConstraint_t> TheConstraints;
	
	obstacleMap2D StaticObstacleMap;
	
} ConfigFileInfo;

// -----------------------------------




typedef struct 
{
	int stateID;
	int X;
	int Y;
	int Timet;
} EnvNAV3DXYTHashEntry_t;



//variables that dynamically change (e.g., array of states, ...)
typedef struct
{

	int startstateid;
	int goalstateid;

	//hash table of size x_size*y_size. Maps from coords to stateId	
	int HashTableSize;
	vector<EnvNAV3DXYTHashEntry_t*>* Coord2StateIDHashTable;

	//vector that maps from stateID to coords	
	vector<EnvNAV3DXYTHashEntry_t*> StateID2CoordTable;

	//any additional variables

}EnvironmentNAV3DXYT_t;



class EnvironmentNAV3DXYT : public DiscreteSpaceInformation
{

public:

	EnvNAV3DXYTConfig_t EnvNAV3DXYTCfg;

	bool InitializeEnv(const char* sEnvFile); // This function is of no use! - Just because it is inherited from DiscreteSpaceInformation
	bool InitializeMDPCfg(MDPConfig *MDPCfg);
	
	int  HuristicFunction(int X1, int Y1, int X2, int Y2);
	int  GetFromToHeuristic(int FromStateID, int ToStateID);
	int  GetGoalHeuristic(int stateID);
	int  GetStartHeuristic(int stateID);
	
	void SetAllActionsandAllOutcomes(CMDPSTATE* state);
	void SetAllPreds(CMDPSTATE* state);
	void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
	void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);

	int	 SizeofCreatedEnv();
	void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);
	void GetTrajectoryFromSolutionStateIDs(vector<int> solution_stateIDs_V, EnvNAV3DXYT_pos_trajectory* posTraj);
	void PrintEnv_Config(FILE* fOut);

    //TODO - add perimeter, goal with tols
    // bool InitializeEnv(int width, int height,
    //                   const unsigned char* mapdata,
    //                   double startx, double starty, double startTimet,
    //                   double goalx, double goaly, double goalTimet,
	//				   double goaltol_x, double goaltol_y, double goaltol_Timet,
	//				   vector<sbpl_2Dpt_t> perimeterptsV,
	//				   double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs);

	// bool InitializeEnv(const char* sEnvFile, vector<EnvNAV3DXYT_pos_trajectory> otherBots_trajectories,
	//				vector<EnvNAV3DXYT_dist_trajectory> distConstraint_trajectories,
	//				vector<float> penaltyWeights);

	/*bool InitializeEnv(vector<int> otherBots_identifiers,
					vector<EnvNAV3DXYT_pos_trajectory> otherBots_trajectories,
					vector<EnvNAV3DXYT_dist_trajectory> distConstraint_trajectories,
					vector<float> penaltyWeights);*/

	bool InitializeEnv(ConfigFileInfo* CfgInfo, CentralizedInfo_t* CentralizedInfo, int robotIndex,
										vector<int> otherBots_identifiers,
										vector<EnvNAV3DXYT_pos_trajectory> otherBots_trajectories,
										vector<EnvNAV3DXYT_dist_trajectory> distConstraint_trajectories);
										
	bool SetPenaltyWeights(vector<float> penaltyWeights);

    int SetStart(double x, double y, double Timet);
    int SetGoal(double x, double y, double Timet);
    // bool UpdateCost(int x, int y, int new_status);
	// void GetPredsofChangedEdges(vector<nav2dcell_t>* changedcellsV, vector<int> *preds_of_changededgesIDV);


	void GetCoordFromState(int stateID, int& x, int& y, int& Timet) const;

	int GetStateFromCoord(int x, int y, int Timet);

	// bool IsObstacle(int x, int y);
	void GetEnvParms(int *size_x, int *size_y, double* startx, double* starty, double*startTimet, 
							double* goalx, double* goaly, double* goalTimet, double* cellsize_m);

	const EnvNAV3DXYTConfig_t* GetEnvNavConfig();


    ~EnvironmentNAV3DXYT(){};

    void PrintTimeStat(FILE* fOut);
    
   	vector<int> ComputeCellConstraintViolationCost(int X, int Y, int T, bool CheckJointStatespaceObstacle=true);
	
 //private:

	//member data
	// EnvNAV3DXYTConfig_t EnvNAV3DXYTCfg;
	EnvironmentNAV3DXYT_t EnvNAV3DXYT;

	
	void ReadConfiguration(FILE* fCfg);

	void InitializeEnvConfig();

	unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Timet);

	void PrintHashTableHist();
	bool CheckQuant(FILE* fOut);

	void SetConfiguration_constraints(vector<int> otherBots_identifiers,
					vector<EnvNAV3DXYT_pos_trajectory> otherBots_trajectories,
					vector<EnvNAV3DXYT_dist_trajectory> distConstraint_trajectories,
					vector<float> penaltyWeights);

	void SetConfiguration_all(int width, int height, int maxTime,
					int startx, int starty, int startTimet,
					int goalx, int goaly, int goalTimet,
					double cellsize_m, double timestepsize_m,
					vector<EnvNAV3DXYT_pos_trajectory> otherBots_trajectories,
					vector<EnvNAV3DXYT_dist_trajectory> distConstraint_trajectories,
					vector<float> penaltyWeights);
	
	bool InitGeneral();



	EnvNAV3DXYTHashEntry_t* GetHashEntry(int X, int Y, int Timet);

	EnvNAV3DXYTHashEntry_t* CreateNewHashEntry(int X, int Y, int Timet);


	void CreateStartandGoalStates();

	void InitializeEnvironment();

	void ComputeHeuristicValues();

	bool IsValidCell(int X, int Y, int T, bool CheckStaticObstacle = true, bool CheckDynamicObstacle = true);

	bool IsWithinMapCell(int X, int Y, int T);

	// void CalculateFootprintForPose(EnvNAV3DXYT3Dpt_t pose, vector<sbpl_2Dcell_t>* footprint);

	int GetActionCost(int SourceX, int SourceY, int SourceTimet, EnvNAV3DXYTAction_t* action);
	
	// int GetActionCost(int SourceX, int SourceY, int SourceTimet, int TargetX, int TargetY, int TargetTimet);


};

// ------------------------------------------


void ReadConfigurationFile(const char* sEnvFile, ConfigFileInfo* theInfo);
EnvNAV3DXYT_dist_trajectory* GenerateDistanceConstraint(EnvNAV3DXYT_pos_trajectory RefPosTraj, int dist);

bool IsNearTrajectories(EnvNAV3DXYT_pos_trajectory traj1, EnvNAV3DXYT_pos_trajectory traj2, int MismatchThresh);



// ----

void CentralizedInfo_t_InitiatePenaltyWeights(CentralizedInfo_t* theCentralInfo, ConfigFileInfo theInfo);
void CentralizedInfo_t_UpdatePenaltyWeights(CentralizedInfo_t* theCentralInfo, ConfigFileInfo theInfo, int ActiveRobot, EnvNAV3DXYT_pos_trajectory* traj, EnvironmentNAV3DXYT* env, bool isSummetricWeights=true);

//vector<int> ComputeTrajectoryCost(EnvNAV3DXYT_pos_trajectory* traj, EnvironmentNAV3DXYT* env, int* Cp, int* Cc);
void ComputeTrajectoryCost(EnvNAV3DXYT_pos_trajectory* traj, EnvironmentNAV3DXYT* env, int* Cp, vector<int>* Cc, vector< vector<int> >* ConstraintViolationIndices);
vector<EnvNAV3DXYT_pos_trajectory> FindVariationsOfATrajectory(EnvNAV3DXYT_pos_trajectory* traj, EnvironmentNAV3DXYT* env, vector<int> varIndices);
//?float FindPenaltyWeightIncrement(float oldPenaltyWeight, EnvNAV3DXYT_pos_trajectory* traj, EnvironmentNAV3DXYT* env);
//float SuggestNextPenaltyWeight(float oldPenaltyWeight, EnvNAV3DXYT_pos_trajectory* traj, EnvironmentNAV3DXYT* env);
vector<float> SuggestNextPenaltyWeight(EnvNAV3DXYT_pos_trajectory* traj, EnvironmentNAV3DXYT* env);

void EvaluateTrajectoryChange(EnvNAV3DXYT_pos_trajectory* newTraj, EnvNAV3DXYT_pos_trajectory* oldTraj, EnvironmentNAV3DXYT* env, vector<int>* ConstraintNegociationFlag, float thresh);
bool AreTrajectoriesSame(EnvNAV3DXYT_pos_trajectory* newTraj, EnvNAV3DXYT_pos_trajectory* oldTraj);


// --------------------

class OutputFile
{
public:
	FILE* fOut;
	
	OutputFile(const char* filename);
	
	void WriteTrajectories(vector<EnvNAV3DXYT_pos_trajectory> pos_trajectories, int IterNo, int ActiveRobot, vector<float> PenaltyWeight, vector< vector<int> > PenaltyWeightRobotsThisIteration = vector< vector<int> >());
	void WriteStaticObstacles(obstacleMap2D obs);
	void WriteConstraints(vector<EnvNAV3DXYT_AParticularConstraint_t> TheConstraints);
	void Close(void);
	
};

// =================================

// void ComputeTrajectoryCost(EnvNAV3DXYT_pos_trajectory* traj, EnvironmentNAV3DXYT* env, int* Cp, int* Cc, vector<int>* NonzeroViolationIndices);


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
