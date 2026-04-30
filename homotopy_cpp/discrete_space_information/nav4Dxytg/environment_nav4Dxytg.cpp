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
#include "../../sbpl/headers.h"


#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif


//-------------------problem specific and local functions---------------------


static unsigned int inthash(unsigned int key)
{
  key += (key << 12);
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}

//examples of hash functions: map state coordinates onto a hash value
//#define GETHASHBIN(X, Y) (Y*WIDTH_Y+X) 
//here we have state coord: <X1, X2, X3, X4>
unsigned int EnvironmentNAV4DXYTG::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int Timet, unsigned int G)
{

	return inthash(inthash(X1)+(inthash(X2)<<1)+(inthash(Timet)<<2)+(inthash(G)<<3)) & (EnvNAV4DXYTG.HashTableSize-1);
}



void EnvironmentNAV4DXYTG::PrintHashTableHist()
{
	int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;

	for(int  j = 0; j < EnvNAV4DXYTG.HashTableSize; j++)
	{
		if((int)EnvNAV4DXYTG.Coord2StateIDHashTable[j].size() == 0)
			s0++;
		else if((int)EnvNAV4DXYTG.Coord2StateIDHashTable[j].size() < 50)
			s1++;
		else if((int)EnvNAV4DXYTG.Coord2StateIDHashTable[j].size() < 100)
			s50++;
		else if((int)EnvNAV4DXYTG.Coord2StateIDHashTable[j].size() < 200)
			s100++;
		else if((int)EnvNAV4DXYTG.Coord2StateIDHashTable[j].size() < 300)
			s200++;
		else if((int)EnvNAV4DXYTG.Coord2StateIDHashTable[j].size() < 400)
			s300++;
		else
			slarge++;
	}
	printf("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d\n",
		s0,s1, s50, s100, s200,s300,slarge);
}


EnvNAV4DXYTGHashEntry_t* EnvironmentNAV4DXYTG::GetHashEntry(int X, int Y, int Timet, int G, complex<double> Lval, bool isGoal)
{
	int a;
	bool isNewExplore=false;
	int MaxExploreCount = EnvNAV4DXYTGCfg.GlobalParams.EXPLORE_HOMOTOPY_CLASSES;

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

	// if isGoal, just return goal state. If goal state has not been initiated, return NULL
	if (isGoal)
		return EnvNAV4DXYTG.CopyOfGoalState;
	
//	printf("++ state: %d,%d,%d,%d,(%f,%f),%d\n", X,Y,Timet,G,Lval.real(),Lval.imag(),isGoal);
	// Check if the state (X,Y,Timet,G) is a goal - check for permitted LVals
	bool isThisTheGoal = false;
	if (EnvNAV4DXYTG.CopyOfGoalState!=NULL 
			&& EnvNAV4DXYTG.CopyOfGoalState->X==X 
			&& EnvNAV4DXYTG.CopyOfGoalState->Y==Y 
			&& EnvNAV4DXYTG.CopyOfGoalState->Timet==Timet 
			&& EnvNAV4DXYTG.CopyOfGoalState->G==G)
	{
		isThisTheGoal = true;
		//printf("\n++++Possible goal state query: %d, %d\n", EnvNAV4DXYTGCfg.RobotHomotopyInfo.BlockedHomotopyClass_LVals.size(), EnvNAV4DXYTGCfg.RobotHomotopyInfo.ConstrainHomotopyClass_LVals.size());
		
		
		for (a=0; a<EnvNAV4DXYTGCfg.RobotHomotopyInfo.BlockedHomotopyClass_LVals.size(); a++)
			if (abs(EnvNAV4DXYTGCfg.RobotHomotopyInfo.BlockedHomotopyClass_LVals[a]-Lval) < LVAL_EQUAL_THRESH)
			{
				isThisTheGoal = false;
				break;
			}
			
		if (isThisTheGoal)
			for (a=0; a<EnvNAV4DXYTGCfg.RobotHomotopyInfo.ConstrainHomotopyClass_LVals.size(); a++)
				if (abs(EnvNAV4DXYTGCfg.RobotHomotopyInfo.ConstrainHomotopyClass_LVals[a]-Lval) < LVAL_EQUAL_THRESH)
				{
					isThisTheGoal = true;
					break;
				}
		
		
		if (MaxExploreCount>0) {
		
			isThisTheGoal = false;
			// Save this state/path
			if (ExplorationInfo.ExploredLVals.size()<MaxExploreCount && (ExplorationInfo.ExploredLVals.size()==0 || abs(ExplorationInfo.ExploredLVals[ExplorationInfo.ExploredLVals.size()-1]-Lval)>=LVAL_EQUAL_THRESH) )
			{
				isNewExplore = true;
				for (a=ExplorationInfo.ExploredLVals.size()-1; a>=0; a--)
					if (abs(ExplorationInfo.ExploredLVals[a]-Lval)<LVAL_EQUAL_THRESH) {
						isNewExplore = false;
						break;
					}
				if (isNewExplore) {
					printf("----@@@@ A new goal state (%d,%d,%d,%d) expanded: LVal = %f+%fi ; time = %f ; expansions = %d \n", X,Y,Timet,G, Lval.real(),Lval.imag(), ((float)(clock()-StartTime))/CLOCKS_PER_SEC , ARAPlannerPointer->expands );	
					ExplorationInfo.ExploredLVals.push_back(Lval);
					ExplorationInfo.ExplorationTime.push_back(((float)(clock()-StartTime))/CLOCKS_PER_SEC);
					ExplorationInfo.ExplorationExpansions.push_back(ARAPlannerPointer->expands);
				}
			}
		
		
			if (ExplorationInfo.ExploredLVals.size()==MaxExploreCount && abs(ExplorationInfo.ExploredLVals[MaxExploreCount-1]-Lval)<LVAL_EQUAL_THRESH)
			{
				isThisTheGoal = true;
				printf("Time:\n");
				for (a=0; a<ExplorationInfo.ExploredLVals.size(); a++)  printf("%f\n", ExplorationInfo.ExplorationTime[a]);
				printf("Expansions:\n");
				for (a=0; a<ExplorationInfo.ExploredLVals.size(); a++)  printf("%d\n", ExplorationInfo.ExplorationExpansions[a]);
			}
		
		}
		
		
		if (isThisTheGoal)
		{
			EnvNAV4DXYTG.CopyOfGoalState->Lval = Lval;
			return EnvNAV4DXYTG.CopyOfGoalState;
		}
		
	}
	

	int binid = GETHASHBIN(X, Y, Timet, G);
	

	//iterate over the states in the bin and select the perfect match
	for(int ind = 0; ind < (int)EnvNAV4DXYTG.Coord2StateIDHashTable[binid].size(); ind++)
	{
	
		if( EnvNAV4DXYTG.Coord2StateIDHashTable[binid][ind]->X == X 
			&& EnvNAV4DXYTG.Coord2StateIDHashTable[binid][ind]->Y == Y
			&& EnvNAV4DXYTG.Coord2StateIDHashTable[binid][ind]->Timet == Timet
			&& EnvNAV4DXYTG.Coord2StateIDHashTable[binid][ind]->G == G
			&& ((EnvNAV4DXYTGCfg.RobotHomotopyInfo.isActive() && abs(EnvNAV4DXYTG.Coord2StateIDHashTable[binid][ind]->Lval-Lval) <= LVAL_EQUAL_THRESH) || !EnvNAV4DXYTGCfg.RobotHomotopyInfo.isActive()) )
		{
			return EnvNAV4DXYTG.Coord2StateIDHashTable[binid][ind];
		}
	}

	
	
	return NULL;	  
}


EnvNAV4DXYTGHashEntry_t* EnvironmentNAV4DXYTG::CreateNewHashEntry(int X, int Y, int Timet, int G, complex<double> Lval, bool isGoal) 
{
	int i;

#if TIME_DEBUG	
	clock_t currenttime = clock();
#endif

	EnvNAV4DXYTGHashEntry_t* HashEntry = new EnvNAV4DXYTGHashEntry_t;

	HashEntry->X = X;
	HashEntry->Y = Y;
	HashEntry->Timet = Timet;
	HashEntry->G = G;
	HashEntry->Lval = Lval;

	HashEntry->stateID = EnvNAV4DXYTG.StateID2CoordTable.size();

	//insert into the tables
	EnvNAV4DXYTG.StateID2CoordTable.push_back(HashEntry);
	// If it's a Goal state, record that
	if (isGoal)
	{
		EnvNAV4DXYTG.CopyOfGoalState = HashEntry;
		printf("++ Setting goal at ID=%d: X=%d, Y=%d, Timet=%d, G=%d\n", EnvNAV4DXYTG.CopyOfGoalState->stateID, EnvNAV4DXYTG.CopyOfGoalState->X, EnvNAV4DXYTG.CopyOfGoalState->Y, EnvNAV4DXYTG.CopyOfGoalState->Timet, EnvNAV4DXYTG.CopyOfGoalState->G);
	}

	//get the hash table bin
	i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Timet, HashEntry->G); 

	//insert the entry into the bin
	EnvNAV4DXYTG.Coord2StateIDHashTable[i].push_back(HashEntry);

	//insert into and initialize the mappings
	int* entry = new int [NUMOFINDICES_STATEID2IND];
	StateID2IndexMapping.push_back(entry);
	for(i = 0; i < NUMOFINDICES_STATEID2IND; i++)
	{
		StateID2IndexMapping[HashEntry->stateID][i] = -1;
	}

	if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
	{
		printf("ERROR in Env... function: last state has incorrect stateID: HashEntry->stateID=%d ; StateID2IndexMapping.size()-1=%d\n", 
						HashEntry->stateID, (int)StateID2IndexMapping.size()-1);
		exit(1);	
	}

#if TIME_DEBUG
	time_createhash += clock()-currenttime;
#endif

	return HashEntry;
}



void EnvironmentNAV4DXYTG::InitializeEnvironment()
{
	EnvNAV4DXYTGHashEntry_t* HashEntry;

	//initialize the map from Coord to StateID
	EnvNAV4DXYTG.HashTableSize = 512*1024; //64 should be power of two
	EnvNAV4DXYTG.Coord2StateIDHashTable = new vector<EnvNAV4DXYTGHashEntry_t*>[EnvNAV4DXYTG.HashTableSize];
	
	//printf("AAAAAA: %x ; size=%d \n", EnvNAV4DXYTG.Coord2StateIDHashTable, (int)sizeof(EnvNAV4DXYTG.Coord2StateIDHashTable)/sizeof(vector<EnvNAV4DXYTGHashEntry_t*>));
	
	//initialize the map from StateID to Coord
	EnvNAV4DXYTG.StateID2CoordTable.clear();

	//create start state 
	HashEntry = CreateNewHashEntry(EnvNAV4DXYTGCfg.StartX_c, EnvNAV4DXYTGCfg.StartY_c, EnvNAV4DXYTGCfg.StartTimet_c, EnvNAV4DXYTGCfg.StartG_c, complex<double>(0.0,0.0));
	EnvNAV4DXYTG.startstateid = HashEntry->stateID;

	//create goal state 
	HashEntry = CreateNewHashEntry(EnvNAV4DXYTGCfg.EndX_c, EnvNAV4DXYTGCfg.EndY_c, EnvNAV4DXYTGCfg.EndTimet_c, EnvNAV4DXYTGCfg.EndG_c, complex<double>(), true);
	EnvNAV4DXYTG.goalstateid = HashEntry->stateID;
}

// ========================================================================================
// ========================================================================================

EnvironmentNAV4DXYTG::~EnvironmentNAV4DXYTG()
{
	int a, b;
	
	if (this)
	{
		if (EnvNAV4DXYTG.grid2DsearchFwd)
			delete EnvNAV4DXYTG.grid2DsearchFwd;
		if (EnvNAV4DXYTG.grid2DsearchBak)
			delete EnvNAV4DXYTG.grid2DsearchBak;
		if (EnvNAV4DXYTG.grid2DsearchFwdTasks)
			delete EnvNAV4DXYTG.grid2DsearchFwdTasks;
		if (EnvNAV4DXYTG.Coord2StateIDHashTable)
		{
			//printf("MMMMMM: %x ; size=%d \n", EnvNAV4DXYTG.Coord2StateIDHashTable, (int)sizeof(EnvNAV4DXYTG.Coord2StateIDHashTable)/sizeof(vector<EnvNAV4DXYTGHashEntry_t*>));
			for (a=0; a<(int)sizeof(EnvNAV4DXYTG.Coord2StateIDHashTable)/sizeof(vector<EnvNAV4DXYTGHashEntry_t*>); a++)
				for (b=0; b<EnvNAV4DXYTG.Coord2StateIDHashTable[a].size(); b++)
					delete EnvNAV4DXYTG.Coord2StateIDHashTable[a][b];
				//printf("MMM: bin %d contains %d elements \n", a, EnvNAV4DXYTG.Coord2StateIDHashTable[a].size());
			delete[] EnvNAV4DXYTG.Coord2StateIDHashTable;
		}
		
		for (b=0; b<EnvNAV4DXYTG.StateID2CoordTable.size(); b++)
			delete EnvNAV4DXYTG.StateID2CoordTable[b];
    }
}  // MEM_CLEAR

void EnvironmentNAV4DXYTG::SetConfiguration_all(int width, int height, int maxTime,
					int startx, int starty, int startTimet, int startg,
					int goalx, int goaly, int goalTimet, int goalg,
					double cellsize_m, double timestepsize_m,
					vector<EnvNAV4DXYTG_pos_trajectory> otherBots_trajectories,
					vector<EnvNAV4DXYTG_dist_trajectory> distConstraint_trajectories,
					vector<float> penaltyWeights) {

  EnvNAV4DXYTGCfg.EnvWidth_c = width;
  EnvNAV4DXYTGCfg.EnvHeight_c = height;
  EnvNAV4DXYTGCfg.EnvMaxTime_c = maxTime;
  EnvNAV4DXYTGCfg.StartX_c = startx;
  EnvNAV4DXYTGCfg.StartY_c = starty;
  EnvNAV4DXYTGCfg.StartTimet_c = startTimet;
  EnvNAV4DXYTGCfg.StartG_c = startg;
 
  if(EnvNAV4DXYTGCfg.StartX_c < 0 || EnvNAV4DXYTGCfg.StartX_c >= EnvNAV4DXYTGCfg.EnvWidth_c) {
    printf("ERROR: illegal start coordinates\n");
    exit(1);
  }
  if(EnvNAV4DXYTGCfg.StartY_c < 0 || EnvNAV4DXYTGCfg.StartY_c >= EnvNAV4DXYTGCfg.EnvHeight_c) {
    printf("ERROR: illegal start coordinates\n");
    exit(1);
  }
  if(EnvNAV4DXYTGCfg.StartTimet_c < 0 || EnvNAV4DXYTGCfg.StartTimet_c >= EnvNAV4DXYTGCfg.EnvMaxTime_c) {
    printf("ERROR: illegal start coordinates for Timet\n");
    exit(1);
  }
  
  EnvNAV4DXYTGCfg.EndX_c = goalx;
  EnvNAV4DXYTGCfg.EndY_c = goaly;
  EnvNAV4DXYTGCfg.EndTimet_c = goalTimet;
  EnvNAV4DXYTGCfg.EndG_c = goalg;

  if(EnvNAV4DXYTGCfg.EndX_c < 0 || EnvNAV4DXYTGCfg.EndX_c >= EnvNAV4DXYTGCfg.EnvWidth_c) {
    printf("ERROR: illegal goal coordinates\n");
    exit(1);
  }
  if(EnvNAV4DXYTGCfg.EndY_c < 0 || EnvNAV4DXYTGCfg.EndY_c >= EnvNAV4DXYTGCfg.EnvHeight_c) {
    printf("ERROR: illegal goal coordinates\n");
    exit(1);
  }
  if(EnvNAV4DXYTGCfg.EndTimet_c < 0 || EnvNAV4DXYTGCfg.EndTimet_c >= EnvNAV4DXYTGCfg.EnvMaxTime_c) {
    printf("ERROR: illegal goal coordinates for Timet\n");
    exit(1);
  }

  EnvNAV4DXYTGCfg.cellsize_m = cellsize_m;
  EnvNAV4DXYTGCfg.timestepsize_m = timestepsize_m;

  EnvNAV4DXYTGCfg.otherBots_trajectories = otherBots_trajectories;
  EnvNAV4DXYTGCfg.distConstraint_trajectories = distConstraint_trajectories;
  EnvNAV4DXYTGCfg.penaltyWeights = penaltyWeights;
  if(EnvNAV4DXYTGCfg.otherBots_trajectories.size() != EnvNAV4DXYTGCfg.distConstraint_trajectories.size() || 
		EnvNAV4DXYTGCfg.otherBots_trajectories.size() != EnvNAV4DXYTGCfg.penaltyWeights.size() ) {
    printf("ERROR: number of trajectories, constraints and weights must be equal!\n");
    exit(1);
  }

}

/*
void EnvironmentNAV4DXYTG::SetConfiguration_constraints(vector<int> otherBots_identifiers,
					vector<EnvNAV4DXYTG_pos_trajectory> otherBots_trajectories,
					vector<EnvNAV4DXYTG_dist_trajectory> distConstraint_trajectories,
					vector<float> penaltyWeights) {

  EnvNAV4DXYTGCfg.otherBots_identifiers = otherBots_identifiers;
  EnvNAV4DXYTGCfg.otherBots_trajectories = otherBots_trajectories;
  EnvNAV4DXYTGCfg.distConstraint_trajectories = distConstraint_trajectories;
  EnvNAV4DXYTGCfg.penaltyWeights = penaltyWeights;
  if(EnvNAV4DXYTGCfg.otherBots_trajectories.size() != EnvNAV4DXYTGCfg.distConstraint_trajectories.size() || 
		EnvNAV4DXYTGCfg.otherBots_trajectories.size() != EnvNAV4DXYTGCfg.penaltyWeights.size() ) {
    printf("ERROR: number of trajectories, constraints and weights must be equal!\n");
    exit(1);
  }

}
*/


// This function should ideally go separate
void EnvironmentNAV4DXYTG::ReadConfiguration(FILE* fCfg)
{
	//read in the configuration of environment and initialize  EnvNAV4DXYTGCfg structure
	char sTemp[1024], sTemp1[1024];
	int dTemp;
	int X, Y, a;
	int time_level, patch_size, newActionIndex;
	int dTimet, dX, dY;

	//discretization(cells)
	fscanf(fCfg, "%s", sTemp);
	strcpy(sTemp1, "discretization(cells):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		printf("ERROR: configuration file has incorrect format\n");
		printf("Expected %s got %s\n", sTemp1, sTemp);
		exit(1);
	}
	fscanf(fCfg, "%s", sTemp);
	EnvNAV4DXYTGCfg.EnvWidth_c = atoi(sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV4DXYTGCfg.EnvHeight_c = atoi(sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV4DXYTGCfg.EnvMaxTime_c = atoi(sTemp);

	//cellsize
	fscanf(fCfg, "%s", sTemp);
	strcpy(sTemp1, "cellsize(meters,seconds):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		printf("ERROR: configuration file has incorrect format\n");
		printf("Expected %s got %s\n", sTemp1, sTemp);
		exit(1);
	}
	fscanf(fCfg, "%s", sTemp);
	EnvNAV4DXYTGCfg.cellsize_m = atof(sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV4DXYTGCfg.timestepsize_m = atof(sTemp);
	
	#if DEBUG2
		printf("In environment_nav4Dxytg.cpp-L325 : cellsize_m=%f , timestepsize_m=%f\n", EnvNAV4DXYTGCfg.cellsize_m, EnvNAV4DXYTGCfg.timestepsize_m);
	#endif

	//start(meters,secs): 
	fscanf(fCfg, "%s", sTemp);
	strcpy(sTemp1, "start(meters,secs):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		printf("ERROR: configuration file has incorrect format\n");
		printf("Expected %s got %s\n", sTemp1, sTemp);
		exit(1);
	}
	fscanf(fCfg, "%s", sTemp);
	EnvNAV4DXYTGCfg.StartX_c = CONTXY2DISC(atof(sTemp),EnvNAV4DXYTGCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV4DXYTGCfg.StartY_c = CONTXY2DISC(atof(sTemp),EnvNAV4DXYTGCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV4DXYTGCfg.StartTimet_c = CONTXY2DISC(atof(sTemp),EnvNAV4DXYTGCfg.timestepsize_m);

	#if DEBUG2
		printf("In environment_nav4Dxytg.cpp-L345 : StartX_c=%d , StartY_c=%d, StartTimet_c=%d\n", 
										EnvNAV4DXYTGCfg.StartX_c, EnvNAV4DXYTGCfg.StartY_c, EnvNAV4DXYTGCfg.StartTimet_c);
	#endif

	if(EnvNAV4DXYTGCfg.StartX_c < 0 || EnvNAV4DXYTGCfg.StartX_c >= EnvNAV4DXYTGCfg.EnvWidth_c)
	{
		printf("ERROR: illegal start coordinates\n");
		exit(1);
	}
	if(EnvNAV4DXYTGCfg.StartY_c < 0 || EnvNAV4DXYTGCfg.StartY_c >= EnvNAV4DXYTGCfg.EnvHeight_c)
	{
		printf("ERROR: illegal start coordinates\n");
		exit(1);
	}
	if(EnvNAV4DXYTGCfg.StartTimet_c < 0 || EnvNAV4DXYTGCfg.StartTimet_c >= EnvNAV4DXYTGCfg.EnvMaxTime_c)
	{
		printf("ERROR: illegal start coordinates for Timet\n");
		exit(1);
	}

	//end(meters,secs): 
	fscanf(fCfg, "%s", sTemp);
	strcpy(sTemp1, "end(meters,secs):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		printf("ERROR: configuration file has incorrect format\n");
		printf("Expected %s got %s\n", sTemp1, sTemp);
		exit(1);
	}
	fscanf(fCfg, "%s", sTemp);
	EnvNAV4DXYTGCfg.EndX_c = CONTXY2DISC(atof(sTemp),EnvNAV4DXYTGCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV4DXYTGCfg.EndY_c = CONTXY2DISC(atof(sTemp),EnvNAV4DXYTGCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV4DXYTGCfg.EndTimet_c = CONTXY2DISC(atof(sTemp), EnvNAV4DXYTGCfg.timestepsize_m);
	
	#if DEBUG2
		printf("In environment_nav4Dxytg.cpp-L345 : EndX_c=%d , EndY_c=%d, EndTimet_c=%d\n", 
										EnvNAV4DXYTGCfg.EndX_c, EnvNAV4DXYTGCfg.EndY_c, EnvNAV4DXYTGCfg.EndTimet_c);
	#endif

	if(EnvNAV4DXYTGCfg.EndX_c < 0 || EnvNAV4DXYTGCfg.EndX_c >= EnvNAV4DXYTGCfg.EnvWidth_c)
	{
		printf("ERROR: illegal end coordinates\n");
		exit(1);
	}
	if(EnvNAV4DXYTGCfg.EndY_c < 0 || EnvNAV4DXYTGCfg.EndY_c >= EnvNAV4DXYTGCfg.EnvHeight_c)
	{
		printf("ERROR: illegal end coordinates\n");
		exit(1);
	}
	if(EnvNAV4DXYTGCfg.EndTimet_c < 0 || EnvNAV4DXYTGCfg.EndTimet_c >= EnvNAV4DXYTGCfg.EnvMaxTime_c)
	{
		printf("ERROR: illegal goal coordinates for Timet\n");
		exit(1);
	}
	#if DEBUG2
		printf("In environment_nav4Dxytg.cpp-L390 : Workspace configuration, Initial and goal states read!\n");
	#endif


	for (a = 0; a < 10 ; a++)
	{

		fscanf(fCfg, "%s", sTemp);
		strcpy(sTemp1, "-END-");
		if(strcmp(sTemp1, sTemp) == 0)
		{
			break;
		}
		
		#if DEBUG2
			//printf("In environment_nav4Dxytg.cpp-L405 : Just read - %s !\n", sTemp);
		#endif

		strcpy(sTemp1, "connectivity(time_level,patch_size):");
		if(strcmp(sTemp1, sTemp) != 0)
		{
			printf("ERROR: configuration file has incorrect format\n");
			printf("Expected %s got %s\n", sTemp1, sTemp);
			exit(1);
		}

		fscanf(fCfg, "%s", sTemp);
		time_level = atoi(sTemp);

		fscanf(fCfg, "%s", sTemp);
		patch_size = atoi(sTemp);
		
		#if DEBUG2
			//printf("In environment_nav4Dxytg.cpp-L423 : Just read - %d, %d !\n", time_level, patch_size);
		#endif

		//connectivity:
		strcpy(sTemp1, "1");
		for (Y = 0; Y < patch_size; Y++)
			for (X = 0; X < patch_size; X++)
			{
				fscanf(fCfg, "%s", sTemp);
				if(strcmp(sTemp1, sTemp) == 0)
				{
					dTimet = time_level;
					dX = X - (patch_size-1)/2;
					dY = Y - (patch_size-1)/2;
					newActionIndex = EnvNAV4DXYTGCfg.ActionsV.size();
					EnvNAV4DXYTGAction_t* TmpActionsV = new EnvNAV4DXYTGAction_t;
					TmpActionsV->dTimet = dTimet;
					TmpActionsV->dX = dX;
					TmpActionsV->dY = dY;
					TmpActionsV->cost = TRANSITIONCOST_XYT(dX, dY, dTimet);
						
					if (TmpActionsV->cost == 0)
						printf("WARNING: Zero transition cost!!!\n");
						
					EnvNAV4DXYTGCfg.ActionsV.push_back(*TmpActionsV);
					#if DEBUG2
						printf("In environment_nav4Dxytg.cpp-L439 : New action read : dX=%d, dY=%d, dTimet=%d, cost=%d\n", 
								TmpActionsV->dX, TmpActionsV->dY, TmpActionsV->dTimet, TmpActionsV->cost);
					#endif
				}
			}
	}
}

// ========================================================================================
// ========================================================================================

bool CentralizedInfo_t::isPointInJointStatespaceObstacles(JointStatespacePoint_t pt, int flag, int radi, bool debug)
{
	int a, b, c;
	bool isMatch, isThisPlanningRobotsInThisConstraint, isAllPlanningRobotsInThisConstraint;
	int EucDistSq;
	vector<JointStatespacePoint_t>* JointStatespaceObstacles_toSearch;
	
	if (flag==0)
		JointStatespaceObstacles_toSearch = &JointStatespaceObstacles_temp;
	else if (flag==1)
		JointStatespaceObstacles_toSearch = &JointStatespaceObstacles;
	
		
	for (a=0; a<JointStatespaceObstacles_toSearch->size(); a++)
	{
		if (pt.Timet != JointStatespaceObstacles_toSearch->at(a).Timet)
			continue;
		
		if (pt.ViolatingRobotIndices.size()>0)
		{
			isAllPlanningRobotsInThisConstraint = true;
			for (b=0; b<pt.ViolatingRobotIndices.size(); b++)
			{
				isThisPlanningRobotsInThisConstraint = false;
				for (c=0; c<JointStatespaceObstacles_toSearch->at(a).ViolatingRobotIndices.size(); c++)
					if (pt.ViolatingRobotIndices[b]==JointStatespaceObstacles_toSearch->at(a).ViolatingRobotIndices[c])
					{
						isThisPlanningRobotsInThisConstraint = true;
						break;
					}
				isAllPlanningRobotsInThisConstraint = isAllPlanningRobotsInThisConstraint & isThisPlanningRobotsInThisConstraint;
				if (!isAllPlanningRobotsInThisConstraint)
					break;
			}
			if (!isAllPlanningRobotsInThisConstraint)
				continue;
		}
			
		if (radi==0)
		{
			isMatch = true;
			for (b=0; b<JointStatespaceObstacles_toSearch->at(a).RobotStateX.size(); b++)
				if (pt.RobotStateX[b]!=JointStatespaceObstacles_toSearch->at(a).RobotStateX[b] || pt.RobotStateY[b]!=JointStatespaceObstacles_toSearch->at(a).RobotStateY[b])
				{
					isMatch = false;
					break;
				}
		}
		else
		{
			int EucDistSq = 0;
			for (b=0; b<JointStatespaceObstacles_toSearch->at(a).RobotStateX.size(); b++)
			{
				if (JointStatespaceObstacles_toSearch->at(a).RobotStateX[b]<0 || JointStatespaceObstacles_toSearch->at(a).RobotStateY[b]<0)
					continue;
				
				EucDistSq = EucDistSq + (int)(pow((float)(pt.RobotStateX[b]-JointStatespaceObstacles_toSearch->at(a).RobotStateX[b]),2)) + 
											(int)(pow((float)(pt.RobotStateY[b]-JointStatespaceObstacles_toSearch->at(a).RobotStateY[b]),2));
				if (debug)
					printf("\nDiff: %d, %d ; Pows: %d, %d", pt.RobotStateX[b]-JointStatespaceObstacles_toSearch->at(a).RobotStateX[b], 
							pt.RobotStateY[b]-JointStatespaceObstacles_toSearch->at(a).RobotStateY[b], (int)(pow((float)(pt.RobotStateX[b]-JointStatespaceObstacles_toSearch->at(a).RobotStateX[b]),2)), 
											(int)(pow((float)(pt.RobotStateY[b]-JointStatespaceObstacles_toSearch->at(a).RobotStateY[b]),2)));
			}
			if (debug)
				printf(" &&& EucDistSq , ((int)sqrt((double)EucDistSq)) , radi : %d, %d, %d\n", EucDistSq, ((int)sqrt((float)(EucDistSq))), radi);
			if (((int)(sqrt((double)(EucDistSq)))) <= radi)
				isMatch = true;
			else
				isMatch = false;
		}
		
		if (isMatch)
		{
			//printf("\n *** 6D pt obstacle just checked and found:");
			//for (b=0; b<pt.RobotStateX.size(); b++)
			//	printf(" %d %d,", pt.RobotStateX[b], pt.RobotStateY[b]);
			return true;
		}
	}
	
	return false;
}


int CentralizedInfo_t::AddObstaclePointsToJointStatespace(vector<EnvNAV4DXYTG_pos_trajectory> posTrajs, 
												vector<EnvNAV4DXYTG_AParticularConstraint_t> theConstraints, 
												int TimetStart, int TimetEnd, bool isMethod2)
{
	int a, b, c, t;
	int DX, DY, DT;
	int ViolationCount = 0;
	vector<int> ViolatingRobotIndices;
	bool isObstacle;
	
	//EnvNAV4DXYTG_pos_t* tempPos;
	//EnvNAV4DXYTG_pos_trajectory* tempPosTraj;
	EnvNAV4DXYTG_dist_t* InterpedDist = new EnvNAV4DXYTG_dist_t;
	JointStatespacePoint_t JointStatespacePoint, JointStatespacePoint_BallPt;
	JointStatespacePoint.RobotStateX.resize(posTrajs.size(), -1);
	JointStatespacePoint.RobotStateY.resize(posTrajs.size(), -1);
	//JointStatespacePoint_BallPt = JointStatespacePoint;
	
	/*
	vector<int> BallDims;
	for (a=0; a<2*posTrajs.size(); a++)
		BallDims.push_back((radi-1)*2 + 1);
	IndexTracker ballTracker(BallDims);
	printf("&&&&&&&& %d\n", ballTracker.tracker_max);
	*/
	
	//EnvNAV4DXYTG_dist_t* InterpedDist;
	vector<EnvNAV4DXYTG_pos_t> NowPoses(posTrajs.size());
	
	//for (int m=0; m<=theConstraints[0].constraint.dist_t.size(); m++)
	//	printf("#### Conatrsint pt: Timet=%d, D=%ld\n", theConstraints[0].constraint.dist_t[m].Timet, theConstraints[0].constraint.dist_t[m].D);
	
	for (t = TimetStart; t <= TimetEnd; t++)
	{
	for (c=0; c<theConstraints.size(); c++)
	{
		ViolatingRobotIndices.clear();
		JointStatespacePoint.RobotStateX.clear();
		JointStatespacePoint.RobotStateY.clear();
		JointStatespacePoint.RobotStateX.resize(posTrajs.size(), -1);
		JointStatespacePoint.RobotStateY.resize(posTrajs.size(), -1);
		
		for (a=0; a<posTrajs.size(); a++)
		{
			//*tempPosTraj = posTraj[a];
			//FindAndInterpInTrajectory(tempPosTraj, t, tempPos);
			//NowPoses[a] = *tempPos;
			//tempPos = new EnvNAV4DXYTG_pos_t;
			FindAndInterpInTrajectory(&posTrajs[a], t, &NowPoses[a]);
			//NowPoses[a] = *tempPos;
		}
		
		isObstacle = false;
		//ViolatingRobotIndices.clear();
		//for (c=0; c<theConstraints.size(); c++)
		//{
			FindAndInterpInTrajectory(&theConstraints[c].constraint, t, InterpedDist);
			DX = NowPoses[theConstraints[c].Robot1].X - NowPoses[theConstraints[c].Robot2].X;
			DY = NowPoses[theConstraints[c].Robot1].Y - NowPoses[theConstraints[c].Robot2].Y;
			DT = NowPoses[theConstraints[c].Robot1].Timet - NowPoses[theConstraints[c].Robot2].Timet;
			//printf("ViolationCount computation: Constraint=%d, time=%d, robots=%d,%d ; (%d,%d)-(%d,%d) ; %ld\n", c, t, theConstraints[c].Robot1, theConstraints[c].Robot2, NowPoses[theConstraints[c].Robot1].X, NowPoses[theConstraints[c].Robot1].Y, NowPoses[theConstraints[c].Robot2].X, NowPoses[theConstraints[c].Robot2].Y, InterpedDist->D);
			if (CELLCOST_XYT(DX, DY, DT, InterpedDist->D) > 0)
			{
				//printf("ViolationCount recorded: Constraint=%d, time=%d, robots=%d,%d\n", c, t, theConstraints[c].Robot1, theConstraints[c].Robot2);
				isObstacle = true;
				ViolatingRobotIndices.push_back(theConstraints[c].Robot1);
				ViolatingRobotIndices.push_back(theConstraints[c].Robot2);
				ViolationCount++;
				//break;
			}
		//}
		
		if (isObstacle)
		{
			if (isMethod2)
			{
				for (a=0; a<ViolatingRobotIndices.size(); a++)
				{
					JointStatespacePoint.RobotStateX[ViolatingRobotIndices[a]] = NowPoses[ViolatingRobotIndices[a]].X;
					JointStatespacePoint.RobotStateY[ViolatingRobotIndices[a]] = NowPoses[ViolatingRobotIndices[a]].Y;
				}
			}
			else
			{
				for (a=0; a<posTrajs.size(); a++)
				{
					JointStatespacePoint.RobotStateX[a] = NowPoses[a].X;
					JointStatespacePoint.RobotStateY[a] = NowPoses[a].Y;
				}
			}
			JointStatespacePoint.Timet = t;
			JointStatespacePoint.ViolatingRobotIndices = ViolatingRobotIndices;
			/*
			for (ballTracker.reset(); ballTracker.tracker<=ballTracker.tracker_max; ballTracker.next())
				for (b=0; b<posTrajs.size(); b++)
				{
					JointStatespacePoint_BallPt.RobotStateX[b] = JointStatespacePoint.RobotStateX[b] + ballTracker.index[2*b]-radi+1;
					JointStatespacePoint_BallPt.RobotStateY[b] = JointStatespacePoint.RobotStateY[b] + ballTracker.index[2*b+1]-radi+1;
					if (!isPointInJointStatespaceObstacles(JointStatespacePoint_BallPt,0) && !isPointInJointStatespaceObstacles(JointStatespacePoint_BallPt,1))
						JointStatespaceObstacles_temp.push_back(JointStatespacePoint_BallPt);
				}
			*/
			if (!isPointInJointStatespaceObstacles(JointStatespacePoint,0,0) && !isPointInJointStatespaceObstacles(JointStatespacePoint,1,0))
				JointStatespaceObstacles_temp.push_back(JointStatespacePoint);
		}
	}
	}
	
	delete InterpedDist; // DELETE**
	return ViolationCount;
}


void CentralizedInfo_t::ConcatenateJointStatespaceLists(void)
{
	int a;
	for (a=0; a<JointStatespaceObstacles_temp.size(); a++)
		JointStatespaceObstacles.push_back(JointStatespaceObstacles_temp[a]);
	JointStatespaceObstacles_temp.clear();
}

// ----------------------------------------------------------------------------------------

void CentralizedInfo_t_InitiatePenaltyWeights(CentralizedInfo_t* theCentralInfo, ConfigFileInfo theInfo)
{
	int a, b;
	float sumWeights = 0.0;
	
	switch (theInfo.GlobalParams.PENALTY_WEIGHT_INCREMENT_METHOD)
	{
		case 1:
			for (a=0; a<theInfo.TheConstraints.size(); a++)
			{
				theCentralInfo->PenaltyWeights[a] = theInfo.TheConstraints[a].PenaltyWeightParams[0] - theInfo.TheConstraints[a].PenaltyWeightParams[1];
			}
			break;
				
		case 2:
		
			/*
			theCentralInfo->PenaltyWeights.resize(theInfo.TheConstraints.size());
			for (a=0; a<theInfo.TheConstraints.size(); a++)
			{
				theCentralInfo->PenaltyWeights[a] = theInfo.TheConstraints[a].PenaltyWeightParams[0];
				sumWeights = sumWeights + theInfo.TheConstraints[a].PenaltyWeightParams[0];
			}
			
			theCentralInfo->AveragePenaltyWeights_Robots.resize(theInfo.TheRobots.size());
			for (a=0; a<theInfo.TheRobots.size(); a++)
				theCentralInfo->AveragePenaltyWeights_Robots[a] = sumWeights/((float)theInfo.TheConstraints.size());
			*/
			
			theCentralInfo->PenaltyWeightVectors.resize(theInfo.TheRobots.size());
			for (a=0; a<theInfo.TheRobots.size(); a++)
			{
				theCentralInfo->PenaltyWeightVectors[a].resize(theInfo.TheRobots[a].constraints.size());
				for (b=0; b<theInfo.TheRobots[a].constraints.size(); b++)
					theCentralInfo->PenaltyWeightVectors[a][b] = theInfo.TheRobots[a].constraints[b].PenaltyParams[0];
			}
				
			break;
	}
}

void CentralizedInfo_t_UpdatePenaltyWeights(CentralizedInfo_t* theCentralInfo, ConfigFileInfo theInfo, int ActiveRobot, EnvNAV4DXYTG_pos_trajectory* traj, EnvironmentNAV4DXYTG* env, bool isSummetricWeights)
{
	int a, b, theOtherRobotIndex;
	float theIncrement, PenaltySum=0;
	vector<float> theNewPenaltyVector;
	theCentralInfo->PenaltyWeights.resize(theInfo.TheConstraints.size());
	
	
	switch (theInfo.GlobalParams.PENALTY_WEIGHT_INCREMENT_METHOD)
	{
		case 1:
			for (a=0; a<theInfo.TheConstraints.size(); a++)
				theCentralInfo->PenaltyWeights[a] = theCentralInfo->PenaltyWeights[a] + theInfo.TheConstraints[a].PenaltyWeightParams[1];
			break;
				
		case 2:
			//?theIncrement = FindPenaltyWeightIncrement(theCentralInfo->PenaltyWeights[ActiveRobot], traj, env);
			//?for (a=0; a<theInfo.TheConstraints.size(); a++)
				//?theCentralInfo->PenaltyWeights[a] = theCentralInfo->PenaltyWeights[a] + __max(theIncrement, theInfo.TheConstraints[a].PenaltyWeightParams[2]);
			//?break;
			//theNewPenaltyVector = SuggestNextPenaltyWeight(traj, env);
			//for (a=0; a<theNewPenaltyVector.size(); a++)
			for (a=0; a<theInfo.TheRobots[ActiveRobot].constraints.size(); a++)
			{
				if (theCentralInfo->PenaltyWeightVectors[ActiveRobot][a] <= theInfo.TheRobots[ActiveRobot].constraints[a].PenaltyParams[0]) // || true
				{
					theNewPenaltyVector = SuggestNextPenaltyWeight(traj, env);
					theCentralInfo->PenaltyWeightVectors[ActiveRobot][a] = 
							__max(theNewPenaltyVector[a], theCentralInfo->PenaltyWeightVectors[ActiveRobot][a] + theInfo.TheRobots[ActiveRobot].constraints[a].PenaltyParams[3]);
				}
				else
					theCentralInfo->PenaltyWeightVectors[ActiveRobot][a] = theCentralInfo->PenaltyWeightVectors[ActiveRobot][a] + theInfo.TheRobots[ActiveRobot].constraints[a].PenaltyParams[3];
			}
						
						
			// Making the weights symmetric
			if (isSummetricWeights)
				for (a=0; a<theCentralInfo->PenaltyWeightVectors[ActiveRobot].size(); a++)
				{
					theOtherRobotIndex = theInfo.TheRobots[ActiveRobot].constraints[a].RobotIndex;
					for (b=0; b<theCentralInfo->PenaltyWeightVectors[theOtherRobotIndex].size(); b++)
						if (theInfo.TheRobots[theOtherRobotIndex].constraints[b].RobotIndex == ActiveRobot)
								theCentralInfo->PenaltyWeightVectors[theOtherRobotIndex][b] = 
									__max(theCentralInfo->PenaltyWeightVectors[ActiveRobot][a], theCentralInfo->PenaltyWeightVectors[theOtherRobotIndex][b]);
				}
						
			//theNewPenalty = __max(theNewPenalty, theCentralInfo->PenaltyWeights[ActiveRobot]+
			/*
			for (a=0; a<theInfo.TheConstraints.size(); a++)
			{
				theCentralInfo->PenaltyWeights[a] = __max(theNewPenalty, theCentralInfo->AveragePenaltyWeights_Robots[ActiveRobot]+theInfo.TheConstraints[a].PenaltyWeightParams[2]);
				PenaltySum = PenaltySum + theCentralInfo->PenaltyWeights[a];
			}
			theCentralInfo->AveragePenaltyWeights_Robots[ActiveRobot] = PenaltySum / ((float)theInfo.TheConstraints.size());
				//theCentralInfo->PenaltyWeights[a] = theNewPenalty;
			*/
				
			break;
	}
}

// ========================================================================================

void EvaluateTrajectoryChange(EnvNAV4DXYTG_pos_trajectory* newTraj, EnvNAV4DXYTG_pos_trajectory* oldTraj, EnvironmentNAV4DXYTG* env, vector<int>* ConstraintNegociationFlag, float thresh)
{
	// Returns: -1 - no change at all, 0 - all changes within threshold, 1 - any change above threshold
	int a, b;
	float diff;
	vector< vector<int> > ConstraintViolationIndices;
	vector<int> CcOld;
	vector<int> CcNew;
	vector<int> ThisConstraintCostVector;
	
	//ComputeTrajectoryCost(oldTraj, env, &CpOld, &CcOld, &ConstraintViolationIndices);
	ConstraintNegociationFlag->clear();
	ConstraintNegociationFlag->resize(env->EnvNAV4DXYTGCfg.penaltyWeights.size(), -1);
	
	// This is a hack assuming that the trajectories are of equal length
	bool isDifferent = false;
	for (a=0; a < oldTraj->pos_t.size(); a++)
	{
		if (oldTraj->pos_t[a].X!=newTraj->pos_t[a].X || 
				oldTraj->pos_t[a].Y!=newTraj->pos_t[a].Y || 
				oldTraj->pos_t[a].Timet!=newTraj->pos_t[a].Timet)
		{
			isDifferent = true;
			break;
		}
	}
	if (!isDifferent)
		return;
	
	CcOld.resize(env->EnvNAV4DXYTGCfg.penaltyWeights.size(), 0);
	for (a=0; a < oldTraj->pos_t.size(); a++)
	{
		// Add to constraint violation cost
		//printf("---%d\n",a);
		ThisConstraintCostVector = env->ComputeCellConstraintViolationCost(oldTraj->pos_t[a].X, oldTraj->pos_t[a].Y, oldTraj->pos_t[a].Timet, false);
		if (ThisConstraintCostVector[0] != 0)
			for (b=0; b<env->EnvNAV4DXYTGCfg.penaltyWeights.size(); b++)
				if (ThisConstraintCostVector[b+1]>0)
					CcOld[b] = __max(CcOld[b], ThisConstraintCostVector[b+1]);
	}
	
	CcNew.resize(env->EnvNAV4DXYTGCfg.penaltyWeights.size(), 0);
	for (a=0; a < newTraj->pos_t.size(); a++)
	{
		// Add to constraint violation cost
		ThisConstraintCostVector = env->ComputeCellConstraintViolationCost(newTraj->pos_t[a].X, newTraj->pos_t[a].Y, newTraj->pos_t[a].Timet, false);
		if (ThisConstraintCostVector[0] != 0)
			for (b=0; b<env->EnvNAV4DXYTGCfg.penaltyWeights.size(); b++)
				if (ThisConstraintCostVector[b+1]>0)
					CcNew[b] = __max(CcNew[b], ThisConstraintCostVector[b+1]);
	}
	
	for (b=0; b<env->EnvNAV4DXYTGCfg.penaltyWeights.size(); b++)
	{
		diff = ((float)(CcOld[b]-CcNew[b])) / NAV4DXYTG_COSTMULT;
		if (diff==0)
			ConstraintNegociationFlag->at(b) = -1;
		else if (diff<=thresh)
			ConstraintNegociationFlag->at(b) = 0;
		else
			ConstraintNegociationFlag->at(b) = 1;
	}
}


bool AreTrajectoriesSame(EnvNAV4DXYTG_pos_trajectory* newTraj, EnvNAV4DXYTG_pos_trajectory* oldTraj)
{
	// Returns: true if trajectories are different, false if they are exactly the same
	int a, b;
	
	
	// This is a hack assuming that the trajectories are of equal length
	bool isSame = true;
	for (a=0; a < oldTraj->pos_t.size(); a++)
	{
		if (oldTraj->pos_t[a].X!=newTraj->pos_t[a].X || 
				oldTraj->pos_t[a].Y!=newTraj->pos_t[a].Y || 
				oldTraj->pos_t[a].Timet!=newTraj->pos_t[a].Timet)
		{
			isSame = false;
			break;
		}
	}
	
	return isSame;
}

// ----------------------------------------------------------------------------------------
/*
CanonicalSequence_t DetermineCanonicalSequence(EnvNAV4DXYTG_pos_trajectory posTraj, vector<EnvNAV4DXYTG2Dpt_t> ObstacleCenters)
{
	int a;
	EnvNAV4DXYTG2Dpt_t startPt, endPt;
	
	startPt.x = posTraj.pos_t[0].X;
	startPt.y = posTraj.pos_t[0].Y;
	endPt.x = posTraj.pos_t[posTraj.pos_t.size()-1].X;
	endPt.y = posTraj.pos_t[posTraj.pos_t.size()-1].Y;
	
	for (a=0; a<posTraj.pos_t.size(); a++)
	{
		
	}
}
*/
// ----------------------------------------------------------------------------------------

/*
bool CentralizedInfo_t::isPointInBlockedHomotopyClasses(JointStatespacePoint_t pt)
{
	int p, r, c;
	bool isInThisBlockedClass, foundPoint;
	
	for (c=0; c<BlockedHomotopyClasses.size(); c++)
	{
		isInThisBlockedClass = true;
		for (r=0; r<BlockedHomotopyClasses[c].robot.size(); r++)
		{
			foundPoint = false;
			for (p=0; p<BlockedHomotopyClasses[c].robot[r].x.size(); p++)
			{
				if (BlockedHomotopyClasses[c].robot[r].x[p]==pt.RobotStateX[r] && BlockedHomotopyClasses[c].robot[r].y[p]==pt.RobotStateY[r])
				{
					foundPoint = true;
					break;
				}
			}
			if (!foundPoint)
			{
				isInThisBlockedClass = false;
				break;
			}
		}
		if (isInThisBlockedClass)
			return true;
	}
	return false;
}


int CentralizedInfo_t::AddObstaclePointsToBlockedHomotopyClasses(vector<EnvNAV4DXYTG_pos_trajectory> posTrajs, 
												vector<EnvNAV4DXYTG_AParticularConstraint_t> theConstraints, 
												int TimetStart, int TimetEnd, int radi)
{
	int a, b, t;
	int DX, DY, DT;
	int ViolationCount = 0;
	bool isObstacle;
	
	//EnvNAV4DXYTG_pos_t* tempPos;
	//EnvNAV4DXYTG_pos_trajectory* tempPosTraj;
	EnvNAV4DXYTG_dist_t* InterpedDist = new EnvNAV4DXYTG_dist_t;
	JointStatespacePoint_t JointStatespacePoint, JointStatespacePoint_BallPt;
	JointStatespacePoint.RobotStateX.resize(posTrajs.size());
	JointStatespacePoint.RobotStateY.resize(posTrajs.size());
	JointStatespacePoint_BallPt = JointStatespacePoint;
	
	HomotopyClassBlocker_t thisBlockedHomotopyClass;
	
	//EnvNAV4DXYTG_dist_t* InterpedDist;
	vector<EnvNAV4DXYTG_pos_t> NowPoses(posTrajs.size());
	
	for (t = TimetStart; t <= TimetEnd; t++)
	{
		for (a=0; a<posTrajs.size(); a++)
		{
			//*tempPosTraj = posTraj[a];
			//FindAndInterpInTrajectory(tempPosTraj, t, tempPos);
			//NowPoses[a] = *tempPos;
			//tempPos = new EnvNAV4DXYTG_pos_t;
			FindAndInterpInTrajectory(&posTrajs[a], t, &NowPoses[a]);
			//NowPoses[a] = *tempPos;
		}
		
		isObstacle = false;
		for (a=0; a<theConstraints.size(); a++)
		{
			FindAndInterpInTrajectory(&theConstraints[a].constraint, t, InterpedDist);
			DX = NowPoses[theConstraints[a].Robot1].X - NowPoses[theConstraints[a].Robot2].X;
			DY = NowPoses[theConstraints[a].Robot1].Y - NowPoses[theConstraints[a].Robot2].Y;
			DT = NowPoses[theConstraints[a].Robot1].Timet - NowPoses[theConstraints[a].Robot2].Timet;
			if (CELLCOST_XYT(DX, DY, DT, InterpedDist->D) > 0)
			{
				isObstacle = true;
				ViolationCount++;
				break;
			}
		}
		
		if (isObstacle)
		{
			for (a=0; a<posTrajs.size(); a++)
			{
				JointStatespacePoint.RobotStateX[a] = NowPoses[a].X;
				JointStatespacePoint.RobotStateY[a] = NowPoses[a].Y;
			}
			HomotopyClassBlocker_t thisBlockedHomotopyClass.clear();
			for (b=0; b<posTrajs.size(); b++)
			{
				
				JointStatespacePoint_BallPt.RobotStateX[b] = JointStatespacePoint.RobotStateX[b] + ballTracker.index[2*b]-radi+1;
				JointStatespacePoint_BallPt.RobotStateY[b] = JointStatespacePoint.RobotStateY[b] + ballTracker.index[2*b+1]-radi+1;
				if (!isPointInBlockedHomotopyClasses(JointStatespacePoint_BallPt))
					JointStatespaceObstacles_temp.push_back(JointStatespacePoint_BallPt);
			}
		}
	}
	return ViolationCount;
}
*/

// ----------------------------------------------------------------------------------------


vector<int> EnvironmentNAV4DXYTG::ComputeCellConstraintViolationCost(int X, int Y, int T, bool CheckJointStatespaceObstacle)
{
	// Returns a vector - the element '0' is the sum of all the costs
	int a, DX, DY, DT;
	//int violationCost=0
	vector<int> violationCost;
	int constraintViolation;
	int ConstraintsCount = 0;
	int OtherRobotsCount;
	JointStatespacePoint_t JointStatespacePoint;
	EnvNAV4DXYTG_pos_t* InterpedPos = new EnvNAV4DXYTG_pos_t;
	EnvNAV4DXYTG_dist_t* InterpedDist = new EnvNAV4DXYTG_dist_t;

	OtherRobotsCount = EnvNAV4DXYTGCfg.penaltyWeights.size();
	CheckJointStatespaceObstacle = CheckJointStatespaceObstacle & EnvNAV4DXYTGCfg.GlobalParams.DO_JOINTSTATESPACE_LOGGING;
	
	violationCost.resize(OtherRobotsCount+1, 0);
	
	//if(!CheckJointStatespaceObstacle)
	//	printf("--->> OtherRobotsCount=%d\n", OtherRobotsCount);
	
	if (CheckJointStatespaceObstacle && OtherRobotsCount>0)
	{
		JointStatespacePoint.RobotStateX.resize(OtherRobotsCount+1);
		JointStatespacePoint.RobotStateY.resize(OtherRobotsCount+1);
		JointStatespacePoint.RobotStateX[EnvNAV4DXYTGCfg.BotIdentifier] = X;
		JointStatespacePoint.RobotStateY[EnvNAV4DXYTGCfg.BotIdentifier] = Y;
		JointStatespacePoint.Timet = T;
		if (EnvNAV4DXYTGCfg.GlobalParams.JOINTSTATESPACE_LOGGING_METHOD==1 || EnvNAV4DXYTGCfg.GlobalParams.JOINTSTATESPACE_LOGGING_METHOD==2)
			JointStatespacePoint.ViolatingRobotIndices.push_back(EnvNAV4DXYTGCfg.BotIdentifier); // Just comment this line if 2N-D obstacle points are not specific to any robot
	}

	for (a = 0; a < OtherRobotsCount; a++)
	{

		if (EnvNAV4DXYTGCfg.distConstraint_trajectories[a].dist_t.size() == 0)
			continue;
		
		FindAndInterpInTrajectory(&EnvNAV4DXYTGCfg.otherBots_trajectories[a], T, InterpedPos);
		FindAndInterpInTrajectory(&EnvNAV4DXYTGCfg.distConstraint_trajectories[a], T, InterpedDist);
		
		// Check if the joint state-space point is valid
		if (CheckJointStatespaceObstacle)
		{
			JointStatespacePoint.RobotStateX[EnvNAV4DXYTGCfg.otherBots_identifiers[a]] = InterpedPos->X;
			JointStatespacePoint.RobotStateY[EnvNAV4DXYTGCfg.otherBots_identifiers[a]] = InterpedPos->Y;
		}
		
		// Compute constraint violation cost
		DX = InterpedPos->X - X;
		DY = InterpedPos->Y - Y;
		DT = InterpedPos->Timet - T;
		constraintViolation = CELLCOST_XYT(DX, DY, DT, InterpedDist->D);
		//if (!CheckJointStatespaceObstacle)
		//	printf("[][][] %d, %d, %d - %d, %d, %d - %d\n", InterpedPos->X, InterpedPos->Y, InterpedPos->Timet, X, Y, T, InterpedDist->D);
		if (constraintViolation > 0)
		{
			//if (CheckJointStatespaceObstacle)
			//	violationCost += (int) pow(((float) constraintViolation) * EnvNAV4DXYTGCfg.penaltyWeights[a], EnvNAV4DXYTGCfg.GlobalParams.VIOLATION_COST_POWER);
			//else
			//	violationCost += (int) pow(((float) constraintViolation), EnvNAV4DXYTGCfg.GlobalParams.VIOLATION_COST_POWER);
			if (CheckJointStatespaceObstacle)
				violationCost[a+1] = (int) pow(((float) constraintViolation) * EnvNAV4DXYTGCfg.penaltyWeights[a], EnvNAV4DXYTGCfg.GlobalParams.VIOLATION_COST_POWER);
			else
				violationCost[a+1] = (int) pow(((float) constraintViolation), EnvNAV4DXYTGCfg.GlobalParams.VIOLATION_COST_POWER);
			//violationCost += (int) pow((((float) constraintViolation) * EnvNAV4DXYTGCfg.penaltyWeights[a]),2.0);
			//printf("\nPositive violation cost: %d", (int) pow(((float) constraintViolation) * EnvNAV4DXYTGCfg.penaltyWeights[a], EnvNAV4DXYTGCfg.GlobalParams.VIOLATION_COST_POWER));
		}
			
		if (EnvNAV4DXYTGCfg.distConstraint_trajectories[a].dist_t[T].D < INFINITECOST)
			ConstraintsCount++;
			
		#if DEBUG2
			printf("In environment_nav4Dxytg.cpp-L527 (ComputeCellConstraintViolationCost) : (%d,%d,%d) - (%d,%d,%d) - Cost %d\n", 
					X, Y, T, EnvNAV4DXYTGCfg.otherBots_trajectories[a].pos_t[T].X, EnvNAV4DXYTGCfg.otherBots_trajectories[a].pos_t[T].Y, 
						EnvNAV4DXYTGCfg.otherBots_trajectories[a].pos_t[T].Timet, (int) pow(((float) constraintViolation) * EnvNAV4DXYTGCfg.penaltyWeights[a], EnvNAV4DXYTGCfg.GlobalParams.VIOLATION_COST_POWER));
		#endif

	}

	if (CheckJointStatespaceObstacle && EnvNAV4DXYTGCfg.CentralizedInfo->isPointInJointStatespaceObstacles(JointStatespacePoint,1,EnvNAV4DXYTGCfg.GlobalParams.JOINTSTATESPACE_LOGGING_RAD))
		violationCost[0] = INFINITECOST;
	else
		for (a = 0; a < OtherRobotsCount; a++)
			violationCost[0] = violationCost[0] + violationCost[a+1];
	//else if (ConstraintsCount > 0)
	//	violationCost = (int)((float)violationCost / (float)ConstraintsCount);
	
	delete InterpedPos; // DELETE**
	delete InterpedDist; // DELETE**
	
	// TODO: remove this
	violationCost[0] = violationCost[0] + (int)((float)X * (EnvNAV4DXYTGCfg.GlobalParams.LEFT_BASE_WEIGHT*NAV4DXYTG_COSTMULT));
	
	return violationCost;
}


bool EnvironmentNAV4DXYTG::IsValidCell(int X, int Y, int T, bool CheckStaticObstacle, bool CheckDynamicObstacle)
{
	bool retRes = false;
	//printf("^^^ IsValidCell: %d - %d, %d\n", CheckDynamicObstacle, EnvNAV4DXYTGCfg.DynamicObstacleMap.get(30,17,5), EnvNAV4DXYTGCfg.DynamicObstacleMap.get(80,17,5));
	
	//if (X >= 0 && X < EnvNAV4DXYTGCfg.EnvWidth_c && 
	//	Y >= 0 && Y < EnvNAV4DXYTGCfg.EnvHeight_c && 
	//	T >= 0 && T < EnvNAV4DXYTGCfg.EnvMaxTime_c)
	if (X >= 1 && X < EnvNAV4DXYTGCfg.EnvWidth_c-1 && 
		Y >= 1 && Y < EnvNAV4DXYTGCfg.EnvHeight_c-1 && 
		T >= 0 && T < EnvNAV4DXYTGCfg.EnvMaxTime_c)
		retRes = true;
	else
		return false;
	
	if (CheckStaticObstacle && EnvNAV4DXYTGCfg.StaticObstacleMap.get(X,Y))
		return false;
		
	if (CheckDynamicObstacle && EnvNAV4DXYTGCfg.DynamicObstacleMap.get(X,Y,T))
		return false;
	
	return retRes;
}

bool EnvironmentNAV4DXYTG::IsWithinMapCell(int X, int Y, int T)
{
	return (X >= 0 && X < EnvNAV4DXYTGCfg.EnvWidth_c && 
		Y >= 0 && Y < EnvNAV4DXYTGCfg.EnvHeight_c && 
		T >= 0 && T < EnvNAV4DXYTGCfg.EnvMaxTime_c);
}


int EnvironmentNAV4DXYTG::GetActionCost(int SourceX, int SourceY, int SourceTimet, int SourceG, EnvNAV4DXYTGAction_t* action)
{

	int TargetX, TargetY, TargetTime, netCost, actionCost;
	vector<int> netCostVector;

	TargetX = SourceX + action->dX;
	TargetY = SourceY + action->dY;
	TargetTime = SourceTimet + action->dTimet;

	if (IsValidCell(SourceX, SourceY, SourceTimet ))
	{
		netCostVector = ComputeCellConstraintViolationCost(TargetX, TargetY, TargetTime);
		actionCost = action->cost;
		if (SourceX==EnvNAV4DXYTGCfg.EndX_c && SourceY==EnvNAV4DXYTGCfg.EndY_c && action->dX==0 && action->dY==0)
			actionCost = (int) (actionCost / 10);
		netCost = netCostVector[0] + actionCost;
	}
	else
		netCost = INFINITECOST;

	#if DEBUG2
		printf("In environment_nav4Dxytg.cpp-L527 (GetActionCost) : In state (%d,%d,%d) - Action (%d,%d,%d) - NetCost(%d) = NodeCost(%d) + ActionCost(%d)\n", SourceX, SourceX, SourceTimet, 
							action->dX, action->dY, action->dTimet, netCost, ComputeCellConstraintViolationCost(TargetX, TargetY, TargetTime), action->cost);
		char tmpc;
		// scanf("%c", &tmpc);
	#endif

	return (netCost);

}

// =================================
// ----------------------------------------------------------------------------------------
// Code to automatically determine the amount of increment in the penalty weights


void ComputeTrajectoryCost(EnvNAV4DXYTG_pos_trajectory* traj, EnvironmentNAV4DXYTG* env, int* Cp, vector<int>* Cc, vector< vector<int> >* ConstraintViolationIndices)
{
	int a, b, ThisConstraintCost;
	bool foundAction;
	vector<int> NonzeroViolationIndices;
	vector<int> ThisConstraintCostVector;
	
	*Cp = 0;
	Cc->clear();
	ConstraintViolationIndices->clear();
	Cc->resize(env->EnvNAV4DXYTGCfg.penaltyWeights.size(), 0);
	ConstraintViolationIndices->resize(env->EnvNAV4DXYTGCfg.penaltyWeights.size());
	
	for (a=0; a < traj->pos_t.size(); a++)
	{
		// Add transition cost to path cost
		//printf("^^^^^ %d, %d, %d - %ld - ", traj->pos_t[a].X, traj->pos_t[a].Y, traj->pos_t[a].Timet, 
		//		env->EnvNAV4DXYTGCfg.distConstraint_trajectories[0].dist_t[a].D);
		if (a>0)
		{
			foundAction = false;
			for (b=0; b < env->EnvNAV4DXYTGCfg.ActionsV.size(); b++)
			{
				if (traj->pos_t[a-1].X + env->EnvNAV4DXYTGCfg.ActionsV[b].dX == traj->pos_t[a].X && 
						traj->pos_t[a-1].Y + env->EnvNAV4DXYTGCfg.ActionsV[b].dY == traj->pos_t[a].Y && 
						traj->pos_t[a-1].Timet + env->EnvNAV4DXYTGCfg.ActionsV[b].dTimet == traj->pos_t[a].Timet)
				{
					*Cp = *Cp + (int)env->EnvNAV4DXYTGCfg.ActionsV[b].cost;
					foundAction = true;
					break;
				}
			}
			if (!foundAction)
				*Cp = *Cp + TRANSITIONCOST_XYT(traj->pos_t[a].X-traj->pos_t[a-1].X, 
												traj->pos_t[a].Y-traj->pos_t[a-1].Y, traj->pos_t[a].Timet-traj->pos_t[a-1].Timet);
		}
		
		// Add to constraint violation cost
		ThisConstraintCostVector = env->ComputeCellConstraintViolationCost(traj->pos_t[a].X, traj->pos_t[a].Y, traj->pos_t[a].Timet, false);
		//printf("%ld\n", ThisConstraintCost);
		if (ThisConstraintCostVector[0] != 0)
		{
			for (b=0; b<env->EnvNAV4DXYTGCfg.penaltyWeights.size(); b++)
				if (ThisConstraintCostVector[b+1]>0)
				{
					Cc->at(b) = Cc->at(b) + ThisConstraintCostVector[b+1];
					ConstraintViolationIndices->at(b).push_back(a);
				}
		}
	}
	//return NonzeroViolationIndices;
}

/*
vector<int> FindConstraintViolationIndices(EnvNAV4DXYTG_pos_trajectory* traj, EnvironmentNAV4DXYTG* env)
{
	int t;
	vector<int> ViolationIndices;
	
	for (a=0; a<traj.pos_t.size(); a++)
	{
		ThisConstraintCost = env->ComputeCellConstraintViolationCost(traj->pos_t[a].X, traj->pos_t[a].Y, traj->pos_t[a].Timet, false);
		if (ThisConstraintCost != 0)
		{
			ViolationIndices.push_back(a);
		}
	}
	return ViolationIndices;
}
*/

vector<EnvNAV4DXYTG_pos_trajectory> FindVariationsOfATrajectory(EnvNAV4DXYTG_pos_trajectory* traj, EnvironmentNAV4DXYTG* env, vector<int> varIndices)
{
	// Changes the XYT position of the trajectory at varIndices to return new trajectories
	int a, b, v, varIndex;
	int testX, testY, testT, test2X, test2Y, test2T;
	EnvNAV4DXYTG_pos_trajectory tempTraj;
	vector<EnvNAV4DXYTG_pos_trajectory> alteredTrajs;
	
	for (v=0; v<varIndices.size(); v++)
	{
		varIndex = varIndices[v];
		if (varIndex<=0 || varIndex>=(traj->pos_t.size()-1))
			continue;
		
		for (a=0; a < env->EnvNAV4DXYTGCfg.ActionsV.size(); a++)
		{
			tempTraj = *traj;
			testX = traj->pos_t[varIndex-1].X + env->EnvNAV4DXYTGCfg.ActionsV[a].dX;
			testY = traj->pos_t[varIndex-1].Y + env->EnvNAV4DXYTGCfg.ActionsV[a].dY;
			testT = traj->pos_t[varIndex-1].Timet + env->EnvNAV4DXYTGCfg.ActionsV[a].dTimet;
			if ((testX==traj->pos_t[varIndex].X && testY==traj->pos_t[varIndex].Y && testT==traj->pos_t[varIndex].Timet) || 
					!env->IsValidCell(testX, testY, testT, true, false))
				continue;
			for (b=0; b < env->EnvNAV4DXYTGCfg.ActionsV.size(); b++)
			{
				test2X = testX + env->EnvNAV4DXYTGCfg.ActionsV[b].dX;
				test2Y = testY + env->EnvNAV4DXYTGCfg.ActionsV[b].dY;
				test2T = testT + env->EnvNAV4DXYTGCfg.ActionsV[b].dTimet;
				if (test2X==traj->pos_t[varIndex+1].X && test2Y==traj->pos_t[varIndex+1].Y && test2T==traj->pos_t[varIndex+1].Timet)
				{
					tempTraj.pos_t[varIndex].X = testX;
					tempTraj.pos_t[varIndex].Y = testY;
					tempTraj.pos_t[varIndex].Timet = testT;
					alteredTrajs.push_back(tempTraj);
					break;
				}
			}
		}
	}
	return alteredTrajs;
}


vector<float> SuggestNextPenaltyWeight(EnvNAV4DXYTG_pos_trajectory* traj, EnvironmentNAV4DXYTG* env)
{
	int a, b, aa, bb, c, Cp, CpNew, Dp, Dc, DcSum=0, DpMax=0, DpMin=INFINITECOST;
	vector<int> CcNew, Cc;
	vector< vector<int> > ConstraintViolationIndices, dummyConstraintViolationIndices;
	//?float thisDeltaGamma, outDeltaGamma=0;
	float thisGamma;
	vector<float> SuggestedGamma;
	//vector<int> ConstraintViolationIndices;
	vector< vector<EnvNAV4DXYTG_pos_trajectory> > VariedTrajectories;
	
	ComputeTrajectoryCost(traj, env, &Cp, &Cc, &ConstraintViolationIndices);
	//printf("()()()ConstraintViolationIndices.size()=%d, Cp=%d, Cc=%d\n", ConstraintViolationIndices.size(), Cp, Cc);
	//ConstraintViolationIndices = FindConstraintViolationIndices(traj, env);
	/*printf("(====) Original Trajectory:\n");
	printf("       Cp = %d\n", Cp);
	printf("       Cc (indices) = \n");
	for (a=0; a<Cc.size(); a++)
	{
		printf("                      %d (", Cc[a]);
		for (b=0; b<ConstraintViolationIndices[a].size(); b++) printf(" %d ", ConstraintViolationIndices[a][b]);
		printf(")\n");
	}*/
	VariedTrajectories.resize(Cc.size());
	SuggestedGamma.resize(Cc.size(),0.0);
	for (a=0; a<Cc.size(); a++)
		VariedTrajectories[a] = FindVariationsOfATrajectory(traj, env, ConstraintViolationIndices[a]);
	
	//if (VariedTrajectories.size()==0)
	//	return 0;
	
	for (a=0; a<VariedTrajectories.size(); a++)
	{
		DpMin=INFINITECOST;
		DcSum=0;
		for (b=0; b<VariedTrajectories[a].size(); b++)
		{
			dummyConstraintViolationIndices.clear();
			ComputeTrajectoryCost(&VariedTrajectories[a][b], env, &CpNew, &CcNew, &dummyConstraintViolationIndices);
			/*printf("[----] Varied Trajectory:\n");
			printf("       CpNew = %d\n", CpNew);
			printf("       CcNew (indices) = \n");
			for (aa=0; aa<CcNew.size(); aa++)
			{
				printf("                      %d (", CcNew[aa]);
				for (bb=0; bb<dummyConstraintViolationIndices[aa].size(); bb++) printf(" %d ", dummyConstraintViolationIndices[aa][bb]);
				printf(")\n");
			}*/
			Dp = CpNew-Cp;
			Dc = Cc[a]-CcNew[a];
			//printf("{}{}{} Dp=%d , Dc=%d , oldPenaltyWeight=%f\n", Dp, Dc, oldPenaltyWeight);
			if (Dc<=0)
			continue;
			DcSum = DcSum + Dc;
			DpMin = __min(DpMin, Dp);
			//?thisDeltaGamma = ((float)Dp)/((float)Dc) - oldPenaltyWeight;
			//?outDeltaGamma = __max(outDeltaGamma, thisDeltaGamma);
			//thisGamma = ((float)Dp)/((float)Dc);
			//SuggestedGamma = __max(thisGamma, SuggestedGamma);
		}
		if (DcSum>0 && DpMin<INFINITECOST)
			SuggestedGamma[a] = ((float)DpMin)/((float)DcSum);
		else
			SuggestedGamma[a] = 0;
	}
	
	//if (DcSum>0 && DpMin<INFINITECOST)
	//	SuggestedGamma = ((float)DpMin)/((float)DcSum);
	
	printf("Suggested Penalty weights:");
	for (a=0; a<SuggestedGamma.size(); a++)
		printf(" %f ", SuggestedGamma[a]);
	printf("\n");
	//?return outDeltaGamma;
	return SuggestedGamma;
}


// =================================

/*
int EnvironmentNAV4DXYTG::GetActionCost(int SourceX, int SourceY, int SourceTimet, int TargetX, int TargetY, int TargetTimet)
{

	int netCost;

	if (IsValidCell(SourceX, SourceY, SourceTimet))
		netCost = ComputeCellConstraintViolationCost(TargetX, TargetY, TargetTime) + action->cost;
	else
		netCost = INFINITECOST;

	return (netCost);

}
*/

// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// XXX - This function is in no more use!!
static int EuclideanDistance(int X1, int Y1, int X2, int Y2)
{
    int sqdist = ((X1-X2)*(X1-X2)+(Y1-Y2)*(Y1-Y2));
    double dist = sqrt((double)sqdist);
    return (int)(NAV4DXYTG_COSTMULT*dist);

}
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx


int EnvironmentNAV4DXYTG::HeuristicFunction(int X1, int Y1, int X2, int Y2)
{
	int sqdist, DX, DY;
	double dist;
	
	// printf("In HeuristicFunction: HEURISTIC_TYPE=%d\n", EnvNAV4DXYTGCfg.GlobalParams.HEURISTIC_TYPE);
	switch(EnvNAV4DXYTGCfg.GlobalParams.HEURISTIC_TYPE)
	{
		case 1 :
			sqdist = ((X1-X2)*(X1-X2)+(Y1-Y2)*(Y1-Y2));
    		dist = sqrt((double)sqdist);
    		return (int)(NAV4DXYTG_COSTMULT*dist);
			break;
			
		case 2 :  // Only for 8-connected grids
			DX = abs(X1-X2);
			DY = abs(Y1-Y2);
			dist = sqrt(2.0)*((double)(__min(DX,DY))) + ((double)(abs(DX-DY)));
			//printf("In HeuristicFunction: %d, %d, %f, %d\n", DX, DY, dist, (int)(NAV4DXYTG_COSTMULT*dist));
    		return (int)(NAV4DXYTG_COSTMULT*dist);
			break;
			
		/* case 3 : // Use pre-computed heuristics
			int h2D = grid2Dsearch->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
			//int hEuclid = (int)(NAV3DKIN_COSTMULT_MTOMM*EuclideanDistance_m(EnvNAV3DKINCfg.StartX_c, EnvNAV3DKINCfg.StartY_c, HashEntry->X, HashEntry->Y));
			//return (int)(((double)__max(h2D,hEuclid))/EnvNAV3DKINCfg.nominalvel_mpersecs);
			break; */
			
	}
}


/*
void EnvironmentNAV4DXYTG::CalculateFootprintForPose(EnvNAV4DXYTG3Dpt_t pose, vector<sbpl_2Dcell_t>* footprint)
{  

#if DEBUG
  printf("---Calculating Footprint for Pose: %f %f %f---\n",
	 pose.x, pose.y, pose.Timet);
#endif

  //handle special case where footprint is just a point
  if(EnvNAV4DXYTGCfg.FootprintPolygon.size() <= 1){
    sbpl_2Dcell_t cell;
    cell.x = CONTXY2DISC(pose.x, EnvNAV4DXYTGCfg.cellsize_m);
    cell.y = CONTXY2DISC(pose.y, EnvNAV4DXYTGCfg.cellsize_m);
    footprint->push_back(cell);
    return;
  }

  vector<sbpl_2Dpt_t> bounding_polygon;
  unsigned int find;
  double max_x, min_x, max_y, min_y;
  sbpl_2Dpt_t pt;
  for(find = 0; find < EnvNAV4DXYTGCfg.FootprintPolygon.size(); find++){
    
    //rotate and translate the corner of the robot
    pt = EnvNAV4DXYTGCfg.FootprintPolygon[find];
    
    //rotate and translate the point
    sbpl_2Dpt_t corner;
    corner.x = cos(pose.Timet)*pt.x - sin(pose.Timet)*pt.y + pose.x;
    corner.y = sin(pose.Timet)*pt.x + cos(pose.Timet)*pt.y + pose.y;
    bounding_polygon.push_back(corner);
#if DEBUG
    printf("Pt: %f %f, Corner: %f %f\n", pt.x, pt.y, corner.x, corner.y);
#endif
    if(corner.x < min_x || find==0){
      min_x = corner.x;
    }
    if(corner.x > max_x || find==0){
      max_x = corner.x;
    }
    if(corner.y < min_y || find==0){
      min_y = corner.y;
    }
    if(corner.y > max_y || find==0){
      max_y = corner.y;
    }
    
  }

#if DEBUG
  printf("Footprint bounding box: %f %f %f %f\n", min_x, max_x, min_y, max_y);
#endif
  //initialize previous values to something that will fail the if condition during the first iteration in the for loop
  int prev_discrete_x = CONTXY2DISC(pt.x, EnvNAV4DXYTGCfg.cellsize_m) + 1; 
  int prev_discrete_y = CONTXY2DISC(pt.y, EnvNAV4DXYTGCfg.cellsize_m) + 1;
  int prev_inside = 0;
  int discrete_x;
  int discrete_y;

  for(double x=min_x; x<=max_x; x+=EnvNAV4DXYTGCfg.cellsize_m/3){
    for(double y=min_y; y<=max_y; y+=EnvNAV4DXYTGCfg.cellsize_m/3){
      pt.x = x;
      pt.y = y;
      discrete_x = CONTXY2DISC(pt.x, EnvNAV4DXYTGCfg.cellsize_m);
      discrete_y = CONTXY2DISC(pt.y, EnvNAV4DXYTGCfg.cellsize_m);
      
      //see if we just tested this point
      if(discrete_x != prev_discrete_x || discrete_y != prev_discrete_y || prev_inside==0){

#if DEBUG
		printf("Testing point: %f %f Discrete: %d %d\n", pt.x, pt.y, discrete_x, discrete_y);
#endif
	
		if(IsInsideFootprint(pt, &bounding_polygon)){
		//convert to a grid point

#if DEBUG
			printf("Pt Inside %f %f\n", pt.x, pt.y);
#endif

			sbpl_2Dcell_t cell;
			cell.x = discrete_x;
			cell.y = discrete_y;
			footprint->push_back(cell);
			prev_inside = 1;

#if DEBUG
			printf("Added pt to footprint: %f %f\n", pt.x, pt.y);
#endif
		}
		else{
			prev_inside = 0;
		}

      }
	  else
	  {
#if DEBUG
		//rintf("Skipping pt: %f %f\n", pt.x, pt.y);
#endif
      }
      
      prev_discrete_x = discrete_x;
      prev_discrete_y = discrete_y;

    }//over x_min...x_max
  }
}
*/


//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------

void EnvironmentNAV4DXYTG::ComputeHeuristicValues()
{
	//whatever necessary pre-computation of heuristic values is done here 
	printf("Precomputing heuristics...\n");
	unsigned char** Grid2D;
	int x, y;
	
	if (EnvNAV4DXYTGCfg.GlobalParams.PRECOMPUTE_HEURISTIC == 1)
	{
		Grid2D = new unsigned char* [EnvNAV4DXYTGCfg.EnvWidth_c];
		for (x = 0; x < EnvNAV4DXYTGCfg.EnvWidth_c; x++)
		{
			Grid2D[x] = new unsigned char [EnvNAV4DXYTGCfg.EnvHeight_c];
			for (y = 0; y < EnvNAV4DXYTGCfg.EnvHeight_c; y++)
			{
				Grid2D[x][y] = 10 * (unsigned char)EnvNAV4DXYTGCfg.StaticObstacleMap.get(x,y);
			}
		}
		
		EnvNAV4DXYTG.grid2DsearchBak = new SBPL2DGridSearch(EnvNAV4DXYTGCfg.EnvWidth_c, EnvNAV4DXYTGCfg.EnvHeight_c, 1.0);
		EnvNAV4DXYTG.grid2DsearchBak->search(Grid2D, 5, EnvNAV4DXYTGCfg.StartX_c, EnvNAV4DXYTGCfg.StartY_c, EnvNAV4DXYTGCfg.EndX_c, EnvNAV4DXYTGCfg.EndY_c, SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
		
		EnvNAV4DXYTG.grid2DsearchFwd = new SBPL2DGridSearch(EnvNAV4DXYTGCfg.EnvWidth_c, EnvNAV4DXYTGCfg.EnvHeight_c, 1.0);
		EnvNAV4DXYTG.grid2DsearchFwd->search(Grid2D, 5, EnvNAV4DXYTGCfg.EndX_c, EnvNAV4DXYTGCfg.EndY_c, EnvNAV4DXYTGCfg.StartX_c, EnvNAV4DXYTGCfg.StartY_c, SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
		
		EnvNAV4DXYTG.grid2DsearchFwdTasks = new SBPL2DGridSearchWithTasks(EnvNAV4DXYTGCfg.EnvWidth_c, EnvNAV4DXYTGCfg.EnvHeight_c, 1.0);
		EnvNAV4DXYTG.grid2DsearchFwdTasks->PreCompute(Grid2D, 5, EnvNAV4DXYTGCfg.EndX_c, EnvNAV4DXYTGCfg.EndY_c, EnvNAV4DXYTGCfg.Tasks, EnvNAV4DXYTGCfg.StartX_c, EnvNAV4DXYTGCfg.StartY_c, SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
		
		//delete Grid2D; // MEM_CLEAR
		for (x = 0; x < EnvNAV4DXYTGCfg.EnvWidth_c; x++)
			delete[] Grid2D[x];
		delete[] Grid2D;
	}

	printf("done\n");

}

//------------debugging functions---------------------------------------------
bool EnvironmentNAV4DXYTG::CheckQuant(FILE* fOut) 
{

	/*
  for(double Timet  = -10; Timet < 10; Timet += 2.0*PI_CONST/NAV4DXYTG_TimetDIRS*0.01)
    {
		int nTimet = ContTimet2Disc(Timet, NAV4DXYTG_TimetDIRS);
		double newTimet = DiscTimet2Cont(nTimet, NAV4DXYTG_TimetDIRS);
		int nnewTimet = ContTimet2Disc(newTimet, NAV4DXYTG_TimetDIRS);

		fprintf(fOut, "Timet=%f(%f)->%d->%f->%d\n", Timet, Timet*180/PI_CONST, nTimet, newTimet, nnewTimet);

        if(nTimet != nnewTimet)
        {
            printf("ERROR: invalid quantization\n");                     
            return false;
        }
    }
 */
 
  return true;
}



//-----------------------------------------------------------------------------

//-----------interface with outside functions-----------------------------------


bool EnvironmentNAV4DXYTG::InitializeEnv(const char* sEnvFile)
{
	// This function is of no use!!
	// It is here only because of it's declaration in DiscreteSpaceInformation class in environment.h
	return true;
}

/*
bool EnvironmentNAV4DXYTG::InitializeEnv(const char* sEnvFile)
{

	FILE* fCfg = fopen(sEnvFile, "r");
	if(fCfg == NULL)
	{
		printf("ERROR: unable to open %s\n", sEnvFile);
		exit(1);
	}
	ReadConfiguration(fCfg);

	InitGeneral();


	return true;
}


bool EnvironmentNAV4DXYTG::InitializeEnv(const char* sEnvFile, vector<EnvNAV4DXYTG_pos_trajectory> otherBots_trajectories,
					vector<EnvNAV4DXYTG_dist_trajectory> distConstraint_trajectories,
					vector<float> penaltyWeights)
{

	FILE* fCfg = fopen(sEnvFile, "r");
	if(fCfg == NULL)
	{
		printf("ERROR: unable to open %s\n", sEnvFile);
		exit(1);
	}
	ReadConfiguration(fCfg);
	SetConfiguration_constraints(otherBots_trajectories, distConstraint_trajectories, penaltyWeights);

	InitGeneral();


	return true;
}
*/

/*
bool EnvironmentNAV4DXYTG::InitializeEnv(vector<int> otherBots_identifiers,
					vector<EnvNAV4DXYTG_pos_trajectory> otherBots_trajectories,
					vector<EnvNAV4DXYTG_dist_trajectory> distConstraint_trajectories,
					vector<float> penaltyWeights)
{

	//SetConfiguration_constraints(otherBots_identifiers, otherBots_trajectories, distConstraint_trajectories, penaltyWeights);
	EnvNAV4DXYTGCfg.otherBots_identifiers = otherBots_identifiers;
	EnvNAV4DXYTGCfg.otherBots_trajectories = otherBots_trajectories;
	EnvNAV4DXYTGCfg.distConstraint_trajectories = distConstraint_trajectories;
	EnvNAV4DXYTGCfg.penaltyWeights = penaltyWeights;

	// InitGeneral(); // Don't call this multiple times!


	return true;
}
*/

/*
// Initialize everything
bool EnvironmentNAV4DXYTG::InitializeEnv(ConfigFileInfo* CfgInfo, CentralizedInfo_t* CentralizedInfo, int robotIndex,
										vector<int> otherBots_identifiers,
										vector<EnvNAV4DXYTG_pos_trajectory> otherBots_trajectories,
										vector<EnvNAV4DXYTG_dist_trajectory> distConstraint_trajectories,
										vector<float> penaltyWeights)
{
	EnvNAV4DXYTGCfg.EnvWidth_c = CfgInfo->EnvWidth_c;
	EnvNAV4DXYTGCfg.EnvHeight_c = CfgInfo->EnvHeight_c;
	EnvNAV4DXYTGCfg.EnvMaxTime_c = CfgInfo->EnvMaxTime_c;
	
	EnvNAV4DXYTGCfg.cellsize_m = CfgInfo->cellsize_m;
	EnvNAV4DXYTGCfg.timestepsize_m = CfgInfo->timestepsize_m;
	
	EnvNAV4DXYTGCfg.StartX_c = CfgInfo->TheRobots[robotIndex].StartX_c;
	EnvNAV4DXYTGCfg.StartY_c = CfgInfo->TheRobots[robotIndex].StartY_c;
	EnvNAV4DXYTGCfg.StartTimet_c = CfgInfo->TheRobots[robotIndex].StartTimet_c;
	
	EnvNAV4DXYTGCfg.EndX_c = CfgInfo->TheRobots[robotIndex].EndX_c;
	EnvNAV4DXYTGCfg.EndY_c = CfgInfo->TheRobots[robotIndex].EndY_c;
	EnvNAV4DXYTGCfg.EndTimet_c = CfgInfo->TheRobots[robotIndex].EndTimet_c;
	
	EnvNAV4DXYTGCfg.BotIdentifier = robotIndex;
	EnvNAV4DXYTGCfg.GlobalParams = CfgInfo->GlobalParams;
	EnvNAV4DXYTGCfg.CentralizedInfo = CentralizedInfo;
	
	EnvNAV4DXYTGCfg.ActionsV = CfgInfo->TheRobots[robotIndex].ActionsV;
	
	//SetConfiguration_constraints(otherBots_identifiers, otherBots_trajectories, distConstraint_trajectories, penaltyWeights);
	EnvNAV4DXYTGCfg.otherBots_identifiers = otherBots_identifiers;
	EnvNAV4DXYTGCfg.otherBots_trajectories = otherBots_trajectories;
	EnvNAV4DXYTGCfg.distConstraint_trajectories = distConstraint_trajectories;
	EnvNAV4DXYTGCfg.penaltyWeights = penaltyWeights;
	
	EnvNAV4DXYTGCfg.StaticObstacleMap.data = CfgInfo->StaticObstacleMap.data;
	EnvNAV4DXYTGCfg.StaticObstacleMap.size_x = CfgInfo->StaticObstacleMap.size_x;
	EnvNAV4DXYTGCfg.StaticObstacleMap.size_y = CfgInfo->StaticObstacleMap.size_y;
	
	EnvNAV4DXYTGCfg.DynamicObstacleMap.init(CfgInfo->EnvWidth_c, CfgInfo->EnvHeight_c, CfgInfo->EnvMaxTime_c, false);
	EnvNAV4DXYTGCfg.DynamicObstacleMap.ConstructFromTrajectories(otherBots_trajectories, EnvNAV4DXYTGCfg.GlobalParams.COLLISION_CHECK_RADIUS);
	
	InitGeneral();
	
	return true;
}
*/


bool EnvironmentNAV4DXYTG::SetPenaltyWeights(vector<float> penaltyWeights)
{
	EnvNAV4DXYTGCfg.penaltyWeights = penaltyWeights;
	return true;
}



// Initialize everything except Penalty weights
bool EnvironmentNAV4DXYTG::InitializeEnv(ConfigFileInfo* CfgInfo, CentralizedInfo_t* CentralizedInfo, int robotIndex,
										vector<int> otherBots_identifiers,
										vector<EnvNAV4DXYTG_pos_trajectory> otherBots_trajectories,
										vector<EnvNAV4DXYTG_dist_trajectory> distConstraint_trajectories)
{
	StartTime = clock();
	
	EnvNAV4DXYTGCfg.EnvWidth_c = CfgInfo->EnvWidth_c;
	EnvNAV4DXYTGCfg.EnvHeight_c = CfgInfo->EnvHeight_c;
	EnvNAV4DXYTGCfg.EnvMaxTime_c = CfgInfo->EnvMaxTime_c;
	
	EnvNAV4DXYTGCfg.cellsize_m = CfgInfo->cellsize_m;
	EnvNAV4DXYTGCfg.timestepsize_m = CfgInfo->timestepsize_m;
	
	EnvNAV4DXYTGCfg.StartX_c = CfgInfo->TheRobots[robotIndex].StartX_c;
	EnvNAV4DXYTGCfg.StartY_c = CfgInfo->TheRobots[robotIndex].StartY_c;
	EnvNAV4DXYTGCfg.StartTimet_c = CfgInfo->TheRobots[robotIndex].StartTimet_c;
	EnvNAV4DXYTGCfg.StartG_c = 0;
	
	EnvNAV4DXYTGCfg.EndX_c = CfgInfo->TheRobots[robotIndex].EndX_c;
	EnvNAV4DXYTGCfg.EndY_c = CfgInfo->TheRobots[robotIndex].EndY_c;
	EnvNAV4DXYTGCfg.EndTimet_c = CfgInfo->TheRobots[robotIndex].EndTimet_c;
	EnvNAV4DXYTGCfg.EndG_c = (int)pow(2, CfgInfo->TheRobots[robotIndex].Tasks.size()) - 1;
	//printf("Goal G: %d\n", EnvNAV4DXYTGCfg.EndG_c);
	
	EnvNAV4DXYTGCfg.Tasks = CfgInfo->TheRobots[robotIndex].Tasks;
	
	EnvNAV4DXYTGCfg.BotIdentifier = robotIndex;
	EnvNAV4DXYTGCfg.GlobalParams = CfgInfo->GlobalParams;
	
	EnvNAV4DXYTGCfg.CentralizedInfo = CentralizedInfo;
	
	EnvNAV4DXYTGCfg.ActionsV = CfgInfo->TheRobots[robotIndex].ActionsV;
	
	//SetConfiguration_constraints(otherBots_identifiers, otherBots_trajectories, distConstraint_trajectories, penaltyWeights);
	EnvNAV4DXYTGCfg.otherBots_identifiers = otherBots_identifiers;
	EnvNAV4DXYTGCfg.otherBots_trajectories = otherBots_trajectories;
	EnvNAV4DXYTGCfg.distConstraint_trajectories = distConstraint_trajectories;
	//EnvNAV4DXYTGCfg.penaltyWeights = penaltyWeights;
	
	EnvNAV4DXYTGCfg.StaticObstacleMap.data = CfgInfo->StaticObstacleMap.data;
	EnvNAV4DXYTGCfg.StaticObstacleMap.size_x = CfgInfo->StaticObstacleMap.size_x;
	EnvNAV4DXYTGCfg.StaticObstacleMap.size_y = CfgInfo->StaticObstacleMap.size_y;
	
	//EnvNAV4DXYTGCfg.DynamicObstacleMap.init(CfgInfo->EnvWidth_c, CfgInfo->EnvHeight_c, CfgInfo->EnvMaxTime_c, false);
	EnvNAV4DXYTGCfg.DynamicObstacleMap.data = CfgInfo->DynamicObstacleMap.data;
	EnvNAV4DXYTGCfg.DynamicObstacleMap.size_x = CfgInfo->DynamicObstacleMap.size_x;
	EnvNAV4DXYTGCfg.DynamicObstacleMap.size_y = CfgInfo->DynamicObstacleMap.size_y;
	EnvNAV4DXYTGCfg.DynamicObstacleMap.size_tt = CfgInfo->DynamicObstacleMap.size_tt;
	EnvNAV4DXYTGCfg.DynamicObstacleMap.ConstructFromTrajectories(otherBots_trajectories, EnvNAV4DXYTGCfg.GlobalParams.COLLISION_CHECK_RADIUS);
	
	//printf("^^^ B: %d, %d\n", CfgInfo->DynamicObstacleMap.get(30,17,5), CfgInfo->DynamicObstacleMap.get(80,17,5));
	//printf("^^^ A: %d, %d\n", EnvNAV4DXYTGCfg.DynamicObstacleMap.get(30,17,5), EnvNAV4DXYTGCfg.DynamicObstacleMap.get(80,17,5));
	
	EnvNAV4DXYTGCfg.RobotHomotopyInfo.LValDiffMap = CfgInfo->LValDiffMap;
	//printf("%x, %x -- \n", CfgInfo->LValDiffMap, EnvNAV4DXYTGCfg.RobotHomotopyInfo.LValDiffMap);
	EnvNAV4DXYTGCfg.RobotHomotopyInfo.ForcedActive = true; // To be removed
	
	EnvNAV4DXYTGCfg.RobotHomotopyInfo.BlockedHomotopyClass_LVals = CfgInfo->TheRobots[robotIndex].BlockedHomotopyClass_Const_LVals;
	for (int a=0; a<CentralizedInfo->BlockedHomotopyClasses.HomotopyClassList[robotIndex].size(); a++)
		EnvNAV4DXYTGCfg.RobotHomotopyInfo.BlockedHomotopyClass_LVals.push_back(CentralizedInfo->BlockedHomotopyClasses.HomotopyClassList[robotIndex][a]);
	//printf("Number of blocked classes = %d, %d \n", EnvNAV4DXYTGCfg.RobotHomotopyInfo.BlockedHomotopyClass_LVals.size(), CfgInfo->TheRobots[robotIndex].BlockedHomotopyClass_Const_LVals.size());
		
	for (int a=0; a<CentralizedInfo->InitialConstrainHomotopyClass.HomotopyClassList[robotIndex].size(); a++)
		EnvNAV4DXYTGCfg.RobotHomotopyInfo.ConstrainHomotopyClass_LVals.push_back(CentralizedInfo->InitialConstrainHomotopyClass.HomotopyClassList[robotIndex][a]);
	// TODO: Append any additional (dynamic) blocking of homotopy class here
	
	InitGeneral();
	
	return true;
}


/*
bool EnvironmentNAV4DXYTG::InitializeEnv(int width, int height, int MaxTime,
					double startx, double starty, double startTimet,
					double goalx, double goaly, double goalTimet,
				    double goaltol_x, double goaltol_y, double goaltol_Timet,
					double cellsize_m, double nominalvel_mpersecs)
{
	//TODO - need to set the tolerance as well

	SetConfiguration(width, height,
					mapdata,
					CONTXY2DISC(startx, cellsize_m), CONTXY2DISC(starty, cellsize_m), ContTimet2Disc(startTimet, NAV4DXYTG_TimetDIRS),
					CONTXY2DISC(goalx, cellsize_m), CONTXY2DISC(goaly, cellsize_m), ContTimet2Disc(goalTimet, NAV4DXYTG_TimetDIRS),
					cellsize_m, nominalvel_mpersecs, Timetoturn45degsinplace_secs, perimeterptsV);

	InitGeneral();

	return true;
}
*/


bool EnvironmentNAV4DXYTG::InitGeneral() {
  //Initialize other parameters of the environment
  // InitializeEnvConfig();
  
  //initialize Environment
  InitializeEnvironment();
  
  //pre-compute heuristics
  ComputeHeuristicValues();

  return true;
}

bool EnvironmentNAV4DXYTG::InitializeMDPCfg(MDPConfig *MDPCfg)
{
	//initialize MDPCfg with the start and goal ids	
	MDPCfg->goalstateid = EnvNAV4DXYTG.goalstateid;
	MDPCfg->startstateid = EnvNAV4DXYTG.startstateid;

	return true;
}



int EnvironmentNAV4DXYTG::GetFromToHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(FromStateID >= (int)EnvNAV4DXYTG.StateID2CoordTable.size() 
		|| ToStateID >= (int)EnvNAV4DXYTG.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV4DXYTG... function: stateID illegal\n");
		exit(1);
	}
#endif

		
	if (ToStateID==EnvNAV4DXYTG.goalstateid)
		return GetGoalHeuristic(FromStateID);

	if (FromStateID==EnvNAV4DXYTG.startstateid)
		return GetStartHeuristic(ToStateID);


	//get X, Y for the state
	EnvNAV4DXYTGHashEntry_t* FromHashEntry = EnvNAV4DXYTG.StateID2CoordTable[FromStateID];
	EnvNAV4DXYTGHashEntry_t* ToHashEntry = EnvNAV4DXYTG.StateID2CoordTable[ToStateID];
	
	return HeuristicFunction(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X, ToHashEntry->Y);	

}


int EnvironmentNAV4DXYTG::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif

#if DEBUG
	if(stateID >= (int)EnvNAV4DXYTG.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV4DXYTG... function: stateID illegal\n");
		exit(1);
	}
#endif

	EnvNAV4DXYTGHashEntry_t* FromHashEntry = EnvNAV4DXYTG.StateID2CoordTable[stateID];
	//!! EnvNAV4DXYTGHashEntry_t* ToHashEntry = EnvNAV4DXYTG.StateID2CoordTable[EnvNAV4DXYTG.goalstateid];
	//!! int h1 = HeuristicFunction(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X, ToHashEntry->Y);	
	
	//int h2 = EnvNAV4DXYTG.grid2DsearchFwd->getlowerboundoncostfromstart_inmm(FromHashEntry->X, FromHashEntry->Y);
	//printf("%d : %d,%d,%d : %d \n", stateID, FromHashEntry->X, FromHashEntry->Y, FromHashEntry->G, IsValidCell(FromHashEntry->X, FromHashEntry->Y, FromHashEntry->Timet));
	int h2 = EnvNAV4DXYTG.grid2DsearchFwdTasks->getPreComputedHeu(FromHashEntry->X, FromHashEntry->Y, FromHashEntry->G);
	
	//printf("In GetGoalHeuristic: (%d,%d) : h1=%d , h2=%d \n", FromHashEntry->X, FromHashEntry->Y, h1, h2);
	
	//define this function if it used in the planner (heuristic forward search would use it)
    //return GetFromToHeuristic(stateID, EnvNAV4DXYTG.goalstateid);
    return h2;
    
}


int EnvironmentNAV4DXYTG::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(stateID >= (int)EnvNAV4DXYTG.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV4DXYTG... function: stateID illegal\n");
		exit(1);
	}
#endif


	//!! EnvNAV4DXYTGHashEntry_t* FromHashEntry = EnvNAV4DXYTG.StateID2CoordTable[EnvNAV4DXYTG.startstateid];
	EnvNAV4DXYTGHashEntry_t* ToHashEntry = EnvNAV4DXYTG.StateID2CoordTable[stateID];
	//!! int h1 = HeuristicFunction(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X, ToHashEntry->Y);	
    
	int h2 = EnvNAV4DXYTG.grid2DsearchBak->getlowerboundoncostfromstart_inmm(ToHashEntry->X, ToHashEntry->Y);
	
	//printf("In GetStartHeuristic: h2=%d\n", h2);
	
	//define this function if it used in the planner (heuristic backward search would use it)
    // return GetFromToHeuristic(EnvNAV4DXYTG.startstateid, stateID);
    return h2;


}



void EnvironmentNAV4DXYTG::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{

	int cost;

#if DEBUG
	if(state->StateID >= (int)EnvNAV4DXYTG.StateID2CoordTable.size())
	{
		printf("ERROR in Env... function: stateID illegal\n");
		exit(1);
	}

	if((int)state->Actions.size() != 0)
	{
		printf("ERROR in Env_setAllActionsandAllOutcomes: actions already exist for the state\n");
		exit(1);
	}
#endif

	

	//goal state should be absorbing
	if(state->StateID == EnvNAV4DXYTG.goalstateid)
		return;

	//get X, Y for the state
	EnvNAV4DXYTGHashEntry_t* HashEntry = EnvNAV4DXYTG.StateID2CoordTable[state->StateID];

	for (int aind = 0; aind < EnvNAV4DXYTGCfg.ActionsV.size(); aind++)
	{
		EnvNAV4DXYTGAction_t* nav3daction = &EnvNAV4DXYTGCfg.ActionsV[aind];
        int newX = HashEntry->X + nav3daction->dX;
		int newY = HashEntry->Y + nav3daction->dY;
		int newTimet = HashEntry->Timet + nav3daction->dTimet;
		
		complex<double> newLVal = HashEntry->Lval;
		if (EnvNAV4DXYTGCfg.RobotHomotopyInfo.isActive())
			newLVal += EnvNAV4DXYTGCfg.RobotHomotopyInfo.getLValDiff(HashEntry->X, HashEntry->Y, aind);
		
		// Checking if passing through a target. Turn on the corresponding bit in G
		int newG = HashEntry->G;
		for (int targetCheck = 0; targetCheck < EnvNAV4DXYTGCfg.Tasks.size(); targetCheck++)
			if (HashEntry->X == EnvNAV4DXYTGCfg.Tasks[targetCheck].x && HashEntry->Y == EnvNAV4DXYTGCfg.Tasks[targetCheck].y)
			{
				newG = newG | (int)pow((float)2, targetCheck);
				break;
			}
		

		// compute and check cost
		cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Timet, HashEntry->G, nav3daction);
		if(cost >= INFINITECOST)
            	continue;

		// skip invalid moves here
		// A better alternative is to do this in GetActionCost and set cost to INFINITECOST
		// continue;

		//add the action
		CMDPACTION* action = state->AddAction(aind);

#if TIME_DEBUG
		clock_t currenttime = clock();
#endif

		EnvNAV4DXYTGHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(newX, newY, newTimet, newG, newLVal)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(newX, newY, newTimet, newG, newLVal);
		}
		action->AddOutcome(OutHashEntry->stateID, cost, 1.0); 

#if TIME_DEBUG
		time3_addallout += clock()-currenttime;
#endif

	}
}



void EnvironmentNAV4DXYTG::SetAllPreds(CMDPSTATE* state)
{
	//implement this if the planner needs access to predecessors
	
	printf("ERROR in EnvNAV4DXYTG... function: SetAllPreds is undefined\n");
	exit(1);
}


void EnvironmentNAV4DXYTG::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
	int actionCount, aind, cost;
	complex<double> newLVal;

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

	#if DEBUG2
		printf("In environment_nav4Dxytg.cpp-L527 : Inside GetSuccs\n");
	#endif

	actionCount = EnvNAV4DXYTGCfg.ActionsV.size();

	//clear the successor array
	SuccIDV->clear();
	CostV->clear();
	SuccIDV->reserve(actionCount); 
	CostV->reserve(actionCount);

	//goal state should be absorbing
	if(SourceStateID == EnvNAV4DXYTG.goalstateid)
		return;

	//get X, Y for the state
	EnvNAV4DXYTGHashEntry_t* HashEntry = EnvNAV4DXYTG.StateID2CoordTable[SourceStateID];
	

	for (aind = 0; aind < actionCount ; aind++)
	{
		EnvNAV4DXYTGAction_t* nav3daction = &EnvNAV4DXYTGCfg.ActionsV[aind];
		int newX = HashEntry->X + nav3daction->dX;
		int newY = HashEntry->Y + nav3daction->dY;
		int newTimet = HashEntry->Timet + nav3daction->dTimet;
		
		newLVal = HashEntry->Lval;
		if (EnvNAV4DXYTGCfg.RobotHomotopyInfo.isActive())
			newLVal += EnvNAV4DXYTGCfg.RobotHomotopyInfo.getLValDiff(HashEntry->X, HashEntry->Y, aind);
		
		int newG = HashEntry->G;
		for (int targetCheck = 0; targetCheck < EnvNAV4DXYTGCfg.Tasks.size(); targetCheck++)
			if (HashEntry->X == EnvNAV4DXYTGCfg.Tasks[targetCheck].x && HashEntry->Y == EnvNAV4DXYTGCfg.Tasks[targetCheck].y)
			{
				newG = newG | (int)pow((float)2, targetCheck);
				break;
			}

		cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Timet, HashEntry->G, nav3daction);
		if(cost >= INFINITECOST)
			continue;


		EnvNAV4DXYTGHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(newX, newY, newTimet, newG, newLVal)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(newX, newY, newTimet, newG, newLVal);
		}

		SuccIDV->push_back(OutHashEntry->stateID);
		CostV->push_back(cost);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif

}



void EnvironmentNAV4DXYTG::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{

	#if DEBUG2
		printf("In environment_nav4Dxytg.cpp-L527 : Inside GetPreds");
	#endif
	
/*
    int aind;

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

    //clear the successor array
    PredIDV->clear();
    CostV->clear();
    PredIDV->reserve(NAV4DXYTG_ACTIONWIDTH); 
    CostV->reserve(NAV4DXYTG_ACTIONWIDTH);

	//get X, Y for the state
	EnvNAV4DXYTGHashEntry_t* HashEntry = EnvNAV4DXYTG.StateID2CoordTable[TargetStateID];
	
	//no predecessors if obstacle
	if(EnvNAV4DXYTGCfg.Grid2D[HashEntry->X][HashEntry->Y] != 0)
		return;

	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == EnvNAV4DXYTGCfg.EnvWidth_c-1 || //TODO - need to modify to take robot perimeter into account
       HashEntry->Y == 0 || HashEntry->Y == EnvNAV4DXYTGCfg.EnvHeight_c-1)
        bTestBounds = true;

	for (aind = 0; aind < NAV4DXYTG_ACTIONWIDTH; aind++)
	{
		EnvNAV4DXYTGAction_t* nav3daction = &EnvNAV4DXYTGCfg.ActionsV[HashEntry->Timet][aind];
        int predX = HashEntry->X + nav3daction->dX;
		int predY = HashEntry->Y + nav3daction->dY;
		int predTimet = NORMALIZEDISCTimet(HashEntry->Timet + nav3daction->dTimet, NAV4DXYTG_TimetDIRS);	
	

		//TODO - incorrect - have to compute preds array
		
		//skip the invalid cells
		if(bTestBounds){ //TODO - need to modify to take robot perimeter into account
            if(!IsValidCell(predX, predY))
                continue;
        }

		//skip invalid diagonal move
	    if(GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Timet, nav3daction) >= INFINITECOST) //TODO -change after I have explicit backward actions
			continue;
        


    	EnvNAV4DXYTGHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(predX, predY, predTimet)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(predX, predY, predTimet);
		}

        //compute clow 
        int cost = nav3daction->cost;

        PredIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif

*/

}




int EnvironmentNAV4DXYTG::SizeofCreatedEnv()
{
	return (int)EnvNAV4DXYTG.StateID2CoordTable.size();
	
}

void EnvironmentNAV4DXYTG::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
	if(stateID >= (int)EnvNAV4DXYTG.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV4DXYTG... function: stateID illegal (2)\n");
		exit(1);
	}
#endif

	if(fOut == NULL)
		fOut = stdout;

	EnvNAV4DXYTGHashEntry_t* HashEntry = EnvNAV4DXYTG.StateID2CoordTable[stateID];

	if(stateID == EnvNAV4DXYTG.goalstateid && bVerbose)
	{
		fprintf(fOut, "the state is a goal state\n");
	}

    if(bVerbose)
    	fprintf(fOut, "X=%d Y=%d Timet=%d G=%d\n", HashEntry->X, HashEntry->Y, HashEntry->Timet, HashEntry->G);
    else
    	fprintf(fOut, "%d %d %d %d\n", HashEntry->X, HashEntry->Y, HashEntry->Timet, HashEntry->G);

}

void EnvironmentNAV4DXYTG::GetTrajectoryFromSolutionStateIDs(vector<int> solution_stateIDs_V, EnvNAV4DXYTG_pos_trajectory* posTraj)
{
	int a, cost=0;
	//int SourceX, SourceY, SourceTimet;
	EnvNAV4DXYTGHashEntry_t* HashEntry;
	EnvNAV4DXYTG_pos_t* tmpPos = new EnvNAV4DXYTG_pos_t;
	
	for(a=0; a<solution_stateIDs_V.size(); a++)
	{
		HashEntry = EnvNAV4DXYTG.StateID2CoordTable[solution_stateIDs_V[a]];
		//~EnvNAV4DXYTG_pos_t* tmpPos = new EnvNAV4DXYTG_pos_t;
		tmpPos->X = HashEntry->X;
		tmpPos->Y = HashEntry->Y;
		tmpPos->Timet = HashEntry->Timet;
		posTraj->pos_t.push_back(*tmpPos);
		
		/*
		if (a>0)
			cost += GetActionCost(SourceX, SourceY, SourceTimet, tmpPos->X, tmpPos->Y, tmpPos->Timet);

		SourceX = tmpPos->X;
		SourceY = tmpPos->Y;
		SourceTimet = tmpPos->Timet;
		*/
		//~delete tmpPos; // DELETE**
	}
	
	posTraj->Lval = EnvNAV4DXYTG.StateID2CoordTable[solution_stateIDs_V[solution_stateIDs_V.size()-1]]->Lval;
	//printf("------- LVal = (%f,%f)\n", posTraj->Lval.real(), posTraj->Lval.imag());
	
	delete tmpPos;
	
	//PostProcessTrajectory(posTraj);
	
	//return (cost);
	
}

EnvNAV4DXYTG_pos_key GetKeyPt(EnvNAV4DXYTG_pos_t pt, bool isstrong)
{
	EnvNAV4DXYTG_pos_key keyPt;
	keyPt.X = pt.X;
	keyPt.Y = pt.Y;
	keyPt.Timet = pt.Timet;
	keyPt.isStrong = isstrong;
	return keyPt;
}


void EnvironmentNAV4DXYTG::PostProcessTrajectory(EnvNAV4DXYTG_pos_trajectory* posTraj)
{
	EnvNAV4DXYTG_pos_trajectory ProcessedPosTraj;
	vector<EnvNAV4DXYTG_pos_t> tmpSegmentPoses;
	vector<EnvNAV4DXYTG_pos_t> tmpSegmentPoses_previous;
	EnvNAV4DXYTG_pos_t startPos, endPos;
	int startIndx, endIndx, b, validSegLen;
	bool isSegmentValid, isEndTask, isEndGoal, isEndConstrained;
	EnvNAV4DXYTG_dist_t* InterpedDist = new EnvNAV4DXYTG_dist_t;
	
	startIndx = 0;
	endIndx = 0;
	ProcessedPosTraj.KeyPts.clear();
	ProcessedPosTraj.KeyPts.push_back(GetKeyPt(posTraj->pos_t[startIndx],true));
	while (true)
	{
		startPos = posTraj->pos_t[startIndx];
		endPos = posTraj->pos_t[endIndx];
		isEndGoal = (endIndx==(posTraj->pos_t.size()-1));
		
		tmpSegmentPoses_previous.clear();
		tmpSegmentPoses_previous = tmpSegmentPoses;
		tmpSegmentPoses.clear();
		tmpSegmentPoses = GenerateStraightSegment(startPos, endPos, EnvNAV4DXYTGCfg.ActionsV);
		printf("(%d,%d,%d)-(%d,%d,%d) --- %d\n", startPos.X, startPos.Y,startPos.Timet, endPos.X, endPos.Y,endPos.Timet, tmpSegmentPoses.size());
		
		isSegmentValid = true;
		for (b=0; b<tmpSegmentPoses.size(); b++)
		{
			isSegmentValid = IsValidCell(tmpSegmentPoses[b].X, tmpSegmentPoses[b].Y, tmpSegmentPoses[b].Timet);
			if (!isSegmentValid)
				break;
		}
		
		if (!isSegmentValid || tmpSegmentPoses.size()==0)
		{
			//printf("Segment a: %d - %d (%d,%d)\n", startIndx, endIndx-1, isSegmentValid, tmpSegmentPoses.size());
			for (b=0; b<tmpSegmentPoses_previous.size(); b++)
				ProcessedPosTraj.pos_t.push_back(tmpSegmentPoses_previous[b]);
			startIndx = endIndx;
			ProcessedPosTraj.KeyPts.push_back(GetKeyPt(posTraj->pos_t[endIndx-1],true));
			ProcessedPosTraj.KeyPts.push_back(GetKeyPt(posTraj->pos_t[startIndx],true));
		}
		else
		{
			isEndTask = false;
			for (b=0; b<EnvNAV4DXYTGCfg.Tasks.size(); b++)
			{
				isEndTask = (endPos.X==EnvNAV4DXYTGCfg.Tasks[b].x) & (endPos.Y==EnvNAV4DXYTGCfg.Tasks[b].y);
				if (isEndTask)
					break;
			}
			isEndConstrained = false;
			if (!isEndTask)
				for (b=0; b<EnvNAV4DXYTGCfg.penaltyWeights.size(); b++)
					if (EnvNAV4DXYTGCfg.penaltyWeights[b]>0)
					{
						FindAndInterpInTrajectory(&EnvNAV4DXYTGCfg.distConstraint_trajectories[b], endPos.Timet, InterpedDist);
						isEndConstrained = (InterpedDist->D < INFINITECOST/10); // & (InterpedDist->D >= 0);
						if (isEndConstrained)
							break;
					}
			
			//printf("constraint: %d - %d - %d, %d\n", isEndConstrained, endPos.Timet, InterpedDist->D, INFINITECOST/10);
			if (isEndTask || isEndGoal || isEndConstrained)
			{
				printf("isEndTask=%d, isEndGoal=%d, isEndConstrained=%d \n", isEndTask, isEndGoal, isEndConstrained);
				for (b=0; b<tmpSegmentPoses.size(); b++)
					ProcessedPosTraj.pos_t.push_back(tmpSegmentPoses[b]);
				ProcessedPosTraj.KeyPts.push_back(GetKeyPt(posTraj->pos_t[endIndx],true));
				if (!isEndGoal)
				{
					endIndx++;
					startIndx = endIndx;
					if (isEndTask)
						ProcessedPosTraj.KeyPts.push_back(GetKeyPt(posTraj->pos_t[startIndx],true));
					else
						ProcessedPosTraj.KeyPts.push_back(GetKeyPt(posTraj->pos_t[startIndx],false));
				}
				else
					break;
			}
			else
				endIndx++;
		}
		
		if (isEndGoal)
			break;
	}
	
	posTraj->pos_t.clear();
	*posTraj = ProcessedPosTraj;
	delete InterpedDist;
}


vector<EnvNAV4DXYTG_pos_t> GenerateStraightSegment(EnvNAV4DXYTG_pos_t startPos, EnvNAV4DXYTG_pos_t endPos, vector<EnvNAV4DXYTGAction_t> ActionsV)
{
	EnvNAV4DXYTG_pos_t tmpPos;
	vector<EnvNAV4DXYTG_pos_t> retPoses;
	int DelX = endPos.X - startPos.X;
	int absDelX = abs(DelX);
	int DelY = endPos.Y - startPos.Y;
	int absDelY = abs(DelY);
	int DelTimet = endPos.Timet - startPos.Timet;
	int tmpDX, tmpDY;
	
	// This works well only for 8-connected grid - TODO: Generelize this
	
/*	if (__max(absDelX,absDelY) != DelTimet)
	{
		printf("^^ %d, %d, %d \n", absDelX,absDelY,DelTimet);
		return retPoses;
	}
	int StandStillCount = DelTimet - __max(absDelX,absDelY); */

/*	if (absDelX > absDelY)
	{
		MaxParallelCount = absDelX - absDelY;
		MaxDiagonalCount = absDelY;
		Parallel_DX = sign(DelX);
		Parallel_DY = 0;
		Diagonal_DX = sign(DelX);
		Diagonal_DY = sign(DelY);
	}else{
		MaxParallelCount = absDelY - absDelX;
		MaxDiagonalCount = absDelX;
		Parallel_DY = sign(DelY);
		Parallel_DX = 0;
		Diagonal_DX = sign(DelX);
		Diagonal_DY = sign(DelY);
	} */
	
	if (DelTimet<__max(absDelX,absDelY))
		DelTimet = __max(absDelX,absDelY);
	
	for (int tt=0; tt<=DelTimet; tt++)
	{
		if (DelTimet==0)
		{
			tmpDX = 0;
			tmpDY = 0;
		}else{
			tmpDX = (int)round(((double)DelX) * ((double)tt) / ((double)DelTimet));
			tmpDY = (int)round(((double)DelY) * ((double)tt) / ((double)DelTimet));
		}
		tmpPos.X = startPos.X + tmpDX;
		tmpPos.Y = startPos.Y + tmpDY;
		tmpPos.Timet = startPos.Timet + tt;
		retPoses.push_back(tmpPos);
	}
	
	//printf("-- %d\n", retPoses.size());
	return retPoses;
}

// ----------------------------------------

void PostProcessTrajectory_Joint(vector<EnvironmentNAV4DXYTG*> robEnvs, vector<EnvNAV4DXYTG_pos_trajectory*> posTrajs)
{
	// Assumption: All the robot trajectories have non-negative integer values for Timet with increment 1
	int maxTimeT=0, tt, a, b, c;
	int lastIndx, otherRobIdentifier, thisTimet;
	int OldViolation, NewViolation;
	int RobCount = posTrajs.size();
	vector<int> startIndx (RobCount,-1);
	vector<int> endIndx (RobCount,-1);
	//vector<bool> isSegmentConstrained (RobCount, false);
	//vector< vector<int> > constrainedRobIdentifiers (RobCount);
	
	vector<EnvNAV4DXYTG_pos_trajectory> ProcessedPosTraj (RobCount);
	vector<EnvNAV4DXYTG_pos_trajectory> tmpNewTraj (RobCount);
	vector< vector<EnvNAV4DXYTG_pos_t> > tmpSegmentPoses (RobCount);
	vector< vector<EnvNAV4DXYTG_pos_t> > tmpSegmentPoses_previous (RobCount);
	EnvNAV4DXYTG_pos_t startPos, endPos;
	bool isSegmentValid, isEndTask, isConstrainExist;
	EnvNAV4DXYTG_dist_t* InterpedDist = new EnvNAV4DXYTG_dist_t;
	EnvNAV4DXYTG_pos_t OldTrajPosThis, OldTrajPosOther, NewTrajPosThis, NewTrajPosOther;
	
	vector<bool> freezePreviousSegment (RobCount, false);
	vector<bool> freezePreviousSegmentForConstraintViolation (RobCount, false);

	
	for (a=0; a<RobCount; a++)
	{
		lastIndx = posTrajs[a]->pos_t.size() - 1;
		maxTimeT = __max(maxTimeT, posTrajs[a]->pos_t[lastIndx].Timet);
	}
	
	for (tt=0; tt<=maxTimeT; tt++)
	{
	
		// Generate new segment
		for (a=0; a<RobCount; a++)
		{
			freezePreviousSegmentForConstraintViolation[a] = false;
			
			tmpSegmentPoses_previous[a].clear();
			tmpSegmentPoses_previous[a] = tmpSegmentPoses[a];
			tmpSegmentPoses[a].clear();
		
			if (posTrajs[a]->pos_t[0].Timet==tt && startIndx[a]==-1)
			{
				ProcessedPosTraj[a].pos_t.clear();
				ProcessedPosTraj[a].KeyPts.clear();
				ProcessedPosTraj[a].KeyPts.push_back(GetKeyPt(posTrajs[a]->pos_t[0],true));
				startIndx[a] = 0;
				endIndx[a] = 0;
			}
			if (startIndx[a] < 0)
				continue;
			else
				endIndx[a]++; // endIndx in sync
			
			// Leave an empty Segment if past goal
			if (endIndx[a] < posTrajs[a]->pos_t.size())
			{
				startPos = posTrajs[a]->pos_t[startIndx[a]];
				endPos = posTrajs[a]->pos_t[endIndx[a]];
				if (robEnvs.size()>a)
					tmpSegmentPoses[a] = GenerateStraightSegment(startPos, endPos, robEnvs[a]->EnvNAV4DXYTGCfg.ActionsV);
				else
					tmpSegmentPoses[a] = GenerateStraightSegment(startPos, endPos, robEnvs[0]->EnvNAV4DXYTGCfg.ActionsV);
				
				tmpNewTraj[a] = ProcessedPosTraj[a];
				for (b=0; b<tmpSegmentPoses[a].size(); b++)
					tmpNewTraj[a].pos_t.push_back(tmpSegmentPoses[a][b]);
			}
			
		}
		
		
			// -----------------------------
			/*
			isSegmentValid = true;
			for (b=0; b<tmpSegmentPoses[a].size(); b++)
			{
				isSegmentValid = robEnvs[a]->IsValidCell(tmpSegmentPoses[b].X, tmpSegmentPoses[b].Y, tmpSegmentPoses[b].Timet);
				if (!isSegmentValid)
					break;
			}*/
			/*
			if (!isSegmentValid || isSegmentConstrained[a] || tmpSegmentPoses[a].size()==0)
			{
				for (b=0; b<tmpSegmentPoses_previous[a].size(); b++)
					ProcessedPosTraj[a].pos_t.push_back(tmpSegmentPoses_previous[a][b]);
				startIndx = endIndx;
				ProcessedPosTraj[a].KeyPts.push_back(GetKeyPt(posTrajs[a]->pos_t[endIndx-1],true));
				ProcessedPosTraj[a].KeyPts.push_back(GetKeyPt(posTrajs[a]->pos_t[startIndx],true));
				
				constrainedRobIdentifiers[a].clear();
			}
			else
			{
				isEndTask = false;
				for (b=0; b<robEnvs[a]->EnvNAV4DXYTGCfg.Tasks.size(); b++)
				{
					isEndTask = (endPos.X==robEnvs[a]->EnvNAV4DXYTGCfg.Tasks[b].x) & (endPos.Y==robEnvs[a]->EnvNAV4DXYTGCfg.Tasks[b].y);
					if (isEndTask)
						break;
				}
				isEndConstrained = false;
				if (!isEndTask)
				{
					constrainedRobIdentifiers[a].clear();
					constrainedRobMinDists[a].clear();
					for (b=0; b<robEnvs[a]->EnvNAV4DXYTGCfg.penaltyWeights.size(); b++)
						if (robEnvs[a]->EnvNAV4DXYTGCfg.penaltyWeights[b]>0)
						{
							FindAndInterpInTrajectory(&robEnvs[a]->EnvNAV4DXYTGCfg.distConstraint_trajectories[b], endPos.Timet, InterpedDist);
							isEndConstrained = (InterpedDist->D < INFINITECOST/10); // & (InterpedDist->D >= 0);
							if (isEndConstrained)
							{
								constrainedRobIdentifiers[a].push_back(robEnvs[a]->EnvNAV4DXYTGCfg.otherBots_identifiers[b]);
								//constrainedRobMinDists[a].push_back(InterpedDist->D);
							}
								//break;
						}
				}
				*/
				/*
				//printf("constraint: %d - %d - %d, %d\n", isEndConstrained, endPos.Timet, InterpedDist->D, INFINITECOST/10);
				if (isEndTask || isEndGoal || isEndConstrained)
				{
					//printf("Segment b: %d - %d \n", startIndx, endIndx);
					for (b=0; b<tmpSegmentPoses.size(); b++)
						ProcessedPosTraj.pos_t.push_back(tmpSegmentPoses[b]);
					ProcessedPosTraj.KeyPts.push_back(posTraj->pos_t[endIndx]);
					if (!isEndGoal)
					{
						//endIndx++;
						startIndx = endIndx;
						if (isEndConstrained)
							//ProcessedPosTraj.KeyPts.push_back(GetKeyPt(posTraj->pos_t[startIndx],true));
						else
							ProcessedPosTraj.KeyPts.push_back(GetKeyPt(posTraj->pos_t[startIndx],true));
					}
					else
						break;
				}
				else
					//endIndx++;
				
					
			} */
		

		// Check the new segment(s) for validity
		for (a=0; a<RobCount; a++)
		{
			freezePreviousSegment[a] = false;
			
			// Past goal
			if (tmpSegmentPoses[a].size()==0)
			{
				freezePreviousSegment[a] = true;
				continue;
			}
			
			// Checking for intersection with obstacle
			isSegmentValid = true;
			for (b=0; b<tmpSegmentPoses[a].size(); b++)
			{
				isSegmentValid = robEnvs[a]->IsValidCell(tmpSegmentPoses[a][b].X, tmpSegmentPoses[a][b].Y, tmpSegmentPoses[a][b].Timet);
				if (!isSegmentValid)
					break;
			}
			if (!isSegmentValid)
			{
				freezePreviousSegment[a] = true;
				continue;
			}
			
			// Checking for task at the end of the segment
			isEndTask = false;
			endPos = tmpSegmentPoses[a][tmpSegmentPoses[a].size()-1];
			for (b=0; b<robEnvs[a]->EnvNAV4DXYTGCfg.Tasks.size(); b++)
			{
				isEndTask = (endPos.X==robEnvs[a]->EnvNAV4DXYTGCfg.Tasks[b].x) & (endPos.Y==robEnvs[a]->EnvNAV4DXYTGCfg.Tasks[b].y);
				if (isEndTask)
					break;
			}
			if (isEndTask)
			{
				freezePreviousSegment[a] = true;
				continue;
			}
			
			// Checking for constraints - all along the segments
			if (!freezePreviousSegmentForConstraintViolation[a])
				for (b=0; b<robEnvs[a]->EnvNAV4DXYTGCfg.penaltyWeights.size(); b++)
					if (robEnvs[a]->EnvNAV4DXYTGCfg.penaltyWeights[b]>0)
					{
						otherRobIdentifier = robEnvs[a]->EnvNAV4DXYTGCfg.otherBots_identifiers[b]; // Robot with which there is +ve weight
						for (c=tmpSegmentPoses[a].size()-1; c>=0; c--) // Checking each point in the segment
						{
							thisTimet = tmpSegmentPoses[a][c].Timet;
							FindAndInterpInTrajectory(&robEnvs[a]->EnvNAV4DXYTGCfg.distConstraint_trajectories[b], thisTimet, InterpedDist);
							isConstrainExist = (InterpedDist->D < INFINITECOST/10); // & (InterpedDist->D >= 0);
							if (isConstrainExist)
							{
								// Check if the constraint violation in the new trajectory segments at this point are same or zero
								FindAndInterpInTrajectory(posTrajs[a], thisTimet, &OldTrajPosThis);
								FindAndInterpInTrajectory(posTrajs[otherRobIdentifier], thisTimet, &OldTrajPosOther);
								FindAndInterpInTrajectory(&tmpNewTraj[a], thisTimet, &NewTrajPosThis);
								FindAndInterpInTrajectory(&tmpNewTraj[otherRobIdentifier], thisTimet, &NewTrajPosOther);
							
								OldViolation = CELLCOST_XYT(OldTrajPosThis.X-OldTrajPosOther.X, OldTrajPosThis.Y-OldTrajPosOther.Y, 0, InterpedDist->D);
								NewViolation = CELLCOST_XYT(NewTrajPosThis.X-NewTrajPosOther.X, NewTrajPosThis.Y-NewTrajPosOther.Y, 0, InterpedDist->D);
								if ((OldViolation>0 || NewViolation>0) && OldViolation!=NewViolation)
								{
									freezePreviousSegmentForConstraintViolation[a] = true;
									freezePreviousSegmentForConstraintViolation[otherRobIdentifier] = true;
									break;
								}
							
							}
						}
					}
			if (freezePreviousSegmentForConstraintViolation[a])
				freezePreviousSegment[a] = true;
				
		}
			
		
		// Freeze the previous segment
		for (a=0; a<RobCount; a++)
		{
			if (freezePreviousSegment[a] && tmpSegmentPoses_previous[a].size()>0)
			{
				for (b=0; b<tmpSegmentPoses_previous[a].size(); b++)
					ProcessedPosTraj[a].pos_t.push_back(tmpSegmentPoses_previous[a][b]);
				ProcessedPosTraj[a].KeyPts.push_back(GetKeyPt(posTrajs[a]->pos_t[endIndx[a]-1],true));
				startIndx[a] = endIndx[a];
			}
		}
			
		
			//if (isEndGoal)
				//break;
			
			/*
			if (tt >= anchorT[a] && )
			{
				tmpSegmentPoses_previous[a].clear();
				tmpSegmentPoses_previous[a] = tmpSegmentPoses[a];
				tmpSegmentPoses[a].clear();
				startPos = 
				tmpSegmentPoses[a] = GenerateStraightSegment(startPos, endPos, robEnvs[a]->EnvNAV4DXYTGCfg.ActionsV);
		
				isSegmentValid = true;
				for (b=0; b<tmpSegmentPoses.size(); b++)
				{
					isSegmentValid = IsValidCell(tmpSegmentPoses[b].X, tmpSegmentPoses[b].Y, tmpSegmentPoses[b].Timet);
					if (!isSegmentValid)
						break;
				}
			}
			
		}*/
		
		// Check the tmpSegmentPoses for consistency among pairs of activeRobots
	}
	
	for (a=0; a<RobCount; a++)
	{
		posTrajs[a]->pos_t.clear();
		//*posTrajs[a] = ProcessedPosTraj[a];
		posTrajs[a]->pos_t = ProcessedPosTraj[a].pos_t;
		posTrajs[a]->KeyPts = ProcessedPosTraj[a].KeyPts;
	}
	delete InterpedDist;
}

// ----------------------------------------


void EnvironmentNAV4DXYTG::GetCoordFromState(int stateID, int& x, int& y, int& Timet, int& g) const {
  EnvNAV4DXYTGHashEntry_t* HashEntry = EnvNAV4DXYTG.StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
  Timet = HashEntry->Timet;
  g = HashEntry->G;
}

/*
int EnvironmentNAV4DXYTG::GetStateFromCoord(int x, int y, int Timet, int g) {

   EnvNAV4DXYTGHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, Timet, g)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, Timet, g);
    }
    return OutHashEntry->stateID;
}*/

int EnvironmentNAV4DXYTG::GetStateFromCoord(int x, int y, int Timet, int g, complex<double> Lval) {

   EnvNAV4DXYTGHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, Timet, g, Lval)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, Timet, g, Lval);
    }
    return OutHashEntry->stateID;
}


const EnvNAV4DXYTGConfig_t* EnvironmentNAV4DXYTG::GetEnvNavConfig() {
  return &EnvNAV4DXYTGCfg;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAV4DXYTG::SetGoal(double x_m, double y_m, double Timet_rad, int g){

	int x = CONTXY2DISC(x_m, EnvNAV4DXYTGCfg.cellsize_m);
	int y = CONTXY2DISC(x_m, EnvNAV4DXYTGCfg.cellsize_m);
	int Timet = CONTXY2DISC(Timet_rad, EnvNAV4DXYTGCfg.timestepsize_m);

    if(!IsWithinMapCell(x,y,Timet))
        return -1;

    EnvNAV4DXYTGHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, Timet, g, complex<double>(), true)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, Timet, g, complex<double>(), true);
    }
    EnvNAV4DXYTG.goalstateid = OutHashEntry->stateID;

    return EnvNAV4DXYTG.goalstateid;    

}


//returns the stateid if success, and -1 otherwise
int EnvironmentNAV4DXYTG::SetStart(double x_m, double y_m, double Timet_rad, int g, complex<double> LVal){

	int x = CONTXY2DISC(x_m, EnvNAV4DXYTGCfg.cellsize_m);
	int y = CONTXY2DISC(x_m, EnvNAV4DXYTGCfg.cellsize_m);
	int Timet = CONTXY2DISC(Timet_rad, EnvNAV4DXYTGCfg.timestepsize_m);

    if(!IsWithinMapCell(x,y,Timet))
        return -1;

    EnvNAV4DXYTGHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, Timet, g, LVal)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, Timet, g, LVal);
    }
    EnvNAV4DXYTG.startstateid = OutHashEntry->stateID;

    return EnvNAV4DXYTG.startstateid;    

}

/*
bool EnvironmentNAV4DXYTG::UpdateCost(int x, int y, int new_status)
{

    EnvNAV4DXYTGCfg.Grid2D[x][y] = new_status;

    return true;
}
*/


void EnvironmentNAV4DXYTG::PrintEnv_Config(FILE* fOut)
{

	//implement this if the planner needs to print out EnvNAV4DXYTG. configuration
	
	printf("ERROR in EnvNAV4DXYTG... function: PrintEnv_Config is undefined\n");
	exit(1);

}

void EnvironmentNAV4DXYTG::PrintTimeStat(FILE* fOut)
{

#if TIME_DEBUG
    fprintf(fOut, "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, time_getsuccs = %f\n",
            time3_addallout/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC, 
            time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
}


/*
void EnvironmentNAV4DXYTG::GetPredsofChangedEdges(vector<nav2dcell_t>* changedcellsV, vector<int> *preds_of_changededgesIDV)
{
	nav2dcell_t cell;

	for(int i = 0; i < (int)changedcellsV->size(); i++)
	{
		cell = changedcellsV->at(i);
		for(int tind = 0; tind < NAV4DXYTG_TimetDIRS; tind++)
			preds_of_changededgesIDV->push_back(GetStateFromCoord(cell.x,cell.y,tind));
		for(int j = 0; j < 8; j++){
			int affx = cell.x + EnvNAV4DXYTGCfg.dXY[j][0];
			int affy = cell.y + EnvNAV4DXYTGCfg.dXY[j][1];
			if(affx < 0 || affx >= EnvNAV4DXYTGCfg.EnvWidth_c || affy < 0 || affy >= EnvNAV4DXYTGCfg.EnvHeight_c)
				continue;
			for(int tind = 0; tind < NAV4DXYTG_TimetDIRS; tind++)
				preds_of_changededgesIDV->push_back(GetStateFromCoord(affx,affy,tind));
		}
	}
}
*/


/*
bool EnvironmentNAV4DXYTG::IsObstacle(int x, int y)
{

	return (EnvNAV4DXYTGCfg.Grid2D[x][y] != 0);

}
*/


void EnvironmentNAV4DXYTG::GetEnvParms(int *size_x, int *size_y, double* startx, double* starty, double*startTimet, 
							double* goalx, double* goaly, double* goalTimet, double* cellsize_m)
{
	*size_x = EnvNAV4DXYTGCfg.EnvWidth_c;
	*size_y = EnvNAV4DXYTGCfg.EnvHeight_c;

	*startx = DISCXY2CONT(EnvNAV4DXYTGCfg.StartX_c, EnvNAV4DXYTGCfg.cellsize_m);
	*starty = DISCXY2CONT(EnvNAV4DXYTGCfg.StartY_c, EnvNAV4DXYTGCfg.cellsize_m);
	*startTimet = DISCXY2CONT(EnvNAV4DXYTGCfg.StartTimet_c, EnvNAV4DXYTGCfg.timestepsize_m);
	*goalx = DISCXY2CONT(EnvNAV4DXYTGCfg.EndX_c, EnvNAV4DXYTGCfg.cellsize_m);
	*goaly = DISCXY2CONT(EnvNAV4DXYTGCfg.EndY_c, EnvNAV4DXYTGCfg.cellsize_m);
	*goalTimet = DISCXY2CONT(EnvNAV4DXYTGCfg.EndTimet_c, EnvNAV4DXYTGCfg.timestepsize_m);

	*cellsize_m = EnvNAV4DXYTGCfg.cellsize_m;
	// *nominalvel_mpersecs = EnvNAV4DXYTGCfg.nominalvel_mpersecs;
	// *Timetoturn45degsinplace_secs = EnvNAV4DXYTGCfg.Timetoturn45degsinplace_secs;
}

//------------------------------------------------------------------------------
/*
PenaltyTracker_t::PenaltyTracker_t(char* method, char* paramsString)
{
	char checkMethod[1024];
	float f1, f2, f3, f4;
	
	strcpy(PenaltyChangeMethod, method);
	
	strcpy(checkMethod, "<FixedIncrement>");
	if(strcmp(PenaltyChangeMethod, checkMethod) == 0)
	{
		sscanf(paramsString, "%f %f", &f1, &f2);
		PenaltyChangeParams.push_back(f1);
		PenaltyChangeParams.push_back(f2);
		Weight = PenaltyChangeParams[0];
		return;
	}
	
	strcpy(checkMethod, "<AutoIncrement>");
	if(strcmp(PenaltyChangeMethod, checkMethod) == 0)
	{
		sscanf(paramsString, "%f %f", &f1, &f2);
		PenaltyChangeParams.push_back(f1);
		PenaltyChangeParams.push_back(f2);
		Weight = PenaltyChangeParams[0];
		return;
	}
}

void PenaltyTracker_t::next(EnvNAV4DXYTG_pos_trajectory* ThisBotLatestTraj, EnvironmentNAV4DXYTG* env)
{
	char checkMethod[1024];
	
	strcpy(checkMethod, "<FixedIncrement>");
	if(strcmp(PenaltyChangeMethod, checkMethod) == 0)
	{
		Weight = Weight + PenaltyChangeParams[1];
		return;
	}
	
	strcpy(checkMethod, "<AutoIncrement>");
	if(strcmp(PenaltyChangeMethod, checkMethod) == 0)
	{
		Weight = Weight + FindPenaltyWeightIncrement(Weight, EnvNAV4DXYTG_pos_trajectory* traj, EnvironmentNAV4DXYTG* env);
		return;
	}
}

void PenaltyTracker_t::reset(void)
{
	char checkMethod[1024];
	
	strcpy(checkMethod, "<FixedIncrement>");
	if(strcmp(PenaltyChangeMethod, checkMethod) == 0)
	{
		Weight = PenaltyChangeParams[0];
		return;
	}
}
*/
//------------------------------------------------------------------------------
bool RandomizeVals;

void ReadConfigurationFile(const char* sEnvFile, ConfigFileInfo* theInfo)
{
	char thisLine[1024], firstString[1024], checkString[1024];
	char thisLineLong[8000];
	char s1[1024], s2[1024], s3[1024], s4[1024], s5[1024], s6[1024];
	bool FileEnd=false;
	int i1, i2, i3, i4;
	int a, b, c, d, X, Y, dTimet, dX, dY, al, bl, InfRad, tCount;
	float f1, f2, f3, f4;
	vector<float> vf1, vf2, vf3, vf4;
	EnvNAV4DXYTG_dist_t thisDistPoint;
	EnvNAV4DXYTG_AParticularRobot_t* thisRobot;
	EnvNAV4DXYTG2DptInt_t* thisTask;
	EnvNAV4DXYTG_AParticularConstraint_t* thisConstraint;
	EnvNAV4DXYTGAction_t* TmpActionsV;
	// Tracking numbers:
	int robotNo=-1;
	int obstacleNo=-1;
	RandomizeVals = false;
	float RandomizeAmount_xy = 0.5;
	float RandomizeAmount_t = 5.0;
	//InfRad = theInfo->GlobalParams.STATIC_OBSTACLE_INFLATION_RADIUS;
	vector<int> HomotopyCriticalPt_Xs, HomotopyCriticalPt_Ys;
	
	theInfo->RobotCount = 0;
	
	FILE* fCfg = fopen(sEnvFile, "r");
	if(fCfg == NULL)
	{
		printf("ERROR: unable to open %s\n", sEnvFile);
		exit(1);
	}
	
	bool insideROBOT = false;
	bool insideCONSTRAINT = false;
	bool insideSTATICOBSTACLE = false;
	bool insideDYNAMICOBSTACLE = false;
	
	while(!FileEnd)
	{
		strcpy(thisLine,"");
		fgets(thisLine, 1024 , fCfg);
		if(strlen(thisLine)<=2)
			continue;
		sscanf(thisLine, "%s", firstString);
		//printf("%s\n",thisLine);
		//char tmpc;
		//scanf("%c", &tmpc);
		//printf("Parsing line: %sFirst String: %s\n", thisLine, firstString);
		
		/*** Environment size and discretization information ***/
				
		// discretization
		strcpy(checkString, "discretization:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %d %d %d", &i1, &i2, &i3);
			theInfo->EnvWidth_c = i1;
			theInfo->EnvHeight_c = i2;
			theInfo->EnvMaxTime_c = i3;
			theInfo->StaticObstacleMap.init(theInfo->EnvWidth_c, theInfo->EnvHeight_c, false);
			continue;
		}
		
		// cellsize
		strcpy(checkString, "cellsize:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %f %f", &f1, &f2);
			theInfo->cellsize_m = f1;
			theInfo->timestepsize_m =f2;
			continue;
		}
		
		/*** Global Parameters ***/
		
		// ViolationCostPower
		strcpy(checkString, "ViolationCostPower:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %f", &f1);
			theInfo->GlobalParams.VIOLATION_COST_POWER = f1;
			continue;
		}
		
		// HeuristicType
		strcpy(checkString, "HeuristicType:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %d", &i1);
			theInfo->GlobalParams.HEURISTIC_TYPE = i1;
			continue;
		}
		
		// PrecomputeHeuristic
		strcpy(checkString, "PrecomputeHeuristic:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %d", &i1);
			theInfo->GlobalParams.PRECOMPUTE_HEURISTIC = i1;
			continue;
		}
		
		// CollisionCheckRadius
		strcpy(checkString, "CollisionCheckRadius:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %d", &i1);
			theInfo->GlobalParams.COLLISION_CHECK_RADIUS = i1;
			continue;
		}
		
		// IsIterationSymmetric
		strcpy(checkString, "IsIterationSymmetric:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %s", s2);
			strcpy(checkString, "TRUE");
			if(strcmp(s2, checkString) == 0)
				theInfo->GlobalParams.IS_ITERATION_SYMMETRIC = true;
			else
				theInfo->GlobalParams.IS_ITERATION_SYMMETRIC = false;
			continue;
		}
		
		// RandomizeVals
		strcpy(checkString, "RandomizeVals:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %s %d", s2, &i1);
			strcpy(checkString, "TRUE");
			if(strcmp(s2, checkString) == 0)
			{
				RandomizeVals = true;
				srand(i1);
			}
			else
				RandomizeVals = false;
			continue;
		}
		
		// DoJointStatespeceLogging
		strcpy(checkString, "DoJointStatespeceLogging:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %s", s2);
			strcpy(checkString, "TRUE");
			if(strcmp(s2, checkString) == 0)
				theInfo->GlobalParams.DO_JOINTSTATESPACE_LOGGING = true;
			else
				theInfo->GlobalParams.DO_JOINTSTATESPACE_LOGGING = false;
			fgets(thisLine, 1024 , fCfg);
			sscanf(thisLine, "%d %d", &i1, &i2);
			theInfo->GlobalParams.JOINTSTATESPACE_LOGGING_METHOD = i1;
			theInfo->GlobalParams.JOINTSTATESPACE_LOGGING_RAD = i2;
			continue;
		}
		
		// IterationType
		strcpy(checkString, "IterationType:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %d", &i1);
			theInfo->GlobalParams.ITERATION_TYPE = i1;
			fgets(thisLine, 1024 , fCfg);
			sscanf(thisLine, "%d %d %d", &i2, &i3, &i4);
			theInfo->GlobalParams.MAX_ITERATION_COUNT = i2;
			theInfo->GlobalParams.MIN_ITERATION_COUNT = i3;
			theInfo->GlobalParams.CONVERGENCE_CYCLE_COUNT = i4;
			continue;
		}
		
		// MaxSuperIter:
		strcpy(checkString, "MaxSuperIter:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %d", &i1);
			theInfo->GlobalParams.MAX_SUPERITER_COUNT = i1;
			continue;
		}
		
		/*** Robot information ***/
		
		// BEGIN_ROBOT
		strcpy(checkString, "BEGIN_ROBOT:");
		if(strcmp(checkString, firstString) == 0)
		{
			insideROBOT = true;
			thisRobot = new EnvNAV4DXYTG_AParticularRobot_t;
			sscanf(thisLine, "%*s %d", &robotNo);
			if(theInfo->TheRobots.size() >= robotNo)
				theInfo->TheRobots.resize(robotNo+1);
			theInfo->RobotCount = theInfo->TheRobots.size();
			continue;
		}
		
		// start
		strcpy(checkString, "start:");
		if(strcmp(checkString, firstString) == 0 && insideROBOT)
		{
			sscanf(thisLine, "%*s %s %s %s", s1, s2, s3);
			f1 = sampled_atof(s1);
			f2 = sampled_atof(s2);
			f3 = sampled_atof(s3);
			thisRobot->StartX_c = CONTXY2DISC(f1,theInfo->cellsize_m);
			thisRobot->StartY_c = CONTXY2DISC(f2,theInfo->cellsize_m);
			thisRobot->StartTimet_c = CONTXY2DISC(f3,theInfo->timestepsize_m);
			//printf("\n %^%^%^ Start: %d, %d, %d\n", thisRobot->StartX_c, thisRobot->StartY_c, thisRobot->StartTimet_c);
			continue;
		}
		
		// tasks
		strcpy(checkString, "task:");
		if(strcmp(checkString, firstString) == 0 && insideROBOT)
		{
			thisTask = new EnvNAV4DXYTG2DptInt_t;
			sscanf(thisLine, "%*s %s %s", s1, s2);
			f1 = sampled_atof(s1);
			f2 = sampled_atof(s2);
			thisTask->x = CONTXY2DISC(f1,theInfo->cellsize_m);
			thisTask->y = CONTXY2DISC(f2,theInfo->cellsize_m);
			thisRobot->Tasks.push_back(*thisTask);
			continue;
		}
		
		// end
		strcpy(checkString, "end:");
		if(strcmp(checkString, firstString) == 0 && insideROBOT)
		{
			sscanf(thisLine, "%*s %s %s %s", s1, s2, s3);
			f1 = sampled_atof(s1);
			f2 = sampled_atof(s2);
			f3 = sampled_atof(s3);
			thisRobot->EndX_c = CONTXY2DISC(f1,theInfo->cellsize_m);
			thisRobot->EndY_c = CONTXY2DISC(f2,theInfo->cellsize_m);
			thisRobot->EndTimet_c = CONTXY2DISC(f3,theInfo->timestepsize_m);
			//printf("\n %^%^%^ End: %d, %d, %d\n", thisRobot->EndX_c, thisRobot->EndY_c, thisRobot->EndTimet_c);
			continue;
		}
		
		// connectivity
		strcpy(checkString, "connectivity:");
		if(strcmp(checkString, firstString) == 0 && insideROBOT)
		{
			sscanf(thisLine, "%*s %d %d", &i1, &i2);
			for (Y = 0; Y < i2; Y++)
			{
				fgets(thisLine, 1024 , fCfg);
				for (X = 0; X < i2; X++)
				{
					char* formatString = new char[1024];
					for (a=0; a<X; a++)
						strcat(formatString, "%*d ");
					strcat(formatString, "%d");
					sscanf(thisLine, formatString, &i3);
					if(i3 == 1)
					{
						dTimet = i1;
						dX = X - (i2-1)/2;
						dY = Y - (i2-1)/2;
						TmpActionsV = new EnvNAV4DXYTGAction_t;
						TmpActionsV->dTimet = dTimet;
						TmpActionsV->dX = dX;
						TmpActionsV->dY = dY;
						TmpActionsV->cost = TRANSITIONCOST_XYT(dX, dY, dTimet);
							
						thisRobot->ActionsV.push_back(*TmpActionsV);
					}
					//delete[] formatString;  // MEM_CLEAR
				}
			}
			continue;
		}
		
		// BlockedHomotopyClass
		strcpy(checkString, "BlockedHomotopyClass:");
		if(strcmp(checkString, firstString) == 0 && insideROBOT)
		{
			sscanf(thisLine, "%*s %f %f", &f1, &f2);
			thisRobot->BlockedHomotopyClass_Const_LVals.push_back(complex<double>((double)f1,(double)f2));
			continue;
		}
		
		
		
		// END_ROBOT
		strcpy(checkString, "END_ROBOT");
		if(strcmp(checkString, firstString) == 0)
		{
			insideROBOT = false;
			theInfo->TheRobots[robotNo] = *thisRobot;
			continue;
		}
		
		/*** Distance constraint information ***/
		
		// BEGIN_CONSTRAINT
		strcpy(checkString, "BEGIN_CONSTRAINT:");
		if(strcmp(checkString, firstString) == 0)
		{
			insideCONSTRAINT = true;
			thisConstraint = new EnvNAV4DXYTG_AParticularConstraint_t;
			sscanf(thisLine, "%*s %d %d", &i1, &i2);
			thisConstraint->Robot1 = i1;
			thisConstraint->Robot2 = i2;
			EnvNAV4DXYTG_dist_t tmpDist;
			/*for (int m=0; m<(thisRobot->EndTimet_c-thisRobot->StartTimet_c); m++)
			{
				tmpDist.Timet = m + thisRobot->StartTimet_c;
				tmpDist.D = 100 * INFINITECOST / NAV4DXYTG_COSTMULT;
				thisConstraint->constraint.dist_t.push_back(tmpDist);
			}*/
			continue;
		}
		
		/*
		// penaltyweight
		strcpy(checkString, "penaltyweight:");
		if(strcmp(checkString, firstString) == 0 && insideCONSTRAINT)
		{
			sscanf(thisLine, "%*s %s", s1);
			fgets(thisLine, 1024 , fCfg);
			PenaltyTracker_t* pt = new PenaltyTracker_t((char*)s1, (char*)thisLine);
			thisConstraint->Penalty = *pt;
			continue;
		}
		*/
		
		// PenaltyWeightIncrementMethod
		strcpy(checkString, "PenaltyWeightIncrementMethod:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %d", &i1);
			theInfo->GlobalParams.PENALTY_WEIGHT_INCREMENT_METHOD = i1;
			fgets(thisLine, 1024 , fCfg);
			sscanf(thisLine, "%d %f", &i2, &f2);
			theInfo->GlobalParams.PENALTY_BIN_SEARCH_MAXSTEPS = i2;
			theInfo->GlobalParams.PENALTY_BIN_SEARCH_THRESH = f2;
			//theInfo->GlobalParams.PENALTY_INCREMENT_MULT_FAC = f3;
			continue;
		}
		
		// penaltyweightparams
		strcpy(checkString, "penaltyweightparams:");
		if(strcmp(checkString, firstString) == 0 && insideCONSTRAINT)
		{
			sscanf(thisLine, "%*s %f %f %f %f", &f1, &f2, &f3, &f4);
			thisConstraint->PenaltyWeightParams.push_back(f1);
			thisConstraint->PenaltyWeightParams.push_back(f2);
			thisConstraint->PenaltyWeightParams.push_back(f3);
			thisConstraint->PenaltyWeightParams.push_back(f4);
			continue;
		}
		
		// constraintprofile
		strcpy(checkString, "constraintprofile:");
		if(strcmp(checkString, firstString) == 0 && insideCONSTRAINT)
		{
			continue;
		}
		
		// Range
		strcpy(checkString, "Range:");
		if(strcmp(checkString, firstString) == 0 && insideCONSTRAINT)
		{
			sscanf(thisLine, "%*s %s %s %s", s1, s2, s3);
			f1 = sampled_atof(s1);
			f2 = sampled_atof(s2);
			f3 = sampled_atof(s3);
			i1 = CONTXY2DISC(f1,theInfo->timestepsize_m);
			i2 = CONTXY2DISC(f2,theInfo->timestepsize_m);
			//f3 = f3 / theInfo->cellsize_m;
			if(f3<0.0)
				f3 = theInfo->cellsize_m * INFINITECOST / NAV4DXYTG_COSTMULT;
			for(a=i1; a<=i2; a++)
			{
				thisDistPoint.Timet = a;
				thisDistPoint.D = CONTXY2DISC(f3*NAV4DXYTG_COSTMULT,theInfo->cellsize_m);
				PutInTrajectory(&(thisConstraint->constraint), thisDistPoint);
				#if DEBUG4
					printf("Parsing Range: %d - %f %ld | %f, %f\n",thisDistPoint.Timet,f3,thisDistPoint.D, (f3*NAV4DXYTG_COSTMULT/theInfo->cellsize_m), theInfo->cellsize_m);
				#endif
			}
			continue;
		}
		
		// Point
		strcpy(checkString, "Point:");
		if(strcmp(checkString, firstString) == 0 && insideCONSTRAINT)
		{
			sscanf(thisLine, "%*s %s %s", s1, s3);
			f1 = sampled_atof(s1);
			i1 = CONTXY2DISC(f1,theInfo->timestepsize_m);
			f3 = sampled_atof(s3);
			//f3 = f3 / theInfo->cellsize_m;
			if(f3<0.0)
				f3 = 100 * INFINITECOST / NAV4DXYTG_COSTMULT;
			thisDistPoint.Timet = i1;
			thisDistPoint.D = CONTXY2DISC(f3*NAV4DXYTG_COSTMULT,theInfo->cellsize_m);
			PutInTrajectory(&(thisConstraint->constraint), thisDistPoint);
			#if DEBUG4
				printf("Parsing Point: %d - %f %d\n",thisDistPoint.Timet,f3,thisDistPoint.D);
			#endif
			continue;
		}
		
		// END_CONSTRAINT
		strcpy(checkString, "END_CONSTRAINT");
		if(strcmp(checkString, firstString) == 0)
		{
			insideCONSTRAINT = false;
			theInfo->TheConstraints.push_back(*thisConstraint);
			//for (int m=0; m<theInfo->TheConstraints[theInfo->TheConstraints.size()-1].constraint.dist_t.size(); m++)
			//	printf("Conatrsint pt: Timet=%d, D=%ld\n", theInfo->TheConstraints[theInfo->TheConstraints.size()-1].constraint.dist_t[m].Timet, theInfo->TheConstraints[theInfo->TheConstraints.size()-1].constraint.dist_t[m].D);
			AConstraintOfARobot_t aConstraint;
			aConstraint.DistTraj = thisConstraint->constraint;
			aConstraint.PenaltyParams = thisConstraint->PenaltyWeightParams;
			aConstraint.RobotIndex = thisConstraint->Robot1;
			theInfo->TheRobots[thisConstraint->Robot2].constraints.push_back(aConstraint);
			aConstraint.RobotIndex = thisConstraint->Robot2;
			theInfo->TheRobots[thisConstraint->Robot1].constraints.push_back(aConstraint);
			continue;
		}
		
		// BEGIN_STATIC_OBSTACLE
		strcpy(checkString, "BEGIN_STATIC_OBSTACLE:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %d", &i1);
			theInfo->GlobalParams.STATIC_OBSTACLE_INFLATION_RADIUS = i1;
			insideSTATICOBSTACLE = true;
			continue;
		}
		
		// Circle
		strcpy(checkString, "Circle:");
		if(strcmp(checkString, firstString) == 0 && insideSTATICOBSTACLE)
		{
			sscanf(thisLine, "%*s %f %f %f", &f1, &f2, &f3);
			i1 = CONTXY2DISC(f1,theInfo->cellsize_m);
			i2 = CONTXY2DISC(f2,theInfo->cellsize_m);
			i3 = CONTXY2DISC(f3,theInfo->cellsize_m);
			for(a = __max(i1-i3,0); a <= __min(i1+i3,theInfo->EnvWidth_c-1); a++)
			{
				for(b = __max(i2-i3,0); b <= __min(i2+i3,theInfo->EnvHeight_c-1); b++)
				{
					al = a - i1;
					bl = b - i2;
					if(al*al+bl*bl <= i3*i3)
						theInfo->StaticObstacleMap.put(a,b,true);
				}
			}
			/*EnvNAV4DXYTG2Dpt_t* ThisObstacleCenter = new EnvNAV4DXYTG2Dpt_t;
			ThisObstacleCenter->x = i1;
			ThisObstacleCenter->y = i2;
			theInfo->GlobalParams.ObstacleCenters.push_back(ThisObstacleCenter);*/
			continue;
		}
		
		// Rectangle
		strcpy(checkString, "Rectangle:");
		if(strcmp(checkString, firstString) == 0 && insideSTATICOBSTACLE)
		{
			InfRad = theInfo->GlobalParams.STATIC_OBSTACLE_INFLATION_RADIUS;
			sscanf(thisLine, "%*s %f %f %f %f", &f1, &f2, &f3, &f4);
			i1 = CONTXY2DISC(f1,theInfo->cellsize_m);
			i2 = CONTXY2DISC(f2,theInfo->cellsize_m);
			i3 = CONTXY2DISC(f3,theInfo->cellsize_m);
			i4 = CONTXY2DISC(f4,theInfo->cellsize_m);
			for(a = __max(__min(i1,i3),InfRad); a <= __min(__max(i1,i3),theInfo->EnvWidth_c-1-InfRad); a++)
				for(b = __max(__min(i2,i4),InfRad); b <= __min(__max(i2,i4),theInfo->EnvHeight_c-1-InfRad); b++)
					for(c = -InfRad; c<=InfRad; c++)
						for(d = -InfRad; d<=InfRad; d++)
							theInfo->StaticObstacleMap.put(a+c,b+d,true);
			/*EnvNAV4DXYTG2Dpt_t* ThisObstacleCenter = new EnvNAV4DXYTG2Dpt_t;
			ThisObstacleCenter->x = (i1+i3)/2;
			ThisObstacleCenter->y = (i2+i4)/2;
			theInfo->GlobalParams.ObstacleCenters.push_back(ThisObstacleCenter);*/
			//printf("^^^^^ Static obstacle - rectangle in config file!\n");
			continue;
		}
		
		// Map
		strcpy(checkString, "Map:");
		if(strcmp(checkString, firstString) == 0 && insideSTATICOBSTACLE)
		{
			sscanf(thisLine, "%*s %d %d %d %f", &i1, &i2, &i3, &f1);
			//i1 = CONTXY2DISC(f1,theInfo->cellsize_m);
			//i2 = CONTXY2DISC(f2,theInfo->cellsize_m);
			
			float XdownFactor, YdownFactor;
			int xDown, yDown, sampleThresh;
			int* SampledMapSum = new int;
			SampledMapSum = (int*) malloc(theInfo->EnvWidth_c * theInfo->EnvHeight_c * sizeof(int));
			//SampledMapSum = new int[theInfo->EnvWidth_c * theInfo->EnvHeight_c];
			XdownFactor = ((float)i1) / ((float)theInfo->EnvWidth_c);
			YdownFactor = ((float)i2) / ((float)theInfo->EnvHeight_c);
			
			for (yDown = 0; yDown < theInfo->EnvHeight_c; yDown++)
				for (xDown = 0; xDown < theInfo->EnvWidth_c; xDown++)
					*(SampledMapSum + theInfo->EnvWidth_c*yDown + xDown) = 0;
			
			for (Y = 0; Y < i2; Y++)
			{
				fgets(thisLineLong, 8000 , fCfg);
				for (X = 0; X < i1; X++)
				{
					//char* formatString = new char[16384];
					//for (a=0; a<X; a++)
					//	strcat(formatString, "%*d ");
					//strcat(formatString, "%d");
					//sscanf(thisLineLong, formatString, &i3);
					sscanf((thisLineLong+X*4), "%d", &i4);
					if(i4 > i3)
					{
						//theInfo->StaticObstacleMap.put(X,Y,true);
						xDown = (int)((float)X / XdownFactor);
						yDown = (int)((float)Y / YdownFactor);
						(*(SampledMapSum + theInfo->EnvWidth_c*yDown + xDown))++;
					}
					//delete formatString;
				}
			}
			
			// Store the downsampled map in downsampled.map file
			FILE* fDown;
			fDown = fopen("downsampled.map", "w");
			
			sampleThresh = (int)(XdownFactor*YdownFactor/f1);
			for (yDown = 0; yDown < theInfo->EnvHeight_c; yDown++)
			{
				for (xDown = 0; xDown < theInfo->EnvWidth_c; xDown++)
					if (*(SampledMapSum + theInfo->EnvWidth_c*yDown + xDown) >= sampleThresh)
					{
						theInfo->StaticObstacleMap.put(xDown,yDown,true);
						fprintf(fDown, " 255");
					}
					else
						fprintf(fDown, "   0");
						
				fprintf(fDown, "\n");
			}
			
			fclose(fDown);
			//delete SampledMapSum; // MEM_CLEAR
			continue;
		}
		
		// HomotopyCriticalPt
		strcpy(checkString, "HomotopyCriticalPt:");
		if(strcmp(checkString, firstString) == 0 && insideSTATICOBSTACLE)
		{
			sscanf(thisLine, "%*s %f %f", &f1, &f2);
			i1 = CONTXY2DISC(f1,theInfo->cellsize_m);
			i2 = CONTXY2DISC(f2,theInfo->cellsize_m);
			HomotopyCriticalPt_Xs.push_back(i1);
			HomotopyCriticalPt_Ys.push_back(i2);
			continue;
		}
		
		// END_STATIC_OBSTACLE
		strcpy(checkString, "END_STATIC_OBSTACLE");
		if(strcmp(checkString, firstString) == 0)
		{
			insideSTATICOBSTACLE = false;
			continue;
		}
		
		// --------------------------------------------------------------
		
		// BEGIN_DYNAMIC_OBSTACLE
		strcpy(checkString, "BEGIN_DYNAMIC_OBSTACLE:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %d", &i1);
			theInfo->GlobalParams.DYNAMIC_OBSTACLE_INFLATION_RADIUS = i1;
			insideDYNAMICOBSTACLE = true;
			theInfo->DynamicObstacleMap.init(theInfo->EnvWidth_c, theInfo->EnvHeight_c, theInfo->EnvMaxTime_c, false);
			continue;
		}
		
		// Rectangle
		strcpy(checkString, "Rectangle:");
		if(strcmp(checkString, firstString) == 0 && insideDYNAMICOBSTACLE)
		{
			InfRad = theInfo->GlobalParams.DYNAMIC_OBSTACLE_INFLATION_RADIUS;
			sscanf(thisLine, "%*s %s %s %s %s", &s1, &s2, &s3, &s4);
			vf1 = parametrized_atof(s1, theInfo->EnvMaxTime_c);
			vf2 = parametrized_atof(s2, theInfo->EnvMaxTime_c);
			vf3 = parametrized_atof(s3, theInfo->EnvMaxTime_c);
			vf4 = parametrized_atof(s4, theInfo->EnvMaxTime_c);
			for(tCount=0; tCount<theInfo->EnvMaxTime_c; tCount++)
			{
				//printf("DO: tt = %d\n", tCount);
				i1 = CONTXY2DISC(vf1[tCount],theInfo->cellsize_m);
				i2 = CONTXY2DISC(vf2[tCount],theInfo->cellsize_m);
				i3 = CONTXY2DISC(vf3[tCount],theInfo->cellsize_m);
				i4 = CONTXY2DISC(vf4[tCount],theInfo->cellsize_m);
				for(a = __max(__min(i1,i3),InfRad); a <= __min(__max(i1,i3),theInfo->EnvWidth_c-1-InfRad); a++)
					for(b = __max(__min(i2,i4),InfRad); b <= __min(__max(i2,i4),theInfo->EnvHeight_c-1-InfRad); b++)
						for(c = -InfRad; c<=InfRad; c++)
							for(d = -InfRad; d<=InfRad; d++)
								theInfo->DynamicObstacleMap.put(a+c,b+d,tCount,true);
			}
			//printf("^^^^^ Dynamic obstacle in config file: \n");
			continue;
		}
		
		// END_DYNAMIC_OBSTACLE
		strcpy(checkString, "END_DYNAMIC_OBSTACLE");
		if(strcmp(checkString, firstString) == 0)
		{
			insideDYNAMICOBSTACLE = false;
			continue;
		}
		
		// --------------------------------------------------------------
		
		// UseHomotopyClass
		strcpy(checkString, "UseHomotopyClass:");
		if(strcmp(checkString, firstString) == 0)
		{
			LValDiffMap_t* myLValDiffMap;
			
			sscanf(thisLine, "%*s %s", s2);
			strcpy(checkString, "TRUE");
			if(strcmp(s2, checkString) == 0)
			{
				printf("Precomputing LVal Differences... ");
					//char tmpc;
					//scanf("%c", &tmpc);
				
				myLValDiffMap = new LValDiffMap_t(theInfo->EnvWidth_c, theInfo->EnvHeight_c, theInfo->TheRobots[0].ActionsV, HomotopyCriticalPt_Xs, HomotopyCriticalPt_Ys);
				myLValDiffMap->isActive = true;
				//myLValDiffMap->SetCriticalPoints(HomotopyCriticalPt_Xs, HomotopyCriticalPt_Ys);
				//myLValDiffMap->ComputeLValDiffs(theInfo->EnvWidth_c, theInfo->EnvHeight_c, theInfo->TheRobots[0].ActionsV);
				
				//printf("(%f,%f)",(*(theInfo->LValDiffMap->LValDiffs + 5000))[0].real(),(*(theInfo->LValDiffMap->LValDiffs + 5))[0].imag());
				printf("done!\n");
			}
			else
			{
				myLValDiffMap = new LValDiffMap_t;
				myLValDiffMap->isActive = false;
			}
			
			theInfo->LValDiffMap = myLValDiffMap;
			continue;
		}
		
		
		strcpy(checkString, "HomotopyExplore:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %d", &i1);
			theInfo->GlobalParams.EXPLORE_HOMOTOPY_CLASSES = i1;
			continue;
		}
		
		strcpy(checkString, "LeftBaseWeight:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %f", &f1);
			theInfo->GlobalParams.LEFT_BASE_WEIGHT = f1;
			continue;
		}
		
		// -END_FILE-
		strcpy(checkString, "-END_FILE-");
		if(strcmp(checkString, firstString) == 0)
		{
			FileEnd = true;
			continue;
		}
	}
}

// =================================================================

int interpVal(int x1, int y1, int x2, int y2, int xs)
{
	// Given data points (x1,y1) and (x2,y2) finds the value of ys at xs
	float slope;
	
	if (x2==x1)
		return y1;
		
	slope = ((float)(y2-y1)) / ((float)(x2-x1));
	return (y1+(int)(slope*(xs-x1)));
}


// =================================================================

vector<int> FindAndInterpInTrajectory(EnvNAV4DXYTG_dist_trajectory* distTraj, int timet, EnvNAV4DXYTG_dist_t* InterpedDist)
{
	// Finds the elements around timet in distTraj
	// For now doing a very simple search, but ideally should do an efficient binary search
	int a;
	vector<int> indices;
	
	InterpedDist->Timet = timet;
	
	for(a=0; a<distTraj->dist_t.size(); a++)
	{
		if(distTraj->dist_t[a].Timet == timet)
		{
			indices.push_back(a);
			//printf(" %d |", distTraj->dist_t[a].D);
			InterpedDist->D = distTraj->dist_t[a].D;
			return indices;
		}
		if(distTraj->dist_t[a].Timet > timet)
		{
			if (a==0)
			{
				indices.push_back(-1);
				indices.push_back(0);
				InterpedDist->D = interpVal(distTraj->dist_t[0].Timet, distTraj->dist_t[0].D, distTraj->dist_t[1].Timet, distTraj->dist_t[1].D, timet);
				return indices;
			}
			else
			{
				indices.push_back(a-1);
				indices.push_back(a);
				InterpedDist->D = interpVal(distTraj->dist_t[a-1].Timet, distTraj->dist_t[a-1].D, distTraj->dist_t[a].Timet, distTraj->dist_t[a].D, timet);
				return indices;
			}
		}
	}
	
	a = distTraj->dist_t.size() - 1;
	indices.push_back(a);
	indices.push_back(-1);
	InterpedDist->D = interpVal(distTraj->dist_t[a-1].Timet, distTraj->dist_t[a-1].D, distTraj->dist_t[a].Timet, distTraj->dist_t[a].D, timet);
	return indices;
}

// ------------------------

void PutInTrajectory(EnvNAV4DXYTG_dist_trajectory* distTraj, EnvNAV4DXYTG_dist_t distToPut)
{
	int a;
	vector<EnvNAV4DXYTG_dist_t>::iterator Iter;
	
	for(a=0, Iter=distTraj->dist_t.begin(); a<distTraj->dist_t.size() || Iter != distTraj->dist_t.end(); a++, Iter++)
	{
		if(distTraj->dist_t[a].Timet == distToPut.Timet)
		{
			distTraj->dist_t[a] = distToPut;
			return;
		}
		if(distTraj->dist_t[a].Timet > distToPut.Timet)
		{
			distTraj->dist_t.insert(Iter, distToPut);
			return;
		}
	}
	distTraj->dist_t.push_back(distToPut);
}

// =================================================================

vector<int> FindAndInterpInTrajectory(EnvNAV4DXYTG_pos_trajectory* posTraj, int timet, EnvNAV4DXYTG_pos_t* InterpedPos)
{
	// Finds the elements around timet in distTraj
	// For now doing a very simple search, but ideally should do an efficient binary search
	int a;
	vector<int> indices;
	
	InterpedPos->Timet = timet;
	
	for(a=0; a<posTraj->pos_t.size(); a++)
	{
		if(posTraj->pos_t[a].Timet == timet)
		{
			indices.push_back(a);
			InterpedPos->X = posTraj->pos_t[a].X;
			InterpedPos->Y = posTraj->pos_t[a].Y;
			return indices;
		}
		if(posTraj->pos_t[a].Timet > timet)
		{
			if (a==0)
			{
				indices.push_back(-1);
				indices.push_back(0);
				InterpedPos->X = interpVal(posTraj->pos_t[0].Timet, posTraj->pos_t[0].X, posTraj->pos_t[1].Timet, posTraj->pos_t[1].X, timet);
				InterpedPos->Y = interpVal(posTraj->pos_t[0].Timet, posTraj->pos_t[0].Y, posTraj->pos_t[1].Timet, posTraj->pos_t[1].Y, timet);
				return indices;
			}
			else
			{
				indices.push_back(a-1);
				indices.push_back(a);
				InterpedPos->X = interpVal(posTraj->pos_t[a-1].Timet, posTraj->pos_t[a-1].X, posTraj->pos_t[a].Timet, posTraj->pos_t[a].X, timet);
				InterpedPos->Y = interpVal(posTraj->pos_t[a-1].Timet, posTraj->pos_t[a-1].Y, posTraj->pos_t[a].Timet, posTraj->pos_t[a].Y, timet);
				return indices;
			}
		}
	}
	
	a = posTraj->pos_t.size() - 1;
	indices.push_back(a);
	indices.push_back(-1);
	InterpedPos->X = interpVal(posTraj->pos_t[a-1].Timet, posTraj->pos_t[a-1].X, posTraj->pos_t[a].Timet, posTraj->pos_t[a].X, timet);
	InterpedPos->Y = interpVal(posTraj->pos_t[a-1].Timet, posTraj->pos_t[a-1].Y, posTraj->pos_t[a].Timet, posTraj->pos_t[a].Y, timet);
	return indices;
}

// ------------------------

void PutInTrajectory(EnvNAV4DXYTG_pos_trajectory* posTraj, EnvNAV4DXYTG_pos_t posToPut)
{
	int a;
	vector<EnvNAV4DXYTG_pos_t>::iterator Iter;
	
	for(a=0, Iter=posTraj->pos_t.begin(); a<posTraj->pos_t.size() || Iter != posTraj->pos_t.end(); a++, Iter++)
	{
		if(posTraj->pos_t[a].Timet == posToPut.Timet)
		{
			posTraj->pos_t[a] = posToPut;
			return;
		}
		if(posTraj->pos_t[a].Timet > posToPut.Timet)
		{
			posTraj->pos_t.insert(Iter, posToPut);
			return;
		}
	}
	posTraj->pos_t.push_back(posToPut);
}

// =================================================================

LValDiffMap_t::LValDiffMap_t(int xSize, int ySize, vector<EnvNAV4DXYTGAction_t> ActionsVIn, vector<int> Xs, vector<int> Ys, double IntegrationStepSizeIn)
{
	complex<double> CrtPt, fCrtPt, prodDiff;
	int a, b, c;
	size_x = xSize;
	size_y = ySize;
	ActionsV = ActionsVIn;
	IntegrationStepSize = IntegrationStepSizeIn;
	double halfcount;

	CriticalPoints.clear();
	
	for (a=0; a<Xs.size(); a++)
	{
		CrtPt = complex<double>((double)Xs[a], (double)Ys[a]);
		CriticalPoints.push_back(CrtPt);
	}
	halfcount = floor(CriticalPoints.size()/2.0);
	for (a=0; a<Xs.size(); a++)
	{
		fCrtPt = pow(CriticalPoints[a],halfcount) * pow(CriticalPoints[a]-complex<double>((double)size_x, (double)size_y), ((double)CriticalPoints.size())-halfcount-1.0);
		prodDiff = complex<double>(1.0,0.0);
		for (b=0; b<Xs.size(); b++)
			if (b!=a)
				prodDiff = prodDiff * (CriticalPoints[b]-CriticalPoints[a]);
			
		PartialFracCoefs.push_back(fCrtPt/prodDiff);
		
/*		if ( PartialFracCoefs[a].real()!=PartialFracCoefs[a].real() || PartialFracCoefs[a].imag()!=PartialFracCoefs[a].imag() || PartialFracCoefs[a].real()==PartialFracCoefs[a].real()+1.0 || PartialFracCoefs[a].imag()==PartialFracCoefs[a].imag()+1.0 )
		{
			printf("!!! (%f,%f) - (%f,%f), (%f,%f); (%f,%f):(%f,%f):%f\n",
					PartialFracCoefs[a].real(),PartialFracCoefs[a].imag(), 
					fCrtPt.real(),fCrtPt.imag(), 
					prodDiff.real(),prodDiff.imag(),
					CriticalPoints[a].real(),CriticalPoints[a].imag(),
					(double)size_x, (double)size_y,
					halfcount);
			for (b=0; b<Xs.size(); b++)
				printf("-- (%f,%f) -- ", CriticalPoints[b].real(), CriticalPoints[b].imag());
			char tmpc;
			scanf("%c", &tmpc);
		} */
	}
	
	
	ActionV_InvIndex = vector<int>(ActionsV.size(), -1);
	for (a=0; a<ActionsV.size(); a++)
		for (b=0; b<ActionsV.size(); b++)
			if (ActionsV[a].dX==-ActionsV[b].dX && ActionsV[a].dY==-ActionsV[b].dY)
			{
				ActionV_InvIndex[a] = b;
				break;
			}
				
	LValDiffs = new vector< complex<double> >[xSize*ySize];
	for (a=0; a<size_x; a++)
		for (b=0; b<size_y; b++)
		{
			(*(LValDiffs + size_x*b + a)).clear();
			for (c=0; c<ActionsVIn.size(); c++)
				(*(LValDiffs + size_x*b + a)).push_back(complex<double>(0.0,0.0));
		}

}

//-----------------------
/*

void LValDiffMap_t::SetCriticalPoints(vector<int> Xs, vector<int> Ys)
{
	int a;
	complex<double> CrtPt, fCrtPt;

	CriticalPoints.clear();
	//CriticalPoints.push_back(complex<double>(-11.0123,-10.3210)); // A point outside the env
	
	for (a=0; a<Xs.size(); a++)
	{
		CrtPt = complex<double>((double)Xs[a], (double)Ys[a]);
		CriticalPoints.push_back(CrtPt);
	}
	for (a=0; a<Xs.size(); a++)
	{
		fCrtPt = pow(CrtPt,CriticalPoints.size()/2.0) * pow(CrtPt-complex<double>(size_x, size_y),CriticalPoints.size()/2.0);
	}
}



void LValDiffMap_t::ComputeLValDiffs(int xSize, int ySize, vector<EnvNAV4DXYTGAction_t> ActionsVIn, double IntegrationStepSizeIn)
{
	int a, b, c;
	size_x = xSize;
	size_y = ySize;
	ActionsV = ActionsVIn;
	IntegrationStepSize = IntegrationStepSizeIn;
	
	ActionV_InvIndex = vector<int>(ActionsV.size(), -1);
	for (a=0; a<ActionsV.size(); a++)
		for (b=0; b<ActionsV.size(); b++)
			if (ActionsV[a].dX==-ActionsV[b].dX && ActionsV[a].dY==-ActionsV[b].dY)
			{
				ActionV_InvIndex[a] = b;
				break;
			}
				
	
	//data = (bool*) malloc(xSize*ySize*sizeof(bool));
	LValDiffs = new vector< complex<double> >[xSize*ySize];
	for (a=0; a<size_x; a++)
	{
		//printf(" %d ",a);
		for (b=0; b<size_y; b++)
		{
			(*(LValDiffs + size_x*b + a)).clear();
			for (c=0; c<ActionsVIn.size(); c++)
				//(*(LValDiffs + size_x*b + a)).push_back(IntegrateLValDiff(a, b, c));
				(*(LValDiffs + size_x*b + a)).push_back(complex<double>(0.0,0.0));
		}
	}
	//printf("\n\n");
}
*/
//-----------------------


complex<double> LValDiffMap_t::IntegrateLValDiff(int xv, int yv, int av)
{
	complex<double> iPt, diffVec, step, destPt, ff, nu, den;
	complex<double> LValDiff(0.0,0.0);
	int a, StepCount, StepNo, halfcount;
	double diffVecMag, nuLnAbs, denLnAbs, nuTh, denTh, RePart, ImagPart;
	
	iPt = complex<double>((double)xv, (double)yv);
	diffVec = complex<double>((double)ActionsV[av].dX, (double)ActionsV[av].dY);


	diffVecMag = abs(diffVec);
	if (diffVecMag<1e-10)
		return LValDiff;

	destPt = iPt + diffVec;
	for (a=0; a<CriticalPoints.size(); a++)
	{
		nu = destPt - CriticalPoints[a];
		den = iPt - CriticalPoints[a];
		//nuLnAbs = log(abs(nu));
		//denLnAbs = log(abs(den));
		RePart = log(abs(nu/den)); //nuLnAbs - denLnAbs;
		nuTh = atan2(nu.imag(), nu.real());
		denTh = atan2(den.imag(), den.real());
		ImagPart = nuTh - denTh;
		if (ImagPart<-M_PI)
			ImagPart = ImagPart + 2.0*M_PI;
		if (ImagPart>M_PI)
			ImagPart = ImagPart - 2.0*M_PI;
		LValDiff = LValDiff + PartialFracCoefs[a]*(complex<double>(RePart,ImagPart));
		
	}
	


/*
	diffVecMag = abs(diffVec);
	if (diffVecMag<1e-10)
		return LValDiff;
	
	destPt = iPt + diffVec;
	StepCount = round(diffVecMag / IntegrationStepSize);
	step = diffVec / ((double)StepCount);
	iPt = iPt + step/2.0;
	destPt = destPt - step/2.0;
	
	StepNo = 0;
	while (StepNo<StepCount)
	{
		halfcount = floor(CriticalPoints.size()/2.0);
		ff = pow(iPt,halfcount) * pow(iPt-complex<double>(size_x, size_y),CriticalPoints.size()-halfcount-1.0); //pow(iPt,CriticalPoints.size());//complex<double>(1.0,0.0);
		//printf("## (%f,%f) ", iPt.real(),iPt.imag());
		for (a=0; a<CriticalPoints.size(); a++)
			ff = ff / (iPt - CriticalPoints[a]);
		LValDiff = LValDiff + ff*step;
		//printf(" (%f,%f) \n", LValDiff.real(),LValDiff.imag());
		iPt = iPt + step;
		StepNo++;
	}
*/

	return LValDiff;
}


complex<double> RobotHomotopyInfo_t::getLValDiff(int xs, int ys, int ActionInd)
{
	if (!LValDiffMap->isActive)
		return complex<double>(0.0,0.0);
		
	// Do this online - cache
	if (abs((*(LValDiffMap->LValDiffs + LValDiffMap->size_x*ys + xs))[ActionInd]) == 0.0) {
		(*(LValDiffMap->LValDiffs + LValDiffMap->size_x*ys + xs))[ActionInd] = LValDiffMap->IntegrateLValDiff(xs, ys, ActionInd);
		int xx = xs + LValDiffMap->ActionsV[ActionInd].dX;
		int yy = ys + LValDiffMap->ActionsV[ActionInd].dY;
		if (xx>=0 && yy>=0 && xx<LValDiffMap->size_x && yy<LValDiffMap->size_y)
			(*(LValDiffMap->LValDiffs + LValDiffMap->size_x*yy + xx))[LValDiffMap->ActionV_InvIndex[ActionInd]] = -(*(LValDiffMap->LValDiffs + LValDiffMap->size_x*ys + xs))[ActionInd];
	}
	return (*(LValDiffMap->LValDiffs + LValDiffMap->size_x*ys + xs))[ActionInd];
	//printf("%x", LValDiffMap);
}

// =================================================================

obstacleMap2D::obstacleMap2D(int xSize, int ySize)
{
	//data = (bool*) malloc(xSize*ySize*sizeof(bool));
	data = new bool[xSize*ySize];
	size_x = xSize;
	size_y = ySize;
}

obstacleMap2D::obstacleMap2D(int xSize, int ySize, bool initVal)
{
	int a, b;
	size_x = xSize;
	size_y = ySize;
	//data = (bool*) malloc(xSize*ySize*sizeof(bool));
	data = new bool[xSize*ySize];
	for (a=0; a<size_x; a++)
		for (b=0; b<size_y; b++)
			*(data + size_x*b + a) = initVal;
}

void obstacleMap2D::init(int xSize, int ySize, bool initVal)
{
	int a, b;
	size_x = xSize;
	size_y = ySize;
	//data = (bool*) malloc(xSize*ySize*sizeof(bool));
	data = new bool[xSize*ySize];
	for (a=0; a<size_x; a++)
		for (b=0; b<size_y; b++)
			*(data + size_x*b + a) = initVal;
}

bool obstacleMap2D::get(int xx, int yy)
{
	return(*(data + size_x*yy + xx));
}

void obstacleMap2D::put(int xx, int yy, bool flag)
{
	*(data + size_x*yy + xx) = flag;
}

// =================================================================


obstacleMap3D::obstacleMap3D(int xSize, int ySize, int tSize)
{
	//data = (bool*) malloc(xSize*ySize*tSize*sizeof(bool));
	data = new bool[xSize*ySize*tSize];
	size_x = xSize;
	size_y = ySize;
	size_tt = tSize;
}

obstacleMap3D::obstacleMap3D(int xSize, int ySize, int tSize, bool initVal)
{
	int a, b, c;
	size_x = xSize;
	size_y = ySize;
	size_tt = tSize;
	//data = (bool*) malloc(xSize*ySize*tSize*sizeof(bool));
	data = new bool[xSize*ySize*tSize];
	for (a=0; a<size_x; a++)
		for (b=0; b<size_y; b++)
			for (c=0; c<size_tt; c++)
				*(data + size_x*size_y*c + size_x*b + a) = initVal;
}

void obstacleMap3D::init(int xSize, int ySize, int tSize, bool initVal)
{
	int a, b, c;
	size_x = xSize;
	size_y = ySize;
	size_tt = tSize;
	//data = (bool*) malloc(xSize*ySize*tSize*sizeof(bool));
	data = new bool[xSize*ySize*tSize];
	for (a=0; a<size_x; a++)
		for (b=0; b<size_y; b++)
			for (c=0; c<size_tt; c++)
				*(data + size_x*size_y*c + size_x*b + a) = initVal;
}

bool obstacleMap3D::get(int xx, int yy, int tt)
{
	return(*(data + size_x*size_y*tt + size_x*yy + xx));
}

void obstacleMap3D::put(int xx, int yy, int tt, bool flag)
{
	*(data + size_x*size_y*tt + size_x*yy + xx) = flag;
}

void obstacleMap3D::ConstructFromTrajectories(vector<EnvNAV4DXYTG_pos_trajectory> otherBots_trajectories, int collisionCheckRad)
{
	int a, b;
	
	for (a=0; a<otherBots_trajectories.size(); a++)
		for (b=0; b<otherBots_trajectories[a].pos_t.size(); b++)
			if (collisionCheckRad == 1) // Presently only 0 and 1 are only allowed values
				put(otherBots_trajectories[a].pos_t[b].X, otherBots_trajectories[a].pos_t[b].Y, otherBots_trajectories[a].pos_t[b].Timet, true);

}


// =================================================================


//------------------------------------------------------------------------------
/*
EnvNAV4DXYTG_dist_trajectory* GenerateDistanceConstraint(EnvNAV4DXYTG_pos_trajectory RefPosTraj, int dist)
{
	// 'dist' is in cell units (not meters), and NOT multiplied by NAV4DXYTG_COSTMULT
	EnvNAV4DXYTG_dist_trajectory* OutDistTraj = new EnvNAV4DXYTG_dist_trajectory;
	int a;
	
	for(a=0; a<RefPosTraj.pos_t.size(); a++)
	{
		EnvNAV4DXYTG_dist_t* TempDist = new EnvNAV4DXYTG_dist_t;
		TempDist->Timet = RefPosTraj.pos_t[a].Timet;
		//TempDist->D = dist*NAV4DXYTG_COSTMULT;
		if (a == RefPosTraj.pos_t.size()/2)
			TempDist->D = 0;
		else
			TempDist->D = 100*NAV4DXYTG_COSTMULT;
		
		OutDistTraj->dist_t.push_back(*TempDist);
	}
	
	return OutDistTraj;
}
*/
//------------------------------------------------------------------------------

int TRANSITIONCOST_XYT(int DX, int DY, int DT)
{
	if (DX == 0 && DY == 0)
		return (NAV4DXYTG_COSTMULT/5);
	else
	{
		//NEW: return ((int) pow(NAV4DXYTG_COSTMULT * sqrt((double)(DX*DX + DY*DY)),COST_NORM_ORDER));
		return ((int) (NAV4DXYTG_COSTMULT * sqrt((double)(DX*DX + DY*DY))));
	}
}


int CELLCOST_XYT(int DX, int DY, int DT, int ConstraintDist)
{
	//NEW: return (((int) (pow(NAV4DXYTG_COSTMULT * sqrt((double)(DX*DX + DY*DY)),COST_NORM_ORDER))) - pow(ConstraintDist,COST_NORM_ORDER));
	return (((int) (NAV4DXYTG_COSTMULT * sqrt((double)(DX*DX + DY*DY)))) - ConstraintDist);
	//return ((int) NAV4DXYTG_COSTMULT * (((double)(DX*DX + DY*DY)) - ((double) ConstraintDist/NAV4DXYTG_COSTMULT)^2 ));
}

//------------------------------------------------------------------------------

void OutputFile::WriteTrajectories(vector<EnvNAV4DXYTG_pos_trajectory> pos_trajectories, int IterNo, int ActiveRobot, vector<float> PenaltyWeights, vector< vector<int> > PenaltyWeightRobotsThisIteration)
{
	int botNo, consNo, trajPtNo;
	
	fprintf(fOut, "ITERATION %d\n", IterNo);
	fprintf(fOut, "ACTIVE_ROBOT %d\n", ActiveRobot);
	//fprintf(fOut, "PENALTY_WEIGHT %f\n", PenaltyWeight);
	
	for (consNo=0; consNo<PenaltyWeights.size(); consNo++)
	{
		if (PenaltyWeightRobotsThisIteration.size()>0)
			fprintf(fOut, "CONSTRAINT %d %d %f\n", PenaltyWeightRobotsThisIteration[consNo][0], PenaltyWeightRobotsThisIteration[consNo][1], PenaltyWeights[consNo]);
		else
			fprintf(fOut, "CONSTRAINT %d %d %f\n", 0, 0, PenaltyWeights[consNo]);
	}
			
	//printf("%%%%%%% Bot nos: %d\n", pos_trajectories.size());
	for (botNo=0; botNo<pos_trajectories.size(); botNo++)
	{
		//printf("Writing ROBOT %d\n", botNo);
		fprintf(fOut, "ROBOT %d\n", botNo);
		for (trajPtNo=0; trajPtNo<pos_trajectories[botNo].pos_t.size(); trajPtNo++)
		{
			fprintf(fOut, "%d %d %d\n", pos_trajectories[botNo].pos_t[trajPtNo].X, 
							pos_trajectories[botNo].pos_t[trajPtNo].Y, 
							pos_trajectories[botNo].pos_t[trajPtNo].Timet);
		}
		fprintf(fOut, "KEYPOINTS %d\n", botNo);
		for (trajPtNo=0; trajPtNo<pos_trajectories[botNo].KeyPts.size(); trajPtNo++)
		{
			fprintf(fOut, "%d %d %d\n", pos_trajectories[botNo].KeyPts[trajPtNo].X, 
							pos_trajectories[botNo].KeyPts[trajPtNo].Y, 
							pos_trajectories[botNo].KeyPts[trajPtNo].Timet);
		}
	}
	
	fflush(fOut);
}

void OutputFile::WriteStaticObstacles(obstacleMap2D* obs)
{
	int a, b;
	
	fprintf(fOut, "OBSTACLE\n");
	for (a=0; a<obs->size_x; a++)
		for (b=0; b<obs->size_y; b++)
			if(obs->get(a,b))
				fprintf(fOut, "%d %d\n", a, b);
				
	fflush(fOut);
}

void OutputFile::WriteDynamicObstacles(obstacleMap3D* obs)
{
	int a, b, c;
	
	for (c=0; c<obs->size_tt; c++)
	{
		fprintf(fOut, "OBSTACLE %d\n", c);
		for (a=0; a<obs->size_x; a++)
			for (b=0; b<obs->size_y; b++)
				if(obs->get(a,b,c))
					fprintf(fOut, "%d %d\n", a, b);
	}
	
	fflush(fOut);
}

void OutputFile::WriteConstraints(vector<EnvNAV4DXYTG_AParticularConstraint_t> TheConstraints)
{
	int a, b;
	
	for (a=0; a<TheConstraints.size(); a++)
	{
		fprintf(fOut, "CONSTRAINT_DEFINITION %d %d %d\n", a, TheConstraints[a].Robot1, TheConstraints[a].Robot2);
		for (b=0; b<TheConstraints[a].constraint.dist_t.size(); b++)
			fprintf(fOut, "%d %d\n", TheConstraints[a].constraint.dist_t[b].Timet, TheConstraints[a].constraint.dist_t[b].D);
	}
	fflush(fOut);
}

void OutputFile::WriteConfigFileParameters(ConfigFileInfo* theConfigFileInfo)
{
	// ROBOT_TASK <robot_no> <taskX> <taskY>
	int a, b;
	for (a=0; a<theConfigFileInfo->TheRobots.size(); a++)
		for (b=0; b<theConfigFileInfo->TheRobots[a].Tasks.size(); b++)
			fprintf(fOut, "ROBOT_TASK %d %d %d\n", a, theConfigFileInfo->TheRobots[a].Tasks[b].x, theConfigFileInfo->TheRobots[a].Tasks[b].y);
			
	// SCALE_FACTOR <x_scale> <y_scale> <t_scale>
	fprintf(fOut, "SCALE_FACTOR %f %f %f\n", theConfigFileInfo->cellsize_m, theConfigFileInfo->cellsize_m, theConfigFileInfo->timestepsize_m);
	
	fflush(fOut);
}


OutputFile::OutputFile(const char* filename)
{
	fOut = fopen(filename, "w");
}

void OutputFile::Close(void)
{
	fprintf(fOut, "END\n");
	fclose(fOut);
}

// **********************************
// **********************************
// Utilities

IndexTracker::IndexTracker(vector<int> DimSizes)
{
	int a;
	
	dim = DimSizes;
	dim_count = dim.size();
	tracker = 0;
	index.assign(dim_count,0);
	
	tracker_max = 1;
	for (a=0; a<dim_count; a++)
		tracker_max = tracker_max * dim[a];
	tracker_max = tracker_max - 1;
}

void IndexTracker::reset(void)
{
	tracker=0;
	index.assign(dim_count,0);
}

void IndexTracker::next(void)
{
	int a, tmp_tracker;
	
	tracker++;
	tmp_tracker = tracker;
	
	for (a=0; a<dim_count; a++)
	{
		index[a] = tmp_tracker % dim[a];
		tmp_tracker = tmp_tracker / dim[a];
	}
}
// ========================================================================

// Permutes indices 0,1,2,..,(count-1)
vector< vector<int> > IndexPermute(int count)
{
	vector< vector<int> > PermResult;
	int insertPos, SubPermCount;
	
	if(count==1)
	{
		vector<int> thisPerm;
		thisPerm.push_back(0);
		PermResult.push_back(thisPerm);
	}
	else
	{
		vector< vector<int> > SubPermResult;
		SubPermResult = IndexPermute(count-1);
		for(insertPos=0; insertPos<count; insertPos++)
			for(SubPermCount=0; SubPermCount<SubPermResult.size(); SubPermCount++)
			{
				vector<int> thisPerm;
				thisPerm = SubPermResult[SubPermCount];
				thisPerm.insert(thisPerm.begin()+insertPos, count-1);
				PermResult.push_back(thisPerm);
			}
	}
	
	return(PermResult);
}


SBPL2DGridSearchWithTasks::SBPL2DGridSearchWithTasks(int in_width_x, int in_height_y, float in_cellsize_m)
{
	width_x = in_width_x;
	height_y = in_height_y;
	cellsize_m = in_cellsize_m;
	//lastComputedG = -1;
}

SBPL2DGridSearchWithTasks::~SBPL2DGridSearchWithTasks()
{
	int a;
	delete OriginPreSearch;
	for(a=0; a<TasksPreSearch.size(); a++)
		delete TasksPreSearch[a];
}


bool SBPL2DGridSearchWithTasks::PreCompute(unsigned char** Grid2D, unsigned char obsthresh, int ExpandOrigin_x, int ExpandOrigin_y, vector<EnvNAV4DXYTG2DptInt_t> TasksIn, int ExpandStop_x, int ExpandStop_y, SBPL_2DGRIDSEARCH_TERM_CONDITION termination_condition)
{
	int a, x, y, p, minHeu, tsk;
	Tasks = TasksIn;
	TasksCount = Tasks.size();
	
	TasksPreSearch.resize(TasksCount);
	
	for(a=0; a<TasksCount; a++)
		TasksPreSearch[a] = new SBPL2DGridSearch(width_x, height_y, cellsize_m);
	OriginPreSearch = new SBPL2DGridSearch(width_x, height_y, cellsize_m);
		
	for(a=0; a<TasksCount; a++)
		TasksPreSearch[a]->search(Grid2D, obsthresh, Tasks[a].x, Tasks[a].y, ExpandStop_x, ExpandStop_y, termination_condition);
	OriginPreSearch->search(Grid2D, obsthresh, ExpandOrigin_x, ExpandOrigin_y, ExpandStop_x, ExpandStop_y, termination_condition);
	
	
	/*
	PreComputedHeu = new int* [width_x];
	for (x = 0; x < width_x; x++)
		PreComputedHeu[x] = new int [height_y];
	
	
	int thisTestHeu, startIndx, targetIndx;
	vector< vector<int> > PermutedIndices;
	PermutedIndices = IndexPermute(TasksCount);
	
	//vector<int> TaskTravelHeuSum(PermutedIndices.size());
	vector<int> MinTaskTravelHeuSum(TasksCount,INFINITECOST);
	
	for (p=0; p<PermutedIndices.size(); p++)
	{
		printf("Perm: ");
		int thisTaskTravelHeuSum = 0;
		for (tsk=0; tsk<TasksCount-1; tsk++)
		{
			printf(" %d ", PermutedIndices[p][tsk]);
			startIndx = PermutedIndices[p][tsk];
			targetIndx = PermutedIndices[p][tsk+1];
			thisTaskTravelHeuSum += TasksPreSearch[targetIndx]->getlowerboundoncostfromstart_inmm(Tasks[startIndx].x, Tasks[startIndx].y);
		}
		thisTaskTravelHeuSum += OriginPreSearch->getlowerboundoncostfromstart_inmm(Tasks[targetIndx].x, Tasks[targetIndx].y);
		printf(" - thisTaskTravelHeuSum = %d\n", thisTaskTravelHeuSum);
		
		MinTaskTravelHeuSum[PermutedIndices[p][0]] = __min(MinTaskTravelHeuSum[PermutedIndices[p][0]], thisTaskTravelHeuSum);
	}
	
	for (tsk=0; tsk<TasksCount; tsk++)
		printf("MinTaskTravelHeuSum[%d] = %d \n", tsk, MinTaskTravelHeuSum[tsk]);
	
	for (x = 0; x < width_x; x++)
	{
		for (y = 0; y < height_y; y++)
		{
			PreComputedHeu[x][y] = INFINITECOST;
			
			for (tsk=0; tsk<TasksCount; tsk++)
			{
				thisTestHeu = TasksPreSearch[tsk]->getlowerboundoncostfromstart_inmm(x, y) + MinTaskTravelHeuSum[tsk];
				PreComputedHeu[x][y] = __min(PreComputedHeu[x][y], thisTestHeu);
			}
			
		}
	}
	*/
}


int SBPL2DGridSearchWithTasks::getPreComputedHeu(int x, int y, int g)
{
	//printf("PreComputedHeu[%d][%d] = ", x, y);
	//printf("%d \n", PreComputedHeu[x][y]);
	int a, p, minHeu, tsk;
	vector<int> RemTaskIndices;
	vector<int> MinTaskTravelHeuSum;
	int RemTasksCount, thisTaskTravelHeuSum;
	
	// Create remaining task list based on value of 'g' if not already created for this 'g'
	int ComputedIndx = -1;
	for (a=0; a<ComputedGs.size(); a++)
		if (ComputedGs[a]==g)
			ComputedIndx = a;
	
	if (ComputedIndx>-1)
	{
		RemTaskIndices = StoredCompute[ComputedIndx].RemTaskIndices;
		MinTaskTravelHeuSum = StoredCompute[ComputedIndx].MinTaskTravelHeuSum;
		RemTasksCount = StoredCompute[ComputedIndx].RemTasksCount;
	}
	else
	{
	
		RemTasksCount = 0;
		for (a=0; a<TasksCount; a++)
		{
			if ((g & (int)pow((float)2,a)) == 0)
			{
				RemTaskIndices.push_back(a);
				RemTasksCount++;
			}
		}
		
		//printf("g=%d, RemTasksCount=%d\n", g, RemTasksCount);
	
		if (RemTasksCount>0)
		{
			MinTaskTravelHeuSum.clear();
			MinTaskTravelHeuSum.resize(TasksCount,INFINITECOST);
	
			int thisTestHeu, startIndx, targetIndx, FirstTaskIndx;
			vector< vector<int> > PermutedIndices;
			PermutedIndices = IndexPermute(RemTasksCount);
	
			//vector<int> TaskTravelHeuSum(PermutedIndices.size());
	
			for (p=0; p<PermutedIndices.size(); p++)
			{
				//printf("Perm: ");
				thisTaskTravelHeuSum = 0;
				for (tsk=0; tsk<RemTasksCount-1; tsk++)
				{
					//thisTskIndx = RemTaskIndices[tsk];
					//printf(" %d ", PermutedIndices[p][tsk]);
					startIndx = RemTaskIndices[PermutedIndices[p][tsk]];
					targetIndx = RemTaskIndices[PermutedIndices[p][tsk+1]];
					thisTaskTravelHeuSum += TasksPreSearch[targetIndx]->getlowerboundoncostfromstart_inmm(Tasks[startIndx].x, Tasks[startIndx].y);
				}
				targetIndx = RemTaskIndices[PermutedIndices[p][RemTasksCount-1]];
				thisTaskTravelHeuSum += OriginPreSearch->getlowerboundoncostfromstart_inmm(Tasks[targetIndx].x, Tasks[targetIndx].y);
				//printf(" - thisTaskTravelHeuSum = %d\n", thisTaskTravelHeuSum);
		
				FirstTaskIndx = RemTaskIndices[PermutedIndices[p][0]];
				MinTaskTravelHeuSum[FirstTaskIndx] = __min(MinTaskTravelHeuSum[FirstTaskIndx], thisTaskTravelHeuSum);
			}
	
			//for (tsk=0; tsk<TasksCount; tsk++)
				//printf("MinTaskTravelHeuSum[%d] = %d \n", tsk, MinTaskTravelHeuSum[tsk]);
		
			//lastComputedG = g;
			ComputedGs.push_back(g);
			GComputeHistory_t tempCompute;
			tempCompute.RemTaskIndices = RemTaskIndices;
			tempCompute.MinTaskTravelHeuSum = MinTaskTravelHeuSum;
			tempCompute.RemTasksCount = RemTasksCount;
			StoredCompute.push_back(tempCompute);
		}
	}

	int Heu = INFINITECOST;
	int tskIndx, thisTestHeu;
	
	if (RemTasksCount>0)
		for (tsk=0; tsk<RemTasksCount; tsk++)
		{
			tskIndx = RemTaskIndices[tsk];
			thisTestHeu = TasksPreSearch[tskIndx]->getlowerboundoncostfromstart_inmm(x, y) + MinTaskTravelHeuSum[tskIndx];
			Heu = __min(Heu, thisTestHeu);
		}
	else
		Heu = OriginPreSearch->getlowerboundoncostfromstart_inmm(x, y);

	
	return Heu;
}


// ------------------------

float sampled_atof(char* fmtStr)
{
	// Syntaxes: n[Ru] or n[R%u] or n[R+u-v] or n[R%+u-v]
	char *startBrack;
	char *endBrack;
	char *distributionChar;
	char *numStart;
	char *plusChar;
	char *minusChar;
	float u, v, val, res=0.0;
	bool isFraction = false;
	
	sscanf(fmtStr, "%f", &val);
	
	startBrack = strstr(fmtStr, "[");
	endBrack = strstr(fmtStr, "]");
	if ((!RandomizeVals) || startBrack==NULL || endBrack==NULL)
		return atof(fmtStr);
	
	distributionChar = startBrack+1;
	
	if (*distributionChar=='R')
	{
		if (*(distributionChar+1)=='%')
		{
			isFraction = true;
			numStart = distributionChar+2;
		}
		else
			numStart = distributionChar+1;
		
		plusChar = strstr(numStart, "+");
		minusChar = strstr(numStart, "-");
		if (plusChar==NULL || minusChar==NULL)
		{
			sscanf(numStart, "%f", &u);
			v = u;
		}
		else
		{
			sscanf((plusChar+1), "%f", &u);
			sscanf((minusChar+1), "%f", &v);
		}
		
		if (isFraction)
			res = val + (RANDU*(u+v) - v)*val;
		else
			res = val + (RANDU*(u+v) - v);
	
	}
	
	printf("-- %f \n", res);
	return res;
}

// ------------------------

vector<float> parametrized_atof(char* fmtStr, int count)
{
	// Syntax: startVal:endVal
	// Returns a vector of length count, linearly interpolated between startVal & endVal
	char* colonStart;
	float startVal, endVal, stepSize, outVal;
	int StepCount;
	vector<float> outVec;
	
	sscanf(fmtStr, "%f", &startVal);
	colonStart = strstr(fmtStr, ":");
	if (colonStart==NULL)
		endVal = startVal;
	else
		sscanf(colonStart+1, "%f", &endVal);
	stepSize = (endVal-startVal) / (count-1);
	
	outVal = startVal;
	for (StepCount=0; StepCount<count; StepCount++)
	{
		outVec.push_back(outVal);
		outVal += stepSize;
	}
	
	return outVec;
}

