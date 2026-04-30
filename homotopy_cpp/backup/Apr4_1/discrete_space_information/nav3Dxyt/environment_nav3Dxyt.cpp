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
unsigned int EnvironmentNAV3DXYT::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int Timet)
{

	return inthash(inthash(X1)+(inthash(X2)<<1)+(inthash(Timet)<<2)) & (EnvNAV3DXYT.HashTableSize-1);
}



void EnvironmentNAV3DXYT::PrintHashTableHist()
{
	int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;

	for(int  j = 0; j < EnvNAV3DXYT.HashTableSize; j++)
	{
		if((int)EnvNAV3DXYT.Coord2StateIDHashTable[j].size() == 0)
			s0++;
		else if((int)EnvNAV3DXYT.Coord2StateIDHashTable[j].size() < 50)
			s1++;
		else if((int)EnvNAV3DXYT.Coord2StateIDHashTable[j].size() < 100)
			s50++;
		else if((int)EnvNAV3DXYT.Coord2StateIDHashTable[j].size() < 200)
			s100++;
		else if((int)EnvNAV3DXYT.Coord2StateIDHashTable[j].size() < 300)
			s200++;
		else if((int)EnvNAV3DXYT.Coord2StateIDHashTable[j].size() < 400)
			s300++;
		else
			slarge++;
	}
	printf("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d\n",
		s0,s1, s50, s100, s200,s300,slarge);
}


EnvNAV3DXYTHashEntry_t* EnvironmentNAV3DXYT::GetHashEntry(int X, int Y, int Timet)
{

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

	int binid = GETHASHBIN(X, Y, Timet);
	
#if DEBUG
	if ((int)EnvNAV3DXYT.Coord2StateIDHashTable[binid].size() > 500)
	{
		printf("WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n", 
			binid, X, Y, EnvNAV3DXYT.Coord2StateIDHashTable[binid].size());
		
		PrintHashTableHist();		
	}
#endif

	//iterate over the states in the bin and select the perfect match
	for(int ind = 0; ind < (int)EnvNAV3DXYT.Coord2StateIDHashTable[binid].size(); ind++)
	{
		if( EnvNAV3DXYT.Coord2StateIDHashTable[binid][ind]->X == X 
			&& EnvNAV3DXYT.Coord2StateIDHashTable[binid][ind]->Y == Y
			&& EnvNAV3DXYT.Coord2StateIDHashTable[binid][ind]->Timet == Timet)
		{
#if TIME_DEBUG
			time_gethash += clock()-currenttime;
#endif
			return EnvNAV3DXYT.Coord2StateIDHashTable[binid][ind];
		}
	}

#if TIME_DEBUG	
	time_gethash += clock()-currenttime;
#endif

	return NULL;	  
}


EnvNAV3DXYTHashEntry_t* EnvironmentNAV3DXYT::CreateNewHashEntry(int X, int Y, int Timet) 
{
	int i;

#if TIME_DEBUG	
	clock_t currenttime = clock();
#endif

	EnvNAV3DXYTHashEntry_t* HashEntry = new EnvNAV3DXYTHashEntry_t;

	HashEntry->X = X;
	HashEntry->Y = Y;
	HashEntry->Timet = Timet;

	HashEntry->stateID = EnvNAV3DXYT.StateID2CoordTable.size();

	//insert into the tables
	EnvNAV3DXYT.StateID2CoordTable.push_back(HashEntry);


	//get the hash table bin
	i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Timet); 

	//insert the entry into the bin
	EnvNAV3DXYT.Coord2StateIDHashTable[i].push_back(HashEntry);

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


void EnvironmentNAV3DXYT::InitializeEnvironment()
{
	EnvNAV3DXYTHashEntry_t* HashEntry;

	//initialize the map from Coord to StateID
	EnvNAV3DXYT.HashTableSize = 64*1024; //should be power of two
	EnvNAV3DXYT.Coord2StateIDHashTable = new vector<EnvNAV3DXYTHashEntry_t*>[EnvNAV3DXYT.HashTableSize];
	
	//initialize the map from StateID to Coord
	EnvNAV3DXYT.StateID2CoordTable.clear();

	//create start state 
	HashEntry = CreateNewHashEntry(EnvNAV3DXYTCfg.StartX_c, EnvNAV3DXYTCfg.StartY_c, EnvNAV3DXYTCfg.StartTimet_c);
	EnvNAV3DXYT.startstateid = HashEntry->stateID;

	//create goal state 
	HashEntry = CreateNewHashEntry(EnvNAV3DXYTCfg.EndX_c, EnvNAV3DXYTCfg.EndY_c, EnvNAV3DXYTCfg.EndTimet_c);
	EnvNAV3DXYT.goalstateid = HashEntry->stateID;
}

// ========================================================================================
// ========================================================================================


void EnvironmentNAV3DXYT::SetConfiguration_all(int width, int height, int maxTime,
					int startx, int starty, int startTimet,
					int goalx, int goaly, int goalTimet,
					double cellsize_m, double timestepsize_m,
					vector<EnvNAV3DXYT_pos_trajectory> otherBots_trajectories,
					vector<EnvNAV3DXYT_dist_trajectory> distConstraint_trajectories,
					vector<float> penaltyWeights) {

  EnvNAV3DXYTCfg.EnvWidth_c = width;
  EnvNAV3DXYTCfg.EnvHeight_c = height;
  EnvNAV3DXYTCfg.EnvMaxTime_c = maxTime;
  EnvNAV3DXYTCfg.StartX_c = startx;
  EnvNAV3DXYTCfg.StartY_c = starty;
  EnvNAV3DXYTCfg.StartTimet_c = startTimet;
 
  if(EnvNAV3DXYTCfg.StartX_c < 0 || EnvNAV3DXYTCfg.StartX_c >= EnvNAV3DXYTCfg.EnvWidth_c) {
    printf("ERROR: illegal start coordinates\n");
    exit(1);
  }
  if(EnvNAV3DXYTCfg.StartY_c < 0 || EnvNAV3DXYTCfg.StartY_c >= EnvNAV3DXYTCfg.EnvHeight_c) {
    printf("ERROR: illegal start coordinates\n");
    exit(1);
  }
  if(EnvNAV3DXYTCfg.StartTimet_c < 0 || EnvNAV3DXYTCfg.StartTimet_c >= EnvNAV3DXYTCfg.EnvMaxTime_c) {
    printf("ERROR: illegal start coordinates for Timet\n");
    exit(1);
  }
  
  EnvNAV3DXYTCfg.EndX_c = goalx;
  EnvNAV3DXYTCfg.EndY_c = goaly;
  EnvNAV3DXYTCfg.EndTimet_c = goalTimet;

  if(EnvNAV3DXYTCfg.EndX_c < 0 || EnvNAV3DXYTCfg.EndX_c >= EnvNAV3DXYTCfg.EnvWidth_c) {
    printf("ERROR: illegal goal coordinates\n");
    exit(1);
  }
  if(EnvNAV3DXYTCfg.EndY_c < 0 || EnvNAV3DXYTCfg.EndY_c >= EnvNAV3DXYTCfg.EnvHeight_c) {
    printf("ERROR: illegal goal coordinates\n");
    exit(1);
  }
  if(EnvNAV3DXYTCfg.EndTimet_c < 0 || EnvNAV3DXYTCfg.EndTimet_c >= EnvNAV3DXYTCfg.EnvMaxTime_c) {
    printf("ERROR: illegal goal coordinates for Timet\n");
    exit(1);
  }

  EnvNAV3DXYTCfg.cellsize_m = cellsize_m;
  EnvNAV3DXYTCfg.timestepsize_m = timestepsize_m;

  EnvNAV3DXYTCfg.otherBots_trajectories = otherBots_trajectories;
  EnvNAV3DXYTCfg.distConstraint_trajectories = distConstraint_trajectories;
  EnvNAV3DXYTCfg.penaltyWeights = penaltyWeights;
  if(EnvNAV3DXYTCfg.otherBots_trajectories.size() != EnvNAV3DXYTCfg.distConstraint_trajectories.size() || 
		EnvNAV3DXYTCfg.otherBots_trajectories.size() != EnvNAV3DXYTCfg.penaltyWeights.size() ) {
    printf("ERROR: number of trajectories, constraints and weights must be equal!\n");
    exit(1);
  }

}

/*
void EnvironmentNAV3DXYT::SetConfiguration_constraints(vector<int> otherBots_identifiers,
					vector<EnvNAV3DXYT_pos_trajectory> otherBots_trajectories,
					vector<EnvNAV3DXYT_dist_trajectory> distConstraint_trajectories,
					vector<float> penaltyWeights) {

  EnvNAV3DXYTCfg.otherBots_identifiers = otherBots_identifiers;
  EnvNAV3DXYTCfg.otherBots_trajectories = otherBots_trajectories;
  EnvNAV3DXYTCfg.distConstraint_trajectories = distConstraint_trajectories;
  EnvNAV3DXYTCfg.penaltyWeights = penaltyWeights;
  if(EnvNAV3DXYTCfg.otherBots_trajectories.size() != EnvNAV3DXYTCfg.distConstraint_trajectories.size() || 
		EnvNAV3DXYTCfg.otherBots_trajectories.size() != EnvNAV3DXYTCfg.penaltyWeights.size() ) {
    printf("ERROR: number of trajectories, constraints and weights must be equal!\n");
    exit(1);
  }

}
*/


// This function should ideally go separate
void EnvironmentNAV3DXYT::ReadConfiguration(FILE* fCfg)
{
	//read in the configuration of environment and initialize  EnvNAV3DXYTCfg structure
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
	EnvNAV3DXYTCfg.EnvWidth_c = atoi(sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DXYTCfg.EnvHeight_c = atoi(sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DXYTCfg.EnvMaxTime_c = atoi(sTemp);

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
	EnvNAV3DXYTCfg.cellsize_m = atof(sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DXYTCfg.timestepsize_m = atof(sTemp);
	
	#if DEBUG2
		printf("In environment_nav3Dxyt.cpp-L325 : cellsize_m=%f , timestepsize_m=%f\n", EnvNAV3DXYTCfg.cellsize_m, EnvNAV3DXYTCfg.timestepsize_m);
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
	EnvNAV3DXYTCfg.StartX_c = CONTXY2DISC(atof(sTemp),EnvNAV3DXYTCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DXYTCfg.StartY_c = CONTXY2DISC(atof(sTemp),EnvNAV3DXYTCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DXYTCfg.StartTimet_c = CONTXY2DISC(atof(sTemp),EnvNAV3DXYTCfg.timestepsize_m);

	#if DEBUG2
		printf("In environment_nav3Dxyt.cpp-L345 : StartX_c=%d , StartY_c=%d, StartTimet_c=%d\n", 
										EnvNAV3DXYTCfg.StartX_c, EnvNAV3DXYTCfg.StartY_c, EnvNAV3DXYTCfg.StartTimet_c);
	#endif

	if(EnvNAV3DXYTCfg.StartX_c < 0 || EnvNAV3DXYTCfg.StartX_c >= EnvNAV3DXYTCfg.EnvWidth_c)
	{
		printf("ERROR: illegal start coordinates\n");
		exit(1);
	}
	if(EnvNAV3DXYTCfg.StartY_c < 0 || EnvNAV3DXYTCfg.StartY_c >= EnvNAV3DXYTCfg.EnvHeight_c)
	{
		printf("ERROR: illegal start coordinates\n");
		exit(1);
	}
	if(EnvNAV3DXYTCfg.StartTimet_c < 0 || EnvNAV3DXYTCfg.StartTimet_c >= EnvNAV3DXYTCfg.EnvMaxTime_c)
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
	EnvNAV3DXYTCfg.EndX_c = CONTXY2DISC(atof(sTemp),EnvNAV3DXYTCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DXYTCfg.EndY_c = CONTXY2DISC(atof(sTemp),EnvNAV3DXYTCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DXYTCfg.EndTimet_c = CONTXY2DISC(atof(sTemp), EnvNAV3DXYTCfg.timestepsize_m);
	
	#if DEBUG2
		printf("In environment_nav3Dxyt.cpp-L345 : EndX_c=%d , EndY_c=%d, EndTimet_c=%d\n", 
										EnvNAV3DXYTCfg.EndX_c, EnvNAV3DXYTCfg.EndY_c, EnvNAV3DXYTCfg.EndTimet_c);
	#endif

	if(EnvNAV3DXYTCfg.EndX_c < 0 || EnvNAV3DXYTCfg.EndX_c >= EnvNAV3DXYTCfg.EnvWidth_c)
	{
		printf("ERROR: illegal end coordinates\n");
		exit(1);
	}
	if(EnvNAV3DXYTCfg.EndY_c < 0 || EnvNAV3DXYTCfg.EndY_c >= EnvNAV3DXYTCfg.EnvHeight_c)
	{
		printf("ERROR: illegal end coordinates\n");
		exit(1);
	}
	if(EnvNAV3DXYTCfg.EndTimet_c < 0 || EnvNAV3DXYTCfg.EndTimet_c >= EnvNAV3DXYTCfg.EnvMaxTime_c)
	{
		printf("ERROR: illegal goal coordinates for Timet\n");
		exit(1);
	}
	#if DEBUG2
		printf("In environment_nav3Dxyt.cpp-L390 : Workspace configuration, Initial and goal states read!\n");
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
			//printf("In environment_nav3Dxyt.cpp-L405 : Just read - %s !\n", sTemp);
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
			//printf("In environment_nav3Dxyt.cpp-L423 : Just read - %d, %d !\n", time_level, patch_size);
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
					newActionIndex = EnvNAV3DXYTCfg.ActionsV.size();
					EnvNAV3DXYTAction_t* TmpActionsV = new EnvNAV3DXYTAction_t;
					TmpActionsV->dTimet = dTimet;
					TmpActionsV->dX = dX;
					TmpActionsV->dY = dY;
					TmpActionsV->cost = TRANSITIONCOST_XYT(dX, dY, dTimet);
						
					if (TmpActionsV->cost == 0)
						printf("WARNING: Zero transition cost!!!\n");
						
					EnvNAV3DXYTCfg.ActionsV.push_back(*TmpActionsV);
					#if DEBUG2
						printf("In environment_nav3Dxyt.cpp-L439 : New action read : dX=%d, dY=%d, dTimet=%d, cost=%d\n", 
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


int CentralizedInfo_t::AddObstaclePointsToJointStatespace(vector<EnvNAV3DXYT_pos_trajectory> posTrajs, 
												vector<EnvNAV3DXYT_AParticularConstraint_t> theConstraints, 
												int TimetStart, int TimetEnd, bool isMethod2)
{
	int a, b, c, t;
	int DX, DY, DT;
	int ViolationCount = 0;
	vector<int> ViolatingRobotIndices;
	bool isObstacle;
	
	//EnvNAV3DXYT_pos_t* tempPos;
	//EnvNAV3DXYT_pos_trajectory* tempPosTraj;
	EnvNAV3DXYT_dist_t* InterpedDist = new EnvNAV3DXYT_dist_t;
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
	
	//EnvNAV3DXYT_dist_t* InterpedDist;
	vector<EnvNAV3DXYT_pos_t> NowPoses(posTrajs.size());
	
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
			//tempPos = new EnvNAV3DXYT_pos_t;
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
			if (CELLCOST_XYT(DX, DY, DT, InterpedDist->D) > 0)
			{
				//printf("ViolationCount recorded: Constraint=%d, time=%d, robots=%d,%d\n", a, t, theConstraints[a].Robot1, theConstraints[a].Robot2);
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

void CentralizedInfo_t_UpdatePenaltyWeights(CentralizedInfo_t* theCentralInfo, ConfigFileInfo theInfo, int ActiveRobot, EnvNAV3DXYT_pos_trajectory* traj, EnvironmentNAV3DXYT* env, bool isSummetricWeights)
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

void EvaluateTrajectoryChange(EnvNAV3DXYT_pos_trajectory* newTraj, EnvNAV3DXYT_pos_trajectory* oldTraj, EnvironmentNAV3DXYT* env, vector<int>* ConstraintNegociationFlag, float thresh)
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
	ConstraintNegociationFlag->resize(env->EnvNAV3DXYTCfg.penaltyWeights.size(), -1);
	
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
	
	CcOld.resize(env->EnvNAV3DXYTCfg.penaltyWeights.size(), 0);
	for (a=0; a < oldTraj->pos_t.size(); a++)
	{
		// Add to constraint violation cost
		//printf("---%d\n",a);
		ThisConstraintCostVector = env->ComputeCellConstraintViolationCost(oldTraj->pos_t[a].X, oldTraj->pos_t[a].Y, oldTraj->pos_t[a].Timet, false);
		if (ThisConstraintCostVector[0] != 0)
			for (b=0; b<env->EnvNAV3DXYTCfg.penaltyWeights.size(); b++)
				if (ThisConstraintCostVector[b+1]>0)
					CcOld[b] = __max(CcOld[b], ThisConstraintCostVector[b+1]);
	}
	
	CcNew.resize(env->EnvNAV3DXYTCfg.penaltyWeights.size(), 0);
	for (a=0; a < newTraj->pos_t.size(); a++)
	{
		// Add to constraint violation cost
		ThisConstraintCostVector = env->ComputeCellConstraintViolationCost(newTraj->pos_t[a].X, newTraj->pos_t[a].Y, newTraj->pos_t[a].Timet, false);
		if (ThisConstraintCostVector[0] != 0)
			for (b=0; b<env->EnvNAV3DXYTCfg.penaltyWeights.size(); b++)
				if (ThisConstraintCostVector[b+1]>0)
					CcNew[b] = __max(CcNew[b], ThisConstraintCostVector[b+1]);
	}
	
	for (b=0; b<env->EnvNAV3DXYTCfg.penaltyWeights.size(); b++)
	{
		diff = ((float)(CcOld[b]-CcNew[b])) / NAV3DXYT_COSTMULT;
		if (diff==0)
			ConstraintNegociationFlag->at(b) = -1;
		else if (diff<=thresh)
			ConstraintNegociationFlag->at(b) = 0;
		else
			ConstraintNegociationFlag->at(b) = 1;
	}
}


bool AreTrajectoriesSame(EnvNAV3DXYT_pos_trajectory* newTraj, EnvNAV3DXYT_pos_trajectory* oldTraj)
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
CanonicalSequence_t DetermineCanonicalSequence(EnvNAV3DXYT_pos_trajectory posTraj, vector<EnvNAV3DXYT2Dpt_t> ObstacleCenters)
{
	int a;
	EnvNAV3DXYT2Dpt_t startPt, endPt;
	
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


int CentralizedInfo_t::AddObstaclePointsToBlockedHomotopyClasses(vector<EnvNAV3DXYT_pos_trajectory> posTrajs, 
												vector<EnvNAV3DXYT_AParticularConstraint_t> theConstraints, 
												int TimetStart, int TimetEnd, int radi)
{
	int a, b, t;
	int DX, DY, DT;
	int ViolationCount = 0;
	bool isObstacle;
	
	//EnvNAV3DXYT_pos_t* tempPos;
	//EnvNAV3DXYT_pos_trajectory* tempPosTraj;
	EnvNAV3DXYT_dist_t* InterpedDist = new EnvNAV3DXYT_dist_t;
	JointStatespacePoint_t JointStatespacePoint, JointStatespacePoint_BallPt;
	JointStatespacePoint.RobotStateX.resize(posTrajs.size());
	JointStatespacePoint.RobotStateY.resize(posTrajs.size());
	JointStatespacePoint_BallPt = JointStatespacePoint;
	
	HomotopyClassBlocker_t thisBlockedHomotopyClass;
	
	//EnvNAV3DXYT_dist_t* InterpedDist;
	vector<EnvNAV3DXYT_pos_t> NowPoses(posTrajs.size());
	
	for (t = TimetStart; t <= TimetEnd; t++)
	{
		for (a=0; a<posTrajs.size(); a++)
		{
			//*tempPosTraj = posTraj[a];
			//FindAndInterpInTrajectory(tempPosTraj, t, tempPos);
			//NowPoses[a] = *tempPos;
			//tempPos = new EnvNAV3DXYT_pos_t;
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


vector<int> EnvironmentNAV3DXYT::ComputeCellConstraintViolationCost(int X, int Y, int T, bool CheckJointStatespaceObstacle)
{
	// Returns a vector - the element '0' is the sum of all the costs
	int a, DX, DY, DT;
	//int violationCost=0
	vector<int> violationCost;
	int constraintViolation;
	int ConstraintsCount = 0;
	int OtherRobotsCount;
	JointStatespacePoint_t JointStatespacePoint;
	EnvNAV3DXYT_pos_t* InterpedPos = new EnvNAV3DXYT_pos_t;
	EnvNAV3DXYT_dist_t* InterpedDist = new EnvNAV3DXYT_dist_t;

	OtherRobotsCount = EnvNAV3DXYTCfg.penaltyWeights.size();
	CheckJointStatespaceObstacle = CheckJointStatespaceObstacle & EnvNAV3DXYTCfg.GlobalParams.DO_JOINTSTATESPACE_LOGGING;
	
	violationCost.resize(OtherRobotsCount+1, 0);
	
	//if(!CheckJointStatespaceObstacle)
	//	printf("--->> OtherRobotsCount=%d\n", OtherRobotsCount);
	
	if (CheckJointStatespaceObstacle && OtherRobotsCount>0)
	{
		JointStatespacePoint.RobotStateX.resize(OtherRobotsCount+1);
		JointStatespacePoint.RobotStateY.resize(OtherRobotsCount+1);
		JointStatespacePoint.RobotStateX[EnvNAV3DXYTCfg.BotIdentifier] = X;
		JointStatespacePoint.RobotStateY[EnvNAV3DXYTCfg.BotIdentifier] = Y;
		JointStatespacePoint.Timet = T;
		if (EnvNAV3DXYTCfg.GlobalParams.JOINTSTATESPACE_LOGGING_METHOD==1 || EnvNAV3DXYTCfg.GlobalParams.JOINTSTATESPACE_LOGGING_METHOD==2)
			JointStatespacePoint.ViolatingRobotIndices.push_back(EnvNAV3DXYTCfg.BotIdentifier); // Just comment this line if 2N-D obstacle points are not specific to any robot
	}

	for (a = 0; a < OtherRobotsCount; a++)
	{

		if (EnvNAV3DXYTCfg.distConstraint_trajectories[a].dist_t.size() == 0)
			continue;
		
		FindAndInterpInTrajectory(&EnvNAV3DXYTCfg.otherBots_trajectories[a], T, InterpedPos);
		FindAndInterpInTrajectory(&EnvNAV3DXYTCfg.distConstraint_trajectories[a], T, InterpedDist);
		
		// Check if the joint state-space point is valid
		if (CheckJointStatespaceObstacle)
		{
			JointStatespacePoint.RobotStateX[EnvNAV3DXYTCfg.otherBots_identifiers[a]] = InterpedPos->X;
			JointStatespacePoint.RobotStateY[EnvNAV3DXYTCfg.otherBots_identifiers[a]] = InterpedPos->Y;
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
			//	violationCost += (int) pow(((float) constraintViolation) * EnvNAV3DXYTCfg.penaltyWeights[a], EnvNAV3DXYTCfg.GlobalParams.VIOLATION_COST_POWER);
			//else
			//	violationCost += (int) pow(((float) constraintViolation), EnvNAV3DXYTCfg.GlobalParams.VIOLATION_COST_POWER);
			if (CheckJointStatespaceObstacle)
				violationCost[a+1] = (int) pow(((float) constraintViolation) * EnvNAV3DXYTCfg.penaltyWeights[a], EnvNAV3DXYTCfg.GlobalParams.VIOLATION_COST_POWER);
			else
				violationCost[a+1] = (int) pow(((float) constraintViolation), EnvNAV3DXYTCfg.GlobalParams.VIOLATION_COST_POWER);
			//violationCost += (int) pow((((float) constraintViolation) * EnvNAV3DXYTCfg.penaltyWeights[a]),2.0);
			//printf("\nPositive violation cost: %d", (int) pow(((float) constraintViolation) * EnvNAV3DXYTCfg.penaltyWeights[a], EnvNAV3DXYTCfg.GlobalParams.VIOLATION_COST_POWER));
		}
			
		if (EnvNAV3DXYTCfg.distConstraint_trajectories[a].dist_t[T].D < INFINITECOST)
			ConstraintsCount++;
			
		#if DEBUG2
			printf("In environment_nav3Dxyt.cpp-L527 (ComputeCellConstraintViolationCost) : (%d,%d,%d) - (%d,%d,%d) - Cost %d\n", 
					X, Y, T, EnvNAV3DXYTCfg.otherBots_trajectories[a].pos_t[T].X, EnvNAV3DXYTCfg.otherBots_trajectories[a].pos_t[T].Y, 
						EnvNAV3DXYTCfg.otherBots_trajectories[a].pos_t[T].Timet, (int) pow(((float) constraintViolation) * EnvNAV3DXYTCfg.penaltyWeights[a], EnvNAV3DXYTCfg.GlobalParams.VIOLATION_COST_POWER));
		#endif

	}

	if (CheckJointStatespaceObstacle && EnvNAV3DXYTCfg.CentralizedInfo->isPointInJointStatespaceObstacles(JointStatespacePoint,1,EnvNAV3DXYTCfg.GlobalParams.JOINTSTATESPACE_LOGGING_RAD))
		violationCost[0] = INFINITECOST;
	else
		for (a = 0; a < OtherRobotsCount; a++)
			violationCost[0] = violationCost[0] + violationCost[a+1];
	//else if (ConstraintsCount > 0)
	//	violationCost = (int)((float)violationCost / (float)ConstraintsCount);
	
	delete InterpedPos; // DELETE**
	delete InterpedDist; // DELETE**
	return violationCost;
}


bool EnvironmentNAV3DXYT::IsValidCell(int X, int Y, int T, bool CheckStaticObstacle, bool CheckDynamicObstacle)
{
	bool retRes = false;
	
	if (X >= 0 && X < EnvNAV3DXYTCfg.EnvWidth_c && 
		Y >= 0 && Y < EnvNAV3DXYTCfg.EnvHeight_c && 
		T >= 0 && T < EnvNAV3DXYTCfg.EnvMaxTime_c)
		retRes = true;
	else
		return false;
	
	if (CheckStaticObstacle && EnvNAV3DXYTCfg.StaticObstacleMap.get(X,Y))
		return false;
		
	if (CheckDynamicObstacle && EnvNAV3DXYTCfg.DynamicObstacleMap.get(X,Y,T))
		return false;
	
	return retRes;
}

bool EnvironmentNAV3DXYT::IsWithinMapCell(int X, int Y, int T)
{
	return (X >= 0 && X < EnvNAV3DXYTCfg.EnvWidth_c && 
		Y >= 0 && Y < EnvNAV3DXYTCfg.EnvHeight_c && 
		T >= 0 && T < EnvNAV3DXYTCfg.EnvMaxTime_c);
}


int EnvironmentNAV3DXYT::GetActionCost(int SourceX, int SourceY, int SourceTimet, EnvNAV3DXYTAction_t* action)
{

	int TargetX, TargetY, TargetTime, netCost;
	vector<int> netCostVector;

	TargetX = SourceX + action->dX;
	TargetY = SourceY + action->dY;
	TargetTime = SourceTimet + action->dTimet;

	if (IsValidCell(SourceX, SourceY, SourceTimet ))
	{
		netCostVector = ComputeCellConstraintViolationCost(TargetX, TargetY, TargetTime);
		netCost = netCostVector[0] + action->cost;
	}
	else
		netCost = INFINITECOST;

	#if DEBUG2
		printf("In environment_nav3Dxyt.cpp-L527 (GetActionCost) : In state (%d,%d,%d) - Action (%d,%d,%d) - NetCost(%d) = NodeCost(%d) + ActionCost(%d)\n", SourceX, SourceX, SourceTimet, 
							action->dX, action->dY, action->dTimet, netCost, ComputeCellConstraintViolationCost(TargetX, TargetY, TargetTime), action->cost);
		char tmpc;
		// scanf("%c", &tmpc);
	#endif

	return (netCost);

}

// =================================
// ----------------------------------------------------------------------------------------
// Code to automatically determine the amount of increment in the penalty weights


void ComputeTrajectoryCost(EnvNAV3DXYT_pos_trajectory* traj, EnvironmentNAV3DXYT* env, int* Cp, vector<int>* Cc, vector< vector<int> >* ConstraintViolationIndices)
{
	int a, b, ThisConstraintCost;
	bool foundAction;
	vector<int> NonzeroViolationIndices;
	vector<int> ThisConstraintCostVector;
	
	*Cp = 0;
	Cc->clear();
	ConstraintViolationIndices->clear();
	Cc->resize(env->EnvNAV3DXYTCfg.penaltyWeights.size(), 0);
	ConstraintViolationIndices->resize(env->EnvNAV3DXYTCfg.penaltyWeights.size());
	
	for (a=0; a < traj->pos_t.size(); a++)
	{
		// Add transition cost to path cost
		//printf("^^^^^ %d, %d, %d - %ld - ", traj->pos_t[a].X, traj->pos_t[a].Y, traj->pos_t[a].Timet, 
		//		env->EnvNAV3DXYTCfg.distConstraint_trajectories[0].dist_t[a].D);
		if (a>0)
		{
			foundAction = false;
			for (b=0; b < env->EnvNAV3DXYTCfg.ActionsV.size(); b++)
			{
				if (traj->pos_t[a-1].X + env->EnvNAV3DXYTCfg.ActionsV[b].dX == traj->pos_t[a].X && 
						traj->pos_t[a-1].Y + env->EnvNAV3DXYTCfg.ActionsV[b].dY == traj->pos_t[a].Y && 
						traj->pos_t[a-1].Timet + env->EnvNAV3DXYTCfg.ActionsV[b].dTimet == traj->pos_t[a].Timet)
				{
					*Cp = *Cp + (int)env->EnvNAV3DXYTCfg.ActionsV[b].cost;
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
			for (b=0; b<env->EnvNAV3DXYTCfg.penaltyWeights.size(); b++)
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
vector<int> FindConstraintViolationIndices(EnvNAV3DXYT_pos_trajectory* traj, EnvironmentNAV3DXYT* env)
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

vector<EnvNAV3DXYT_pos_trajectory> FindVariationsOfATrajectory(EnvNAV3DXYT_pos_trajectory* traj, EnvironmentNAV3DXYT* env, vector<int> varIndices)
{
	// Changes the XYT position of the trajectory at varIndices to return new trajectories
	int a, b, v, varIndex;
	int testX, testY, testT, test2X, test2Y, test2T;
	EnvNAV3DXYT_pos_trajectory tempTraj;
	vector<EnvNAV3DXYT_pos_trajectory> alteredTrajs;
	
	for (v=0; v<varIndices.size(); v++)
	{
		varIndex = varIndices[v];
		if (varIndex<=0 || varIndex>=(traj->pos_t.size()-1))
			continue;
		
		for (a=0; a < env->EnvNAV3DXYTCfg.ActionsV.size(); a++)
		{
			tempTraj = *traj;
			testX = traj->pos_t[varIndex-1].X + env->EnvNAV3DXYTCfg.ActionsV[a].dX;
			testY = traj->pos_t[varIndex-1].Y + env->EnvNAV3DXYTCfg.ActionsV[a].dY;
			testT = traj->pos_t[varIndex-1].Timet + env->EnvNAV3DXYTCfg.ActionsV[a].dTimet;
			if ((testX==traj->pos_t[varIndex].X && testY==traj->pos_t[varIndex].Y && testT==traj->pos_t[varIndex].Timet) || 
					!env->IsValidCell(testX, testY, testT, true, false))
				continue;
			for (b=0; b < env->EnvNAV3DXYTCfg.ActionsV.size(); b++)
			{
				test2X = testX + env->EnvNAV3DXYTCfg.ActionsV[b].dX;
				test2Y = testY + env->EnvNAV3DXYTCfg.ActionsV[b].dY;
				test2T = testT + env->EnvNAV3DXYTCfg.ActionsV[b].dTimet;
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


vector<float> SuggestNextPenaltyWeight(EnvNAV3DXYT_pos_trajectory* traj, EnvironmentNAV3DXYT* env)
{
	int a, b, aa, bb, c, Cp, CpNew, Dp, Dc, DcSum=0, DpMax=0, DpMin=INFINITECOST;
	vector<int> CcNew, Cc;
	vector< vector<int> > ConstraintViolationIndices, dummyConstraintViolationIndices;
	//?float thisDeltaGamma, outDeltaGamma=0;
	float thisGamma;
	vector<float> SuggestedGamma;
	//vector<int> ConstraintViolationIndices;
	vector< vector<EnvNAV3DXYT_pos_trajectory> > VariedTrajectories;
	
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
int EnvironmentNAV3DXYT::GetActionCost(int SourceX, int SourceY, int SourceTimet, int TargetX, int TargetY, int TargetTimet)
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
    return (int)(NAV3DXYT_COSTMULT*dist);

}
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx


int EnvironmentNAV3DXYT::HuristicFunction(int X1, int Y1, int X2, int Y2)
{
	int sqdist, DX, DY;
	double dist;
	
	switch(EnvNAV3DXYTCfg.GlobalParams.HURISTIC_TYPE)
	{
		case 1 :
			sqdist = ((X1-X2)*(X1-X2)+(Y1-Y2)*(Y1-Y2));
    		dist = sqrt((double)sqdist);
    		return (int)(NAV3DXYT_COSTMULT*dist);
			break;
			
		case 2 :  // Only for 8-connected grids
			DX = abs(X1-X2);
			DY = abs(Y1-Y2);
			dist = sqrt(2.0)*((double)(__min(DX,DY))) + ((double)(abs(DX-DY)));
    		return (int)(NAV3DXYT_COSTMULT*dist);
			break;
	}
}


/*
void EnvironmentNAV3DXYT::CalculateFootprintForPose(EnvNAV3DXYT3Dpt_t pose, vector<sbpl_2Dcell_t>* footprint)
{  

#if DEBUG
  printf("---Calculating Footprint for Pose: %f %f %f---\n",
	 pose.x, pose.y, pose.Timet);
#endif

  //handle special case where footprint is just a point
  if(EnvNAV3DXYTCfg.FootprintPolygon.size() <= 1){
    sbpl_2Dcell_t cell;
    cell.x = CONTXY2DISC(pose.x, EnvNAV3DXYTCfg.cellsize_m);
    cell.y = CONTXY2DISC(pose.y, EnvNAV3DXYTCfg.cellsize_m);
    footprint->push_back(cell);
    return;
  }

  vector<sbpl_2Dpt_t> bounding_polygon;
  unsigned int find;
  double max_x, min_x, max_y, min_y;
  sbpl_2Dpt_t pt;
  for(find = 0; find < EnvNAV3DXYTCfg.FootprintPolygon.size(); find++){
    
    //rotate and translate the corner of the robot
    pt = EnvNAV3DXYTCfg.FootprintPolygon[find];
    
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
  int prev_discrete_x = CONTXY2DISC(pt.x, EnvNAV3DXYTCfg.cellsize_m) + 1; 
  int prev_discrete_y = CONTXY2DISC(pt.y, EnvNAV3DXYTCfg.cellsize_m) + 1;
  int prev_inside = 0;
  int discrete_x;
  int discrete_y;

  for(double x=min_x; x<=max_x; x+=EnvNAV3DXYTCfg.cellsize_m/3){
    for(double y=min_y; y<=max_y; y+=EnvNAV3DXYTCfg.cellsize_m/3){
      pt.x = x;
      pt.y = y;
      discrete_x = CONTXY2DISC(pt.x, EnvNAV3DXYTCfg.cellsize_m);
      discrete_y = CONTXY2DISC(pt.y, EnvNAV3DXYTCfg.cellsize_m);
      
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

void EnvironmentNAV3DXYT::ComputeHeuristicValues()
{
	//whatever necessary pre-computation of heuristic values is done here 
	printf("Precomputing heuristics...\n");
	


	printf("done\n");

}

//------------debugging functions---------------------------------------------
bool EnvironmentNAV3DXYT::CheckQuant(FILE* fOut) 
{

	/*
  for(double Timet  = -10; Timet < 10; Timet += 2.0*PI_CONST/NAV3DXYT_TimetDIRS*0.01)
    {
		int nTimet = ContTimet2Disc(Timet, NAV3DXYT_TimetDIRS);
		double newTimet = DiscTimet2Cont(nTimet, NAV3DXYT_TimetDIRS);
		int nnewTimet = ContTimet2Disc(newTimet, NAV3DXYT_TimetDIRS);

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


bool EnvironmentNAV3DXYT::InitializeEnv(const char* sEnvFile)
{
	// This function is of no use!!
	// It is here only because of it's declaration in DiscreteSpaceInformation class in environment.h
	return true;
}

/*
bool EnvironmentNAV3DXYT::InitializeEnv(const char* sEnvFile)
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


bool EnvironmentNAV3DXYT::InitializeEnv(const char* sEnvFile, vector<EnvNAV3DXYT_pos_trajectory> otherBots_trajectories,
					vector<EnvNAV3DXYT_dist_trajectory> distConstraint_trajectories,
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
bool EnvironmentNAV3DXYT::InitializeEnv(vector<int> otherBots_identifiers,
					vector<EnvNAV3DXYT_pos_trajectory> otherBots_trajectories,
					vector<EnvNAV3DXYT_dist_trajectory> distConstraint_trajectories,
					vector<float> penaltyWeights)
{

	//SetConfiguration_constraints(otherBots_identifiers, otherBots_trajectories, distConstraint_trajectories, penaltyWeights);
	EnvNAV3DXYTCfg.otherBots_identifiers = otherBots_identifiers;
	EnvNAV3DXYTCfg.otherBots_trajectories = otherBots_trajectories;
	EnvNAV3DXYTCfg.distConstraint_trajectories = distConstraint_trajectories;
	EnvNAV3DXYTCfg.penaltyWeights = penaltyWeights;

	// InitGeneral(); // Don't call this multiple times!


	return true;
}
*/

/*
// Initialize everything
bool EnvironmentNAV3DXYT::InitializeEnv(ConfigFileInfo* CfgInfo, CentralizedInfo_t* CentralizedInfo, int robotIndex,
										vector<int> otherBots_identifiers,
										vector<EnvNAV3DXYT_pos_trajectory> otherBots_trajectories,
										vector<EnvNAV3DXYT_dist_trajectory> distConstraint_trajectories,
										vector<float> penaltyWeights)
{
	EnvNAV3DXYTCfg.EnvWidth_c = CfgInfo->EnvWidth_c;
	EnvNAV3DXYTCfg.EnvHeight_c = CfgInfo->EnvHeight_c;
	EnvNAV3DXYTCfg.EnvMaxTime_c = CfgInfo->EnvMaxTime_c;
	
	EnvNAV3DXYTCfg.cellsize_m = CfgInfo->cellsize_m;
	EnvNAV3DXYTCfg.timestepsize_m = CfgInfo->timestepsize_m;
	
	EnvNAV3DXYTCfg.StartX_c = CfgInfo->TheRobots[robotIndex].StartX_c;
	EnvNAV3DXYTCfg.StartY_c = CfgInfo->TheRobots[robotIndex].StartY_c;
	EnvNAV3DXYTCfg.StartTimet_c = CfgInfo->TheRobots[robotIndex].StartTimet_c;
	
	EnvNAV3DXYTCfg.EndX_c = CfgInfo->TheRobots[robotIndex].EndX_c;
	EnvNAV3DXYTCfg.EndY_c = CfgInfo->TheRobots[robotIndex].EndY_c;
	EnvNAV3DXYTCfg.EndTimet_c = CfgInfo->TheRobots[robotIndex].EndTimet_c;
	
	EnvNAV3DXYTCfg.BotIdentifier = robotIndex;
	EnvNAV3DXYTCfg.GlobalParams = CfgInfo->GlobalParams;
	EnvNAV3DXYTCfg.CentralizedInfo = CentralizedInfo;
	
	EnvNAV3DXYTCfg.ActionsV = CfgInfo->TheRobots[robotIndex].ActionsV;
	
	//SetConfiguration_constraints(otherBots_identifiers, otherBots_trajectories, distConstraint_trajectories, penaltyWeights);
	EnvNAV3DXYTCfg.otherBots_identifiers = otherBots_identifiers;
	EnvNAV3DXYTCfg.otherBots_trajectories = otherBots_trajectories;
	EnvNAV3DXYTCfg.distConstraint_trajectories = distConstraint_trajectories;
	EnvNAV3DXYTCfg.penaltyWeights = penaltyWeights;
	
	EnvNAV3DXYTCfg.StaticObstacleMap.data = CfgInfo->StaticObstacleMap.data;
	EnvNAV3DXYTCfg.StaticObstacleMap.size_x = CfgInfo->StaticObstacleMap.size_x;
	EnvNAV3DXYTCfg.StaticObstacleMap.size_y = CfgInfo->StaticObstacleMap.size_y;
	
	EnvNAV3DXYTCfg.DynamicObstacleMap.init(CfgInfo->EnvWidth_c, CfgInfo->EnvHeight_c, CfgInfo->EnvMaxTime_c, false);
	EnvNAV3DXYTCfg.DynamicObstacleMap.ConstructFromTrajectories(otherBots_trajectories, EnvNAV3DXYTCfg.GlobalParams.COLLISION_CHECK_RADIUS);
	
	InitGeneral();
	
	return true;
}
*/


bool EnvironmentNAV3DXYT::SetPenaltyWeights(vector<float> penaltyWeights)
{
	EnvNAV3DXYTCfg.penaltyWeights = penaltyWeights;
	return true;
}



// Initialize everything except Penalty weights
bool EnvironmentNAV3DXYT::InitializeEnv(ConfigFileInfo* CfgInfo, CentralizedInfo_t* CentralizedInfo, int robotIndex,
										vector<int> otherBots_identifiers,
										vector<EnvNAV3DXYT_pos_trajectory> otherBots_trajectories,
										vector<EnvNAV3DXYT_dist_trajectory> distConstraint_trajectories)
{
	EnvNAV3DXYTCfg.EnvWidth_c = CfgInfo->EnvWidth_c;
	EnvNAV3DXYTCfg.EnvHeight_c = CfgInfo->EnvHeight_c;
	EnvNAV3DXYTCfg.EnvMaxTime_c = CfgInfo->EnvMaxTime_c;
	
	EnvNAV3DXYTCfg.cellsize_m = CfgInfo->cellsize_m;
	EnvNAV3DXYTCfg.timestepsize_m = CfgInfo->timestepsize_m;
	
	EnvNAV3DXYTCfg.StartX_c = CfgInfo->TheRobots[robotIndex].StartX_c;
	EnvNAV3DXYTCfg.StartY_c = CfgInfo->TheRobots[robotIndex].StartY_c;
	EnvNAV3DXYTCfg.StartTimet_c = CfgInfo->TheRobots[robotIndex].StartTimet_c;
	
	EnvNAV3DXYTCfg.EndX_c = CfgInfo->TheRobots[robotIndex].EndX_c;
	EnvNAV3DXYTCfg.EndY_c = CfgInfo->TheRobots[robotIndex].EndY_c;
	EnvNAV3DXYTCfg.EndTimet_c = CfgInfo->TheRobots[robotIndex].EndTimet_c;
	
	EnvNAV3DXYTCfg.BotIdentifier = robotIndex;
	EnvNAV3DXYTCfg.GlobalParams = CfgInfo->GlobalParams;
	EnvNAV3DXYTCfg.CentralizedInfo = CentralizedInfo;
	
	EnvNAV3DXYTCfg.ActionsV = CfgInfo->TheRobots[robotIndex].ActionsV;
	
	//SetConfiguration_constraints(otherBots_identifiers, otherBots_trajectories, distConstraint_trajectories, penaltyWeights);
	EnvNAV3DXYTCfg.otherBots_identifiers = otherBots_identifiers;
	EnvNAV3DXYTCfg.otherBots_trajectories = otherBots_trajectories;
	EnvNAV3DXYTCfg.distConstraint_trajectories = distConstraint_trajectories;
	//EnvNAV3DXYTCfg.penaltyWeights = penaltyWeights;
	
	EnvNAV3DXYTCfg.StaticObstacleMap.data = CfgInfo->StaticObstacleMap.data;
	EnvNAV3DXYTCfg.StaticObstacleMap.size_x = CfgInfo->StaticObstacleMap.size_x;
	EnvNAV3DXYTCfg.StaticObstacleMap.size_y = CfgInfo->StaticObstacleMap.size_y;
	
	EnvNAV3DXYTCfg.DynamicObstacleMap.init(CfgInfo->EnvWidth_c, CfgInfo->EnvHeight_c, CfgInfo->EnvMaxTime_c, false);
	EnvNAV3DXYTCfg.DynamicObstacleMap.ConstructFromTrajectories(otherBots_trajectories, EnvNAV3DXYTCfg.GlobalParams.COLLISION_CHECK_RADIUS);
	
	InitGeneral();
	
	return true;
}


/*
bool EnvironmentNAV3DXYT::InitializeEnv(int width, int height, int MaxTime,
					double startx, double starty, double startTimet,
					double goalx, double goaly, double goalTimet,
				    double goaltol_x, double goaltol_y, double goaltol_Timet,
					double cellsize_m, double nominalvel_mpersecs)
{
	//TODO - need to set the tolerance as well

	SetConfiguration(width, height,
					mapdata,
					CONTXY2DISC(startx, cellsize_m), CONTXY2DISC(starty, cellsize_m), ContTimet2Disc(startTimet, NAV3DXYT_TimetDIRS),
					CONTXY2DISC(goalx, cellsize_m), CONTXY2DISC(goaly, cellsize_m), ContTimet2Disc(goalTimet, NAV3DXYT_TimetDIRS),
					cellsize_m, nominalvel_mpersecs, Timetoturn45degsinplace_secs, perimeterptsV);

	InitGeneral();

	return true;
}
*/


bool EnvironmentNAV3DXYT::InitGeneral() {
  //Initialize other parameters of the environment
  // InitializeEnvConfig();
  
  //initialize Environment
  InitializeEnvironment();
  
  //pre-compute heuristics
  ComputeHeuristicValues();

  return true;
}

bool EnvironmentNAV3DXYT::InitializeMDPCfg(MDPConfig *MDPCfg)
{
	//initialize MDPCfg with the start and goal ids	
	MDPCfg->goalstateid = EnvNAV3DXYT.goalstateid;
	MDPCfg->startstateid = EnvNAV3DXYT.startstateid;

	return true;
}



int EnvironmentNAV3DXYT::GetFromToHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(FromStateID >= (int)EnvNAV3DXYT.StateID2CoordTable.size() 
		|| ToStateID >= (int)EnvNAV3DXYT.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV3DXYT... function: stateID illegal\n");
		exit(1);
	}
#endif

	//get X, Y for the state
	EnvNAV3DXYTHashEntry_t* FromHashEntry = EnvNAV3DXYT.StateID2CoordTable[FromStateID];
	EnvNAV3DXYTHashEntry_t* ToHashEntry = EnvNAV3DXYT.StateID2CoordTable[ToStateID];
	

	return HuristicFunction(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X, ToHashEntry->Y);	

}


int EnvironmentNAV3DXYT::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif

#if DEBUG
	if(stateID >= (int)EnvNAV3DXYT.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV3DXYT... function: stateID illegal\n");
		exit(1);
	}
#endif


	//define this function if it used in the planner (heuristic forward search would use it)
    return GetFromToHeuristic(stateID, EnvNAV3DXYT.goalstateid);

}


int EnvironmentNAV3DXYT::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(stateID >= (int)EnvNAV3DXYT.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV3DXYT... function: stateID illegal\n");
		exit(1);
	}
#endif

    


	//define this function if it used in the planner (heuristic backward search would use it)
    return GetFromToHeuristic(EnvNAV3DXYT.startstateid, stateID);


}



void EnvironmentNAV3DXYT::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{

	int cost;

#if DEBUG
	if(state->StateID >= (int)EnvNAV3DXYT.StateID2CoordTable.size())
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
	if(state->StateID == EnvNAV3DXYT.goalstateid)
		return;

	//get X, Y for the state
	EnvNAV3DXYTHashEntry_t* HashEntry = EnvNAV3DXYT.StateID2CoordTable[state->StateID];

	for (int aind = 0; aind < EnvNAV3DXYTCfg.ActionsV.size(); aind++)
	{
		EnvNAV3DXYTAction_t* nav3daction = &EnvNAV3DXYTCfg.ActionsV[aind];
        	int newX = HashEntry->X + nav3daction->dX;
		int newY = HashEntry->Y + nav3daction->dY;
		int newTimet = HashEntry->Timet + nav3daction->dTimet;	

		// compute and check cost
		cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Timet, nav3daction);
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

		EnvNAV3DXYTHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(newX, newY, newTimet)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(newX, newY, newTimet);
		}
		action->AddOutcome(OutHashEntry->stateID, cost, 1.0); 

#if TIME_DEBUG
		time3_addallout += clock()-currenttime;
#endif

	}
}



void EnvironmentNAV3DXYT::SetAllPreds(CMDPSTATE* state)
{
	//implement this if the planner needs access to predecessors
	
	printf("ERROR in EnvNAV3DXYT... function: SetAllPreds is undefined\n");
	exit(1);
}


void EnvironmentNAV3DXYT::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
	int actionCount, aind, cost;

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

	#if DEBUG2
		printf("In environment_nav3Dxyt.cpp-L527 : Inside GetSuccs\n");
	#endif

	actionCount = EnvNAV3DXYTCfg.ActionsV.size();

	//clear the successor array
	SuccIDV->clear();
	CostV->clear();
	SuccIDV->reserve(actionCount); 
	CostV->reserve(actionCount);

	//goal state should be absorbing
	if(SourceStateID == EnvNAV3DXYT.goalstateid)
		return;

	//get X, Y for the state
	EnvNAV3DXYTHashEntry_t* HashEntry = EnvNAV3DXYT.StateID2CoordTable[SourceStateID];
	

	for (aind = 0; aind < actionCount ; aind++)
	{
		EnvNAV3DXYTAction_t* nav3daction = &EnvNAV3DXYTCfg.ActionsV[aind];
		int newX = HashEntry->X + nav3daction->dX;
		int newY = HashEntry->Y + nav3daction->dY;
		int newTimet = HashEntry->Timet + nav3daction->dTimet;

		cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Timet, nav3daction);
		if(cost >= INFINITECOST)
			continue;


		EnvNAV3DXYTHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(newX, newY, newTimet)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(newX, newY, newTimet);
		}

		SuccIDV->push_back(OutHashEntry->stateID);
		CostV->push_back(cost);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif

}



void EnvironmentNAV3DXYT::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{

	#if DEBUG2
		printf("In environment_nav3Dxyt.cpp-L527 : Inside GetPreds");
	#endif
	
/*
    int aind;

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

    //clear the successor array
    PredIDV->clear();
    CostV->clear();
    PredIDV->reserve(NAV3DXYT_ACTIONWIDTH); 
    CostV->reserve(NAV3DXYT_ACTIONWIDTH);

	//get X, Y for the state
	EnvNAV3DXYTHashEntry_t* HashEntry = EnvNAV3DXYT.StateID2CoordTable[TargetStateID];
	
	//no predecessors if obstacle
	if(EnvNAV3DXYTCfg.Grid2D[HashEntry->X][HashEntry->Y] != 0)
		return;

	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == EnvNAV3DXYTCfg.EnvWidth_c-1 || //TODO - need to modify to take robot perimeter into account
       HashEntry->Y == 0 || HashEntry->Y == EnvNAV3DXYTCfg.EnvHeight_c-1)
        bTestBounds = true;

	for (aind = 0; aind < NAV3DXYT_ACTIONWIDTH; aind++)
	{
		EnvNAV3DXYTAction_t* nav3daction = &EnvNAV3DXYTCfg.ActionsV[HashEntry->Timet][aind];
        int predX = HashEntry->X + nav3daction->dX;
		int predY = HashEntry->Y + nav3daction->dY;
		int predTimet = NORMALIZEDISCTimet(HashEntry->Timet + nav3daction->dTimet, NAV3DXYT_TimetDIRS);	
	

		//TODO - incorrect - have to compute preds array
		
		//skip the invalid cells
		if(bTestBounds){ //TODO - need to modify to take robot perimeter into account
            if(!IsValidCell(predX, predY))
                continue;
        }

		//skip invalid diagonal move
	    if(GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Timet, nav3daction) >= INFINITECOST) //TODO -change after I have explicit backward actions
			continue;
        


    	EnvNAV3DXYTHashEntry_t* OutHashEntry;
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




int EnvironmentNAV3DXYT::SizeofCreatedEnv()
{
	return (int)EnvNAV3DXYT.StateID2CoordTable.size();
	
}

void EnvironmentNAV3DXYT::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
	if(stateID >= (int)EnvNAV3DXYT.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV3DXYT... function: stateID illegal (2)\n");
		exit(1);
	}
#endif

	if(fOut == NULL)
		fOut = stdout;

	EnvNAV3DXYTHashEntry_t* HashEntry = EnvNAV3DXYT.StateID2CoordTable[stateID];

	if(stateID == EnvNAV3DXYT.goalstateid && bVerbose)
	{
		fprintf(fOut, "the state is a goal state\n");
	}

    if(bVerbose)
    	fprintf(fOut, "X=%d Y=%d Timet=%d\n", HashEntry->X, HashEntry->Y, HashEntry->Timet);
    else
    	fprintf(fOut, "%d %d %d\n", HashEntry->X, HashEntry->Y, HashEntry->Timet);

}

void EnvironmentNAV3DXYT::GetTrajectoryFromSolutionStateIDs(vector<int> solution_stateIDs_V, EnvNAV3DXYT_pos_trajectory* posTraj)
{
	int a, cost=0;
	//int SourceX, SourceY, SourceTimet;
	
	for(a=0; a<solution_stateIDs_V.size(); a++)
	{
		EnvNAV3DXYTHashEntry_t* HashEntry = EnvNAV3DXYT.StateID2CoordTable[solution_stateIDs_V[a]];
		EnvNAV3DXYT_pos_t* tmpPos = new EnvNAV3DXYT_pos_t;
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
		delete tmpPos; // DELETE**
	}
	
	//return (cost);
	
}


void EnvironmentNAV3DXYT::GetCoordFromState(int stateID, int& x, int& y, int& Timet) const {
  EnvNAV3DXYTHashEntry_t* HashEntry = EnvNAV3DXYT.StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
  Timet = HashEntry->Timet;
}

int EnvironmentNAV3DXYT::GetStateFromCoord(int x, int y, int Timet) {

   EnvNAV3DXYTHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, Timet)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, Timet);
    }
    return OutHashEntry->stateID;
}

const EnvNAV3DXYTConfig_t* EnvironmentNAV3DXYT::GetEnvNavConfig() {
  return &EnvNAV3DXYTCfg;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAV3DXYT::SetGoal(double x_m, double y_m, double Timet_rad){

	int x = CONTXY2DISC(x_m, EnvNAV3DXYTCfg.cellsize_m);
	int y = CONTXY2DISC(x_m, EnvNAV3DXYTCfg.cellsize_m);
	int Timet = CONTXY2DISC(Timet_rad, EnvNAV3DXYTCfg.timestepsize_m);

    if(!IsWithinMapCell(x,y,Timet))
        return -1;

    EnvNAV3DXYTHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, Timet)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, Timet);
    }
    EnvNAV3DXYT.goalstateid = OutHashEntry->stateID;

    return EnvNAV3DXYT.goalstateid;    

}


//returns the stateid if success, and -1 otherwise
int EnvironmentNAV3DXYT::SetStart(double x_m, double y_m, double Timet_rad){

	int x = CONTXY2DISC(x_m, EnvNAV3DXYTCfg.cellsize_m);
	int y = CONTXY2DISC(x_m, EnvNAV3DXYTCfg.cellsize_m);
	int Timet = CONTXY2DISC(Timet_rad, EnvNAV3DXYTCfg.timestepsize_m);

    if(!IsWithinMapCell(x,y,Timet))
        return -1;

    EnvNAV3DXYTHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, Timet)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, Timet);
    }
    EnvNAV3DXYT.startstateid = OutHashEntry->stateID;

    return EnvNAV3DXYT.startstateid;    

}

/*
bool EnvironmentNAV3DXYT::UpdateCost(int x, int y, int new_status)
{

    EnvNAV3DXYTCfg.Grid2D[x][y] = new_status;

    return true;
}
*/


void EnvironmentNAV3DXYT::PrintEnv_Config(FILE* fOut)
{

	//implement this if the planner needs to print out EnvNAV3DXYT. configuration
	
	printf("ERROR in EnvNAV3DXYT... function: PrintEnv_Config is undefined\n");
	exit(1);

}

void EnvironmentNAV3DXYT::PrintTimeStat(FILE* fOut)
{

#if TIME_DEBUG
    fprintf(fOut, "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, time_getsuccs = %f\n",
            time3_addallout/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC, 
            time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
}


/*
void EnvironmentNAV3DXYT::GetPredsofChangedEdges(vector<nav2dcell_t>* changedcellsV, vector<int> *preds_of_changededgesIDV)
{
	nav2dcell_t cell;

	for(int i = 0; i < (int)changedcellsV->size(); i++)
	{
		cell = changedcellsV->at(i);
		for(int tind = 0; tind < NAV3DXYT_TimetDIRS; tind++)
			preds_of_changededgesIDV->push_back(GetStateFromCoord(cell.x,cell.y,tind));
		for(int j = 0; j < 8; j++){
			int affx = cell.x + EnvNAV3DXYTCfg.dXY[j][0];
			int affy = cell.y + EnvNAV3DXYTCfg.dXY[j][1];
			if(affx < 0 || affx >= EnvNAV3DXYTCfg.EnvWidth_c || affy < 0 || affy >= EnvNAV3DXYTCfg.EnvHeight_c)
				continue;
			for(int tind = 0; tind < NAV3DXYT_TimetDIRS; tind++)
				preds_of_changededgesIDV->push_back(GetStateFromCoord(affx,affy,tind));
		}
	}
}
*/


/*
bool EnvironmentNAV3DXYT::IsObstacle(int x, int y)
{

	return (EnvNAV3DXYTCfg.Grid2D[x][y] != 0);

}
*/


void EnvironmentNAV3DXYT::GetEnvParms(int *size_x, int *size_y, double* startx, double* starty, double*startTimet, 
							double* goalx, double* goaly, double* goalTimet, double* cellsize_m)
{
	*size_x = EnvNAV3DXYTCfg.EnvWidth_c;
	*size_y = EnvNAV3DXYTCfg.EnvHeight_c;

	*startx = DISCXY2CONT(EnvNAV3DXYTCfg.StartX_c, EnvNAV3DXYTCfg.cellsize_m);
	*starty = DISCXY2CONT(EnvNAV3DXYTCfg.StartY_c, EnvNAV3DXYTCfg.cellsize_m);
	*startTimet = DISCXY2CONT(EnvNAV3DXYTCfg.StartTimet_c, EnvNAV3DXYTCfg.timestepsize_m);
	*goalx = DISCXY2CONT(EnvNAV3DXYTCfg.EndX_c, EnvNAV3DXYTCfg.cellsize_m);
	*goaly = DISCXY2CONT(EnvNAV3DXYTCfg.EndY_c, EnvNAV3DXYTCfg.cellsize_m);
	*goalTimet = DISCXY2CONT(EnvNAV3DXYTCfg.EndTimet_c, EnvNAV3DXYTCfg.timestepsize_m);

	*cellsize_m = EnvNAV3DXYTCfg.cellsize_m;
	// *nominalvel_mpersecs = EnvNAV3DXYTCfg.nominalvel_mpersecs;
	// *Timetoturn45degsinplace_secs = EnvNAV3DXYTCfg.Timetoturn45degsinplace_secs;
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

void PenaltyTracker_t::next(EnvNAV3DXYT_pos_trajectory* ThisBotLatestTraj, EnvironmentNAV3DXYT* env)
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
		Weight = Weight + FindPenaltyWeightIncrement(Weight, EnvNAV3DXYT_pos_trajectory* traj, EnvironmentNAV3DXYT* env);
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

void ReadConfigurationFile(const char* sEnvFile, ConfigFileInfo* theInfo)
{
	char thisLine[1024], firstString[1024], checkString[1024];
	char thisLineLong[8000];
	char s1[1024], s2[1024];
	bool FileEnd=false;
	int i1, i2, i3, i4;
	int a, b, c, d, X, Y, dTimet, dX, dY, al, bl, InfRad;
	float f1, f2, f3, f4;
	EnvNAV3DXYT_dist_t thisDistPoint;
	EnvNAV3DXYT_AParticularRobot_t* thisRobot;
	EnvNAV3DXYT_AParticularConstraint_t* thisConstraint;
	EnvNAV3DXYTAction_t* TmpActionsV;
	// Tracking numbers:
	int robotNo=-1;
	int obstacleNo=-1;
	//InfRad = theInfo->GlobalParams.STATIC_OBSTACLE_INFLATION_RADIUS;
	
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
	
	while(!FileEnd)
	{
		
		fgets(thisLine, 1024 , fCfg);
		if(strlen(thisLine)<=2)
			continue;
		sscanf(thisLine, "%s", firstString);
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
		
		// HuristicType
		strcpy(checkString, "HuristicType:");
		if(strcmp(checkString, firstString) == 0)
		{
			sscanf(thisLine, "%*s %d", &i1);
			theInfo->GlobalParams.HURISTIC_TYPE = i1;
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
		
		
		/*** Robot information ***/
		
		// BEGIN_ROBOT
		strcpy(checkString, "BEGIN_ROBOT:");
		if(strcmp(checkString, firstString) == 0)
		{
			insideROBOT = true;
			thisRobot = new EnvNAV3DXYT_AParticularRobot_t;
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
			sscanf(thisLine, "%*s %f %f %f", &f1, &f2, &f3);
			thisRobot->StartX_c = CONTXY2DISC(f1,theInfo->cellsize_m);
			thisRobot->StartY_c = CONTXY2DISC(f2,theInfo->cellsize_m);
			thisRobot->StartTimet_c = CONTXY2DISC(f3,theInfo->timestepsize_m);
			//printf("\n %^%^%^ Start: %d, %d, %d\n", thisRobot->StartX_c, thisRobot->StartY_c, thisRobot->StartTimet_c);
			continue;
		}
		
		// end
		strcpy(checkString, "end:");
		if(strcmp(checkString, firstString) == 0 && insideROBOT)
		{
			sscanf(thisLine, "%*s %f %f %f", &f1, &f2, &f3);
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
						TmpActionsV = new EnvNAV3DXYTAction_t;
						TmpActionsV->dTimet = dTimet;
						TmpActionsV->dX = dX;
						TmpActionsV->dY = dY;
						TmpActionsV->cost = TRANSITIONCOST_XYT(dX, dY, dTimet);
							
						thisRobot->ActionsV.push_back(*TmpActionsV);
					}
				}
			}
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
			thisConstraint = new EnvNAV3DXYT_AParticularConstraint_t;
			sscanf(thisLine, "%*s %d %d", &i1, &i2);
			thisConstraint->Robot1 = i1;
			thisConstraint->Robot2 = i2;
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
			sscanf(thisLine, "%*s %f %f %f", &f1, &f2, &f3);
			i1 = CONTXY2DISC(f1,theInfo->timestepsize_m);
			i2 = CONTXY2DISC(f2,theInfo->timestepsize_m);
			//f3 = f3 / theInfo->cellsize_m;
			if(f3<0.0)
				f3 = 100 * INFINITECOST / NAV3DXYT_COSTMULT;
			for(a=i1; a<=i2; a++)
			{
				thisDistPoint.Timet = a;
				thisDistPoint.D = CONTXY2DISC(f3*NAV3DXYT_COSTMULT,theInfo->cellsize_m);
				PutInTrajectory(&(thisConstraint->constraint), thisDistPoint);
				#if DEBUG4
					printf("Parsing Range: %d - %f %d\n",thisDistPoint.Timet,f3,thisDistPoint.D);
				#endif
			}
			continue;
		}
		
		// Point
		strcpy(checkString, "Point:");
		if(strcmp(checkString, firstString) == 0 && insideCONSTRAINT)
		{
			sscanf(thisLine, "%*s %f %f", &f1, &f3);
			i1 = CONTXY2DISC(f1,theInfo->timestepsize_m);
			//f3 = f3 / theInfo->cellsize_m;
			if(f3<0.0)
				f3 = 100 * INFINITECOST / NAV3DXYT_COSTMULT;
			thisDistPoint.Timet = i1;
			thisDistPoint.D = CONTXY2DISC(f3*NAV3DXYT_COSTMULT,theInfo->cellsize_m);
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
			/*EnvNAV3DXYT2Dpt_t* ThisObstacleCenter = new EnvNAV3DXYT2Dpt_t;
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
			/*EnvNAV3DXYT2Dpt_t* ThisObstacleCenter = new EnvNAV3DXYT2Dpt_t;
			ThisObstacleCenter->x = (i1+i3)/2;
			ThisObstacleCenter->y = (i2+i4)/2;
			theInfo->GlobalParams.ObstacleCenters.push_back(ThisObstacleCenter);*/
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
			delete SampledMapSum;
			continue;
		}
		
		// END_STATIC_OBSTACLE
		strcpy(checkString, "END_STATIC_OBSTACLE:");
		if(strcmp(checkString, firstString) == 0)
		{
			insideSTATICOBSTACLE = false;
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
	
	slope = ((float)(y2-y1)) / ((float)(x2-x1));
	return (y1+(int)(slope*(xs-x1)));
}


// =================================================================

vector<int> FindAndInterpInTrajectory(EnvNAV3DXYT_dist_trajectory* distTraj, int timet, EnvNAV3DXYT_dist_t* InterpedDist)
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

void PutInTrajectory(EnvNAV3DXYT_dist_trajectory* distTraj, EnvNAV3DXYT_dist_t distToPut)
{
	int a;
	vector<EnvNAV3DXYT_dist_t>::iterator Iter;
	
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

vector<int> FindAndInterpInTrajectory(EnvNAV3DXYT_pos_trajectory* posTraj, int timet, EnvNAV3DXYT_pos_t* InterpedPos)
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

void PutInTrajectory(EnvNAV3DXYT_pos_trajectory* posTraj, EnvNAV3DXYT_pos_t posToPut)
{
	int a;
	vector<EnvNAV3DXYT_pos_t>::iterator Iter;
	
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

obstacleMap2D::obstacleMap2D(int xSize, int ySize)
{
	data = (bool*) malloc(xSize*ySize*sizeof(bool));
	size_x = xSize;
	size_y = ySize;
}

obstacleMap2D::obstacleMap2D(int xSize, int ySize, bool initVal)
{
	int a, b;
	size_x = xSize;
	size_y = ySize;
	data = (bool*) malloc(xSize*ySize*sizeof(bool));
	for (a=0; a<size_x; a++)
		for (b=0; b<size_y; b++)
			*(data + size_x*b + a) = initVal;
}

void obstacleMap2D::init(int xSize, int ySize, bool initVal)
{
	int a, b;
	size_x = xSize;
	size_y = ySize;
	data = (bool*) malloc(xSize*ySize*sizeof(bool));
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
	data = (bool*) malloc(xSize*ySize*tSize*sizeof(bool));
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
	data = (bool*) malloc(xSize*ySize*tSize*sizeof(bool));
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
	data = (bool*) malloc(xSize*ySize*tSize*sizeof(bool));
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

void obstacleMap3D::ConstructFromTrajectories(vector<EnvNAV3DXYT_pos_trajectory> otherBots_trajectories, int collisionCheckRad)
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
EnvNAV3DXYT_dist_trajectory* GenerateDistanceConstraint(EnvNAV3DXYT_pos_trajectory RefPosTraj, int dist)
{
	// 'dist' is in cell units (not meters), and NOT multiplied by NAV3DXYT_COSTMULT
	EnvNAV3DXYT_dist_trajectory* OutDistTraj = new EnvNAV3DXYT_dist_trajectory;
	int a;
	
	for(a=0; a<RefPosTraj.pos_t.size(); a++)
	{
		EnvNAV3DXYT_dist_t* TempDist = new EnvNAV3DXYT_dist_t;
		TempDist->Timet = RefPosTraj.pos_t[a].Timet;
		//TempDist->D = dist*NAV3DXYT_COSTMULT;
		if (a == RefPosTraj.pos_t.size()/2)
			TempDist->D = 0;
		else
			TempDist->D = 100*NAV3DXYT_COSTMULT;
		
		OutDistTraj->dist_t.push_back(*TempDist);
	}
	
	return OutDistTraj;
}
*/
//------------------------------------------------------------------------------

int TRANSITIONCOST_XYT(int DX, int DY, int DT)
{
	if (DX == 0 && DY == 0)
		return (NAV3DXYT_COSTMULT/5);
	else
	{
		//NEW: return ((int) pow(NAV3DXYT_COSTMULT * sqrt((double)(DX*DX + DY*DY)),COST_NORM_ORDER));
		return ((int) (NAV3DXYT_COSTMULT * sqrt((double)(DX*DX + DY*DY))));
	}
}


int CELLCOST_XYT(int DX, int DY, int DT, int ConstraintDist)
{
	//NEW: return (((int) (pow(NAV3DXYT_COSTMULT * sqrt((double)(DX*DX + DY*DY)),COST_NORM_ORDER))) - pow(ConstraintDist,COST_NORM_ORDER));
	return (((int) (NAV3DXYT_COSTMULT * sqrt((double)(DX*DX + DY*DY)))) - ConstraintDist);
	//return ((int) NAV3DXYT_COSTMULT * (((double)(DX*DX + DY*DY)) - ((double) ConstraintDist/NAV3DXYT_COSTMULT)^2 ));
}

//------------------------------------------------------------------------------

void OutputFile::WriteTrajectories(vector<EnvNAV3DXYT_pos_trajectory> pos_trajectories, int IterNo, int ActiveRobot, vector<float> PenaltyWeights, vector< vector<int> > PenaltyWeightRobotsThisIteration)
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
	}
	
	fflush(fOut);
}

void OutputFile::WriteStaticObstacles(obstacleMap2D obs)
{
	int a, b;
	
	fprintf(fOut, "OBSTACLE\n");
	for (a=0; a<obs.size_x; a++)
		for (b=0; b<obs.size_y; b++)
			if(obs.get(a,b))
				fprintf(fOut, "%d %d\n", a, b);
				
	fflush(fOut);
}

void OutputFile::WriteConstraints(vector<EnvNAV3DXYT_AParticularConstraint_t> TheConstraints)
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
