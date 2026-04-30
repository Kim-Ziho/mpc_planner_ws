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

#include <iostream>
#include <time.h>
#include "../sbpl/headers.h"

//#define ROBOT_COUNT 3
//#define ROBOT_ORDERED_PAIR_INDEX(R1,R2) ((R1)*ROBOT_COUNT+(R2))
//#define ROBOT_PAIR_INDEX(R1,R2) ((((R1)<(R2))?((R1)*ROBOT_COUNT+(R2)):((R2)*ROBOT_COUNT+(R1)))-1)

int ROBOT_COUNT;

void PrintUsage(char *argv[])
{
	printf("USAGE: %s <cfg file>\n", argv[0]);
}



int planandnavigate3Dxyt(int argc, char *argv[])
{

	int bRet = 0, Timet, a, b, r1, r2, SolnCost;
	int ThisOtherRobotIndex;
	double allocated_time_secs = 1000.0; //in seconds
	char checkString[1024];
	int ViolationCount;
	bool isSummetricWeights;
	int subIterationCount, maxSubIteration;
	FILE * testfile;
	time_t StartTime, EndTime;
	bool restartIterations_LvalChange=false;
	
	StartTime = time(NULL);
	
	
	// Initiate output file
	char checkOutFileName[1024], outFileName[1024];
	int randSeed = 0;
	
	for (a=1; a<=1000; a++)
	{
		sprintf(checkOutFileName, "out_files/%s_%d.out", argv[1], a);
		if (testfile = fopen(checkOutFileName, "r"))
		{
		    fclose(testfile);
		    continue;
		}
		else
		{
			strcpy (outFileName,checkOutFileName);
			randSeed = a;
			break;
		}
	}
	printf("Output file: %s\n", outFileName);
	OutputFile OutFile(outFileName);
	srand(randSeed);
	
	
	// Read Configuration file
	ConfigFileInfo* theConfigFileInfo = new ConfigFileInfo;
	ReadConfigurationFile(argv[1], theConfigFileInfo);
	ROBOT_COUNT = theConfigFileInfo->RobotCount;
	
	if(theConfigFileInfo->GlobalParams.IS_ITERATION_SYMMETRIC)
		maxSubIteration = ROBOT_COUNT;
	else
		maxSubIteration = 1;
	
	vector<int> OtherBots_identifiers(ROBOT_COUNT-1);
	vector<EnvNAV4DXYTG_pos_trajectory> OtherBots_posLatest_trajectories(ROBOT_COUNT-1);
	vector<EnvNAV4DXYTG_dist_trajectory> OtherBots_distConstraint_trajectories(ROBOT_COUNT-1);
	vector<float> OtherBots_penaltyWeights(ROBOT_COUNT-1);
	vector<EnvNAV4DXYTG_pos_trajectory*> solutionTraj(maxSubIteration);
	EnvNAV4DXYTG_pos_trajectory baseSolnTraj;
	
	vector<float> BackupPenaltyWeights;
	vector<int> ConstraintNegociationFlag;
	vector<float> upperBraket_PenaltySearch, lowerBraket_PenaltySearch, midPoint_PenaltySearch;
	vector<bool> foundUpperBraket, foundLowerBraket, foundWeight;
	
	vector<int> solution_stateIDs_V;
	vector<float> TheInitialPenaltyWeights;
	int iterCount=0, ActiveRobot, NextActiveRobot;
	
	
	// Centrally shared information structure - a pointer to it
	CentralizedInfo_t* CentralizedInfo = new CentralizedInfo_t(ROBOT_COUNT);
	
	// Debug
	//theConfigFileInfo->TheRobots[0]->StartX_c
	for (a=0; a<=(ROBOT_COUNT-1); a++)
	{
		printf("Robot %d: Start: (%d,%d)\n",a,theConfigFileInfo->TheRobots[a].StartX_c, theConfigFileInfo->TheRobots[a].StartY_c);
		if(!theConfigFileInfo->StaticObstacleMap.get(theConfigFileInfo->TheRobots[a].StartX_c, theConfigFileInfo->TheRobots[a].StartY_c))
			printf("Robot %d: Start OK\n",a);
		else
			printf("Robot %d: Start NOT OK\n",a);
			
		printf("Robot %d: End: (%d,%d)\n",a,theConfigFileInfo->TheRobots[a].EndX_c, theConfigFileInfo->TheRobots[a].EndY_c);
		if(!theConfigFileInfo->StaticObstacleMap.get(theConfigFileInfo->TheRobots[a].EndX_c, theConfigFileInfo->TheRobots[a].EndY_c))
			printf("Robot %d: End OK\n",a);
		else
			printf("Robot %d: End NOT OK\n",a);
	}
	
	//for (int m=0; m<theConfigFileInfo->TheConstraints[theConfigFileInfo->TheConstraints.size()-1].constraint.dist_t.size(); m++)
	//	printf("######Conatrsint pt: Timet=%d, D=%ld\n", theConfigFileInfo->TheConstraints[theConfigFileInfo->TheConstraints.size()-1].constraint.dist_t[m].Timet, theConfigFileInfo->TheConstraints[theConfigFileInfo->TheConstraints.size()-1].constraint.dist_t[m].D);
		
	
	// Declare variables required by the planner
	MDPConfig MDPCfg;
	ARAPlanner* planner;
	vector<EnvironmentNAV4DXYTG*> environment_robot(ROBOT_COUNT);
	vector<MDPConfig*> MDPConfig_robot(ROBOT_COUNT);
	vector<bool> robotDefined(ROBOT_COUNT, false);
	
	// Declare the trajectory database and the penalty weights
	vector<EnvNAV4DXYTG_pos_trajectory> posLatest_trajectories(ROBOT_COUNT * theConfigFileInfo->GlobalParams.EXPLORE_HOMOTOPY_CLASSES);
	//vector<float> penaltyWeights;	
	
	
	
	OutFile.WriteStaticObstacles(theConfigFileInfo->StaticObstacleMap);
	OutFile.WriteConstraints(theConfigFileInfo->TheConstraints);
	OutFile.WriteConfigFileParameters(theConfigFileInfo);
	//printf("Start penalty weight: %f\n", theConfigFileInfo->TheConstraints[0].Penalty.Weight);
	//exit(1);
	

	
	
	// *****************************************************************************************
	// Generate initial trajectories for the robots
	
	vector<EnvNAV4DXYTG_pos_trajectory> empty_pos_trajectory;
	vector<EnvNAV4DXYTG_dist_trajectory> empty_dist_trajectory;
	//vector<float> empty_penaltyWeights;
	vector<int> empty_identifiers;
	EnvNAV4DXYTG_pos_trajectory* initSolutionTraj = new EnvNAV4DXYTG_pos_trajectory;
	bool PENALTY_WEIGHT_INCREMENT_METHOD_2_isActive = true;
	
	int SuperIterCount = -1;
LABEL_ITER0:
	SuperIterCount++;
if (SuperIterCount < theConfigFileInfo->GlobalParams.MAX_SUPERITER_COUNT)
{
	printf("\n\n\n############################\nStarting super-iteration %d\n", SuperIterCount);
	
	if (SuperIterCount>0 && theConfigFileInfo->GlobalParams.PENALTY_WEIGHT_INCREMENT_METHOD == 2)
		PENALTY_WEIGHT_INCREMENT_METHOD_2_isActive = false;
	
	// Initiating the position trajectories of the robots
	for (a = 0; a < ROBOT_COUNT; a++)
	{
		int HomotopyExploreCount = 0;
		while (theConfigFileInfo->GlobalParams.EXPLORE_HOMOTOPY_CLASSES > 0) // TODO: Comment this
		{
	
		printf("\n### Determining the first unconstrained solution for Robot %d\n", a);
		
		// Initiate environment
		for (b = 0; b < ROBOT_COUNT; b++)
			if (robotDefined[b] && b==a)
			{
				delete MDPConfig_robot[b];
				delete environment_robot[b];
				robotDefined[b] = false;
			}
		MDPConfig_robot[a] = new MDPConfig;
		environment_robot[a] = new EnvironmentNAV4DXYTG;
		robotDefined[a] = true;
		environment_robot[a]->InitializeEnv(theConfigFileInfo, CentralizedInfo, a, empty_identifiers, empty_pos_trajectory, empty_dist_trajectory);
		
		// Do the initial planning and generate trajectory
		environment_robot[a]->InitializeMDPCfg(MDPConfig_robot[a]);
		bool bforwardsearch = true; //false;
		planner = new ARAPlanner(environment_robot[a], bforwardsearch);
		planner->set_start(MDPConfig_robot[a]->startstateid);
		planner->set_goal(MDPConfig_robot[a]->goalstateid);
		environment_robot[a]->ARAPlannerPointer = planner;
		//vector<int> solution_stateIDs_V;
		solution_stateIDs_V.clear();
		bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V, &SolnCost);

		delete initSolutionTraj;
		initSolutionTraj = new EnvNAV4DXYTG_pos_trajectory;
		environment_robot[a]->GetTrajectoryFromSolutionStateIDs(solution_stateIDs_V, initSolutionTraj);
		//posLatest_trajectories.push_back(*initSolutionTraj);
		posLatest_trajectories[a+HomotopyExploreCount] = *initSolutionTraj;
		
		//environment_robot[a]->PostProcessTrajectory(&posLatest_trajectories[a+HomotopyExploreCount]); //Remove this
		
		printf("Initial trajectory planned for robot %d - Cost = %d, LVal = (%f,%f)\n", a, SolnCost, posLatest_trajectories[a].Lval.real(), posLatest_trajectories[a].Lval.imag());
		//printf("Heuristic value to goal = %d\n", environment_robot[a]->GetGoalHeuristic(MDPConfig_robot[a]->startstateid));
		//char tmpc;
		//scanf("%c", &tmpc);
		//delete planner;
		HomotopyExploreCount++;
		theConfigFileInfo->GlobalParams.EXPLORE_HOMOTOPY_CLASSES--;
		//break;
		}
		
	}
	for (a = 0; a < ROBOT_COUNT; a++)
		CentralizedInfo->InitialConstrainHomotopyClass.HomotopyClassList[a].clear();
	printf("\n --------------------------------- \n");
	
	
	// Post-process trajectories
/*	vector<EnvNAV4DXYTG_pos_trajectory*> posTrajPointers (posLatest_trajectories.size());
	for (a=0; a<posLatest_trajectories.size(); a++)
		posTrajPointers[a] = &posLatest_trajectories[a];
	PostProcessTrajectory_Joint(environment_robot, posTrajPointers);
	*/
/*	for (a=0; a<posLatest_trajectories.size(); a++)
		environment_robot[0]->PostProcessTrajectory(&posLatest_trajectories[a]); */
	
	// Writing the initial trajectories to output file
	TheInitialPenaltyWeights.clear();
	TheInitialPenaltyWeights.resize(theConfigFileInfo->TheConstraints.size());
	for (a=0; a<theConfigFileInfo->TheConstraints.size(); a++)
		TheInitialPenaltyWeights[a] = 0.0;
	OutFile.WriteTrajectories(posLatest_trajectories, 0, -1, TheInitialPenaltyWeights);
	
	// *****************************************************************************************
	char tmpc;
	OutFile.Close();
	scanf("%c", &tmpc);
	
	

	iterCount=0;

	//if (!restartIterations_LvalChange)
	CentralizedInfo_t_InitiatePenaltyWeights(CentralizedInfo, *theConfigFileInfo);
	
	/*
	vector<int> OtherBots_identifiers(ROBOT_COUNT-1);
	vector<EnvNAV4DXYTG_pos_trajectory> OtherBots_posLatest_trajectories(ROBOT_COUNT-1);
	vector<EnvNAV4DXYTG_dist_trajectory> OtherBots_distConstraint_trajectories(ROBOT_COUNT-1);
	vector<float> OtherBots_penaltyWeights(ROBOT_COUNT-1);
	vector<EnvNAV4DXYTG_pos_trajectory*> solutionTraj(maxSubIteration);
	EnvNAV4DXYTG_pos_trajectory baseSolnTraj;
	
	
	vector<float> BackupPenaltyWeights;
	vector<int> ConstraintNegociationFlag;
	vector<float> upperBraket_PenaltySearch, lowerBraket_PenaltySearch, midPoint_PenaltySearch;
	vector<bool> foundUpperBraket, foundLowerBraket, foundWeight;
	*/
	
	
	bool hasConverged = false;
	int cycleState = 0;
	
	// The main iteration loop
	bool DoIteration = true;
	while(DoIteration)
	{

		printf("### Starting Iteration: %d-%d\n", SuperIterCount, iterCount);

		for (a = 0; a < maxSubIteration; a++)
		{
			delete solutionTraj[a];
			solutionTraj[a] = new EnvNAV4DXYTG_pos_trajectory;
		}
		
		
		for (subIterationCount = 0; subIterationCount < maxSubIteration; subIterationCount++)
		{
			cycleState++;
			
			// Determine which robot will modify its trajectory in this iteration
			if(theConfigFileInfo->GlobalParams.IS_ITERATION_SYMMETRIC)
				ActiveRobot = (subIterationCount % ROBOT_COUNT);
			else
				ActiveRobot = ((iterCount+SuperIterCount) % ROBOT_COUNT);
				
		
			// ------------------------------------------------------------------------------------
			
			// Gather the trajectory information and distance constraints with other robots
			/*
			for (a = 0; a < ROBOT_COUNT; a++)
			{
				if (a==ActiveRobot)
					continue;
				
				ThisOtherRobotIndex = ((a<ActiveRobot)? a : (a-1));
				
				OtherBots_identifiers[ThisOtherRobotIndex] = a;
				OtherBots_posLatest_trajectories[ThisOtherRobotIndex] = posLatest_trajectories[a];
				OtherBots_distConstraint_trajectories[ThisOtherRobotIndex].dist_t.clear();
				for (b=0; b<theConfigFileInfo->TheConstraints.size(); b++)
				{
					if ((theConfigFileInfo->TheConstraints[b].Robot1 == ActiveRobot && theConfigFileInfo->TheConstraints[b].Robot2 == a)
							|| (theConfigFileInfo->TheConstraints[b].Robot2 == ActiveRobot && theConfigFileInfo->TheConstraints[b].Robot1 == a))
						OtherBots_distConstraint_trajectories[ThisOtherRobotIndex] = (theConfigFileInfo->TheConstraints[b].constraint);
				}
			}
			*/
			OtherBots_identifiers.resize(theConfigFileInfo->TheRobots[ActiveRobot].constraints.size());
			OtherBots_posLatest_trajectories.resize(theConfigFileInfo->TheRobots[ActiveRobot].constraints.size());
			OtherBots_distConstraint_trajectories.resize(theConfigFileInfo->TheRobots[ActiveRobot].constraints.size());
			OtherBots_penaltyWeights.resize(theConfigFileInfo->TheRobots[ActiveRobot].constraints.size());
			for (a=0; a<theConfigFileInfo->TheRobots[ActiveRobot].constraints.size(); a++)
			{
				OtherBots_identifiers[a] = theConfigFileInfo->TheRobots[ActiveRobot].constraints[a].RobotIndex;
				OtherBots_posLatest_trajectories[a] = posLatest_trajectories[OtherBots_identifiers[a]];
				OtherBots_distConstraint_trajectories[a] = theConfigFileInfo->TheRobots[ActiveRobot].constraints[a].DistTraj;
			}
			
			
			// ------------------------------------------------------------------------------------
			int PenaltySearchStepCount = -1;
LABEL_PLANNING:
			PenaltySearchStepCount++;
			
			
			// Initiate the planning environment for the Active Robot
			for (b = 0; b < ROBOT_COUNT; b++)
				if (robotDefined[b] && b==ActiveRobot)
				{
					delete MDPConfig_robot[b];
					delete environment_robot[b];
					robotDefined[b] = false;
				}
			//delete MDPConfig_robot[ActiveRobot];
			//delete environment_robot[ActiveRobot];
			MDPConfig_robot[ActiveRobot] = new MDPConfig;
			environment_robot[ActiveRobot] = new EnvironmentNAV4DXYTG;
			robotDefined[ActiveRobot] = true;
			environment_robot[ActiveRobot]->InitializeEnv(theConfigFileInfo, CentralizedInfo, ActiveRobot, OtherBots_identifiers,
																OtherBots_posLatest_trajectories, OtherBots_distConstraint_trajectories);
			environment_robot[ActiveRobot]->SetPenaltyWeights(OtherBots_penaltyWeights); // This is just a dummy
			environment_robot[ActiveRobot]->InitializeMDPCfg(MDPConfig_robot[ActiveRobot]);
			
			// Compute and update the penalty weights that will be used for planning
			// BackupPenaltyWeights = CentralizedInfo->PenaltyWeightVectors[ActiveRobot];

			// Set the penalty weights
			/*
			for (a = 0; a < ROBOT_COUNT; a++)
			{
				if (a==ActiveRobot)
					continue;
				ThisOtherRobotIndex = ((a<ActiveRobot)? a : (a-1));
				OtherBots_penaltyWeights[ThisOtherRobotIndex] = 0;
				for (b=0; b<theConfigFileInfo->TheConstraints.size(); b++)
				{
					if ((theConfigFileInfo->TheConstraints[b].Robot1 == ActiveRobot && theConfigFileInfo->TheConstraints[b].Robot2 == a)
							|| (theConfigFileInfo->TheConstraints[b].Robot2 == ActiveRobot && theConfigFileInfo->TheConstraints[b].Robot1 == a))
						OtherBots_penaltyWeights[ThisOtherRobotIndex] = CentralizedInfo->PenaltyWeights[b];
				}
			}
			*/
			
			//environment_robot[ActiveRobot]->SetPenaltyWeights(OtherBots_penaltyWeights);
			if (PenaltySearchStepCount==0)
			{
				if (theConfigFileInfo->GlobalParams.PENALTY_WEIGHT_INCREMENT_METHOD == 2 && PENALTY_WEIGHT_INCREMENT_METHOD_2_isActive && 
						((theConfigFileInfo->GlobalParams.IS_ITERATION_SYMMETRIC && iterCount==0) || 
						(!theConfigFileInfo->GlobalParams.IS_ITERATION_SYMMETRIC && iterCount<ROBOT_COUNT)))
					isSummetricWeights = false;
				else
					isSummetricWeights = true;
				
				if (theConfigFileInfo->GlobalParams.PENALTY_WEIGHT_INCREMENT_METHOD==2 || subIterationCount==0)
					CentralizedInfo_t_UpdatePenaltyWeights(CentralizedInfo, *theConfigFileInfo, ActiveRobot, &posLatest_trajectories[ActiveRobot], environment_robot[ActiveRobot], isSummetricWeights);

				environment_robot[ActiveRobot]->SetPenaltyWeights(CentralizedInfo->PenaltyWeightVectors[ActiveRobot]);
			}
			else
			{
				environment_robot[ActiveRobot]->SetPenaltyWeights(midPoint_PenaltySearch);
			}

			// ------------------------------------------------------------------------------------
			// Do the planning
			
			bool bforwardsearch = true; //false;
			//~planner();
			//delete planner;
			planner = new ARAPlanner(environment_robot[ActiveRobot], bforwardsearch);
			planner->set_start(MDPConfig_robot[ActiveRobot]->startstateid);
			planner->set_goal(MDPConfig_robot[ActiveRobot]->goalstateid);
			vector<int> solution_stateIDs_V;
			bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V, &SolnCost);
	
			solutionTraj[subIterationCount]->pos_t.clear();
			environment_robot[ActiveRobot]->GetTrajectoryFromSolutionStateIDs(solution_stateIDs_V, solutionTraj[subIterationCount]);
			delete planner;
			
			// ----------------------------------------------------------------
			// Search for penalty increment
			if (theConfigFileInfo->GlobalParams.PENALTY_WEIGHT_INCREMENT_METHOD == 2 && PENALTY_WEIGHT_INCREMENT_METHOD_2_isActive)
			{
				if ((theConfigFileInfo->GlobalParams.IS_ITERATION_SYMMETRIC && iterCount==0) || 
					(!theConfigFileInfo->GlobalParams.IS_ITERATION_SYMMETRIC && iterCount<ROBOT_COUNT))
				{
					// Initiate the brackets
					if (PenaltySearchStepCount==0)
					{
						lowerBraket_PenaltySearch.clear();
						upperBraket_PenaltySearch.clear();
						midPoint_PenaltySearch.clear();
						foundUpperBraket.clear();
						foundWeight.clear();
						for (a=0; a<CentralizedInfo->PenaltyWeightVectors[ActiveRobot].size(); a++)
						{
							lowerBraket_PenaltySearch.push_back( CentralizedInfo->PenaltyWeightVectors[ActiveRobot][a] ); //theConfigFileInfo->TheRobots[ActiveRobot].constraints[a].PenaltyParams[0];
							upperBraket_PenaltySearch.push_back( CentralizedInfo->PenaltyWeightVectors[ActiveRobot][a] + theConfigFileInfo->TheRobots[ActiveRobot].constraints[a].PenaltyParams[2] );
							midPoint_PenaltySearch.push_back( lowerBraket_PenaltySearch[a] );
							foundLowerBraket.push_back( true ); //][false //[]true
							foundUpperBraket.push_back( false );
							foundWeight.push_back( false );
						}
						//BackupPenaltyWeights = CentralizedInfo->PenaltyWeightVectors[ActiveRobot];
						baseSolnTraj = *solutionTraj[subIterationCount]; //][posLatest_trajectories[ActiveRobot]; //[]*solutionTraj[subIterationCount];
						//environment_robot[ActiveRobot]->SetPenaltyWeights(midPoint_PenaltySearch);
						goto LABEL_PLANNING; //[]goto LABEL_PLANNING; //][
					}
					
					// Check the trajectories and re-adjust bracket
					EvaluateTrajectoryChange(solutionTraj[subIterationCount], &baseSolnTraj, environment_robot[ActiveRobot], 
												&ConstraintNegociationFlag, theConfigFileInfo->GlobalParams.PENALTY_BIN_SEARCH_THRESH);
					bool allFound = true;
					for (a=0; a<CentralizedInfo->PenaltyWeightVectors[ActiveRobot].size(); a++)
					{
						if(foundWeight[a])
							continue;
						
						if (ConstraintNegociationFlag[a]==0)
						{
							foundWeight[a] = true;
							continue;
						}
						
						allFound = false;
						
						if (ConstraintNegociationFlag[a]==-1)
						{
							lowerBraket_PenaltySearch[a] = midPoint_PenaltySearch[a];
							foundLowerBraket[a] = true;
							if (!foundUpperBraket[a])
							{
								upperBraket_PenaltySearch[a] = lowerBraket_PenaltySearch[a] + theConfigFileInfo->TheRobots[ActiveRobot].constraints[a].PenaltyParams[2];
								midPoint_PenaltySearch[a] = upperBraket_PenaltySearch[a];
							}
							else
								midPoint_PenaltySearch[a] = (lowerBraket_PenaltySearch[a]+upperBraket_PenaltySearch[a])/2.0;
						}
						
						if (ConstraintNegociationFlag[a]==1)
						{
							upperBraket_PenaltySearch[a] = midPoint_PenaltySearch[a];
							foundUpperBraket[a] = true;
							if (!foundLowerBraket[a])
							{
								lowerBraket_PenaltySearch[a] = upperBraket_PenaltySearch[a] - theConfigFileInfo->TheRobots[ActiveRobot].constraints[a].PenaltyParams[2];
								midPoint_PenaltySearch[a] = lowerBraket_PenaltySearch[a];
							}
							else
								midPoint_PenaltySearch[a] = (lowerBraket_PenaltySearch[a]+upperBraket_PenaltySearch[a])/2.0;
						}
							
					}
					
					if (allFound)
					{
						printf("!!!!!!! ALL FOUND for robot %d !!!!!!!\n", ActiveRobot);
						CentralizedInfo->PenaltyWeightVectors[ActiveRobot] = midPoint_PenaltySearch;
						posLatest_trajectories[ActiveRobot] = *solutionTraj[subIterationCount];
						for (a=0; a<CentralizedInfo->PenaltyWeightVectors[ActiveRobot].size(); a++)
							theConfigFileInfo->TheRobots[ActiveRobot].constraints[a].PenaltyParams[2] = (upperBraket_PenaltySearch[a]-lowerBraket_PenaltySearch[a])/1.0; //6.0
					}
					else if (PenaltySearchStepCount>theConfigFileInfo->GlobalParams.PENALTY_BIN_SEARCH_MAXSTEPS)
					{
						printf("!!!!!!! Binary Search TERMINATED for robot %d !!!!!!!\n", ActiveRobot);
						for (a=0; a<CentralizedInfo->PenaltyWeightVectors[ActiveRobot].size(); a++)
							theConfigFileInfo->TheRobots[ActiveRobot].constraints[a].PenaltyParams[2] = 
										__min(theConfigFileInfo->TheRobots[ActiveRobot].constraints[a].PenaltyParams[2], upperBraket_PenaltySearch[a]-lowerBraket_PenaltySearch[a])/1.0; //10.0

					}
					else
					{
						printf("---> Step %d in binary search - trying penalty weights: ", PenaltySearchStepCount);
						for (a=0; a<CentralizedInfo->PenaltyWeightVectors[ActiveRobot].size(); a++)
							printf(" %f ", midPoint_PenaltySearch[a]);
						printf("\n");
						//environment_robot[ActiveRobot]->SetPenaltyWeights(midPoint_PenaltySearch);
						goto LABEL_PLANNING;
					}
					
				}
			}
			
			if (theConfigFileInfo->GlobalParams.ITERATION_TYPE==2)
			{
				hasConverged = hasConverged & AreTrajectoriesSame(solutionTraj[subIterationCount], &posLatest_trajectories[ActiveRobot]);
			}
		}
		
		// -------------------------------------------------------------
		// Uplade the trajectory data with the latest solution
				
		if(theConfigFileInfo->GlobalParams.IS_ITERATION_SYMMETRIC)
		{
			vector<float> ThePenaltyWeightsThisIteration(CentralizedInfo->PenaltyWeightVectors[ActiveRobot].size());
			for (a = 0; a < maxSubIteration; a++)
				posLatest_trajectories[a] = *(solutionTraj[a]);
			OutFile.WriteTrajectories(posLatest_trajectories, iterCount+1, -1, ThePenaltyWeightsThisIteration);
		}
		else
		{
			vector<float> ThePenaltyWeightsThisIteration(CentralizedInfo->PenaltyWeightVectors[ActiveRobot].size());
			vector< vector<int> > PenaltyWeightRobotsThisIteration(CentralizedInfo->PenaltyWeightVectors[ActiveRobot].size());
			for (a=0; a<CentralizedInfo->PenaltyWeightVectors[ActiveRobot].size(); a++)
			{
				ThePenaltyWeightsThisIteration[a] = CentralizedInfo->PenaltyWeightVectors[ActiveRobot][a]; //CentralizedInfo->PenaltyWeights[a];
				PenaltyWeightRobotsThisIteration[a].resize(2);
				PenaltyWeightRobotsThisIteration[a][0] = ActiveRobot;
				PenaltyWeightRobotsThisIteration[a][1] = theConfigFileInfo->TheRobots[ActiveRobot].constraints[a].RobotIndex;
			}
			
			// LVal blocking
			restartIterations_LvalChange = false;
			if (abs(posLatest_trajectories[ActiveRobot].Lval - solutionTraj[0]->Lval) > LVAL_EQUAL_THRESH)
			{
				restartIterations_LvalChange = true;
				//CentralizedInfo_t_InitiatePenaltyWeights(CentralizedInfo, *theConfigFileInfo);
				CentralizedInfo->BlockedHomotopyClasses.HomotopyClassList[ActiveRobot].push_back(posLatest_trajectories[ActiveRobot].Lval);
				//CentralizedInfo->InitialConstrainHomotopyClass.HomotopyClassList[ActiveRobot].push_back(solutionTraj[0]->Lval);
			}
			
			posLatest_trajectories[ActiveRobot] = *(solutionTraj[0]);
			
			// Post-process trajectories
			vector<EnvNAV4DXYTG_pos_trajectory*> posTrajPointers (ROBOT_COUNT);
			for (a=0; a<ROBOT_COUNT; a++)
				posTrajPointers[a] = &posLatest_trajectories[a];
			PostProcessTrajectory_Joint(environment_robot, posTrajPointers);
			
			OutFile.WriteTrajectories(posLatest_trajectories, iterCount+1, ActiveRobot, ThePenaltyWeightsThisIteration, PenaltyWeightRobotsThisIteration);
		}
				
		
		// Update the CentralizedInfo
		if (theConfigFileInfo->GlobalParams.DO_JOINTSTATESPACE_LOGGING)
		{
			ViolationCount = CentralizedInfo->AddObstaclePointsToJointStatespace(posLatest_trajectories, 
												theConfigFileInfo->TheConstraints, 0, theConfigFileInfo->EnvMaxTime_c-1, 
													(theConfigFileInfo->GlobalParams.JOINTSTATESPACE_LOGGING_METHOD==2));
			//CentralizedInfo->ConcatenateJointStatespaceLists();
		}

	
		// -------------------------------------------------------------
			
		// Increase the penalty weights
		//for (b=0; b<theConfigFileInfo->TheConstraints.size(); b++)
		//	theConfigFileInfo->TheConstraints[b].Penalty.next();
		
		for (int al=0; al<CentralizedInfo->BlockedHomotopyClasses.HomotopyClassList.size(); al++)
		{
			printf(" - R%d", al);
			for (int bl=0; bl<CentralizedInfo->BlockedHomotopyClasses.HomotopyClassList[al].size(); bl++)
				printf(" (%f,%f)", CentralizedInfo->BlockedHomotopyClasses.HomotopyClassList[al][bl].real(), CentralizedInfo->BlockedHomotopyClasses.HomotopyClassList[al][bl].imag());
		}
		
		printf("!! Iteration: %d-%d\n!! Robot: %d\n!! PenaltyWeights:", SuperIterCount, iterCount, ActiveRobot);
		for (a=0; a<CentralizedInfo->PenaltyWeightVectors[ActiveRobot].size(); a++)
			printf(" %f", CentralizedInfo->PenaltyWeightVectors[ActiveRobot][a]);
		printf("\n!! Cost of trajectory: %d\n", SolnCost);
		printf("!! Lval of trajectory: (%f,%f)\n", posLatest_trajectories[ActiveRobot].Lval.real(), posLatest_trajectories[ActiveRobot].Lval.imag());
		if (theConfigFileInfo->GlobalParams.DO_JOINTSTATESPACE_LOGGING)
			printf("!! ViolationCount: %d\n\n", ViolationCount);
		
		if (restartIterations_LvalChange)
		{
			printf("## Restarting with initial penalty weights and another blocked homotopy class for robot %d \n", ActiveRobot);
			goto LABEL_ITER0;
		}
		
		
		// Just for stats purpose:
		if (ViolationCount==0)
			break;
		//char tmpc;
		//scanf("%c", &tmpc);
		
		
		// =================================
		// Sets up the next iteration
		iterCount++;

		if (theConfigFileInfo->GlobalParams.DO_JOINTSTATESPACE_LOGGING)
		{
			
			if (theConfigFileInfo->GlobalParams.ITERATION_TYPE==1 && iterCount>theConfigFileInfo->GlobalParams.MAX_ITERATION_COUNT 
					&& iterCount>theConfigFileInfo->GlobalParams.MIN_ITERATION_COUNT)
			{
				if (ViolationCount==0)
					DoIteration = false;
				else
				{
					iterCount = 0;
					CentralizedInfo->ConcatenateJointStatespaceLists();
					goto LABEL_ITER0;
				}
			}
			
			if (theConfigFileInfo->GlobalParams.ITERATION_TYPE==2 && iterCount>theConfigFileInfo->GlobalParams.MIN_ITERATION_COUNT && 
					((cycleState>=theConfigFileInfo->GlobalParams.CONVERGENCE_CYCLE_COUNT*ROBOT_COUNT && hasConverged) || iterCount>theConfigFileInfo->GlobalParams.MAX_ITERATION_COUNT))
			{
				if (ViolationCount==0)
					DoIteration = false;
				else
				{
					iterCount = 0;
					CentralizedInfo->ConcatenateJointStatespaceLists();
					for (a=0; a<CentralizedInfo->JointStatespaceObstacles.size(); a++)
					{
						printf("\n $$ 2N-D pt no %d :", a);
						for (b=0; b<CentralizedInfo->JointStatespaceObstacles[a].RobotStateX.size(); b++)
						{
							printf(" %d %d,", CentralizedInfo->JointStatespaceObstacles[a].RobotStateX[b], CentralizedInfo->JointStatespaceObstacles[a].RobotStateY[b]);
						}
						if (CentralizedInfo->isPointInJointStatespaceObstacles(CentralizedInfo->JointStatespaceObstacles[a], 1, theConfigFileInfo->GlobalParams.JOINTSTATESPACE_LOGGING_RAD))
							printf(" -- Tested OK!!");
					}
					printf("\n");
					goto LABEL_ITER0;
				}
			}
			else if (theConfigFileInfo->GlobalParams.ITERATION_TYPE==2 && (cycleState>=theConfigFileInfo->GlobalParams.CONVERGENCE_CYCLE_COUNT*ROBOT_COUNT && !hasConverged))
			{
				cycleState = 0;
				hasConverged = true;
			}
			
		}
		else
		{
			
			if (theConfigFileInfo->GlobalParams.ITERATION_TYPE==1 && iterCount>theConfigFileInfo->GlobalParams.MAX_ITERATION_COUNT 
					&& iterCount>theConfigFileInfo->GlobalParams.MIN_ITERATION_COUNT)
				DoIteration = false;
			
			if (theConfigFileInfo->GlobalParams.ITERATION_TYPE==2 && iterCount>theConfigFileInfo->GlobalParams.MIN_ITERATION_COUNT && 
					((cycleState>=theConfigFileInfo->GlobalParams.CONVERGENCE_CYCLE_COUNT*ROBOT_COUNT && hasConverged) || iterCount>theConfigFileInfo->GlobalParams.MAX_ITERATION_COUNT))
				DoIteration = false;
			else if (theConfigFileInfo->GlobalParams.ITERATION_TYPE==2 && (cycleState>=theConfigFileInfo->GlobalParams.CONVERGENCE_CYCLE_COUNT*ROBOT_COUNT && !hasConverged))
			{
				cycleState = 0;
				hasConverged = true;
			}
			
		}
		
		/*
		// =================================
		if(theConfigFileInfo->GlobalParams.ITERATION_TYPE == 1)
		{
	    	
	    	//if (ViolationCount==0 && iterCount>theConfigFileInfo->GlobalParams.ITERATION_COUNT)
	    	//	break;
	    	//else
	    	//	continue;
	    	//if (iterCount>theConfigFileInfo->GlobalParams.ITERATION_COUNT)
	    	//	break;

	    			if (cycleState==ROBOT_COUNT)
		{
			if (hasConverged)
		}
			
			if (iterCount>theConfigFileInfo->GlobalParams.ITERATION_COUNT && theConfigFileInfo->GlobalParams.DO_JOINTSTATESPACE_LOGGING)
			{
				if (ViolationCount==0)
					DoIteration = false;
				else
				{
					//for (b=0; b<theConfigFileInfo->TheConstraints.size(); b++)
						//theConfigFileInfo->TheConstraints[b].Penalty.reset();
					iterCount = 0;
					CentralizedInfo->ConcatenateJointStatespaceLists();
					/.*for (a=0; a<CentralizedInfo->JointStatespaceObstacles.size(); a++)
					{
						printf("\n $$ 2N-D pt no %d :", a);
						for (b=0; b<CentralizedInfo->JointStatespaceObstacles[a].RobotStateX.size(); b++)
						{
							printf(" %d %d,", CentralizedInfo->JointStatespaceObstacles[a].RobotStateX[b], CentralizedInfo->JointStatespaceObstacles[a].RobotStateY[b]);
						}
						if (CentralizedInfo->isPointInJointStatespaceObstacles(CentralizedInfo->JointStatespaceObstacles[a], 1, theConfigFileInfo->GlobalParams.JOINTSTATESPACE_LOGGING_RAD))
							printf(" -- Tested OK!!");
					}
					printf("\n");*./
					goto LABEL_ITER0;
				}
			}
			else if (iterCount>theConfigFileInfo->GlobalParams.ITERATION_COUNT)
				DoIteration = false;
		}*/
	
	}


}
	
	EndTime = time(NULL);
    
    FILE* fStats;
    char timestamp_str[1024];
	fStats = fopen("RunStats.txt", "a+");
	time_t rawtime;
	time ( &rawtime );
	strncpy(timestamp_str, ctime(&rawtime), strlen(ctime(&rawtime))-1);
	// <out_file_name> -- timestamp -- [<super_iter>,<iter>](<violation_count>) -- <run_time>
	fprintf(fStats, "%s -- %s -- [%d,%d](%d) -- %f\n", outFileName, timestamp_str, SuperIterCount, iterCount, ViolationCount, difftime(EndTime, StartTime));
	fclose(fStats);

		
	OutFile.Close();
	
	printf("\n\nSUMMARY:\n");
	printf("Output file: %s\n", outFileName);
	printf("Time stamp: %s\n", timestamp_str);
	printf("Iteration: %d,%d \nViolation count: %d \n", SuperIterCount, iterCount, ViolationCount);
	printf("Run time (s): %f\n", difftime(EndTime, StartTime));
	printf("\n");
	
	/*
	for (int a=environment_robot1.EnvNAV4DXYTGCfg.StartTimet_c; a <= environment_robot1.EnvNAV4DXYTGCfg.EndTimet_c; a++)
	{
		#if DEBUG2
			//printf("In main.cpp-L75 : a = %d\n", a);
			//break;
		#endif
		Timet = a; //(int) (a / environment_robot1.EnvNAV4DXYTGCfg.timestepsize_m);
		EnvNAV4DXYTG_pos_t* Tmppos_t = new EnvNAV4DXYTG_pos_t;
		EnvNAV4DXYTG_dist_t* Tmpdist_t = new EnvNAV4DXYTG_dist_t;
		Tmppos_t->X = 3;
		Tmppos_t->Y = 4 + a;
		Tmppos_t->Timet = Timet;
		Tmpdist_t->D = 8*NAV4DXYTG_COSTMULT;
		Tmpdist_t->Timet = Timet;
		otherBots_trajectories[0].pos_t.push_back(*Tmppos_t);
		distConstraint_trajectories[0].dist_t.push_back(*Tmpdist_t);
	}
	penaltyWeights[0] = 1;
	#if DEBUG2
		printf("In main.cpp-L76 : Trajectories and constraints w.r.t other bots defined!\n");
	#endif
	*/
	

	//Initialize constraint  Info
	/*
	if(!environment_robot1.InitializeEnv(otherBots_trajectories, distConstraint_trajectories, penaltyWeights))
	{
		printf("ERROR: failed to initialize constraints\n");
		exit(1);
	}
	#if DEBUG2
		printf("In main.cpp-L87 : Initialization done with trajectories and constraints w.r.t other bots!\n");
	#endif
	*/
	
	/*
	//Initialize MDP Info
	if(!environment_robot1.InitializeMDPCfg(&MDPCfg))
	{
		printf("ERROR: InitializeMDPCfg failed\n");
		exit(1);
	}
	#if DEBUG2
		printf("In main.cpp-L98 : MDPCfg Initialized!\n");
	#endif


	//plan a path
	vector<int> solution_stateIDs_V;
	bool bforwardsearch = true; //false;
	ARAPlanner planner(&environment_robot1, bforwardsearch);
	#if DEBUG2
		printf("In main.cpp-L107 : ARAPlanner Initialized!\n");
	#endif

    if(planner.set_start(MDPCfg.startstateid) == 0)
        {
            printf("ERROR: failed to set start state\n");
            exit(1);
        }

    if(planner.set_goal(MDPCfg.goalstateid) == 0)
        {
            printf("ERROR: failed to set goal state\n");
            exit(1);
        }

    printf("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    FILE* fSol = fopen("sol.txt", "w");
	for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
	  environment_robot1.PrintState(solution_stateIDs_V[i], true, fSol);
	}
    fclose(fSol);


 	//print a path
	if(bRet)
	{
		//print the solution
		printf("Solution is found\n");
	}
	else
		printf("Solution does not exist\n");

	fflush(NULL);


    return bRet;
    */
        
    return 1;
}



int main(int argc, char *argv[])
{

	#if DEBUG2
		printf("Entered main.cpp\n");
	#endif
	
	if(argc != 2)
	{
		PrintUsage(argv);
		exit(1);
	}

    //3D planning
    planandnavigate3Dxyt(argc, argv);


	
	return 0;
}

// =================================================

