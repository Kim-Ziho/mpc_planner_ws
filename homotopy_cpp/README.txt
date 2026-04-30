******************************************************************************************
*                                                                                        *
*    A* search with homotopy class constraints                                           *
*    Copyright (C) 2010  Subhrajit Bhattacharya                                          *
*                                                                                        *
*    This program is free software: you can redistribute it and/or modify                *
*    it under the terms of the GNU General Public License as published by                *
*    the Free Software Foundation, either version 3 of the License, or                   *
*    (at your option) any later version.                                                 *
*                                                                                        *
*    This program is distributed in the hope that it will be useful,                     *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of                      *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                       *
*    GNU General Public License for more details <http://www.gnu.org/licenses/>.         *
*                                                                                        *
*                                                                                        *
*    Contact: subhrajit@gmail.com, http://fling.seas.upenn.edu/~subhrabh/                *
*                                                                                        *
*                                                                                        *
******************************************************************************************

The entry point to the code is in the main.cpp under 'test_xytg' folder.
The program requires a configuration file to run.
Output files are stored in folder 'out_files'. The 'PlotData.m' can be used to view the output files in MATLAB.

This is a modified versin of an original code meant for distributed optimization. This version of the code is just to demonstrate the use of homotopy class constraints. The original code is used just as a base on top of which development was easy. So the overall code may have lots of redundent componets and hence appear messy.


Compilation:
make clean
make

Execution (supplied cfg files):
./main MovingObstacle.cfg [This one has 2 dynamic obstacles]
./main NonEuclideanCost.cfg [Non-euclidean distance as cost]
./main HomotopyExplore_2.cfg [This one finds the different homotopy classes]

View corresponding output files in MATLAB:
PlotData('out_files/MovingObstacle.cfg_41.out')
PlotData('out_files/NonEuclideanCost.cfg_21.out')
PlotData('out_files/HomotopyExplore_2.cfg_25.out')

