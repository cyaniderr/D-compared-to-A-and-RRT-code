%% preamble
close all;
clear all;
clc;
map1;

% initialize variables and validator
Validator = validatorOccupancyMap(stateSpaceSE2);
Validator.Map = mymap;
Validator.ValidationDistance = inf;
startPoint = [1 1 pi/2];
goalPoint = [95 95 -pi/2];
Bench = plannerBenchmark(Validator,startPoint,goalPoint);

% initialize path planning algorithms functions
HYBAstarPathPlanner = @(Validator)plannerHybridAStar(Validator);

% initialize plan function
planFncn = @(initOut,s,g)plan(initOut,s,g);

% add the algorithms to the benchmark
addPlanner(Bench,planFncn,HYBAstarPathPlanner,PlannerName="HYBAStarPathPlanner");


rng("shuffle")

% run benchmark
Count = 10;
runPlanner(Bench,Count)

% visualize data
[pathLengthSummary,pathLengthData] = metric(Bench,"pathLength");
[clearanceSummary,clearanceData] = metric(Bench,"clearance");
[ExecutionTimeSummary,ExecutionTimeData] = metric(Bench,"executionTime");
[initializationTimeSummary,initializationTimeData] = metric(Bench,"initializationTime");
[isPathValidSummary,isPathValidData] = metric(Bench,"isPathValid");
[smoothnessSummary,smoothnessData] = metric(Bench,"smoothness");
