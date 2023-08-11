%% preamble
close all;
clear all;
clc;
% load map file
map3;
% Initialize variables
tic
in_use1 = monitor_memory_whos; %memory check
StateSpace = stateSpaceSE2;
Validator = validatorOccupancyMap(StateSpace);
Validator.Map = mymap;
Validator.ValidationDistance = inf;
StateSpace.StateBounds = [mymap.XWorldLimits; mymap.YWorldLimits; [-pi pi]];

AstarPlanner = plannerHybridAStar(Validator);

startPoint = [2 2 pi/2];
goalPoint = [95 95 -pi/2];


path = plan(AstarPlanner,startPoint,goalPoint,SearchMode='greedy');  


show(mymap);hold on;title('');grid on
plot(path.States(:,1),path.States(:,2))
xlabel('Y(meters)'); ylabel('X(meters)');
hold on
plot(2,2,'g*');
plot(95,95,'b*');

time_taken = toc;
in_use2 = monitor_memory_whos;

best_path = [path.States(:,1),path.States(:,2)];
diff = diff(best_path); 
path_length = sum(sqrt(sum(diff.*diff,2)));
smoothness = calculate_smoothness(best_path); 

memory_used_in_Megabytes=(in_use2-in_use1);
disp(path_length)
disp(smoothness)
disp('time')
disp(time_taken)
disp(memory_used_in_Megabytes)