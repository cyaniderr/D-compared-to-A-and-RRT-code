% preamble 
clearvars
close all
clc;
map3; %load map file


%variable initialization
xstart=2; ystart=2;          
xGoal=95; yGoal=95;      
Tolerance=5;                 
maxConnectionDistance= 5;
map = Mapmatrix;
Max_iterations = 3000;
i=1;
goalpoint = [xGoal,yGoal];


%Tree struct initialization
Tree.b(1).x = xstart;        
Tree.b(1).y = ystart; 
Tree.b(1).PrevX= xstart;     
Tree.b(1).PrevY = ystart;
Tree.b(1).distance=0;          
Tree.b(1).indexPrev = 0; 


%plot start
xmax=size(map,1);
ymax=size(map,2);
figure;
show(mymap);hold on;title('');grid on
xlabel('Y(meters)'); ylabel('X(meters)');
hold on
plot(xstart, ystart,'go', 'MarkerSize',5, 'MarkerFaceColor','g');
plot(xGoal, yGoal,'bo' ,'MarkerSize',5, 'MarkerFaceColor','b');


tic
in_use1 = monitor_memory_whos; %memory check
%% Algorithm main
for ii = 1:Max_iterations
    xrand=[];
    xrand(1) = xmax*rand; 
    xrand(2) = ymax*rand;
    xnear=[];
    min_distance = 1000000;
    near_i = 1;
    %finding x_near
    [~,N]=size(Tree.b);
    for j = 1:N
       xnear(1) = Tree.b(j).x;
       xnear(2) = Tree.b(j).y;
       distance = norm(xrand - xnear);
       if min_distance > distance
           min_distance = distance;
           near_i = j;
       end
    end
    xnear(1) = Tree.b(near_i).x;
    xnear(2) = Tree.b(near_i).y;
    %finding x_new
    xnew=[];
    near_to_rand = [xrand(1)-xnear(1),xrand(2)-xnear(2)];
    normlized = near_to_rand / norm(near_to_rand) * maxConnectionDistance;
    xnew = xnear + normlized;
    %obstacle detection
    if ~obstacleDetect(xnear,xnew,map) 
       continue;
    end
    i=i+1;
    %Adding x_new to the tree
    Tree.b(i).x = xnew(1);
    Tree.b(i).y = xnew(2); 
    Tree.b(i).PrevX = xnear(1);     
	Tree.b(i).PrevY = xnear(2);
    Tree.b(i).distance= norm(xnew - xnear);          
    Tree.b(i).indexPrev = near_i;   
    plot([xnear(1),xnew(1)],[xnear(2),xnew(2)],'-r');
    hold on;
    plot(xnew(1),xnew(2),'.r');
    hold on;

    %pathfound?
    if norm(xnew - goalpoint) < Tolerance
        break;
    end
    % % pause(0.1); %uncomment for animation
end
time_taken = toc;
in_use2 = monitor_memory_whos;

if ii < Max_iterations
    path.p(1).x = xGoal; path.p(1).y = yGoal;
    path.p(2).x = Tree.b(end).x; path.p(2).y = Tree.b(end).y;
    pathIndex = Tree.b(end).indexPrev; 
    j=0;
    while 1
        path.p(j+3).x = Tree.b(pathIndex).x;
        path.p(j+3).y = Tree.b(pathIndex).y;
        pathIndex = Tree.b(pathIndex).indexPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  
    path.p(end+1).x = xstart; path.p(end).y = ystart; 
    for j = 2:length(path.p)
        plot([path.p(j).x; path.p(j-1).x;], [path.p(j).y; path.p(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end
best_path = [path.p.x; path.p.y];
best_path = best_path';
best_path = flip(best_path);
diff = diff(best_path); 
path_length = sum(sqrt(sum(diff.*diff,2)));
smoothness = calculate_smoothness(best_path);
memory_used_in_Megabytes=(in_use2-in_use1);
disp(path_length)
disp(smoothness)
disp('time')
disp(time_taken)
disp(memory_used_in_Megabytes)


