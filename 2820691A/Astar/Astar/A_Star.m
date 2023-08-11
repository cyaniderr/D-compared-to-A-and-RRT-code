% preamble 
clearvars
close all
clc;
map3;


% % define obstacles
% obs = {[5,70,20,20];[40,70,20,20];[75,70,20,20];[5,40,20,20];[40,40,20,20];[75,40,20,20];[5,10,20,20];[40,10,20,20];[75,10,20,20]};
% 
% for k = 1:length(obs)
% 
%     for i = obs{k}(1):(obs{k}(1)+obs{k}(3))
%         for j = obs{k}(2):(obs{k}(2)+obs{k}(4))
%             A_starmap(i,j) = 1;
%         end
%     end
% end

A_starmap = Mapmatrix;

start_point = [2,2];
goal_point = [95,95];

xgoal=goal_point(1);%X Coordinate of the Target
ygoal=goal_point(2);%Y Coordinate of the Target

A_starmap(xgoal,ygoal)=3;%Initialize A_starmap with location of the target



xstart=start_point(1);%Starting Position
ystart=start_point(2);%Starting Position

A_starmap(xstart,ystart)=2;

tic
in_use1 = monitor_memory_whos; %memory check
%OPEN LIST COMPOSITION
%--------------------------------------------------------------------------
%IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
%--------------------------------------------------------------------------
OPEN_LIST=[];
%CLOSED LIST COMPOSITION
%--------------
%X val | Y val |
%--------------
CLOSED_LIST=[];
ObstAstar.xaxis = [];
ObstAstar.yaxis = [];

%Put all obstacles on the Closed list
k=1;
for i=1:x_max
    for j=1:y_max
        if(A_starmap(i,j) == 1)
            ObstAstar.xaxis = [ObstAstar.xaxis j];
            ObstAstar.yaxis = [ObstAstar.yaxis i];
            CLOSED_LIST(k,1)=i; 
            CLOSED_LIST(k,2)=j; 
            k=k+1;
        end
    end
end
CLOSED_COUNT=size(CLOSED_LIST,1);

%set the starting node as the first node
xNode=xstart;
path_cost=0;
yNode=ystart;
OPEN_COUNT=1;
goal_distance1=distance1(xNode,yNode,xgoal,ygoal);
OPEN_LIST(OPEN_COUNT,:)=populate(xNode,yNode,xNode,yNode,path_cost,goal_distance1,goal_distance1);
CLOSED_COUNT=CLOSED_COUNT+1;
CLOSED_LIST(CLOSED_COUNT,1)=xNode;
CLOSED_LIST(CLOSED_COUNT,2)=yNode;
NoPathcreated=1;

% ALGORITHM MAIN
while((xNode ~= xgoal || yNode ~= ygoal) && NoPathcreated == 1)

 exp_array=expand(xNode,yNode,path_cost,xgoal,ygoal,CLOSED_LIST,x_max,y_max);
 exp_count=size(exp_array,1);
 %UPDATE LIST OPEN WITH THE SUCCESSOR NODES
 %EXP ARRAY FORMAT
 %--------------------------------
 %|X val |Y val ||h(n) |g(n)|f(n)|
 %--------------------------------
 for i=1:exp_count
    flag=0;
    for j=1:OPEN_COUNT
        if(exp_array(i,1) == OPEN_LIST(j,2) && exp_array(i,2) == OPEN_LIST(j,3) )
            OPEN_LIST(j,8)=min(OPEN_LIST(j,8),exp_array(i,5)); 
            if OPEN_LIST(j,8)== exp_array(i,5)
                %UPDATE PARENTS,gn,hn
                OPEN_LIST(j,4)=xNode;
                OPEN_LIST(j,5)=yNode;
                OPEN_LIST(j,6)=exp_array(i,3);
                OPEN_LIST(j,7)=exp_array(i,4);
            end                               
            flag=1;
        end

    end
    if flag == 0
        OPEN_COUNT = OPEN_COUNT+1;
        OPEN_LIST(OPEN_COUNT,:)=populate(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
    end
 end

 %Find out the node with the smallest fn cost 
  index_min_node = min_fc(OPEN_LIST,OPEN_COUNT,xgoal,ygoal);
  if (index_min_node ~= -1)    
   %Set xNode and yNode to the node with minimum fn
   xNode=OPEN_LIST(index_min_node,2);
   yNode=OPEN_LIST(index_min_node,3);
   path_cost=OPEN_LIST(index_min_node,6);%Update the cost of reaching the parent node
  %Move the Node to list CLOSED
  CLOSED_COUNT=CLOSED_COUNT+1;
  CLOSED_LIST(CLOSED_COUNT,1)=xNode;
  CLOSED_LIST(CLOSED_COUNT,2)=yNode;
  OPEN_LIST(index_min_node,1)=0;
  else
     
      NoPathcreated=0;%Exit the loop
  end
end
i=size(CLOSED_LIST,1);
best_path=[];
xval=CLOSED_LIST(i,1);
yval=CLOSED_LIST(i,2);
i=1;
best_path(i,1)=xval;
best_path(i,2)=yval;
i=i+1;

if ( (xval == xgoal) && (yval == ygoal))
    inode=0;
   %Traverse OPEN and determine the parent nodes
   parent_x=OPEN_LIST(index(OPEN_LIST,xval,yval),4); %index returns the index of the node
   parent_y=OPEN_LIST(index(OPEN_LIST,xval,yval),5);
   
   while( parent_x ~= xstart || parent_y ~= ystart)
           best_path(i,2) = parent_x;
           best_path(i,1) = parent_y;
           %Get the grandparents
           inode=index(OPEN_LIST,parent_x,parent_y); %index returns the index of the node
           parent_x=OPEN_LIST(inode,4);                  
           parent_y=OPEN_LIST(inode,5);
           i=i+1;
   end
time_taken = toc;
in_use2 = monitor_memory_whos;
 
else
 pause(1);
 disp('Sorry, No path exists to the Target!');
end

 % plot map/obstacles  
    xMin = 1;
    xMax = x_max;
    yMin = 1;
    yMax = y_max;
    obstX = ObstAstar.xaxis;
    obstY = ObstAstar.yaxis;
    xs = start_point(1);
    ys = start_point(2);
    xt = goal_point(1);
    yt = goal_point(2);

    figure(1)
    grid on;
    axis equal
    axis([xMin - 1, xMax + 1, yMin - 1, yMax + 1])
    box on
    hold on

    % g=gca;                   %looks terrible as a figure in report
    % g.XTick=0:xMax;
    % g.YTick=0:yMax;

    % start
    plot(xs, ys, 'g*') 
    % target
    plot(xt, yt, 'b*')
  

    % Obstacles
    plot(obstX, obstY, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k');

    % walls
    rectangle('Position', [xMin - 0.5 yMin - 0.5 (xMax - xMin + 1) (yMax - yMin + 1)], 'LineWidth', 5)
    xlabel('Y(meters)'); ylabel('X(meters)');

    % plot path
    best_path = flip(best_path);
    best_path = [start_point;best_path]; 
    x = best_path(:, 1);
    y = best_path(:, 2);

    % path
    plot(x, y, 'b', 'LineWidth', 2);
    plot(x(2:end - 1), y(2:end - 1),  'b', 'LineWidth', 2);


    diff = diff(best_path);
    path_length = sum(sqrt(sum(diff.*diff,2)));
    smoothness = calculate_smoothness(best_path);

    memory_used_in_Megabytes=(in_use2-in_use1);
    disp(path_length)
    disp(smoothness)
    disp('time')
    disp(time_taken)
    disp(memory_used_in_Megabytes)

