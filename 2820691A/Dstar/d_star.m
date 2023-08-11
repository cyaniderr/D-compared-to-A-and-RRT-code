
% preamble 
clearvars
close all
clc;

map1;

D_starmap = Mapmatrix;

j=0;

start_point = [2,2];
goal_point = [95,95];

xgoal=goal_point(1);%X Coordinate of the Target
ygoal=goal_point(2);%Y Coordinate of the Target

D_starmap(xgoal,ygoal)=3;%Initialize MAP with location of the target

xStart=start_point(1);%Starting Position x
yStart=start_point(2);%Starting Position y

D_starmap(xStart,yStart)=2;
valid=[];                            % x | y | parents_x | parents_y | h(n) | g(n) | f(n) 

in_valid=[]; % x | y 
ObstDstar.xaxis = [];
ObstDstar.yaxis = [];

k=1;
for i=1:x_max
    for j=1:y_max
        if(D_starmap(i,j) == 1) % check for obstacles in mapmatrix
            ObstDstar.xaxis = [ObstDstar.xaxis j];
            ObstDstar.yaxis = [ObstDstar.yaxis i];
            in_valid(k,1)=i; 
            in_valid(k,2)=j; 
            k=k+1;
        end
    end
end
in_valid_size=size(in_valid,1);

cell_x=xStart;
cell_y=yStart;
valid_size=1; % initialize size of the valid_list
path_cost=0;
goal_distance=sqrt((cell_x-xgoal)^2 + (cell_y-ygoal)^2);

new_row=[1,8];
new_row(1,1)=1;
new_row(1,2)=cell_x;
new_row(1,3)=cell_y;
new_row(1,4)=cell_x; % parents x
new_row(1,5)=cell_y; % parents y 
new_row(1,6)=path_cost; %  hcost
new_row(1,7)=goal_distance; % gcsost
new_row(1,8)=goal_distance; % fcost

valid(valid_size,:)=new_row; % initializing path with start position
valid(valid_size,1)=0;

in_valid_size=in_valid_size+1;
in_valid(in_valid_size,1)=cell_x; 
in_valid(in_valid_size,2)=cell_y;

path_not_found=1;
obstacle_was_detected = 0;

% plot map/obstacles  
    xMin = 1;
    xMax = x_max;
    yMin = 1;
    yMax = y_max;
    obstX = ObstDstar.xaxis;
    obstY = ObstDstar.yaxis;
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

    % g=gca;
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



tic
in_use1 = monitor_memory_whos;
while 1 % broken if no new obstacle gets detected, making it work like A* 

while((cell_x ~= xgoal || cell_y ~= ygoal) && path_not_found == 1)
    successors=explore(cell_x,cell_y,path_cost,xgoal,ygoal,in_valid,x_max,y_max);
    successors_size=size(successors,1);
    for i=1:successors_size
    flag=0;
    for j=1:valid_size
        if(successors(i,1) == valid(j,2) && successors(i,2) == valid(j,3) ) % if successor same as already existing cell 

            valid(j,8)=min(valid(j,8),successors(i,5)); % check for minimum fc 
            if valid(j,8) == successors(i,5)
                valid(j,4)=cell_x;% parent x
                valid(j,5)=cell_y;% parent y
                valid(j,6)=successors(i,3); % h
                valid(j,7)=successors(i,4); % g
            end
            flag=1;
        end
    end
    if flag == 0 % if new cell with minimum f(n) then add to valid_path
        valid_size= valid_size+1;
        new_row=[1,8];
        new_row(1,1)=1;
        new_row(1,2)=successors(i,1);
        new_row(1,3)=successors(i,2);
        new_row(1,4)=cell_x; % parent x
        new_row(1,5)=cell_y; % parent y
        new_row(1,6)=successors(i,3); % h
        new_row(1,7)=successors(i,4); % g
        new_row(1,8)=successors(i,5); % f
        valid(valid_size,:)= new_row;
     end
    end

    index_min_cell = min_fcd(valid,valid_size,xgoal,ygoal);
    if (index_min_cell ~= -1) % if index with minimum fc is obstacle no path exists    
        cell_x=valid(index_min_cell,2);
        cell_y=valid(index_min_cell,3);
        path_cost=valid(index_min_cell,6);

        in_valid_size=in_valid_size+1; % put the cell in_valid so we dont come back on it again
        in_valid(in_valid_size,1)=cell_x;
        in_valid(in_valid_size,2)=cell_y;
        valid(index_min_cell,1)=0;
       
    else
        path_not_found=0;
    end
end

% backtracking to find the path

i=size(in_valid,1);
best_path=[];
xval=in_valid(i,1); % pick last in in_valid_list that must be target
yval=in_valid(i,2);
i=1;
best_path(i,1)=xval;
best_path(i,2)=yval;
i=i+1;

if ( (xval == xgoal) && (yval == ygoal))
    obstacle map 1 
    newob = size(in_valid,1) + 1;
    for p = 60:76                 
        for q = 60:70              
            D_starmap(p,q) = 1;
            in_valid(newob,1) = p;
            in_valid(newob,2) = q;   
            newob = newob + 1;

        end
    end

    % obstacle map 2
    % newob = size(in_valid,1) + 1;
    % obs = {[30,30,40,40]};
    % 
    % for k = 1:length(obs)
    % 
    % for p = obs{k}(1):(obs{k}(1)+obs{k}(3))
    %     for q = obs{k}(2):(obs{k}(2)+obs{k}(4))
    %         D_starmap(p,q) = 1;
    %         in_valid(newob,1) = p;
    %          in_valid(newob,2) = q;   
    %          newob = newob + 1;
    %     end
    % end
    % end

    % obstacle map 3
    % newob = size(in_valid,1) + 1;
    % obs = {[40,40,10,2]};
    % 
    % for k = 1:length(obs)
    % 
    % for p = obs{k}(1):(obs{k}(1)+obs{k}(3))
    %     for q = obs{k}(2):(obs{k}(2)+obs{k}(4))
    %         D_starmap(p,q) = 1;
    %         in_valid(newob,1) = p;
    %          in_valid(newob,2) = q;   
    %          newob = newob + 1;
    %     end
    % end
    % end
    in_valid_size= size(in_valid,1);
    obstacle_map = binaryOccupancyMap(D_starmap);
    
   inode=0;
   parents_x=valid(find((valid(:,2) == xval) & (valid(:,3) == yval),1),4);
   parents_y=valid(find((valid(:,2) == xval) & (valid(:,3) == yval),1),5);
   
   while( parents_x ~= xStart || parents_y ~= yStart)
           best_path(i,1) = parents_x;
           best_path(i,2) = parents_y;
           
           inode=find((valid(:,2) == parents_x) & (valid(:,3) == parents_y),1);
           parents_x=valid(inode,4);
           parents_y=valid(inode,5);
           i=i+1;
   end
   pathCoords = [];
    flipped_yet_again = flip(best_path);
    pathCoords(:,1) = flipped_yet_again(:,2);
    pathCoords(:,2) = flipped_yet_again(:,1);
    anotha_one = flip(pathCoords);

 % plotting path
 obstacle_was_detected = 0;
 j=size(pathCoords,1);
 p=plot(pathCoords(:,1),pathCoords(:,2),'r-','LineWidth',1.5);
 q=plot(anotha_one(j,1),anotha_one(j,2),'bsquare');

 for i=j:-1:1
     
  % pause(0.1);
 
  
  % -------------------------check if D_starmap is altered ------------- 
  if D_starmap(best_path(i,1),best_path(i,2)) == 1
      % map 1
      % x = [60, 70, 70, 60];
      % y = [62, 62, 75, 75];
      % fill(x,y,'k'); 
       
      % map 2
      % x = [30, 70, 70, 30];
      % y = [30, 30, 70, 70];
      % fill(x,y,'k');  

      % map 3
      x = [40, 42, 42, 40];
      y = [40, 40, 50, 50];
      fill(x,y,'k'); 
      
    disp ('obstacle detected');
    if(obstacle_was_detected == 0)
        
        % start computing the path from the parents path from where obstacle was first found
        cell_x = valid(find((valid(:,2) == best_path(i,1)) & (valid(:,3) == best_path(i,2)),1),4); 
        cell_y=valid(find((valid(:,2) == best_path(i,1)) & (valid(:,3) == best_path(i,2)),1),5);
        path_cost=valid(find((valid(:,2) == best_path(i,1)) & (valid(:,3) == best_path(i,2)),1),6);
   
    end
    
    obstacle_was_detected = 1;
    
    % remove obstacles from valid list
    [valid, valid_size] = remove(valid,best_path(i,1),best_path(i,2));
    valid = increase_fc(valid,best_path(i,1), best_path(i,2),in_valid,x_max,y_max);
    
   
  elseif obstacle_was_detected == 0 
      
    set(p,'XData',pathCoords(i,1),'YData',pathCoords(i,2));
    set(q,'XData',anotha_one(i,1),'YData',anotha_one(i,2));
    drawnow ;
    
  else


    in_valid(find((in_valid(:,1) == best_path(i,1)) & (in_valid(:,2) == best_path(i,2)),1),:) = [];
    valid = increase_fc(valid,best_path(i,1), best_path(i,2),in_valid,x_max,y_max); % updating costs
    
  end
  
 end
 if obstacle_was_detected == 0
     
    % plot path
   
    x = pathCoords(:, 1);
    y = pathCoords(:, 2);

    % path
    plot(x, y, 'b', 'LineWidth', 2);
    plot(x(2:end - 1), y(2:end - 1),  'b', 'LineWidth', 2);
    break
    
 else
     
     continue
     
 end
 
else
    
    disp( 'Sorry, No path exists to the Target!');
    break
    
end

end
time_taken = toc;
in_use2 = monitor_memory_whos;
best_path = [best_path(:,2),best_path(:,1)];
best_path = [best_path; start_point];
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

