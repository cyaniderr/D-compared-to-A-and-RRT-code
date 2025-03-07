function i_min = min_fc(OPEN,OPEN_COUNT,xTarget,yTarget)
% This function takes the list OPEN as its input and returns the index of the node that has the least cost
 temp_array=[];
 k=1;
 flag=0;
 goal_index=0;
 for j=1:OPEN_COUNT
     if (OPEN(j,1)==1)
         temp_array(k,:)=[OPEN(j,:) j]; %#ok<*AGROW>
         if (OPEN(j,2)==xTarget && OPEN(j,3)==yTarget)
             flag=1;
             goal_index=j;%Store the index of the goal node
         end
         k=k+1;
     end
 end%Get all nodes that are on the list open
 if flag == 1 % one of the successors is the goal node so send this node
     i_min=goal_index;
 end
 %Send the index of the smallest node
 if size(temp_array )~= 0
  [min_fc,temp_min]=min(temp_array(:,8));%Index of the smallest node in  array
  i_min=temp_array(temp_min,9);%Index of the smallest node in the OPEN array
 else
     i_min=-1;%The temp array is empty i.e No more paths are available.
 end