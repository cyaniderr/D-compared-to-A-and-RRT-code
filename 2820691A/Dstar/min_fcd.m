function index_of_min = min_fcd(valid,valid_len,x_target,y_target)

 temp_array=[];
 k=1;
 flag=0;
 goal_index=0;
 for j=1:valid_len
     if (valid(j,1)==1)
         temp_array(k,:)=[valid(j,:) j]; 
         if (valid(j,2)==x_target && valid(j,3)==y_target)
             flag=1;
             goal_index=j;
         end;
         k=k+1;
     end;
 end;
 if flag == 1 
     index_of_min=goal_index;
 end
 
 if size(temp_array ~= 0)
  [min_fcd,temp_min]=min(temp_array(:,8));
  
  index_of_min=temp_array(temp_min,9);
  
 else
     index_of_min=-1;
 end;