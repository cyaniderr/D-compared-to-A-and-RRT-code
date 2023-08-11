function [valid,valid_size] = remove(valid,cell_x,cell_y)
   
   
    valid(find((valid(:,2) == cell_x) & (valid(:,3) == cell_y),1),:) = [];
   [valid_size,~] = size(valid);
   
   
end