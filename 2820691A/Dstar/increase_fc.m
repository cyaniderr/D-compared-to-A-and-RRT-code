function valid = increase_fc(valid,cell_x,cell_y,in_valid,max_x,max_y)

    list_children = find((valid(:,4) == cell_x) & (valid(:,5) == cell_y));
    
    valid(find((valid(:,2) == cell_x) & (valid(:,3) == cell_y),1),8) = 1000;
    valid(find((valid(:,2) == cell_x) & (valid(:,3) == cell_y),1),1) = 1;
    
    fprintf('generating new path\n');    
    for i = 1:size(list_children)
        valid = increase_fc(valid,valid(list_children(i),2),valid(list_children(i),3),in_valid,max_x,max_y);
    end
