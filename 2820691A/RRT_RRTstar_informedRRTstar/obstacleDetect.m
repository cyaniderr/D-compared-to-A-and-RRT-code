function feasible=obstacleDetect(startPose,goalPose,map)

feasible=true;
heading=atan2(goalPose(1)-startPose(1),goalPose(2)-startPose(2));
for r=0:0.5:sqrt(sum((startPose-goalPose).^2))
    pCheck = startPose + r.*[sin(heading) cos(heading)];
    if ~(feasiblePoint(ceil(pCheck),map) && feasiblePoint(floor(pCheck),map) && ...
            feasiblePoint([ceil(pCheck(1)) floor(pCheck(2))],map) && feasiblePoint([floor(pCheck(1)) ceil(pCheck(2))],map))
        feasible=false;break;
    end
    if ~feasiblePoint([floor(goalPose(1)),ceil(goalPose(2))],map), feasible=false; end

end

function feasible=feasiblePoint(point,map)
feasible=true;
if ~(point(1)>=1 &&  point(1)<=size(map,2) && point(2)>=1 && point(2)<=size(map,1) && map(point(2),point(1))==0)
    feasible=false;
end