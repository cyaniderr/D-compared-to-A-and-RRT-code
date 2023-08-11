clearvars
close all
clc;
map3;
%%%%%%% make sure to use the right dynamic obstacle for the right map
LPAMap = Mapmatrix;
LPAbase.distType = 'euclidean'; % euclidean or manhattan;
LPAbase.adjType = 'Diag'; % nDiag or Diag

%% Map Size
[H, W] = size(LPAMap);
Map.xMin = 1;
Map.yMin = 1;
Map.xMax = W;
Map.yMax = H;
Map.lim = max(W, H);
Map.nX = Map.xMax - Map.xMin + 1;
Map.nY = Map.yMax - Map.yMin + 1;

%% Obstacles
Obst.radius = 0.25;
Obst.xaxis = [];
Obst.yaxis = [];
Obst.nodeNumber = [];
Nodes.count = H * W;
Nodes.cord = zeros(2, Nodes.count);
Nodes.number = zeros(1, Nodes.count);

iNodeNumber = 1;

for i = 1:H

    for j = 1:W
        Nodes.number(1, iNodeNumber) = iNodeNumber;
        Nodes.cord(:, iNodeNumber) = [j, i]';

        if LPAMap(i, j) == 1
            Obst.xaxis = [Obst.xaxis j];
            Obst.yaxis = [Obst.yaxis i];
            Obst.nodeNumber = [Obst.nodeNumber iNodeNumber];
        end

        iNodeNumber = iNodeNumber + 1;
    end

end

% if isempty(Obst.nodeNumber)               %% Bandaid fix for case where the initial obstacle planned on an empty map is not obstructed at all(MAP2)
%     Obst.nodeNumber = 7071;
% end



%% update model struct
LPAbase.Nodes = Nodes;
LPAbase.Obsts = Obst;
LPAbase.Map = Map;



% start & goal - coordinates
R.xstart = 2;
R.ystart = 2;
R.xtarget = 95;
R.ytarget = 95;

%  start & goal - node numbers
R.startNode = (R.ystart - LPAbase.Map.yMin) * (LPAbase.Map.nX) + R.xstart - LPAbase.Map.xMin + 1;
R.targetNode = (R.ytarget - LPAbase.Map.yMin) * (LPAbase.Map.nX) + R.xtarget - LPAbase.Map.xMin + 1;

%% update model struct
LPAbase.R = R;

%% edge costs, G, RHS
Nodes = LPAbase.Nodes;

switch LPAbase.adjType
    case 'nDiag'
        ixy = [1 0; 0 1; 0 -1; -1 0];
        nAdj = 4;
    case 'Diag'
        ixy = [1 0; 0 1; 0 -1; -1 0; 1 1; -1 -1; 1 -1; -1 1];
        nAdj = 8;
end

% euclidean or manhattan costs
switch LPAbase.distType
    case 'manhattan'
        edgeLength = 2;
    case 'euclidean'
        edgeLength = sqrt(2);
end

nNodes = LPAbase.Nodes.count;
Successors = cell(nNodes, 2);
Predecessors = cell(nNodes, 2);

for iNode = 1:nNodes

    if ~any(iNode == LPAbase.Obsts.nodeNumber)
        xNode = Nodes.cord(1, iNode);
        yNode = Nodes.cord(2, iNode);

        for iAdj = 1:nAdj
            ix = ixy(iAdj, 1);
            iy = ixy(iAdj, 2);
            newX = xNode + ix;
            newY = yNode + iy;

            % check if the Node is within array bound
            if (newX >= LPAbase.Map.xMin && newX <= LPAbase.Map.xMax) && (newY >= LPAbase.Map.yMin && newY <= LPAbase.Map.yMax)
                newNodeNumber = iNode + ix + iy * (LPAbase.Map.nX);

                if ~any(newNodeNumber == LPAbase.Obsts.nodeNumber)
                    Successors{iNode, 1} = [Successors{iNode}, newNodeNumber];
                    Predecessors{newNodeNumber, 1} = [Predecessors{newNodeNumber, 1}, iNode];

                    if ix ~= 0 && iy ~= 0
                        cost = edgeLength;
                    else
                        cost = 1;
                    end

                    Successors{iNode, 2} = [Successors{iNode, 2}, cost];
                    Predecessors{newNodeNumber, 2} = [Predecessors{newNodeNumber, 2}, cost];
                end

            end

        end

    end

end

% G, RHS
G = inf(1, nNodes);
RHS = inf(1, nNodes);

%% dynamic obsts
LPAbase.NewObsts.count = 0;

%% save model
LPAbase.startNode = LPAbase.R.startNode;
LPAbase.Predecessors = Predecessors;
LPAbase.Successors = Successors;
LPAbase.RHS = RHS;
LPAbase.G = G;


%% uncomment section to add dynamic obstacle at a time t
% obstacle map 1
% t = zeros(1, 154)+ 15;
% 
% 
% a = zeros(1,11)+61;
% b = zeros(1,11)+62;
% c = zeros(1,11)+63;
% d = zeros(1,11)+64;
% e = zeros(1,11)+65;
% f = zeros(1,11)+66;
% g = zeros(1,11)+67;
% h = zeros(1,11)+68;
% i = zeros(1,11)+69;
% j = zeros(1,11)+70;
% k = zeros(1,11)+71;
% l = zeros(1,11)+72;
% m = zeros(1,11)+73;
% n = zeros(1,11)+74;
% 
% y = [a b c d e f g h i j k l m n];
% x = [60 61 62 63 64 65 66 67 68 69 70 60 61 62 63 64 65 66 67 68 69 70 60 61 62 63 64 65 66 67 68 69 70 60 61 62 63 64 65 66 67 68 69 70 60 61 62 63 64 65 66 67 68 69 70 60 61 62 63 64 65 66 67 68 69 70 60 61 62 63 64 65 66 67 68 69 70 60 61 62 63 64 65 66 67 68 69 70 60 61 62 63 64 65 66 67 68 69 70 60 61 62 63 64 65 66 67 68 69 70 60 61 62 63 64 65 66 67 68 69 70 60 61 62 63 64 65 66 67 68 69 70 60 61 62 63 64 65 66 67 68 69 70 60 61 62 63 64 65 66 67 68 69 70];
 
% obstacle map 2
% obs = {[30,30,40,40]};
% k = length(obs);
% xL = (obs{k}(1)+obs{k}(3)-obs{k}(1))+1;
% yL = (obs{k}(2)+obs{k}(4)-obs{k}(2))+1;
% p = 0;
% q = 0;
% for i = obs{k}(1):(obs{k}(1)+obs{k}(3))
%     p = p+1;
%     x1(p) = i;
% end
% 
% x = [];
% y = [];
% for j = obs{k}(2):(obs{k}(2)+obs{k}(4))
%     y1 = j;
%     for i = 1:xL
%         y = [y y1];
%     end
% 
% 
% 
% end
% 
% for i = 1:yL
%     x = [x x1];
% end
% r = numel(y);
% t = zeros(1,r) + 10;

% obstacle map 3
obs = {[40,43,2,6]};
k = length(obs);
xL = (obs{k}(1)+obs{k}(3)-obs{k}(1))+1;
yL = (obs{k}(2)+obs{k}(4)-obs{k}(2))+1;
p = 0;
q = 0;
for i = obs{k}(1):(obs{k}(1)+obs{k}(3))
    p = p+1;
    x1(p) = i;
end

x = [];
y = [];
for j = obs{k}(2):(obs{k}(2)+obs{k}(4))
    y1 = j;
    for i = 1:xL
        y = [y y1];
    end



end

for i = 1:yL
    x = [x x1];
end
r = numel(y);
t = zeros(1,r) + 10;
    
%%
count = numel(t);
nodeNumbers = zeros(count);

for z = 1:count
    nodeNumbers(z) = (y(z) - LPAbase.Map.yMin) * LPAbase.Map.nX + x(z) - LPAbase.Map.xMin + 1;
end

% NewObsts
NewObsts.t = t;
NewObsts.x = x;
NewObsts.y = y;
NewObsts.count = count;
NewObsts.nodeNumbers = nodeNumbers;

% update LPAbase
LPAbase.NewObsts = NewObsts;
   
% dynamic obst section end
%% ALGORITHM START
tic
in_use1 = monitor_memory_whos;
% G, RHS
G = LPAbase.G;
RHS = LPAbase.RHS;

% Open: count, List
% node key, ind

% set the starting node as the first node in Open
TopNode.nodeNumber = LPAbase.R.startNode;
hCost = LPAcalDistance(LPAbase.R.xstart, LPAbase.R.ystart, LPAbase.R.xtarget, LPAbase.R.ytarget, LPAbase.distType);
TopNode.key = [hCost; 0];
RHS(TopNode.nodeNumber) = 0;
TopNode.ind = 1;

% insert start node in Open list
Open.List(1) = TopNode;
Open.count = 1;

t = 1;
finalPathNodeNumbers = [LPAbase.R.startNode;];

while LPAbase.startNode ~= LPAbase.R.targetNode


    
    % select top key
    keys = [[Open.List.key]', rand(Open.count, 1)];

    % search for node with min cost
    [~, sortInds] = sortrows(keys);
    topKeyInd = sortInds(1);

    TopNode = Open.List(topKeyInd);
    TopNode.ind = topKeyInd;

    if keys(topKeyInd, 1) == inf
        disp('No Path!')
    end


    % update goal_key
    Goal.nodeNumber = LPAbase.R.targetNode;
    Goal.key = min(G(Goal.nodeNumber), RHS(Goal.nodeNumber)) * [1; 1];

    while LPAcompareKeys(TopNode.key, Goal.key) || RHS(Goal.nodeNumber) ~= G(Goal.nodeNumber)

        % remove topkey from open
        Open.List(TopNode.ind) = [];
        Open.count = Open.count - 1;

        % update vertex
        nodesForUpdate = LPAbase.Successors{TopNode.nodeNumber, 1};

        if G(TopNode.nodeNumber) > RHS(TopNode.nodeNumber)
            G(TopNode.nodeNumber) = RHS(TopNode.nodeNumber);
        else
            G(TopNode.nodeNumber) = inf;
            nodesForUpdate(end + 1) = TopNode.nodeNumber;
        end

        [Open, RHS] = LPAupdateVertex(Open, RHS, G, nodesForUpdate, LPAbase);

        % select top key
        keys = [[Open.List.key]', rand(Open.count, 1)];

        % search for node with min cost
        [~, sortInds] = sortrows(keys);
        topKeyInd = sortInds(1);

        TopNode = Open.List(topKeyInd);
        TopNode.ind = topKeyInd;

        if keys(topKeyInd, 1) == inf
            disp('No Path!')
        end

        % update goal_key
        Goal.key = min(G(Goal.nodeNumber), RHS(Goal.nodeNumber)) * [1; 1];

    end

    % optimal paths nodes
    i = 1;
    pathNodes = [];
    nodeNumber = LPAbase.R.targetNode;

    pathNodes(i) = nodeNumber;

            %expand
            while nodeNumber ~= LPAbase.startNode
                i = i + 1;
                predNodes = LPAbase.Predecessors{nodeNumber, 1};
                [~, indMinG] = min(G(predNodes) + LPAbase.Predecessors{nodeNumber, 2});
                nodeNumber = predNodes(indMinG);
                pathNodes(i) = nodeNumber;
            end

       

    pathNodes = flip(pathNodes);

    % move R to new startNode
    LPAbase.startNode = pathNodes(2);
    finalPathNodeNumbers(end + 1) = LPAbase.startNode;
    t = t + 1;

    % check for update in edge costs (obstacles)
    for i = 1:LPAbase.NewObsts.count

        if t == LPAbase.NewObsts.t(i)
            newObstNode = LPAbase.NewObsts.nodeNumbers(i);
            LPAbase.Predecessors{newObstNode, 2} = LPAbase.Predecessors{newObstNode, 2} + inf;

            % update vertex
            [Open, RHS] = LPAupdateVertex(Open, RHS, G, newObstNode, LPAbase);

        end

    end
end
in_use2 = monitor_memory_whos;
best_Path.nodeNumbers = finalPathNodeNumbers;
best_Path.coords = LPAnodes2coords(best_Path.nodeNumbers, LPAbase);


%% benchmarks
memory_used_in_Megabytes=(in_use2-in_use1);
Soln = best_Path;
Soln.runTime = toc;
Soln.cost = calCostLPA(Soln.coords);
Soln.smoothness = calSmoothnessLPA(Soln.coords);
diff = diff(best_Path.coords);
Soln.path_length = sum(sqrt(sum(diff.*diff,2)));
best_path = best_Path.coords;
disp(memory_used_in_Megabytes)

%% display and plot
disp(Soln)

plotLPA(LPAbase)
plotSolution(Soln.coords)
plotAnimation2(LPAbase, Soln.coords)

