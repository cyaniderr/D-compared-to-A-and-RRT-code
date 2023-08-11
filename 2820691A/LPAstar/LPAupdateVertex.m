function [Open, RHS] = LPAupdateVertex(Open, RHS, G, nodesForUpdate, LPAbase)

    for nodeNumber = nodesForUpdate

        if nodeNumber ~= LPAbase.startNode
            predNodes = LPAbase.Predecessors{nodeNumber, 1};
            [valMinG, ~] = min(G(predNodes) + LPAbase.Predecessors{nodeNumber, 2});
            RHS(nodeNumber) = valMinG;
        end

        checkInOpen = nodeNumber == [Open.List.nodeNumber];

        if any(checkInOpen)
            Open.List(checkInOpen) = [];
            Open.count = Open.count - 1;
        end

        if G(nodeNumber) ~= RHS(nodeNumber)
            Open.count = Open.count + 1;
            op.nodeNumber = nodeNumber;
            nodeXY = LPAbase.Nodes.cord(:, nodeNumber);
            hCost = LPAcalDistance(nodeXY(1), nodeXY(2), LPAbase.R.xtarget, LPAbase.R.ytarget, LPAbase.distType);
            op.key = min(G(nodeNumber), RHS(nodeNumber)) + [hCost; 0];
            op.ind = Open.count;
            Open.List(op.ind) = op;
        end

    end

end
