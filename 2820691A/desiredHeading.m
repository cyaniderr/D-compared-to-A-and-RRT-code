function [AtCheckpoint, desHeading] = desiredHeading(xpos, ypos, checkpoint, tolerance)

%calculate distance from checkpoint
Dposition = checkpoint - [xpos,ypos];
distanceFromCheckpoint = norm(Dposition);

% check if robot is within specified tolerance range of distance from
% checkpoint

if distanceFromCheckpoint<= tolerance
    AtCheckpoint = 1;
else 
    AtCheckpoint = 0;
end

% Calculate new heading
CalcHeading = atan2( Dposition(2), Dposition(1) );

%incase tan inverse returns NaN
if isnan(CalcHeading)
    if Dposition(2) >= 0
        desHeading = pi/2;
    else
        desHeading = -pi/2;
    end
else
    desHeading = CalcHeading;
end

end