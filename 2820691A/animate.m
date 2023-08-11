function [] = animate(robot_radius, robot_x, robot_y, headingAngle, colour)

%% Generate Circle
theta    = linspace(0, 2*pi, 360);
circle_x = robot_radius * cos(theta);
circle_y = robot_radius * sin(theta);

%% Calculate end points of the line
linendpoint_x = cos(headingAngle) * (robot_radius + 0.01);
linendpoint_y = sin(headingAngle) * (robot_radius + 0.01);

%% Draw robot
plot((circle_x + robot_x), (circle_y + robot_y), colour);

% Draw line from center of robot in the direction of the current heading
line([robot_x, (linendpoint_y + robot_x)], [robot_y, (linendpoint_x + robot_y)], 'color', colour);

end