% Adapted from robot_4wd_full_VER2.m

function [xdot, xo] = Rover_Rigid_Body_Model_v1(xcur,torques,unmatched)

%--------------------%
% Used to keep psi between -180 -> +180
if(xcur(12)>=(pi))
   xcur(12)=xcur(12)-(2*pi);
elseif(xcur(12)<-pi)
   xcur(12)=xcur(12)+(2*pi);
end
%--------------------%

%
fz = unmatched(1);
tx = unmatched(2);
ty = unmatched(3);

%---------------------------------------------------------------------%
% States
u = xcur(1);   % forward velocity(surge)
v = xcur(2);   % lateral velocity(sway)
w = xcur(3);   % vertical velocity(heave)
p = xcur(4);   % rotational velocities
q = xcur(5);   % rotational velocities
r = xcur(6);   % rotational velocities
x = xcur(7);  
y = xcur(8);
z = xcur(9);
phi = xcur(10);   % roll
theta = xcur(11); % pitch
psi = xcur(12);   % heading(yaw)
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Setup Cosines and Sines
% Cosines
cphi = cos(phi); ctheta = cos(theta); cpsi = cos(psi);

% Sines
sphi = sin(phi); stheta = sin(theta); spsi = sin(psi);
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Robot Specifications
m = 2.148;          % Mass of robot, kg
wheel_r = 0.0635;   % Radius of Wheel, m
x_area = 0.0316;    % Area presented on the x-axis, m.m
y_area = 0.0448;    % Area presented on the y-axis, m.m
Jx = 0.0140;        % Moment of Inertia about the x-axis, kg.m.m
Jy = 0.0252;        % Moment of Inertia about the y-axis, kg.m.m
Jz = 0.0334;        % Moment of Inertia about the z-axis, kg.m.m
mr = 0.1245;        % Moment Arm

% Constants
g = 9.81;           % Gravity, m/s.s
Cd = 0.89;          % Drag Coefficent
rho= 1.29;          % Air density
W = 21.0719;        % m*g; %Weight
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Input Forces
% Calculated from the torques generated by the wheels. 
force_l1 = (torques(1)/wheel_r);
force_l2 = (torques(2)/wheel_r);
force_r1 = (torques(3)/wheel_r);
force_r2 = (torques(4)/wheel_r);
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Slip
% calculates the slip angle for each wheel
bottom = sqrt((u^2)+(v^2));   % calculate denominator

if bottom == 0,                  % check if 0,
    beta = 0;                    %   if 0 then no movement, no slip
else
    beta = asin(v/bottom);       % else calculate slip
end;

if (abs(beta) > pi)
    beta =  0;
end
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Propulsion Forces
surge = (force_l1+force_l2+force_r1+force_r2)*cos(beta);
sway = (force_l1+force_l2+force_r1+force_r2)*sin(beta);
heave = fz;
roll = tx;
pitch = ty;
yaw = ((force_l1+force_l2)-(force_r1+force_r2))*mr;
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Dampening Terms
% fric_k = 0.35;
% fric_m = 0.44;
% fric_x = 0.22;
% fric_n = 0.18;
% fric_y = 1;
% fric_z = 0.3;

% Friction
Fx_fric = 4.6358*u;     % W*fric_x*u;
Fy_fric = 21.0719*v;    % W*fric_y*v;
Fz_fric = 6.3216*w;     % W*fric_z*w;
K_fric = 0.9182*p;      % W*fric_k*mr*p;
M_fric = 1.1543*q;      % W*fric_m*mr*q;
N_fric = 0.4722*r;      % W*fric_n*mr*r;

% Air Resistance
% x-axis
Fx_ar = 0.0181*u*abs(u); % 0.5*Cd*x_area*rho*u*abs(u);

% Total Dampening
X_damp = Fx_fric+Fx_ar;
Y_damp = Fy_fric;
Z_damp = Fz_fric;
K_damp = K_fric;
M_damp = M_fric;
N_damp = N_fric;
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Gravity Terms
X_grav = W*stheta;
Y_grav = W*sphi*ctheta;
Z_grav = (W*ctheta*cphi)-W;
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Forces and Torques
% Forces
X = surge-X_damp+X_grav;
Y = sway-Y_damp+Y_grav;
Z = heave-Z_damp+Z_grav;

% Torques
K = roll-K_damp;
M = pitch-M_damp;
N = yaw-N_damp;
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Equations of Motion
% Linear Accelerations
udot = (X/m)+(v*r)-(w*q);
vdot = (Y/m)+(w*p)-(u*r);
wdot = (Z/m)+(u*q)-(v*p);

% Rotational Accelerations
pdot = (K-((Jz-Jy)*q*r))/Jx;
qdot = (M-((Jx-Jz)*r*p))/Jy;
rdot = (N-((Jy-Jx)*p*q))/Jz;
 %---------------------------------------------------------------------%
 
%---------------------------------------------------------------------%
% Kinematics
% Linear Kinematics
% xxdot = ((cpsi*ctheta)*u)+(((-spsi*cphi)-(cpsi*stheta*sphi))*v)+(((spsi*sphi)-(cpsi*stheta*cphi))*w); 
%     % xxdot because xdot is the output matrix
% ydot = ((spsi*ctheta)*u)+(((cpsi*cphi)-(spsi*stheta*sphi))*v)+(((-sphi*cpsi)-(spsi*cphi*stheta))*w);
% zdot = ((stheta)*u)+((ctheta*sphi)*v)+((ctheta*cphi)*w);

% From the equations given in the paper, the equations should really be:
%
xxdot = ((cpsi*ctheta)*u)+(((-spsi*cphi)+(cpsi*stheta*sphi))*v)+(((spsi*sphi)+(cpsi*stheta*cphi))*w); 
    % xxdot because xdot is the output matrix
ydot = ((spsi*ctheta)*u)+(((cpsi*cphi)+(spsi*stheta*sphi))*v)+(((-sphi*cpsi)+(spsi*cphi*stheta))*w);
zdot = ((-stheta)*u)+((ctheta*sphi)*v)+((ctheta*cphi)*w);


% Angular Kinematics
% In theory +/-90 degrees for pitch is undefined but matlab tan()
% gives it a figure. Also this situation should not occur.
% ttheta = tan(theta);
% phidot = p+((-sphi*ttheta)*q)+((cphi*ttheta)*r); 
% thetadot = ((cphi)*q)+((sphi)*r);
% psidot = ((-sphi/ctheta)*q)+((cphi/ctheta)*r);

% Same here:
%
ttheta = tan(theta);
phidot = p+((sphi*ttheta)*q)+((cphi*ttheta)*r); 
thetadot = ((cphi)*q)+((-sphi)*r);
psidot = ((sphi/ctheta)*q)+((cphi/ctheta)*r);

%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Assign variables to output
xdot = [udot; vdot; wdot; pdot; qdot; rdot; xxdot; ydot; zdot; phidot; thetadot; psidot];
%---------------------------------------------------------------------%

xo = xcur;