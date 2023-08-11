%% preamble
close all;
clear all;
clc;
% RRTstar;
% RRT;
% A_Star;
% d_star;
% informed;
% HybAstar;
LPAstar;

%% Simulation setup
% Time
endtime = 1000;           
stepsize = 0.01;
time = 0;

%% initial states and controls
unmatched = zeros(1,3);
x_init = [0;0;0;0;0;0;2;2;0;0;0;0];
i_init = zeros(1,4);
omega_init = zeros(1,4);
% adjust start and endpoints as needed
start = [2,2];
goal = [95,95];
v_left = 5;
v_right = 5;
% checkpoint = [0 0 ; 1 1; 3 -1; -1 -1; -3 2; 0 4];
checkpoint = best_path;
checkcheck = 1;
torques = zeros(1,4);
a = 0;
robot_radius = 0.3;
init = 0;
e = 0;
% test 1
des_V = 0.6;
maxdist = 0.5;
Turn = 0;
initH = 0;
eH = 0;

%VELOCITY CONTROLLER GAINS
% kp = 6;
% ki = 10;
% kd = 0;
kp = 6;
ki = 10;
kd = 0;

%HEADING CONTROLLER GAINS
kpH = 4;
kiH = 0;
kdH = 0;




%% main simulation
% initialize simulation
x = x_init;
i = i_init;
omega=omega_init;
total_timesteps = endtime/stepsize;



% run simulation
for timestep = 1:total_timesteps
    a = a+1;
    
    % data collection
    T(a) = timestep;
    dpsi(a,:) = x(12);
    psi = x(12);
    dxaxis(a,:) = x(7);
    dyaxis(a,:) = x(8);
    Vs = x(1);
    dx(a,:) = x;    
    di(a,:) = i;
    domega(a,:) = omega;
    dtorques(a,:) = torques;
    desHeading = 0;
    AtCheckpoint = 0;
  
    if checkcheck > length(checkpoint)     %terminate loop at the end of checkpoint
        break
    end

    [AtCheckpoint, desHeading] = desiredHeading(dx(timestep,7), dx(timestep,8), checkpoint(checkcheck,:), maxdist);

    if AtCheckpoint == 1
    checkcheck = checkcheck+1;  
    end
    % % test 2
    % desHeading = deg2rad(45);
    % % test 3
    % if timestep > 0.5 * total_timesteps
    %     des_V = 0.25;
    % end
    % % % test 4/5
    % if timestep > 0.5 * total_timesteps
    %     desHeading = deg2rad(-45);
    % end
    Turn =  wrapToPi(desHeading-dpsi(timestep,1));
    dTurn(a,:) = Turn;

    



    

    %% controller

    % open loop controller
    % if timestep < 1000
    %     v_left = 6;
    %     v_right = 6;
    % elseif (300 < timestep) && (timestep < 600)
    %     v_left = 1;
    %     v_right = 6;
    % else
    %     v_left = 6;
    %     v_right = 1;

    % end

    % % closed loop control

    % velocity PID 
    old_e = e;
    e = des_V-Vs;
    init = init + (e *stepsize);

    P = kp*e;
    I = ki*init;
    D = kd*((e - old_e)/stepsize);

    PID = P+I+D;

    % Heading PID 
    old_eH = eH;
    eH = Turn;
    initH = initH + (eH *stepsize);

    PH = kpH*eH;
    IH = kiH*initH;
    DH = kdH*((eH - old_eH)/stepsize);

    PIDH = PH+IH+DH;


    v_left=PID+PIDH;
    v_right=PID-PIDH;

    %% data recording
    v = [v_left,v_left,v_right,v_right];
    deH(a,:) = eH;
    dv(a,:) = v_right+v_left;
    ddes_v(a,:) = des_V;
    ddesheading(a,:) = desHeading;

    
    %% processes
    % run motor
    [torques,idot,omegadot] = Rover_Motor_Model_v1(v,i,omega);
    % run rigidbody
    [xdot, xo] = Rover_Rigid_Body_Model_v1(x,torques,unmatched);
    % Euler intergration 
    x = x + (xdot * stepsize);
    i = i + (idot * stepsize);
    omega = omega + (omegadot * stepsize);
    time(a)    = timestep * stepsize;
    % robot animation
   % figure;
   % show(mymap);hold on;title('');grid on
   % xlabel('Y(meters)'); ylabel('X(meters)');
   % animate(robot_radius, dx(timestep,8), dx(timestep,7), dx(timestep,12), 'k');
    




end

smoothnessRobot = calculate_smoothness([dxaxis,dyaxis]);
disp(smoothnessRobot)
disp(time(end))
% graphs
% figure; hold on; grid on;axis([0,endtime,0,1])
%  plot(time,dx(:,1),'--'); 
%  plot(time,ddes_v(:,1),'--');
%  xlabel('time,s'); ylabel('velocity,m/s');
 % % figure(3); hold on; grid on;
 % % plot(time,dv(:,1));
 % % xlabel('time'); ylabel('voltage');
 % figure; hold on; grid on; axis([0,endtime,-90,90])
 % plot(time,rad2deg(dpsi(:,1)),'--');
 % plot(time,rad2deg(ddesheading(:,1)),'--');
 % xlabel('time,s'); ylabel('heading,degrees');

figure;                                  % robot path plot
show(mymap);hold on;title('');grid on
xlabel('Y(meters)'); ylabel('X(meters)');
hold on
%if this returns an error its because of capitalization(case) of variables.
plot(start(1), start(2),'go', 'MarkerSize',5, 'MarkerFaceColor','g');
plot(goal(1), goal(2),'bo' ,'MarkerSize',5, 'MarkerFaceColor','b');
plot(dxaxis(:,1),dyaxis(:,1),'g',LineWidth=2);
xlabel('Y(meters)'); ylabel('X(meters)');


