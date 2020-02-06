%function MainDexrov
addpath('./simulation_scripts');
clc;
clear;
close all

% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 35;
loop = 1;
maxloops = ceil(end_time/deltat);

% this struct can be used to evolve what the UVMS has to do
mission.phase = 1;
mission.phase_time = 0;

% Rotation matrix to convert coordinates between Unity and the <w> frame
% do not change
wuRw = rotation(0,-pi/2,pi/2);
vRvu = rotation(-pi/2,0,-pi/2);

% pipe parameters
u_pipe_center = [-10.66 31.47 -1.94]'; % in unity coordinates
pipe_center = wuRw'*u_pipe_center;     % in world frame coordinates
pipe_radius = 0.3;

% UDP Connection with Unity viewer v2
uArm = udp('127.0.0.1',15000,'OutputDatagramPacketSize',28);
uVehicle = udp('127.0.0.1',15001,'OutputDatagramPacketSize',24);
fopen(uVehicle);
fopen(uArm);

% initialize uvms structure
uvms = InitUVMS('DexROV');
% uvms.q 
% Initial joint positions. You can change these values to initialize the simulation with a 
% different starting position for the arm
uvms.q = [-0.0031 1.2586 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]';
% uvms.p
% initial position of the vehicle
% the vector contains the values in the following order
% [x y z r(rot_x) p(rot_y) y(rot_z)]
% RPY angles are applied in the following sequence
% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)
uvms.p = [-1.9379 10.4813-6.1 -29.7242-0.1 0 0 0]';

% initial goal position definition
% slightly over the top of the pipe
distanceGoalWrtPipe = 0.3;
uvms.goalPosition = pipe_center + (pipe_radius + distanceGoalWrtPipe)*[0 0 1]';
uvms.wRg = rotation(pi,0,0);
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

% defines the tool control point
uvms.eTt = eye(4);

% defines the target position for the vehicle position task      
uvms.targetPosition = pipe_center + (pipe_radius + 1.5)*[0 0 1]';
targetRotation = [0, -0.06, 0.5];
uvms.wRtarget = rotation(0, -0.06, 0.5);
uvms.wTtarget = [uvms.wRtarget uvms.targetPosition; 0 0 0 1]; % transf. matrix w->target

exercise = input('Enter the exercise: ','s');
% Definition and initialization of missions 
mission = InitMissionPhase2(exercise);
% mission = InitMissionPhase(exercise);

% Preallocation
plt = InitDataPlot(maxloops, mission);
plt.targetPosition = uvms.targetPosition; % vehicle target position
plt.targetRotation = targetRotation;
plt.goalPosition = uvms.goalPosition; % tool goal position
%plt.goalRotation = goalRotation;
uvms = ComputeActivationFunctions(uvms, mission);

tic
for t = 0:deltat:end_time
    % update all the involved variables
    uvms = UpdateTransforms(uvms);
    uvms = ComputeJacobians(uvms);
    uvms = ComputeTaskReferences(uvms);
    uvms = ComputeActivationFunctions(uvms, mission);
 
    mission.current_time = t;
    
    if strcmp(exercise,'5.1') || strcmp(exercise,'5.2')
        % Exercise 5
        % the sequence of iCAT_task calls defines the priority
        [uvms] = taskSequence(uvms, mission);
    else
        % Exercise 6
        if mission.phase == 1
            % the sequence of iCAT_task calls defines the priority
            [uvms] = taskSequence(uvms, mission);
        else
            % save the current vehicle velocity in a constant variable
            current_v = uvms.p_dot;

            % TPIK 1
            [uvms] = taskSequence(uvms, mission);
            v1 = uvms.p_dot;
            % TPIK 2
            [uvms] = taskSequence(uvms, mission, current_v);
            q_dot2 = uvms.q_dot;

            % get the two variables for integration
            uvms.q_dot = q_dot2;
            % sinusoidal velocity disturbance wrt world frame
            a_x = 1;    % amplitude of the sine along x
            a_y = 1;    % amplitude of the sine along y
            w = pi;     % frequency of the sine
            disturbance = sin(w*t)*[a_x a_y 0 0 0 0]';
            % sinusoidal velocity disturbance wrt vehicle frame
            disturbance_v = [uvms.vTw(1:3,1:3)  zeros(3); ...
                             zeros(3)           uvms.vTw(1:3,1:3)]*disturbance;
            uvms.p_dot = v1 + disturbance_v;

        end
    end
    % Integration
	uvms.q = uvms.q + uvms.q_dot*deltat;
    % beware: p_dot should be projected on <v>
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat);
    
    % check if the mission phase should be changed
    [uvms, mission,plt] = UpdateMissionPhase(uvms, mission, plt);
    
    % send packets to Unity viewer
    SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);
        
    % collect data for plots
    plt = UpdateDataPlot(plt,uvms,t,loop, mission);
    loop = loop + 1;
   
    % add debug prints here
    if (mod(t,0.1) == 0 && ~(plt.goalreached))
        disp(['Time:             ',num2str(t)]);
        disp(['Phase:            ',num2str(mission.phase)]);
        disp(['Vehicle position: ',num2str(uvms.p')]);
        disp('- - - - - - - - -');
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

PrintPlot(plt);

%end