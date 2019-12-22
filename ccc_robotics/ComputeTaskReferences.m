function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

%% reference for manipulability
uvms.xdot.mu = 0.1 * (0.12 - uvms.mu);

%% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.5 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

%% reference for vehicle-frame position control task
[ang_v, lin_v] = CartError(uvms.vTtarget , eye(4));
% we don't care about the arm 
uvms.xdot.target = 0.5 * [lin_v; ang_v];
% limit the requested velocities...
uvms.xdot.target(1:3) = Saturate(uvms.xdot.target(1:3), 0.9); % m/s
uvms.xdot.target(4:6) = Saturate(uvms.xdot.target(4:6), 0.9);

%% reference for horizontal attitude
uvms.xdot.ha = -0.1 * norm(uvms.phi); 

%% reference for minimum altitude
% Compute the current altitude from the seafloor by taking into account
% that the vehicle can be oriented in a different way with respect to world
% frame
uvms.altitude = [0 0 1]*uvms.wTv(1:3,1:3)*[0 0 uvms.sensorDistance]';
uvms.xdot.minalt = -0.5*(uvms.altitude - uvms.minAltitude);

%% reference for altitude
% the altitude of the vehicle converges to 0
uvms.xdot.alt = -0.5*(uvms.altitude);%  - 0.1);

%% reference for longitudinal alignment
% uvms.xdot.la = -0.5 * norm(uvms.misalignment); % Norm
uvms.xdot.la = -0.5 * uvms.misalignment;

%% reference for fixing the vehicle in the current position
uvms.xdot.fixvehicle = zeros(6,1); % no velocity

%% reference for joint limit
uvms.xdot.jl = -0.7*(uvms.q - ((uvms.jlmax - uvms.jlmin)./2)); % -lamda (current_q - desired_q)

%% reference for optimize configuration of the first 4 joints
preferred_shape = [-0.0031 1.2586 0.0128 -1.2460]';
uvms.xdot.opt = -0.4*(uvms.q(1:4) - preferred_shape);
