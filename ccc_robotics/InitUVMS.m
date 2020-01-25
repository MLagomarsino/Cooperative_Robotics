function [uvms] = InitUVMS(robotname)

% uvms.vTb
% transformation matrix betwene the arm base wrt vehicle frame
% expresses how the base of the arm is attached to the vehicle
% do NOT change, since it must be coherent with the visualization tool
if (strcmp(robotname, 'DexROV'))    
    % do NOT change
    uvms.vTb = [rotation(pi, 0, pi) [0.167 0 -0.43]'; 0 0 0 1]; 
else
    if (strcmp(robotname, 'Robust'))
        % do NOT change
        uvms.vTb = [rotation(0, 0, pi) [0.85 0 -0.42]'; 0 0 0 1];
    end
end

uvms.q_dot = [0 0 0 0 0 0 0]';
uvms.q_ddot = [0 0 0 0 0 0 0]';
uvms.p_dot = [0 0 0 0 0 0]';
uvms.p_ddot = [0 0 0 0 0 0]';

% joint limits corresponding to the actual MARIS arm configuration
uvms.jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
uvms.jlmax  = [2.9;1.65;2.9;0.01;2.9;1.25;2.8];

% to be compute at each time step
uvms.wTv = eye(4,4);
uvms.wTt = eye(4,4);
uvms.vTw = eye(4,4);
uvms.vTe = eye(4,4);
uvms.vTt = eye(4,4);
uvms.vTg = eye(4,4);
uvms.Ste = eye(6,6);
uvms.bTe = eye(4,4);
% ---
uvms.vTtarget = eye(4,4);
uvms.eTg = eye(4,4);
uvms.wTt = eye(4,4);

uvms.bJe = eye(6,7);
uvms.djdq = zeros(6,7,7);
uvms.mu  = 0;
uvms.phi = zeros(3,1);
uvms.psi = zeros(3,1);
uvms.xi = zeros(3,1);
uvms.virtualFrameVelocity = zeros(6,1);
uvms.sensorDistance = 0; % distance of the vehicle from the seafloor measured by the sensor
% ----
uvms.minAltitude = 10; % minimum altitude from the seafloor
uvms.misalignment = zeros(3,1); % misalignment of the longitudinal axis
uvms.altitude = uvms.minAltitude + 0.5;

uvms.Jjl = [];
uvms.Jmu = [];
uvms.Jcc = [];
uvms.Jha = [];
uvms.Jt_a = [];
uvms.Jt_v = [];
uvms.Jt = [];
uvms.Jc = [];
uvms.Jca = [];
% ----
uvms.Jvpos = [];
uvms.Jminalt = [];
uvms.Jarmposture = [];
uvms.Jalt = [];
uvms.Jla = [];
uvms.Jfixvehicle = [];
uvms.Jopt = [];
    
uvms.xdot.jl = [];
uvms.xdot.mu = [];
uvms.xdot.cc = [];
uvms.xdot.ha = [];
uvms.xdot.t = [];
uvms.xdot.c = [];
uvms.xdot.ca = [];
% ----
uvms.xdot.target = [];
uvms.xdot.minalt = [];
uvms.xdot.alt = [];
uvms.xdot.la = [];

uvms.A.jl = zeros(7,7);     uvms.A.jl_t = zeros(7,7);
uvms.A.mu = 0;              uvms.A.mu_t = 0;
uvms.A.cc = zeros(1,1);     %?????? what is
uvms.A.ha = zeros(1,1);     uvms.A.ha_t = zeros(1,1);
uvms.A.t = zeros(6,6);      uvms.A.t = zeros(6,6);
uvms.A.c = [];     %?????? what is
uvms.A.ca = zeros(3,3);     %?????? what is
% ----
uvms.A.target = zeros(6,6);         uvms.A.target_t = zeros(6,6);
uvms.A.minalt = zeros(1,1);         uvms.A.minalt_t = zeros(1,1);
uvms.A.alt = zeros(1);              uvms.A.alt_t = zeros(1);
% uvms.Aexternal.target = zeros(1);
% uvms.Aexternal.alt = zeros(1);
uvms.A.la = zeros(1,1);             uvms.A.la_t = zeros(1,1);
uvms.A.fixvehicle = zeros(1);       uvms.A.fixvehicle_t = zeros(1);
uvms.A.opt = zeros(4);              uvms.A.opt_t = zeros(4);
uvms.A.constrained_vel = zeros(6);

uvms.toolFrameError = zeros(6,1);
uvms.totalError = zeros(6,1);
end

