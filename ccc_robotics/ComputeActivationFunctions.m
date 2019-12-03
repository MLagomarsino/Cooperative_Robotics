function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% example: manipulability
% if mu < 0.02, A = 1;
% if mu > 0.05, A = 0;
% in between, there is a smooth behavior.
uvms.A.mu = 1;%DecreasingBellShapedFunction(0.02, 0.05, 0, 1, uvms.mu);

% phi: misalignment vector between the horizontal plane and the
% longitudinal axis of the vehicle
% if norm(phi) > 0.1,   A = 1;
% if norm(phi) < 0.025, A = 0;
% in between, there is a smooth behavior.
uvms.A.ha = 1;%IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.phi));

% arm tool position control
% always active
uvms.A.t = eye(6);

% vehicle position control 
uvms.A.target = eye(6);% (uvms.Aexternal.target).*eye(6); % equality objective

% minimum altitude
% if altitude < -1.5, A = 0;
% if altitude > -1, A = 1;
% in between, there is a smooth behavior.
uvms.A.minalt = 1;%DecreasingBellShapedFunction(uvms.minAltitude, uvms.minAltitude + 0.5, 0, 1, uvms.altitude);

% altitude control
uvms.A.alt = 1;%*(uvms.Aexternal.alt); % equality objective

% longitudinal axis of the vehicle aligned to the rocket
uvms.A.la = 1;%IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.misalignment));


