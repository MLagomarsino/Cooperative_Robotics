function [uvms] = ComputeActivationFunctions(uvms, mission)
% The function computes the activation functions

% manipulability
% if mu < 0.02, A = 1;
% if mu > 0.05, A = 0;
% in between, there is a smooth behavior.
uvms.A.mu = DecreasingBellShapedFunction(0.02, 0.05, 0, 1, uvms.mu);

% horizontal attitude control
% if norm(phi) > 0.1,   A = 1;
% if norm(phi) < 0.025, A = 0;
% in between, there is a smooth behavior.
uvms.A.ha = IncreasingBellShapedFunction(0.01, 0.02, 0, 1, norm(uvms.phi));

% arm tool position control
uvms.A.t = eye(6); % always active (equality objective)

% vehicle position control 
[rho, basic_vector] = CartError(eye(4), uvms.vTtarget); 
uvms.A.target = [eye(3)*IncreasingBellShapedFunction(0.05, 0.25, 0, 1, norm(basic_vector)),       zeros(3); ...
                 zeros(3),                  eye(3)*IncreasingBellShapedFunction(0.005, 0.01, 0, 1, norm(rho))]; % (uvms.Aexternal.target);

% minimum altitude
% if altitude < -1.5, A = 0;
% if altitude > -1, A = 1;
% in between, there is a smooth behavior.
uvms.A.minalt = DecreasingBellShapedFunction(uvms.minAltitude, uvms.minAltitude + 0.5, 0, 1, uvms.altitude);

% altitude control
uvms.A.alt = 1; %*(uvms.Aexternal.alt); % equality objective

% longitudinal axis of the vehicle aligned to the nodule
uvms.A.la = eye(3)*IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.misalignment));

% fix vehicle velocity
uvms.A.fixvehicle = eye(6);

% joint limit
for i = 1:length(uvms.q)
    uvms.A.jl(i,i) = DecreasingBellShapedFunction(uvms.jlmin(i),uvms.jlmin(i) + 0.6,0,1,uvms.q(i)) + ...
                    IncreasingBellShapedFunction(uvms.jlmax(i) - 0.6,uvms.jlmax(i),0,1,uvms.q(i));
end

% arm preferred shape
error_q =  uvms.preferred_shape - uvms.q(1:length(uvms.preferred_shape));
for i = 1:length(uvms.preferred_shape)
    uvms.A.opt(i,i) = IncreasingBellShapedFunction(uvms.preferred_shape(i) - 0.1, uvms.preferred_shape(i),0,1,norm(error_q(i)));
end

end