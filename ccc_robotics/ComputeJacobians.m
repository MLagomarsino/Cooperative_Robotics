function [uvms] = ComputeJacobians(uvms)
% compute the relevant Jacobians here
% joint limits
% manipulability
% tool-frame position control
% vehicle-frame position control
% horizontal attitude 
% minimum altitude
% preferred arm posture ( [-0.0031 1.2586 0.0128 -1.2460] )
%
% remember: the control vector is:
% [q_dot; p_dot] 
% [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
%
% therefore all task jacobians should be of dimensions
% m x 13
% where m is the row dimension of the task, and of its reference rate

%% Joint limit Jacobian
% uvms.Jjl ? 

%% Manipulability Jacobian
[Jmu_a, uvms.mu] = ComputeManipulability(uvms.bJe, uvms.djdq);
uvms.Jmu = [Jmu_a zeros(1,6)];

%% Tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]
%
% Ste is the rigid body transformation from vehicle-frame to end-effector
% frame projected on <v>
% ---- uvms.vTe(1:3,1:3) = rotation matrix of e wrt v
% ---- uvms.eTt(1:3,4) = basic vector e->t
% ---- > basic vector v->t
uvms.Ste = [eye(3) zeros(3);  -skew(uvms.vTe(1:3,1:3)*uvms.eTt(1:3,4)) eye(3)];
% uvms.bJe contains the arm end-effector Jacobian (6x7) wrt arm base
% top three rows are angular velocities, bottom three linear velocities
% ---- uvms.vTb(1:3,1:3) = rotation matrix of base b wrt v
uvms.Jt_a  = uvms.Ste * [uvms.vTb(1:3,1:3) zeros(3,3); zeros(3,3) uvms.vTb(1:3,1:3)] * uvms.bJe;
% vehicle contribution is simply a rigid body transformation from vehicle
% frame to tool frame. Notice that linear and angular velocities are
% swapped due to the different definitions of the task and control
% variables
uvms.Jt_v = [zeros(3) eye(3); eye(3) -skew(uvms.vTt(1:3,4))];
% juxtapose the two Jacobians to obtain the global one
uvms.Jt = [uvms.Jt_a uvms.Jt_v];

%% Vehicle-frame position Jacobian
uvms.Jvpos = [zeros(6,7) eye(6,6)];
 
%% Horizontal attitude Jacobian
kv   = [0 0 1]';
w_kw = [0 0 1]'; % unit vector k of world frame wrt to w
v_kw = (uvms.wTv(1:3,1:3))' * w_kw;
% ---- misalignment vector axis k of the world frame wrt v
uvms.phi   = ReducedVersorLemma(v_kw, kv); % only for k
if (norm(uvms.phi) > 0)
    nphi = uvms.phi/norm(uvms.phi); % unit vector
else
    nphi = [0 0 0]';
end
uvms.Jha = [zeros(1,7) nphi'*[zeros(3) eye(3)]]; % 1x13
 
%% Minimum altitude Jacobian
% only the z position of the vehicles counts
uvms.Jminalt = [zeros(1,7) uvms.wTv(3,1:3) zeros(1,3)];

%% Altitude Jacobian
% only the z position of the vehicles counts
uvms.Jalt = [zeros(1,7) uvms.wTv(3,1:3) zeros(1,3)];

%% Preferred arm posture Jacobian
% uvms.Jarmposture 

end