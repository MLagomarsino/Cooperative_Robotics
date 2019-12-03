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
uvms.Jminalt = [0 0 1]*[zeros(3,7) uvms.wTv(1:3,1:3) zeros(3,3)];

%% Altitude Jacobian
% only the z position of the vehicles counts
uvms.Jalt = [0 0 1]*[zeros(3,7) uvms.wTv(1:3,1:3) zeros(3,3)];

%% Alignment of longitudinal axis of the vehicle towards the nodule
rock_center = [12.2025   37.3748  -39.8860]';
% vector joining the vehicle frame to the nodule frame wrt w
basic_vector_vn = rock_center - uvms.wTv(1:3,4);
% projected on v
v_bv_n = (uvms.wTv(1:3,1:3))' * basic_vector_vn;
% projection on the inertial horizontal plane (third component = 0)
% v_bv_n(3) = 0;
v_bv_n = [1 0 0; 0 1 0; 0 0 0] * v_bv_n;
% unit vector joining the vehicle frame to the nodule frame
v_uv_n = v_bv_n/norm(v_bv_n);%b

% vector axis i of the vehicle frame
iv = [1 0 0]'; %a

% a^b
unit_n = cross(iv, v_uv_n);
unit_n = unit_n/norm(unit_n);

omega_bw = (norm(uvms.p_dot(1:3))/norm(basic_vector_vn)) * ...
            cross(uvms.p_dot(1:3),basic_vector_vn);
        
omega_aw = (norm(uvms.p_dot(1:3))/norm(uvms.wTv(1:3,4))) * ...
            cross(uvms.p_dot(1:3),uvms.wTv(1:3,4));
        
theta_dot = dot(unit_n, (omega_bw - omega_aw));
rho_derivative = unit_n * theta_dot;

uvms.Jla = [zeros(1,7) rho_derivative' eye(1,3)]; % 1x13
    
% ---- misalignment vector axis i of the vehicle frame wrt projection, 
% on the inertial horizontal plane, of the unit vector joining the vehicle 
% frame to the nodule frame.
% uvms.misalignment   = ReducedVersorLemma(v_uv_n, iv); % only for i
% if (norm(uvms.misalignment) > 0)
%     nmisalignment = uvms.misalignment/norm(uvms.misalignment); % unit vector
% else
%     nmisalignment = [0 0 0]';
% end
% uvms.Jla = [zeros(1,7) nmisalignment'*[zeros(3) eye(3)]]; % 1x13
 
%% Preferred arm posture Jacobian
% uvms.Jarmposture 

end