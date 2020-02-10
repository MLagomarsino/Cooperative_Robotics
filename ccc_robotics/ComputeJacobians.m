function [uvms] = ComputeJacobians(uvms)
% The function computes the relevant Jacobians
%  - joint limits
%  - manipulability
%  - tool-frame position control
%  - vehicle-frame position control
%  - horizontal attitude 
%  - minimum altitude
%  - preferred arm posture ( [-0.0031 1.2586 0.0128 -1.2460] )
%
% the control vector is:
% [q_dot; p_dot] 
% [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]

%% Joint limit Jacobian
uvms.Jjl =  [eye(7) zeros(7,6)]; % 7 joints

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
% uvms.Jminalt = [0 0 1]*[zeros(3,7) eye(3) zeros(3,3)];

%% Altitude Jacobian
% only the z position of the vehicles counts
uvms.Jalt = [0 0 1]*[zeros(3,7) uvms.wTv(1:3,1:3) zeros(3,3)];

%% Alignment of longitudinal axis of the vehicle towards the nodule
rock_center = [12.2025   37.3748  -39.8860]';
% vector joining the vehicle frame to the nodule frame wrt w
w_vector_vn = rock_center - uvms.wTv(1:3,4);
% projected on v
v_vector_vn = (uvms.wTv(1:3,1:3))' * w_vector_vn;

% Projection matrix on inertial xy plane
Pxy = [1 0 0; 0 1 0; 0 0 0];

% projection on the inertial horizontal plane (third component = 0)
w_projection_vector_vn = Pxy * w_vector_vn;
% projection projected on v
v_projection_vector_vn = (uvms.wTv(1:3,1:3))' * w_projection_vector_vn;
% unit vector joining the vehicle frame to the nodule frame
v_uv_n = v_projection_vector_vn/norm(v_projection_vector_vn); % b

% vector axis i of the vehicle frame
iv = [1 0 0]'; % a

% Misalignment vector
uvms.misalignment = ReducedVersorLemma(v_uv_n, iv);

% % Misalignment vector projected on the w
% w_misalignment = (uvms.wTv(1:3,1:3))'*uvms.misalignment;

theta = norm(uvms.misalignment);
unit_n = uvms.misalignment/theta;

% Projection matrix on unit_n
Pn = unit_n * unit_n';

% Controlling the norm
% uvms.Jla = unit_n'*[zeros(3,7)  Pn*(1/(norm(v_vector_vn)^2))*skew(v_vector_vn)*Pxy  Pn]; % 1x13
% uvms.Jla = unit_n'*[zeros(3,7)  zeros(3)  Pn];
% uvms.Jla = unit_n'*[zeros(3,7)  Pn*(1/(norm(v_vector_vn)^2))*skew(v_vector_vn)*Pxy  zeros(3)];

% Controlling 3 degrees of freedom : wb/a = unit_n*lambda*theta
uvms.Jla = [zeros(3,7)  (1/(norm(v_vector_vn)^2))*skew(v_vector_vn)*Pxy  eye(3)]; % 3x13
% uvms.Jla = [zeros(3,7)  (1/(norm(v_vector_vn)^2))*skew(v_vector_vn)*Pxy zeros(3)]; % linear control
% uvms.Jla = [zeros(3,7)  zeros(3) eye(3)]; % angular control

%% Fixing vehicle velocity Jacobian
uvms.Jfixvehicle = [zeros(6,7) eye(6)];

%% Preferred arm posture Jacobian
uvms.Jopt =  [eye(4) zeros(4,3) zeros(4,6)];

end