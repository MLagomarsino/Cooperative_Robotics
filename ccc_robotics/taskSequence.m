function [Qp, rhop, uvms] = taskSequence(uvms, mission)

    % main kinematic algorithm initialization
    % rhop order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    rhop = zeros(13,1);
    Qp = eye(13); 
    
    % minimum altitude task
    uvms.A.minalt = UpdateActivation(uvms.A.minalt, mission, 1);
    [Qp, rhop] = iCAT_task(uvms.A.minalt,   uvms.Jminalt,   Qp, rhop, uvms.xdot.minalt, 0.0001, 0.01, 10);
    % horizontal attitude task
    uvms.A.ha = UpdateActivation(uvms.A.ha, mission, 2);
    [Qp, rhop] = iCAT_task(uvms.A.ha,   uvms.Jha,   Qp, rhop, uvms.xdot.ha, 0.0001,   0.01, 10);
    % vehicle position control task
    uvms.A.target = UpdateActivation(uvms.A.target, mission, 3);
    [Qp, rhop] = iCAT_task(uvms.A.target, uvms.Jvpos, Qp, rhop, uvms.xdot.target, 0.0001,   0.01, 10);
    % vehicle longitudinal alignment to the nodule
    uvms.A.la = UpdateActivation(uvms.A.la, mission, 4);
    [Qp, rhop] = iCAT_task(uvms.A.la, uvms.Jla, Qp, rhop, uvms.xdot.la, 0.0001,   0.01, 10);
    % altitude control objective
    uvms.A.alt = UpdateActivation(uvms.A.alt, mission, 5);
    [Qp, rhop] = iCAT_task(uvms.A.alt,   uvms.Jalt,   Qp, rhop, uvms.xdot.alt, 0.0001,   0.01, 10);

%     % joint limit task
%     uvms.A.target_t = updateActivation(uvms.A.target, mission, 3);
%     [Qp, rhop] = iCAT_task(uvms.A.jl_t,    uvms.Jjl,    Qp, rhop, uvms.xdot.jl,  0.0001,   0.01, 10);
%     % tool-frame position control task
%     uvms.A.target_t = updateActivation(uvms.A.target, mission, 3);
%     [Qp, rhop] = iCAT_task(uvms.A.t_t,    uvms.Jt,    Qp, rhop, uvms.xdot.t,  0.0001,   0.01, 10);
%     % manipulability
%     uvms.A.target_t = updateActivation(uvms.A.target, mission, 3);
%     [Qp, rhop] = iCAT_task(uvms.A.mu_t,   uvms.Jmu,   Qp, rhop, uvms.xdot.mu, 0.000001, 0.0001, 10);
%     % this task should be the last one
%     [Qp, rhop] = iCAT_task(eye(13),     eye(13),    Qp, rhop, zeros(13,1),  0.0001,   0.01, 10);    
end