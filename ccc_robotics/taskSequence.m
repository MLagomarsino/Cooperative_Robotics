function [uvms] = taskSequence(uvms, mission, current_v)

    % main kinematic algorithm initialization
    % rhop order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    rhop = zeros(13,1);
    Qp = eye(13); 
        
    if nargin == 3 % additional input for implementing Arm-Vehicle Coordination Scheme
        % TPIK 2
        % Constrained vehicle velocity to a given value current_v
        [Qp, rhop] = iCAT_task(uvms.A.fixvehicle, uvms.Jfixvehicle, Qp, rhop, current_v,  0.0001,   0.01, 10);
    end
    
    % joint limit task
    uvms.A.jl_t = UpdateActivation(uvms.A.jl, mission, 1);
    [Qp, rhop] = iCAT_task(uvms.A.jl_t, uvms.Jjl, Qp, rhop, uvms.xdot.jl,  0.0001,   0.01, 10);
    % manipulability
    uvms.A.mu_t = UpdateActivation(uvms.A.mu, mission, 2);
    [Qp, rhop] = iCAT_task(uvms.A.mu_t, uvms.Jmu, Qp, rhop, uvms.xdot.mu, 0.000001, 0.0001, 10);
    % minimum altitude task
    uvms.A.minalt_t = UpdateActivation(uvms.A.minalt, mission, 3);
    [Qp, rhop] = iCAT_task(uvms.A.minalt_t, uvms.Jminalt, Qp, rhop, uvms.xdot.minalt, 0.0001, 0.01, 10);
    % horizontal attitude task
    uvms.A.ha_t = UpdateActivation(uvms.A.ha, mission, 4);
    [Qp, rhop] = iCAT_task(uvms.A.ha_t, uvms.Jha, Qp, rhop, uvms.xdot.ha, 0.0001,   0.01, 10);
    % vehicle position control task
    uvms.A.target_t = UpdateActivation(uvms.A.target, mission, 5);
    [Qp, rhop] = iCAT_task(uvms.A.target_t, uvms.Jvpos, Qp, rhop, uvms.xdot.target, 0.0001,   0.01, 10);
    % vehicle longitudinal alignment to the nodule
    uvms.A.la_t = UpdateActivation(uvms.A.la, mission, 6);
    [Qp, rhop] = iCAT_task(uvms.A.la_t, uvms.Jla, Qp, rhop, uvms.xdot.la, 0.0001,   0.01, 10);
    % altitude control objective
    uvms.A.alt_t = UpdateActivation(uvms.A.alt, mission, 7);
    [Qp, rhop] = iCAT_task(uvms.A.alt_t,   uvms.Jalt,   Qp, rhop, uvms.xdot.alt, 0.0001,   0.01, 10);
    % tool-frame position control task
    uvms.A.t_t = UpdateActivation(uvms.A.t, mission, 8);
    [Qp, rhop] = iCAT_task(uvms.A.t_t, uvms.Jt, Qp, rhop, uvms.xdot.t,  0.0001,   0.01, 10);
    % fix vehicle position
    uvms.A.fixvehicle_t = UpdateActivation(uvms.A.fixvehicle, mission, 9);
    [Qp, rhop] = iCAT_task(uvms.A.fixvehicle_t,    uvms.Jfixvehicle,    Qp, rhop, uvms.xdot.fixvehicle,  0.0001,   0.01, 10);
    % optimization control task
    uvms.A.opt_t = UpdateActivation(uvms.A.opt, mission, 10);
    [Qp, rhop] = iCAT_task(uvms.A.opt_t, uvms.Jopt, Qp, rhop, uvms.xdot.opt,  0.0001,   0.01, 10);
   
    % this task should be the last one
    [Qp, rhop] = iCAT_task(eye(13), eye(13), Qp, rhop, zeros(13,1),  0.0001,   0.01, 11);    
    
    % get the two variables for integration
    uvms.q_dot = rhop(1:7);
    uvms.p_dot = rhop(8:13);
end