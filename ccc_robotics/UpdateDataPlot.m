function [ plt ] = UpdateDataPlot( plt, uvms, t, loop, mission)

% this function samples the variables contained in the structure uvms
% and saves them in arrays inside the struct plt
% this allows to have the time history of the data for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script

plt.t(loop) = t;

plt.toolPos(:, loop) = uvms.wTt(1:3,4);
plt.toolOrient(:, loop) = rotm2eul(uvms.wTt(1:3,1:3),'XYZ');

plt.q(:, loop) = uvms.q;
plt.q_dot(:, loop) = uvms.q_dot;
plt.q_ddot(:, loop) = uvms.q_ddot;

plt.p(:, loop) = uvms.p;
plt.p_dot(:, loop) = uvms.p_dot;
plt.p_ddot(:, loop) = uvms.p_ddot;

%plt.xdot_jl(:, loop) = uvms.xdot.jl;
plt.xdot_mu(:, loop) = uvms.xdot.mu;
plt.xdot_t(:, loop) =  blkdiag(uvms.wTv(1:3,1:3), uvms.wTv(1:3,1:3))*uvms.xdot.t;

plt.a(1:7, loop) = diag(uvms.A.jl_t);   % joint limits
plt.a(8, loop) = uvms.A.mu_t;           % manipulability
plt.a(9, loop) = uvms.A.minalt_t(1,1);  % minimum altitude
plt.a(10, loop) = uvms.A.ha_t(1,1);     % horizontal attitude
plt.a(11,loop) = uvms.A.target_t(1,1);  % position vehicle
plt.a(12,loop) = uvms.A.target_t(4,4);  % orientation vehicle
plt.a(13,loop) = uvms.A.la_t(1,1);      % longitudinal alignment
plt.a(14,loop) = uvms.A.alt_t(1,1);     % landing
plt.a(15,loop) = uvms.A.t_t(1,1);       % tool
plt.a(16,loop) = uvms.A.fixvehicle_t(1,1); % fix vehicle
plt.a(17:20, loop) = diag(uvms.A.opt_t);   % preferred shape

plt.toolFrameError(:, loop) = uvms.toolFrameError;
plt.totalError(:, loop) = uvms.totalError;

plt.toolx(:,loop) = uvms.wTt(1,4);
plt.tooly(:,loop) = uvms.wTt(2,4);

plt.altitude(1, loop) = uvms.altitude;
plt.sensorDistance(1, loop) = uvms.sensorDistance; % add
plt.misalignment(:, loop) = uvms.misalignment;
plt.goalreached = (mission.task_completed == sum(mission.tasksPerPhase(mission.Nphases,:)));

plt.jointMax = uvms.jlmax;
plt.jointMin = uvms.jlmin;

end