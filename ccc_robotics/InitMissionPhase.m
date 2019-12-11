function [mission] = InitMissionPhase()
    
    % Definition of the mission (fixed)
    mission.transition_interval = 5; % interval of the transition between 2 phases
    mission.Nphases = 3;    % number of phases
    mission.Nobjectives = 8;     % total number of tasks 

    % Time variables (changing during execution)
    mission.current_time = 0;
    mission.phase = 1;
    mission.task_completed = 0;
    mission.goal_reached = 0;
    
    mission.in_transition = 0;
    mission.start_transition = 0;
    mission.end_transition = 0;

    % matrix identifying the active tasks in each phase
    mission.tasksPerPhase = zeros(mission.Nphases, mission.Nobjectives);
    mission.exit_conditions = cell(mission.Nphases,1);

    %% First phase
    % active tasks 
%     mission.tasksPerPhase(1,:) = [0 1 1 0 0 0 0 0]; % Ex2.2
    mission.tasksPerPhase(1,:) = [0 1 1 0 0 0 0 0]; % Ex3
    % exit condition from first phase
    mission.exit_conditions(1) = {@exit_phase_target}; % callback
    
    %% Second phase
    mission.tasksPerPhase(2,:) = [0 1 0 1 1 0 0 0];
    % mission.exit_conditions(2) = {@exit_phase_landing}; % Ex2.2
    mission.exit_conditions(2) = {@exit_phase_landing_withAlignment}; % Ex3
    
    %% Third phase
    % mission.tasksPerPhase(3,:) = [0 0 0 0 0 0 0 1]; % Ex3
    mission.tasksPerPhase(3,:) = [0 0 0 0 0 1 0 1]; % Ex4
    mission.exit_conditions(3) = {@exit_endeffector_pos}; % Ex3 & Ex4
end
% Callback for exiting the target reaching phase for the vehicle position
function [output] = exit_phase_target(uvms)
    output = norm(uvms.vTtarget(1:3,4)) < 0.5;
end

function [output] = exit_phase_landing(uvms)
    output = (uvms.altitude <= 0.1);
end

function [output] = exit_phase_landing_withAlignment(uvms)
    cond1 = (uvms.altitude <= 0.1);
    cond2 = (norm(uvms.misalignment) <= 0.1);
    output = cond1 && cond2;
end
% Callback for exiting phase: move end effector to a target position
function [output] = exit_endeffector_pos(uvms)
    cond1 = ((uvms.goalPosition - uvms.q(1:3))<= 0.1);
    % cond2 = ((uvms.**** - uvms.q(4:7))<= 0.1);
    output = cond1;
end