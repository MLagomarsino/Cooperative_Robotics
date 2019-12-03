function [mission] = InitMissionPhase()
    
    % Definition of the mission (fixed)
    mission.transition_interval = 5; % interval of the transition between 2 phases
    mission.Nphases = 2;    % number of phases
    mission.Nobjectives = 5;     % total number of tasks 

    % Time variables (changing during execution)
    mission.current_time = 0;
    mission.phase = 1;
    mission.task_completed = 0;
    
    mission.in_transition = 0;
    mission.start_transition = 0;
    mission.end_transition = 0;

    % matrix identifying the active tasks in each phase
    mission.tasksPerPhase = zeros(mission.Nphases, mission.Nobjectives);
    mission.exit_conditions = cell(mission.Nphases,1);

    %% First phase
    % active tasks 
    mission.tasksPerPhase(1,:) = [0 1 1 0 0]; % active task = 1
    % exit condition from first phase
    mission.exit_conditions(1) = {@exit_phase_1}; % callback
    
    %% Second phase
    mission.tasksPerPhase(2,:) = [0 1 0 0 1];
    mission.exit_conditions(2) = {@exit_phase_2};
end

function [output] = exit_phase_1(uvms)
%     cond = norm(uvms.vTtarget(1:3,4)) < 0.5;
%     output = cond;
    output = norm(uvms.vTtarget(1:3,4)) < 0.5;
end

function [output] = exit_phase_2(uvms)
    cond = (uvms.altitude <= 0.1);
    output = cond;
end