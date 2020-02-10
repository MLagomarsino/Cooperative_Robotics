function [mission] = InitMissionPhase(nPhases, activeTasks, exitCondition)
    
    % Definition of the mission (fixed)
    mission.transition_interval = 5; % interval of the transition between 2 phases
    mission.Nphases = nPhases;    % number of phases
    mission.Nobjectives = 11;     % total number of tasks 

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
    
    for i = 1:nPhases
        % active tasks 
        mission.tasksPerPhase(i,:) = [activeTasks(i,:) 1];
        % exit condition
        if (exitCondition(i) == 1)
            mission.exit_conditions(i) = {@exit_phase_target};
        elseif (exitCondition(i) == 2)
            mission.exit_conditions(i) = {@exit_phase_landing};
        elseif (exitCondition(i) == 3)
            mission.exit_conditions(i) = {@exit_phase_landing_withAlignment};
        else
            mission.exit_conditions(i) = {@exit_endeffector_pos};
        end
    end
end

%% Callback for exiting the target reaching phase for the vehicle position
function [output] = exit_phase_target(uvms)
    output = norm(uvms.vTtarget(1:3,4)) < 0.4;
end

function [output] = exit_phase_landing(uvms)
    cond1 = (uvms.t >= 1);
    cond2 = (uvms.altitude <= 0.2);
    output = cond1 && cond2;
end

function [output] = exit_phase_landing_withAlignment(uvms)
    cond1 = (uvms.altitude <= 0.15);
    cond2 = (norm(uvms.misalignment) <= 0.1);
    output = cond1 && cond2;
end
% Callback for exiting phase: move end effector to a target position
function [output] = exit_endeffector_pos(uvms)
    [rho, basic_vector] = CartError(uvms.wTg, uvms.wTt); 
    % orientation
    cond1 = (norm(rho) <= 0.4);
    % position
    cond2 = (norm(basic_vector) <= 0.2);
    output = cond1 && cond2;
end