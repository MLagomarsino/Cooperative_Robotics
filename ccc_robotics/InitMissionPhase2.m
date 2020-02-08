function [mission] = InitMissionPhase2(exercise)
    
    % Definition of the mission (fixed)
    mission.transition_interval = 5; % interval of the transition between 2 phases
    mission.Nobjectives = 11;        % total number of tasks 

    % Time variables (changing during execution)
    mission.current_time = 0;
    mission.phase = 1;
    mission.task_completed = 0;
    mission.goal_reached = 0;
    
    mission.in_transition = 0;
    mission.start_transition = 0;
    mission.end_transition = 0;

    switch(exercise)
        case '1.1'
            mission.Nphases = 1;    % number of phases
            % matrix identifying the active tasks in each phase
            mission.tasksPerPhase = zeros(mission.Nphases, mission.Nobjectives);
            mission.exit_conditions = cell(mission.Nphases,1);
            % active tasks 
            mission.tasksPerPhase(1,:) = [0 0 0 1 1 0 0 0 0 0 1]; % Ex1.1
            % exit condition from first phase
            mission.exit_conditions(1) = {@exit_phase_target}; % callback
        case '1.2'
            mission.Nphases = 1;
            mission.tasksPerPhase = zeros(mission.Nphases, mission.Nobjectives);
            mission.exit_conditions = cell(mission.Nphases,1);
            mission.tasksPerPhase(1,:) = [0 0 1 1 1 0 0 0 0 0 1];
            mission.exit_conditions(1) = {@exit_phase_target};
        case '2.1'
            mission.Nphases = 1;
            mission.tasksPerPhase = zeros(mission.Nphases, mission.Nobjectives);
            mission.exit_conditions = cell(mission.Nphases,1);
            mission.tasksPerPhase(1,:) = [0 0 0 1 0 0 1 0 0 0 1];
            mission.exit_conditions(1) = {@exit_phase_landing};
        case '2.2'
            mission.Nphases = 2;
            mission.tasksPerPhase = zeros(mission.Nphases, mission.Nobjectives);
            mission.exit_conditions = cell(mission.Nphases,1);
            % First phase
            mission.tasksPerPhase(1,:) = [0 0 0 1 1 0 0 0 0 0 1];
            mission.exit_conditions(1) = {@exit_phase_target};
            % Second phase
            mission.tasksPerPhase(2,:) = [0 0 0 1 0 0 1 0 0 0 1];
            mission.exit_conditions(2) = {@exit_phase_landing};
        case '3'
            mission.Nphases = 2;
            mission.tasksPerPhase = zeros(mission.Nphases, mission.Nobjectives);
            mission.exit_conditions = cell(mission.Nphases,1);
            % First phase
            mission.tasksPerPhase(1,:) = [0 0 0 1 1 0 0 0 0 0 1];
            mission.exit_conditions(1) = {@exit_phase_target};
            % Second phase
            mission.tasksPerPhase(2,:) = [0 0 0 1 0 1 1 0 0 0 1];
            mission.exit_conditions(2) = {@exit_phase_landing_withAlignment};
        case '3.1.4'
            mission.Nphases = 3;
            mission.tasksPerPhase = zeros(mission.Nphases, mission.Nobjectives);
            mission.exit_conditions = cell(mission.Nphases,1);
            % First phase
            mission.tasksPerPhase(1,:) = [0 0 0 1 1 0 0 0 0 0 1];
            mission.exit_conditions(1) = {@exit_phase_target};
            % Second phase
            mission.tasksPerPhase(2,:) = [0 0 0 1 0 1 1 0 0 0 1];
            mission.exit_conditions(2) = {@exit_phase_landing_withAlignment};
            % Third phase
            mission.tasksPerPhase(3,:) = [0 0 0 1 0 1 1 1 0 0 1];
            % mission.tasksPerPhase(3,:) = [0 0 0 0 0 0 0 1 0 0 1];
            mission.exit_conditions(3) = {@exit_endeffector_pos};
        case '4.1'
            mission.Nphases = 3;
            mission.tasksPerPhase = zeros(mission.Nphases, mission.Nobjectives);
            mission.exit_conditions = cell(mission.Nphases,1);
            % First phase
            mission.tasksPerPhase(1,:) = [0 0 0 1 1 0 0 0 0 0 1];
            mission.exit_conditions(1) = {@exit_phase_target};
            % Second phase
            mission.tasksPerPhase(2,:) = [0 0 0 1 0 1 1 0 0 0 1];
            mission.exit_conditions(2) = {@exit_phase_landing};
            % Third phase
            mission.tasksPerPhase(3,:) = [0 0 0 0 0 0 0 1 1 0 1];
            mission.exit_conditions(3) = {@exit_endeffector_pos};
        case '4.2'
            mission.Nphases = 3;
            mission.tasksPerPhase = zeros(mission.Nphases, mission.Nobjectives);
            mission.exit_conditions = cell(mission.Nphases,1);
            % First phase
            mission.tasksPerPhase(1,:) = [0 0 0 1 1 0 0 0 0 0 1];
            mission.exit_conditions(1) = {@exit_phase_target};
            % Second phase
            mission.tasksPerPhase(2,:) = [0 0 0 1 0 1 1 0 0 0 1];
            mission.exit_conditions(2) = {@exit_phase_landing};
            % Third phase
            mission.tasksPerPhase(3,:) = [1 1 0 0 0 0 0 1 1 0 1];
            mission.exit_conditions(3) = {@exit_endeffector_pos};
        case '5.1'
            mission.Nphases = 1;
            mission.tasksPerPhase = zeros(mission.Nphases, mission.Nobjectives);
            mission.exit_conditions = cell(mission.Nphases,1);
            mission.tasksPerPhase(1,:) = [1 1 0 1 0 0 0 1 0 1 1];
            mission.exit_conditions(1) = {@exit_endeffector_pos};
        case '5.2'
            mission.Nphases = 2;
            mission.tasksPerPhase = zeros(mission.Nphases, mission.Nobjectives);
            mission.exit_conditions = cell(mission.Nphases,1);
            % First phase
            mission.tasksPerPhase(1,:) = [0 0 0 1 1 0 0 0 0 0 1];
            mission.exit_conditions(1) = {@exit_phase_target};
            % Second phase
            mission.tasksPerPhase(2,:) = [1 1 0 1 0 0 0 1 1 1 1];
            mission.exit_conditions(2) = {@exit_endeffector_pos};
        case '6'
            mission.Nphases = 2;
            mission.tasksPerPhase = zeros(mission.Nphases, mission.Nobjectives);
            mission.exit_conditions = cell(mission.Nphases,1);
            % First phase
            mission.tasksPerPhase(1,:) = [0 0 0 1 1 0 0 0 0 0 1];
            mission.exit_conditions(1) = {@exit_phase_target};
            % Second phase
            mission.tasksPerPhase(2,:) = [1 1 0 1 0 0 0 1 0 1 1]; % check!
            mission.exit_conditions(2) = {@exit_endeffector_pos}; % check!
        otherwise
            disp(['Exercise ',exercise,' doesn''t exist'])
            exercise = input('Enter the exercise: ','s');
            mission = InitMissionPhase2(exercise);
    end
end
% Callback for exiting the target reaching phase for the vehicle position
function [output] = exit_phase_target(uvms)
    [rho, basic_vector] = CartError(uvms.wTv, uvms.wTtarget); 
    cond1 = (norm(rho) <= 0.3); % orientation
    cond2 = (norm(basic_vector) <= 0.3); % position
    output = cond1 && cond2;
end
% Callback for exiting phase: landing on the seafloor
function [output] = exit_phase_landing(uvms)
    cond1 = (uvms.t >= 1); % sensor provides strange measurements at the beginning
    cond2 = (uvms.altitude <= 0.2); % distance from the seafloor
    output = cond1 && cond2;
end
% Callback for exiting phase: landing on the seafloor aligning to the nodule 
function [output] = exit_phase_landing_withAlignment(uvms)
    cond1 = (uvms.altitude <= 0.15); % distance from the seafloor
    cond2 = (norm(uvms.misalignment) <= 0.1); % longitudinal alignment to the nodule
    output = cond1 && cond2;
end
% Callback for exiting phase: move end effector to a target position
function [output] = exit_endeffector_pos(uvms)
    [rho, basic_vector] = CartError(uvms.wTg, uvms.wTt); 
    cond1 = (norm(rho) <= 0.4); % orientation
    cond2 = (norm(basic_vector) <= 0.2); % position
    output = cond1 && cond2;
end