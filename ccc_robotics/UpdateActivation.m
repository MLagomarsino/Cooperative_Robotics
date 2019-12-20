function [currentA] = UpdateActivation(A, mission, task)
    % [newA] = UpdateActivation(A, mission, task)
    %       A = activation function for a given task
    %       mission = structure containing info about the mission
    %       task = number identifying the selected task
    
    
    % Current state of the selected task 
    %   1 -> active;    0 -> inactive;
    current_state = mission.tasksPerPhase(mission.phase, task);

    % Initialization with state at time t
    A_t = current_state;
    
    % During a transition
    if mission.in_transition
        % Compute the end of the transition
        end_transition = mission.start_transition + mission.transition_interval;
        
        % State of the selected task at the previous step
        previous_state = mission.tasksPerPhase(mission.phase-1, task);
    
        % transition task inactive -> active
        if previous_state < current_state
            A_t = IncreasingBellShapedFunction(mission.start_transition,end_transition,0,1,mission.current_time);
        % transition task active -> inactive
        elseif  previous_state > current_state
            A_t = DecreasingBellShapedFunction(mission.start_transition,end_transition,0,1,mission.current_time);
        end
    end
    % Update activation function
    currentA = A * A_t;
end