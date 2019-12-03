function [uvms, mission] = UpdateMissionPhase(uvms, mission)

    if ~mission.task_completed
        % Check if the transition is finished
        if mission.in_transition       
            if mission.current_time >= mission.end_transition
                fprintf(2,"---------------- End of transition ----------------");
                mission.in_transition = 0;
                mission.start_transition = 0;
                mission.end_transition = 0;
            end
        else
        % Check if there is a transition

            % Callback to check if the end condition is met
            exit_phase = mission.exit_conditions{mission.phase};

            if exit_phase(uvms)
                if (mission.phase == mission.Nphases)
                    fprintf(2,"---------------- Goal is reached ----------------\n");
                    mission.task_completed = 1;
                else
                    fprintf(2,"---------------- Start transition ----------------");
                    mission.phase = mission.phase + 1;
                    mission.in_transition = 1;
                    mission.start_transition = mission.current_time;
                    mission.end_transition = mission.start_transition + mission.transition_interval;    
                end
            end
        end
    end
end

