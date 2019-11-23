function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1  
            % add policy for changing phase
            % Reach the vehicle desired position using Safe Waypoint
            % Navition Action
            % basic vector between the target and the vehicle frame
            if norm(uvms.vTtarget(1:3,4)) < 0.5
                mission.phase = 2; % Landing
            end
    end
end

