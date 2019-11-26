function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1  
            % add policy for changing phase
            % Reach the vehicle desired position using Safe Waypoint
            % Navition Action
            % basic vector between the target and the vehicle frame
            if norm(uvms.vTtarget(1:3,4)) < 0.5
                mission.phase = 2; % Landing
            elseif norm(uvms.vTtarget(1:3,4)) >= 0.5
                % Active for a distance > 0.5
                uvms.Aexternal.target = IncreasingBellShapedFunction(0.5, 0.8, 0, 1, norm(uvms.vTtarget(1:3,4)));
                % deactive 
                uvms.Aexternal.alt = DecreasingBellShapedFunction(0.5, 0.8, 0, 1, norm(uvms.vTtarget(1:3,4)));
            end
        case 2
            uvms.Aexternal.target = 0;
            uvms.Aexternal.alt = 1;
    end
end

