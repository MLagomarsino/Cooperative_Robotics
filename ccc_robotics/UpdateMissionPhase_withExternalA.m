function [uvms, mission] = UpdateMissionPhase_withExternalA(uvms, mission)
    switch mission.phase
        % add policy for changing phase
        case 1  
        % Reach the vehicle desired position using Safe Waypoint Navition Action
            
            % External activation functions whose parameter is the norm of 
            %   the basic vector between the target and the vehicle frame
            %   to obtain a smooth transition between actions
            % external activation function of vehicle position control task != 0 for norm > 0.5
            uvms.Aexternal.target = IncreasingBellShapedFunction(0.5, 0.8, 0, 1, norm(uvms.vTtarget(1:3,4)));
            % external act.f. of landing = 0 for norm > 0.5
            uvms.Aexternal.alt = DecreasingBellShapedFunction(0.5, 0.8, 0, 1, norm(uvms.vTtarget(1:3,4)));

            if norm(uvms.vTtarget(1:3,4)) < 0.5
                mission.phase = 2; % Landing
            end
        case 2
        % Landing
            uvms.Aexternal.target = 0;
            uvms.Aexternal.alt = 1;
    end
end