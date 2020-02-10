function [plt] = InitDataPlot( maxloops, mission)
    plt.t = zeros(1, maxloops);
    plt.q = zeros(7, maxloops);
    plt.q_dot = zeros(7, maxloops);
    plt.q_ddot = zeros(7, maxloops);

    plt.p = zeros(6, maxloops);
    plt.p_dot = zeros(6, maxloops);
    plt.p_ddot = zeros(6, maxloops);

    plt.f = zeros(3,maxloops);
    plt.f_mod = zeros(1,maxloops);
    plt.m = zeros(3,maxloops);
    plt.m_mod = zeros(1,maxloops);

    plt.xdot_jl = zeros(7, maxloops);
    plt.xdot_mu = zeros(1, maxloops);
    plt.xdot_t = zeros(6, maxloops);

    plt.a = zeros(20, maxloops); %%%%%%%%%%%% Numero giusto di A

    plt.virtualFrameError = zeros(6, maxloops);
    plt.toolFrameError = zeros(6, maxloops);
    plt.totalError = zeros(6, maxloops);
    
    plt.altitude = zeros(1, maxloops);
    plt.misalignment = zeros(3, maxloops);
    plt.change_phase = zeros(mission.Nphases);
    plt.Nphases = mission.Nphases;
    % position of the tool wrt world
    plt.toolPos = zeros(3, maxloops);
    
end

