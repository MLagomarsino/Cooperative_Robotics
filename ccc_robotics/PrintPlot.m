function [ ] = PrintPlot( plt )

% some predefined plots
% you can add your own

figure(1);
sgtitle('\fontsize{16}Arm')
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
set(hplot, 'LineWidth', 1.5);
legend({'q_1','q_2','q_3','q_4','q_5','q_6','q_7'},'FontSize',14);
title('\fontsize{16}Joint vector'),
xlabel('Time(s)','FontSize',14),ylabel('Joint angle (rad)','FontSize',14)
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1.5);
legend({'qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7'},'FontSize',14);
title('\fontsize{16}Joint velocity vector'),
xlabel('Time(s)','FontSize',14),ylabel('Joint velocities','FontSize',14)


figure(2);
sgtitle('\fontsize{16}Vehicle')
subplot(3,1,1);
hplot = plot(plt.t, plt.p);
set(hplot, 'LineWidth', 1.5);
legend({'x','y','z','roll','pitch','yaw'},'FontSize',14);
title('\fontsize{16}Position and orientation of the vehicle');
xlabel('Time(s)','FontSize',14),ylabel('Position(m) and orientation(rad)','FontSize',14)
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot);
set(hplot, 'LineWidth', 1.5);
legend({'xdot', 'ydot','zdot','omega_x','omega_y','omega_z'},'FontSize',14);
xlabel('Time(s)','FontSize',14),ylabel('Linear (m/s) and angular (rad/s) velocities','FontSize',14);
title('\fontsize{16}Linear and angular velocities of the vehicle');

figure(3);
hplot = plot(plt.t, plt.a(1:7,:));
set(hplot, 'LineWidth', 2);
legend({'Ajl_11','Ajl_22','Ajl_33','Ajl_44','Ajl_55','Ajl_66','Ajl_77'},'FontSize',14);
xlabel('Time(s)','FontSize',14),ylabel('??????Joint limit','FontSize',14)

figure(4);
hplot = plot(plt.t, plt.a([9 10 12 13],:));
set(hplot, 'LineWidth', 2);
legend({'Aha', 'Atarget', 'Aalt', 'Ala'},'FontSize',14);%, 'Aalt');
title('\fontsize{16}Activation functions'); 
xlabel('Time(s)','FontSize',14)

figure(5);
hplot = plot(plt.t, plt.p(4:6,:));
set(hplot, 'LineWidth', 1.5);
legend({'roll','pitch','yaw'},'FontSize',14);
title('\fontsize{16}Orientation of the vehicle'); 
xlabel('Time(s)','FontSize',14),ylabel('Angles(rad)','FontSize',14)

figure(6);
hplot = plot(plt.t, plt.p(1:3,:));
set(hplot, 'LineWidth', 1.5);
legend({'x','y','z'},'FontSize',14);
title('\fontsize{16}Position of the vehicle'); 
xlabel('Time(s)','FontSize',14),ylabel('Position(m)','FontSize',14)

figure(7);
hplot = plot(plt.t, plt.altitude);
set(hplot, 'LineWidth', 1.5);
title('\fontsize{16}Altitude'); 
xlabel('Time(s)','FontSize',14),ylabel('Altitude(m)','FontSize',14)

figure(8);
for i = 1:size(plt.misalignment,2)
    norm_rho(i) = norm(plt.misalignment(:,i));
end
hplot = plot(plt.t, norm_rho);
set(hplot, 'LineWidth', 1.5);
title('\fontsize{16}Misalignment vector'); 
xlabel('Time(s)','FontSize',14),ylabel('Norm(m)','FontSize',14)
xline(plt.change_phase(1), '--r', 'Phase 1');
xline(plt.change_phase(2), '--r', 'Phase 2');
xline(plt.change_phase(3), '--r', 'Phase 3');

end

