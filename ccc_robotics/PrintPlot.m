function [ ] = PrintPlot( plt )

% some predefined plots
% you can add your own

figure(1);
sgtitle('\fontsize{16}Arm')
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
set(hplot, 'LineWidth', 1.5);
legend({'$q_1$','$q_2$','$q_3$','$q_4$','$q_5$','$q_6$','$q_7$'},'interpreter','latex','FontSize',16);
title('\fontsize{16}Joint vector'),
xlabel('Time(s)','FontSize',14),ylabel('Joint angle (rad)','FontSize',14)
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1.5);
for i = 1:plt.Nphases
    xline(plt.change_phase(i), '--r', ['Phase ',num2str(i)]);
end
if plt.goalreached
    xline(plt.change_phase(plt.Nphases+1), '--r', 'Goal reached');
end
legend(hplot(1:7),{'$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','$\dot{q}_4$','$\dot{q}_5$','$\dot{q}_6$','$\dot{q}_7$'},'interpreter','latex','FontSize',16);
title('\fontsize{16}Joint velocity vector'),
xlabel('Time(s)','FontSize',14),ylabel('Joint velocities(rad/s)','FontSize',14)

figure(2);
sgtitle('\fontsize{16}Vehicle')
subplot(3,1,1);
hplot = plot(plt.t, plt.p);
set(hplot, 'LineWidth', 1.5);
legend({'x','y','z','roll','pitch','yaw'},'interpreter','latex','FontSize',16);
title('\fontsize{16}Position and orientation of the vehicle');
xlabel('Time(s)','FontSize',14),ylabel({'Position(m) and',' orientation(rad)'},'FontSize',14)
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot);
set(hplot, 'LineWidth', 1.5);
for i = 1:plt.Nphases
    xline(plt.change_phase(i), '--r', ['Phase ',num2str(i)]);
end
if plt.goalreached
    xline(plt.change_phase(plt.Nphases+1), '--r', 'Goal reached');
end
legend(hplot(1:6),{'$\dot{x}$', '$\dot{y}_2$','$\dot{z}$','$\omega_x$','$\omega_y$','$\omega_z$'},'interpreter','latex','FontSize',16);
xlabel('Time(s)','FontSize',14),ylabel({'Linear (m/s) and','angular (rad/s) velocities'},'FontSize',14);
title('\fontsize{16}Linear and angular velocities of the vehicle');

figure(3);
hplot = plot(plt.t, plt.a(1:7,:));
set(hplot, 'LineWidth', 2);
legend({'Ajl_1','Ajl_2','Ajl_3','Ajl_4','Ajl_5','Ajl_6','Ajl_7'},'FontSize',16);
xlabel('Time(s)','FontSize',14),ylabel('Activation value','FontSize',14);
title('\fontsize{16}Activation functions Joint limit task');

figure(4);
hplot = plot(plt.t(17:end), plt.a([11 9 10],(17:end)));
% hplot = plot(plt.t, plt.a([8 9 15 16],:));
set(hplot, {'LineWidth'}, {2,2,2}', {'LineStyle'}, {'-','-','--'}');
%set(hplot, {'LineWidth'}, {2,2,2,2,2,4}', {'LineStyle'}, {'-','-','-','-','--','--'}');
legend({'Aminalt', 'Aha', 'Atarget'},'interpreter','latex','FontSize',16);
% legend({'Amu','Aha', 'Atarget','Afixvehicle', 'At', 'Aopt'},'interpreter','latex','FontSize',14);
% legend({'Amu', 'Aha', 'At', 'Aopt'},'FontSize',14);
title('\fontsize{16}Activation functions'); 
xlabel('Time(s)','FontSize',14),ylabel('Activation value','FontSize',14);
ylim([0 1])

figure(5);
hplot = plot(plt.t, plt.p(4:6,:));
set(hplot, 'LineWidth', 1.5);
for i = 1:plt.Nphases
    xl = xline(plt.change_phase(i), '--r', ['Phase ',num2str(i)],'FontSize',12);
    xl.LabelVerticalAlignment = 'middle';
end
if plt.goalreached
    xl = xline(plt.change_phase(plt.Nphases+1), '--r', 'Goal reached','FontSize',12);
    xl.LabelVerticalAlignment = 'middle';
end
yline(plt.targetRotation(1),'-','roll_{desired}','LabelHorizontalAlignment', 'left','LabelVerticalAlignment', 'middle','FontSize',14); % horizontal line -> initial x
yline(plt.targetRotation(2),'-','pitch_{desired}','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle','FontSize',14); % horizontal line -> initial y
yline(plt.targetRotation(3),'-','yaw_{desired}','LabelHorizontalAlignment', 'right','LabelVerticalAlignment', 'middle','FontSize',14); % horizontal line -> initial z
legend({'roll','pitch','yaw'},'interpreter','latex','FontSize',16);
title('\fontsize{16}Orientation of the vehicle'); 
xlabel('Time(s)','FontSize',14),ylabel('Angles(rad)','FontSize',14)
%ylim([-0.4 1.6])

figure(6);
hplot = plot(plt.t, plt.p(1:3,:));
set(hplot, 'LineWidth', 1.5);
for i = 1:plt.Nphases
    xl = xline(plt.change_phase(i), '--r', ['Phase ',num2str(i)],'FontSize',12);
    xl.LabelVerticalAlignment = 'middle';
end
if plt.goalreached
    xl = xline(plt.change_phase(plt.Nphases+1), '--r', 'Goal reached','FontSize',12);
    xl.LabelVerticalAlignment = 'middle';
end
% yline(plt.toolPos(1,1),'-','x_0','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle'); % horizontal line -> initial x
% yline(plt.toolPos(2,1),'-','y_0','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle'); % horizontal line -> initial y
% yline(plt.toolPos(3,1),'-','z_0','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle'); % horizontal line -> initial z
yline(plt.targetPosition(1),'-','x_{desired}','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle','FontSize',14); % horizontal line -> initial x
yline(plt.targetPosition(2),'-','y_{desired}','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle','FontSize',14); % horizontal line -> initial y
yline(plt.targetPosition(3),'-','z_{desired}','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle','FontSize',14); % horizontal line -> initial z
legend({'x','y','z'},'interpreter','latex','FontSize',16);
title('\fontsize{16}Position of the vehicle'); 
xlabel('Time(s)','FontSize',14),ylabel('Position(m)','FontSize',14)

figure(7);
hplot = plot(plt.t(17:end), plt.altitude(17:end)); set(hplot, 'LineWidth', 1.5);
yline(plt.minAltitude,'-','minAltitude','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle','FontSize',14);
title('\fontsize{16}Altitude'); 
xlabel('Time(s)','FontSize',14),ylabel('Altitude(m)','FontSize',14)
ylim([0,12])

figure(8);
for i = 1:size(plt.misalignment,2)
    norm_rho(i) = norm(plt.misalignment(:,i));
end
hplot = plot(plt.t, norm_rho);
set(hplot, 'LineWidth', 1.5);
title('\fontsize{16}Misalignment vector'); 
xlabel('Time(s)','FontSize',14),ylabel('Norm(m)','FontSize',14)
for i = 1:plt.Nphases
    xline(plt.change_phase(i), '--r', ['Phase ',num2str(i)],'FontSize',12);
end
if plt.goalreached
    xline(plt.change_phase(plt.Nphases+1), '--r', 'Goal reached','FontSize',12);
end

figure(9);
hplot = plot(plt.t, plt.toolPos);
set(hplot, 'LineWidth', 1.5);
for i = 1:plt.Nphases
    xl = xline(plt.change_phase(i), '--r', ['Phase ',num2str(i)],'FontSize',12);
    xl.LabelVerticalAlignment = 'middle';
end
if plt.goalreached
    xl = xline(plt.change_phase(plt.Nphases+1), '--r', 'Goal reached','FontSize',12);
    xl.LabelVerticalAlignment = 'middle';
end
% yline(plt.toolPos(1,1),'-','x_0','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle'); % horizontal line -> initial x
% yline(plt.toolPos(2,1),'-','y_0','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle'); % horizontal line -> initial y
% yline(plt.toolPos(3,1),'-','z_0','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle'); % horizontal line -> initial z
% horizontal lines -> Desired position
yline(plt.goalPosition(1),'-','x_{desired}','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle','FontSize',14);
yline(plt.goalPosition(2),'-','y_{desired}','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle','FontSize',14);
yline(plt.goalPosition(3),'-','z_{desired}','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle','FontSize',14);
legend({'x','y','z'},'interpreter','latex','FontSize',16);
title('\fontsize{16}Position of the tool wrt world frame'); 
xlabel('Time(s)','FontSize',14),ylabel('Position(m)','FontSize',14)

end

