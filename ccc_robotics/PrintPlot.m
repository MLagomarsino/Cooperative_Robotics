function [ ] = PrintPlot(plt)

figure(1);
sgtitle('\fontsize{16}Arm')
subplot(2,1,1);
hold on    
hplot = plot(plt.t, plt.q);
hold off
set(hplot, 'LineWidth', 1.5);
% for j = 1:7
%     yline(plt.jointMax(j,1),'-','jl_{max}','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle','FontSize',14);
%     yline(plt.jointMin(j,1),'-','jl_{min}','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle','FontSize',14);
% end
for i = 1:plt.Nphases
    xline(plt.change_phase(i), '--r', ['Phase ',num2str(i)]);
end
if plt.goalreached
    xline(plt.change_phase(plt.Nphases+1), '--r', 'Goal reached');
end
% joint limit
% yline(plt.jointMin(6,1),'-','$jl_{6min}$','LabelHorizontalAlignment', 'left','LabelVerticalAlignment', 'middle','interpreter','latex','FontSize',14);
% yline(plt.jointMin(6,1)+0.6,'-','$jl_{6min}+\Delta$','LabelHorizontalAlignment', 'left','LabelVerticalAlignment', 'middle','interpreter','latex','FontSize',14);
% preferred shape
% preferred_shape = [-0.0031 1.2586 0.0128 -1.2460]';
% yline(preferred_shape(1),'-','$\bar{q}_1\ \ \bar{q}_3$','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle','interpreter','latex','FontSize',14);
% yline(preferred_shape(2),'-','$\bar{q}_2$','interpreter','latex','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle','FontSize',14);
% yline(preferred_shape(4),'-','$\bar{q}_4$','interpreter','latex','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle','FontSize',14);
box on
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

% -------------------------------------------------------------------------
figure(2);
sgtitle('\fontsize{16}Vehicle')
subplot(2,1,1);
hplot = plot(plt.t, plt.p);
set(hplot, 'LineWidth', 1.5);
for i = 1:plt.Nphases
    xline(plt.change_phase(i), '--r', ['Phase ',num2str(i)]);
end
if plt.goalreached
    xline(plt.change_phase(plt.Nphases+1), '--r', 'Goal reached');
end
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
legend(hplot(1:6),{'$\dot{x}$', '$\dot{y}$','$\dot{z}$','$\omega_x$','$\omega_y$','$\omega_z$'},'interpreter','latex','FontSize',16);
xlabel('Time(s)','FontSize',14),ylabel({'Linear (m/s) and','angular (rad/s) velocities'},'FontSize',14);
title('\fontsize{16}Linear and angular velocities of the vehicle');

% -------------------------------------------------------------------------
figure(3);
hplot = plot(plt.t, plt.a(1:7,:));
set(hplot,{'LineWidth'}, {2,2,2,2,2,2,2}', {'LineStyle'}, {'-','-','-','-','-','-','-'}');
for i = 1:plt.Nphases
    xline(plt.change_phase(i), '--r', ['Phase ',num2str(i)],'LabelVerticalAlignment', 'middle','FontSize',12);
end
if plt.goalreached
    xline(plt.change_phase(plt.Nphases+1), '--r', 'Goal reached','LabelVerticalAlignment', 'middle','FontSize',12);
end
legend({'Ajl_1','Ajl_2','Ajl_3','Ajl_4','Ajl_5','Ajl_6','Ajl_7'},'FontSize',16);
xlabel('Time(s)','FontSize',14),ylabel('Activation value','FontSize',14);
title('\fontsize{16}Activation function Joint limit task');
ylim([0 1])


% -------------------------------------------------------------------------
figure(11);
hplot = plot(plt.t, plt.a(17:20,:));
set(hplot,{'LineWidth'}, {4,2,2,2}', {'LineStyle'}, {'--','-','-','--'}');
for i = 1:plt.Nphases
    xline(plt.change_phase(i), '--r', ['Phase ',num2str(i)],'LabelVerticalAlignment', 'middle','FontSize',12);
end
if plt.goalreached
    xline(plt.change_phase(plt.Nphases+1), '--r', 'Goal reached','LabelVerticalAlignment', 'middle','FontSize',12);
end
legend({'Aoptq_1', 'Aoptq_2', 'Aoptq_3', 'Aoptq_4'},'FontSize',16);
xlabel('Time(s)','FontSize',14),ylabel('Activation value','FontSize',14);
title('\fontsize{16}Activation function of arm preferred shape');
ylim([0 1])

% -------------------------------------------------------------------------
figure(4);
hplot = plot(plt.t, plt.a([8 9 10 11 12 13 14 15 16],:));
set(hplot,{'LineWidth'}, {2,2,2,2,2,2,2,2,2}', {'LineStyle'}, {'-','-','-','-','-','-','-','-','-'}');
legend({'Amu','AminAlt','Aha','AtargetPos','AtargetOrient','AlongAlign','Aalt','At','AfixVehicle'},'interpreter','latex','FontSize',16);
title('\fontsize{16}Activation functions'); 
xlabel('Time(s)','FontSize',14),ylabel('Activation value','FontSize',14);
ylim([0 1])

% -------------------------------------------------------------------------
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
% yline(plt.targetRotation(1),'-','roll_{desired}','LabelHorizontalAlignment', 'left','LabelVerticalAlignment', 'middle','FontSize',14); % horizontal line -> initial x
% yline(plt.targetRotation(2),'-','pitch_{desired}','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle','FontSize',14); % horizontal line -> initial y
% yline(plt.targetRotation(3),'-','yaw_{desired}','LabelHorizontalAlignment', 'right','LabelVerticalAlignment', 'middle','FontSize',14); % horizontal line -> initial z
legend({'roll','pitch','yaw'},'interpreter','latex','FontSize',16);
title('\fontsize{16}Orientation of the vehicle'); 
xlabel('Time(s)','FontSize',14),ylabel('Angles(rad)','FontSize',14)

% -------------------------------------------------------------------------
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
yline(plt.targetPosition(1),'-','x_{desired}','LabelHorizontalAlignment', 'left','LabelVerticalAlignment', 'middle','FontSize',14);
yline(plt.targetPosition(2),'-','y_{desired}','LabelHorizontalAlignment', 'left','LabelVerticalAlignment', 'middle','FontSize',14);
yline(plt.targetPosition(3),'-','z_{desired}','LabelHorizontalAlignment', 'left','LabelVerticalAlignment', 'middle','FontSize',14);
legend({'x','y','z'},'interpreter','latex','FontSize',16);
title('\fontsize{16}Position of the vehicle'); 
xlabel('Time(s)','FontSize',14),ylabel('Position(m)','FontSize',14)

% -------------------------------------------------------------------------
figure(7);
hplot = plot(plt.t(15:end), plt.altitude(15:end)); set(hplot, 'LineWidth', 1.5);
for i = 1:plt.Nphases
    xl = xline(plt.change_phase(i), '--r', ['Phase ',num2str(i)],'FontSize',12);
    xl.LabelVerticalAlignment = 'middle';
end
if plt.goalreached
    xl = xline(plt.change_phase(plt.Nphases+1), '--r', 'Goal reached','FontSize',12);
    xl.LabelVerticalAlignment = 'middle';
end
% yline(plt.minAltitude,'-','minAltitude','LabelHorizontalAlignment', 'center','LabelVerticalAlignment', 'middle','FontSize',14);
title('\fontsize{16}Altitude'); 
xlabel('Time(s)','FontSize',14),ylabel('Altitude(m)','FontSize',14)
% ylim([0,12])

% -------------------------------------------------------------------------
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

% -------------------------------------------------------------------------
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
% horizontal lines -> Desired position
yline(plt.goalPosition(1),'-','x_{desired}','LabelHorizontalAlignment', 'left','LabelVerticalAlignment', 'middle','FontSize',14);
yline(plt.goalPosition(2),'-','y_{desired}','LabelHorizontalAlignment', 'left','LabelVerticalAlignment', 'middle','FontSize',14);
yline(plt.goalPosition(3),'-','z_{desired}','LabelHorizontalAlignment', 'left','LabelVerticalAlignment', 'middle','FontSize',14);
% yline(plt.goalOrientation(1),'-','roll_{desired}','LabelHorizontalAlignment', 'left','LabelVerticalAlignment', 'middle','FontSize',14);
% yline(plt.goalOrientation(2),'-','yaw_{desired}','LabelHorizontalAlignment', 'left','LabelVerticalAlignment', 'middle','FontSize',14);
% yline(plt.goalOrientation(3),'-','pitch_{desired}','LabelHorizontalAlignment', 'left','LabelVerticalAlignment', 'middle','FontSize',14);
legend({'x','y','z'},'interpreter','latex','FontSize',16);
title('\fontsize{16}Position of the tool wrt world frame'); 
xlabel('Time(s)','FontSize',14),ylabel('Position(m)','FontSize',14)

end

