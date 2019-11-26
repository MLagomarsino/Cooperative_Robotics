function [ ] = PrintPlot( plt )

% some predefined plots
% you can add your own

figure(1);
sgtitle('Arm')
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
title('Joint vector')
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
title('Joint velocity vector')


figure(2);
sgtitle('Vehicle')
subplot(3,1,1);
hplot = plot(plt.t, plt.p);
set(hplot, 'LineWidth', 1);
legend('x','y','z','roll','pitch','yaw');
title('Position and orientation of the vehicle');
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot);
set(hplot, 'LineWidth', 1);
legend('xdot', 'ydot','zdot','omega_x','omega_y','omega_z');
title('Linear and angular velocities of the vehicle');

figure(3);
hplot = plot(plt.t, plt.a(1:7,:));
set(hplot, 'LineWidth', 2);
legend('Ajl_11','Ajl_22','Ajl_33','Ajl_44','Ajl_55','Ajl_66','Ajl_77');

figure(4);
hplot = plot(plt.t, plt.a([9 10 12],:));
set(hplot, 'LineWidth', 2);
legend('Aha', 'Atarget', 'Aalt');%, 'Aalt');
title('Activation functions');

figure(5);
hplot = plot(plt.t, plt.p(4:6,:));
set(hplot, 'LineWidth', 1);
legend('roll','pitch','yaw');
title('Orientation of the vehicle');

figure(6);
hplot = plot(plt.t, plt.p(1:3,:));
set(hplot, 'LineWidth', 1);
legend('x','y','z');
title('Position of the vehicle');

figure(7);
hplot = plot(plt.t, plt.altitude);
set(hplot, 'LineWidth', 1);
title('Altitude');

end

