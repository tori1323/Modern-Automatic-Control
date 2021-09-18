%% Pre-Load Before Starting 
load('Values.mat')
Bus_Architecture()

%% Begin Script
Sim_Time = 60;                               % Initialize Time

% Initial States
x0.x = 0;
x0.theta1 = -.01;
x0.theta2 = -.02;
x0.x_dot = 0;
x0.theta1_dot = 0;
x0.theta2_dot = 0;


sim('Simulink_File.slx',Sim_Time)
logsout = ans.logsout;
%% Plotting Data
% Plot state vs time
hold on
sgtitle('States of the System')
subplot(3,1,1)
plot(logsout{3}.Values.Time',logsout{3}.Values.Data)
xlabel('Time (sec)')
ylabel('x [m]')
%set(gca,'Fontsize',font_size)
subplot(3,1,2)
plot(logsout{1}.Values.Time',logsout{1}.Values.Data*180/pi)
xlabel('Time (sec)')
ylabel('$\theta_1$[degrees]','Interpreter','latex')
%set(gca,'Fontsize',font_size)
subplot(3,1,3)
plot(logsout{2}.Values.Time',logsout{2}.Values.Data*180/pi)
xlabel('Time (sec)')
ylabel('$\theta_2$ [degrees]','Interpreter','latex')
grid
%set(gca,'Fontsize',font_size)
hold off