%% Pre-Load Before Starting 
load('Values_New.mat')
Bus_Architecture()

%% Begin Script
Sim_Time = 15;                               % Initialize Time

% Initial States
x0.x = 0;
x0.theta1 = -.1;
x0.theta2 = -.15;
x0.x_dot = 0;
x0.theta1_dot = 0;
x0.theta2_dot = 0;

% Reference Input
v.one = 0;
v.two = 0;

%% Just Controller
sim('Two_Input_Model.slx',Sim_Time)
logsout = ans.logsout;

% Plot state vs time
figure(1)
hold on
sgtitle('States of the System')
subplot(3,1,1)
hold on
plot(logsout{1}.Values.Time',logsout{1}.Values.Data)
hold off
xlabel('Time (sec)')
ylabel('x [m]')
grid
subplot(3,1,2)
hold on
plot(logsout{2}.Values.Time',logsout{2}.Values.Data*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_1$[degrees]','Interpreter','latex')
grid
subplot(3,1,3)
hold on
plot(logsout{3}.Values.Time',logsout{3}.Values.Data*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_2$ [degrees]','Interpreter','latex')
grid
hold off

% Plot u vs time
figure(2)
hold on
sgtitle('Inputs of the System')
subplot(2,1,1)
plot(logsout{4}.Values.Time',logsout{4}.Values.Data)
xlabel('Time (sec)')
ylabel('Force [kg*m/s^2]')
subplot(2,1,2)
plot(logsout{5}.Values.Time',logsout{5}.Values.Data)
xlabel('Time (sec)')
ylabel('Torque [kg*(m/s)^2]')
grid
hold off

%% Controller-Observer
x0.theta1 = -.08;
x0.theta2 = -.12;
sim('Two_Input_Model_CO.slx',Sim_Time)
logsout = ans.logsout;

close all;
% Plot state vs time
figure(1)
hold on
sgtitle('States of the System')
subplot(3,1,1)
hold on
plot(logsout{1}.Values.Time',logsout{1}.Values.Data)
plot(logsout{1}.Values.Time',logsout{6}.Values.Data)
hold off
xlabel('Time (sec)')
ylabel('x [m]')
grid
subplot(3,1,2)
hold on
plot(logsout{2}.Values.Time',logsout{2}.Values.Data*180/pi)
plot(logsout{2}.Values.Time',logsout{7}.Values.Data*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_1$[degrees]','Interpreter','latex')
grid
subplot(3,1,3)
hold on
plot(logsout{3}.Values.Time',logsout{3}.Values.Data*180/pi)
plot(logsout{3}.Values.Time',logsout{8}.Values.Data*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_2$ [degrees]','Interpreter','latex')
grid
legend('States','Estimates')
hold off

% Plot u vs time
figure(2)
hold on
sgtitle('Inputs of the System')
subplot(2,1,1)
plot(logsout{4}.Values.Time',logsout{4}.Values.Data)
xlabel('Time (sec)')
ylabel('Force [kg*m/s^2]')
subplot(2,1,2)
plot(logsout{5}.Values.Time',logsout{5}.Values.Data)
xlabel('Time (sec)')
ylabel('Torque [kg*(m/s)^2]')
grid
hold off