% Victoria Nagorski - ECE 680
% Version 1.0 - 9/25/2021
% FunWork 3

%% Problem 1
% Show there is no u_e for x_e
% Load Variables from previous part
clear; close all; clc;
load('Values.mat');                 % From Homework # 1
syms u

% Start problem 
f = simplify(f);                    % First simplify non-linear terms

theta1 = -60*pi/180;                % Convert state to radians
theta2 = -45*pi/180;                % Convert state to radians

x_e = [0.1;theta1;theta2;0;0;0];    % Create the x state
g = subs(f,x,x_e);                  % Plug in x_e

f_e = g == [0;0;0;0;0;0];           % Set the equation f(x_e,u_e) = 0
u_e = solve(f_e,u)

%% Problem 2
% Show there is no u_e for x_e
% Load Variables from previous part
clear; close all; clc;
load('Values_Rev2.mat');            % From Homework # 2
syms u

% Start problem 
theta1 = -60*pi/180;                % Convert state to radians
theta2 = -45*pi/180;                % Convert state to radians

x_e = [0.1;theta1;theta2;0;0;0];    % Create the x state
g = subs(f,x,x_e);                  % Plug in x_e

f_e = g == [0;0;0;0;0;0];           % Set the equation f(x_e,u_e) = 0
u_e = solve(f_e,u)

%% Problem 3
% Add Torque to second joint
clear; close all; clc;
syms u

% Problem contants
m1 = 0.5;                           % kg
l1 = 0.5;                           % m
m2 = 0.75;                          % kg
l2 = 0.75;                          % m
M = 1.5;                            % kg
g = 9.81;                           % m/sec^2

% Solve for matrix contants
r1 = M + m1 + m2;
r2 = (m1 + m2) * l1;
r3 = m2*l2;
r4 = (m1 + m2) * l1^2;
r5 = m2 * l1 * l2;
r6 = m2 * l2^2;
f1 = (m1 * l1 + m2 * l1) * g;
f2 = m2 * l2 * g;

% Define Non-Linear Matrices
syms x1 x2 x3 x4 x5 x6 u1 u2 u3
M = [  r1        r2*cos(x2)     r3*cos(x3);
    r2*cos(x2)     r4         r5*cos(x2-x3);
    r3*cos(x3) r5*cos(x2-x3)       r6];
C = [0 -r2*x5*sin(x2)    -r3*x6*sin(x3);
     0       0            r5*x6*sin(x2-x3);
     0 -r5*x5*sin(x2-x3)       0];
G = [      0;
    -f1 * sin(x2);
    -f2 * sin(x3)];
H = [1 0 0;0 1 -1;0 0 1];

% Solve for the Non-Linear Functions
matrix1 = [zeros(3,3)   eye(3);     % Break up equation
           zeros(3,3) -M^-1 * C];   
matrix2 = [zeros(3,1); -M^-1 * G];  % Break up equation
matrix3 = [zeros(3,3); M^-1 * H];   % Break up equation
x = [x1; x2; x3; x4; x5; x6];       % Define x vector
u = [u1; u2; u3];                   % Define state vector
f = simplify(matrix1 * x + matrix2 + matrix3 * u);

% Find the Equilibrium Input
theta1 = -60*pi/180;                % Convert state to radians
theta2 = -45*pi/180;                % Convert state to radians

x_e = [0.1;theta1;theta2;0;0;0];    % Create the x state
g = subs(f,x,x_e);                  % Plug in x_e

f_e = g == [0;0;0;0;0;0];           % Set the equation f(x_e,u_e) = 0

u_temp = struct2cell(solve(f_e,u)); % Currently in a struct
u_e = zeros(3,1);                   % Initialize variable
for i = 1:3
   u_e(i,1) = double(u_temp{i});
end
u_e                                 % Print out u_e for publishing

%% Problem 4
% Linearize around equilirbium points
% Linearize with the Jacobian
equil = [x_e;u_e];                  % Equilibrium point
a = jacobian(f,x);                  % Create Jacobian matrix for A
b = jacobian(f,u);                  % Create Jacobian matrix for B
A = double(subs(a,[x;u],equil))     % Plug in equilibrium points
B = double(subs(b,[x;u],equil))     % Plug in equilibrium points
C = [1 0 0 0 0 0;                   % Define the C matrix for Sys
     0 1 0 0 0 0;
     0 0 1 0 0 0] 
D = zeros(3,3)                      % Define the D matrix for Sys

% Check Observability and Controllability
Co = rank(ctrb(A,B))                % Controllability
O = rank(obsv(A,C))                 % Observability

% Save Matrices for Easy Access
save('Values_Rev3.mat','A','B','C','D','f','x')

%% Problem 5
% Comapare ODEs w/LMI controller
% Preliminaries
clear x
theta1 = -60*pi/180;                % Convert state to radians
theta2 = -45*pi/180;                % Convert state to radians
span = [0 3];
m = 3;
n = 6;

% Solve for Alpha
setlmis([]); 
p = lmivar(1,[6 1]);

lmiterm([1 1 1 0],1);               % P > I : I 
lmiterm([-1 1 1 p],1,1);            % P > I : P 
lmiterm([2 1 1 p],1,A,'s');     	% LFC (lhs) 
lmiterm([-2 1 1 p],1,1);            % LFC (rhs) 
lmis = getlmis;
[alpha,~]=gevp(lmis,1);
alpha

% LMI Solve
cvx_begin sdp quiet

% Variable definiton
variable S(n,n) symmetric
variable Z(m,n) 

% LMIs
A*S + S*A' - B*Z - Z'*B' + alpha*S <= -eps * eye(n)
S >= eps * eye(n)

cvx_end

K = Z*S^-1;                        % Solve for K matrix

% Simulate the Controller
clear x0
% Initial States
x0.x = 0;
x0.theta1 = -50*pi/180;
x0.theta2 = -40*pi/180;
x0.x_dot = 0;
x0.theta1_dot = 0;
x0.theta2_dot = 0;

% Reference Input
v.one = 0;
v.two = 0;
sim('Linear_Controller_Design.slx',3);
open_system('Linear_Controller_Design.slx')

% Pull out Data
data = ans.logsout{1}.Values.Data;
time = ans.logsout{1}.Values.Time';

% Plot State vs Time
figure
hold on
sgtitle('States of the Linear System')
subplot(6,1,1)
plot(time,data(:,1)')
xlabel('Time (sec)')
ylabel('x [m]')
grid
subplot(6,1,2)
plot(time,-data(:,2)'*180/pi)
xlabel('Time (sec)')
ylabel('$\theta_1$[degrees]','Interpreter','latex')
grid
subplot(6,1,3)
plot(time,-data(:,3)'*180/pi)
xlabel('Time (sec)')
ylabel('$\theta_2$ [degrees]','Interpreter','latex')
grid
subplot(6,1,4)
plot(time,data(:,4)')
xlabel('Time (sec)')
ylabel('$\dot{x}$ [m/s]','Interpreter','latex')
grid
subplot(6,1,5)
plot(time,-data(:,5)'*180/pi)
xlabel('Time (sec)')
ylabel('$\dot{\theta_1}$[degrees/s]','Interpreter','latex')
grid
subplot(6,1,6)
plot(time,-data(:,6)'*180/pi)
xlabel('Time (sec)')
ylabel('$\dot{\theta_2}$ [degrees/s]','Interpreter','latex')
grid
hold off

clear x0
x0 = [0;-50*pi/180;-40*pi/180;0;0;0];                       % Create the x state

% Compare odes
[t1,y1] = ode45(@(t1,y1) nonlinear(y1,x_e,u_e,K),span,x0);  % ODE45
[t2,y2] = ode23(@(t2,y2) nonlinear(y2,x_e,u_e,K),span,x0);  % ODE23

% Plot graphs
figure
hold on
sgtitle('States of the System')
subplot(6,1,1)
hold on
plot(t1,y1(:,1))
plot(t2,y2(:,1))
hold off
xlabel('Time (sec)')
ylabel('x [m]')
grid
subplot(6,1,2)
hold on
plot(t1,-y1(:,2)*180/pi)
plot(t2,-y2(:,2)*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_1$[degrees]','Interpreter','latex')
grid
subplot(6,1,3)
hold on
plot(t1,-y1(:,3)*180/pi)
plot(t2,-y2(:,3)*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_2$ [degrees]','Interpreter','latex')
grid
subplot(6,1,4)
hold on
plot(t1,y1(:,4))
plot(t2,y2(:,4))
hold off
xlabel('Time (sec)')
ylabel('$\dot{x}$ [m/s]','Interpreter','latex')
grid
subplot(6,1,5)
hold on
plot(t1,-y1(:,5)*180/pi)
plot(t2,-y2(:,5)*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\dot{\theta_1}$[degrees/s]','Interpreter','latex')
grid
subplot(6,1,6)
hold on
plot(t1,-y1(:,6)*180/pi)
plot(t2,-y2(:,6)*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\dot{\theta_2}$ [degrees/s]','Interpreter','latex')
legend('ODE45','ODE23')
grid
hold off

% Save Matrices for Easy Access
save('Values_Rev3.mat','A','B','C','D','f','K','x_e','u_e','alpha')

%% Problem 6
% Animate the DIPC - Simulation_Main.m
Bus_Architecture()                           % Load Bus Archicture
close all;

% Begin Script
Sim_Time = 15;                               % Initialize Time

clear x0
% Initial States
x0.x = 0;
x0.theta1 = -80*pi/180;
x0.theta2 = -60*pi/180;
x0.x_dot = 0;
x0.theta1_dot = 0;
x0.theta2_dot = 0;

% Run SImulation
sim('Three_Input_Model.slx',Sim_Time)
open_system('Three_Input_Model.slx')
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
plot(logsout{2}.Values.Time',-logsout{2}.Values.Data*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_1$[degrees]','Interpreter','latex')
grid
subplot(3,1,3)
hold on
plot(logsout{3}.Values.Time',-logsout{3}.Values.Data*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_2$ [degrees]','Interpreter','latex')
grid
hold off

%% Problem 7
% Output Feedback Control
p = 3;
clear K
close all;

% LMI Solve
cvx_begin sdp quiet

% Variable definiton
variable P(n,n) symmetric
variable N(m,p) 
variable M(m,m) 

% LMIs
P*A + A'*P - B*N*C - C'*N'*B' <= -eps*eye(n)
B*M == P*B
P >= eps*eye(n)

cvx_end

K = M^-1*N;                                                 % Solve for K matrix

% Initial States
clear x0
x0.x = 0;
x0.theta1 = -50*pi/180;
x0.theta2 = -40*pi/180;
x0.x_dot = 0;
x0.theta1_dot = 0;
x0.theta2_dot = 0;
sim('Linear_Controller_Design_Output.slx',3);
open_system('Linear_Controller_Design_Output.slx')

% Plot State vs Time
figure
hold on
sgtitle('States of the Linear System')
subplot(6,1,1)
plot(time,data(:,1)')
xlabel('Time (sec)')
ylabel('x [m]')
grid
subplot(6,1,2)
plot(time,-data(:,2)'*180/pi)
xlabel('Time (sec)')
ylabel('$\theta_1$[degrees]','Interpreter','latex')
grid
subplot(6,1,3)
plot(time,-data(:,3)'*180/pi)
xlabel('Time (sec)')
ylabel('$\theta_2$ [degrees]','Interpreter','latex')
grid
subplot(6,1,4)
plot(time,data(:,4)')
xlabel('Time (sec)')
ylabel('$\dot{x}$ [m/s]','Interpreter','latex')
grid
subplot(6,1,5)
plot(time,-data(:,5)'*180/pi)
xlabel('Time (sec)')
ylabel('$\dot{\theta_1}$[degrees/s]','Interpreter','latex')
grid
subplot(6,1,6)
plot(time,-data(:,6)'*180/pi)
xlabel('Time (sec)')
ylabel('$\dot{\theta_2}$ [degrees/s]','Interpreter','latex')
grid
hold off

clear y1 y2 t1 t2
clear x0
x0 = [0;-50*pi/180;-40*pi/180;0;0;0];                       % Create the x state

% Compare odes
[t1,y1] = ode45(@(t1,y1) nonlinear_output(y1,x_e,u_e,K,C),span,x0);  % ODE45
[t2,y2] = ode23(@(t2,y2) nonlinear_output(y2,x_e,u_e,K,C),span,x0);  % ODE23

% Plot graphs
figure
hold on
sgtitle('States of the System')
subplot(6,1,1)
hold on
plot(t1,y1(:,1))
plot(t2,y2(:,1))
hold off
xlabel('Time (sec)')
ylabel('x [m]')
grid
subplot(6,1,2)
hold on
plot(t1,-y1(:,2)*180/pi)
plot(t2,-y2(:,2)*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_1$[degrees]','Interpreter','latex')
grid
subplot(6,1,3)
hold on
plot(t1,-y1(:,3)*180/pi)
plot(t2,-y2(:,3)*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_2$ [degrees]','Interpreter','latex')
grid
subplot(6,1,4)
hold on
plot(t1,y1(:,4))
plot(t2,y2(:,4))
hold off
xlabel('Time (sec)')
ylabel('$\dot{x}$ [m/s]','Interpreter','latex')
grid
subplot(6,1,5)
hold on
plot(t1,-y1(:,5)*180/pi)
plot(t2,-y2(:,5)*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\dot{\theta_1}$[degrees/s]','Interpreter','latex')
grid
subplot(6,1,6)
hold on
plot(t1,-y1(:,6)*180/pi)
plot(t2,-y2(:,6)*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\dot{\theta_2}$ [degrees/s]','Interpreter','latex')
legend('ODE45','ODE23')
grid
hold off

%% Problem 8
% Solve for controller-observer compensator
clear; clc; close all;
load('Values_Rev3.mat')
span = [0 3];
m = 3;
n = 6;
p = 3;

cvx_begin sdp quiet

%Variable Definitions
variable P(n,n) symmetric
variable Y(n,p) 

% LMIs
alpha = 12.3326;
P*A + A'*P - Y*C - C'*Y' + 5* alpha * P <= 0
P >= eps*eye(n)

cvx_end
L = P^-1*Y                      % Solve for L

clear x0
x0.x = 0;
x0.theta1 = -50*pi/180;
x0.theta2 = -40*pi/180;
x0.x_dot = 0;
x0.theta1_dot = 0;
x0.theta2_dot = 0;
sim('Linear_Control_ObserverLMI.slx',3);
open_system('Linear_Control_ObserverLMI.slx')


% Pull out Data
data = ans.logsout{1}.Values.Data;
data2 = ans.logsout{3}.Values.Data;
time = ans.logsout{1}.Values.Time';

% Plot State vs Time
figure
hold on
sgtitle('States of the Linear System')
subplot(6,1,1)
hold on
plot(time,data(:,1)')
plot(time,data2(:,1)')
hold off
xlabel('Time (sec)')
ylabel('x [m]')
grid
subplot(6,1,2)
hold on
plot(time,-data(:,2)'*180/pi)
plot(time,-data2(:,2)'*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_1$[degrees]','Interpreter','latex')
grid
subplot(6,1,3)
hold on
plot(time,-data(:,3)'*180/pi)
plot(time,-data2(:,3)'*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_2$ [degrees]','Interpreter','latex')
grid
subplot(6,1,4)
hold on
plot(time,data(:,4)')
plot(time,data2(:,4)')
hold off
xlabel('Time (sec)')
ylabel('$\dot{x}$ [m/s]','Interpreter','latex')
grid
subplot(6,1,5)
hold on
plot(time,-data(:,5)'*180/pi)
plot(time,-data2(:,5)'*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\dot{\theta_1}$[degrees/s]','Interpreter','latex')
grid
subplot(6,1,6)
hold on
plot(time,-data(:,6)'*180/pi)
plot(time,-data2(:,6)'*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\dot{\theta_2}$ [degrees/s]','Interpreter','latex')
grid
legend('True','Observer')
hold off

% Save Matrices for Easy Access
save('Values_Rev3.mat','A','B','C','D','f','K','x_e','u_e','L')

clear x0
x0 = [0;-50*pi/180;-40*pi/180;0;0;0];                       % Create the x state
obs0 = [0;0;0;0;0;0];
initial = [x0;obs0];

% Compare odes
[t1,y1] = ode45(@(t1,y1) nonlinear_cont_obsv(y1,x_e,u_e,K,L,A,C,B),span,initial);  % ODE45
[t2,y2] = ode23(@(t2,y2) nonlinear_cont_obsv(y2,x_e,u_e,K,L,A,C,B),span,initial);  % ODE23

% Plot graphs
figure
hold on
sgtitle('States of the System')
subplot(6,1,1)
hold on
plot(t1,y1(:,1))
plot(t1,y1(:,7))
hold off
xlabel('Time (sec)')
ylabel('x [m]')
grid
subplot(6,1,2)
hold on
plot(t1,-y1(:,2)*180/pi)
plot(t1,-y1(:,8)*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_1$[degrees]','Interpreter','latex')
grid
subplot(6,1,3)
hold on
plot(t1,-y1(:,3)*180/pi)
plot(t1,-y1(:,9)*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_2$ [degrees]','Interpreter','latex')
grid
subplot(6,1,4)
hold on
plot(t1,y1(:,4))
plot(t1,y1(:,10))
hold off
xlabel('Time (sec)')
ylabel('$\dot{x}$ [m/s]','Interpreter','latex')
grid
subplot(6,1,5)
hold on
plot(t1,-y1(:,5)*180/pi)
plot(t1,-y1(:,11)*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\dot{\theta_1}$[degrees/s]','Interpreter','latex')
grid
subplot(6,1,6)
hold on
plot(t1,-y1(:,6)*180/pi)
plot(t1,-y1(:,12)*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\dot{\theta_2}$ [degrees/s]','Interpreter','latex')
legend('Actual','Observer')
grid
hold off

%% Problem 9
% Animate with Optimal Controller-Observer
close all;
Bus_Architecture();

% Initial States
clear x0
x0.x = 0;
x0.theta1 = -30*pi/180;
x0.theta2 = -20*pi/180;
x0.x_dot = 0;
x0.theta1_dot = 0;
x0.theta2_dot = 0;

% Reference Input
v.one = 0;
v.two = 0;
v.three = 0;
% Controller-Observer
sim('Three_Input_Model_CO.slx',15)
open_system('Three_Input_Model_CO.slx')
logsout = ans.logsout;

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
plot(logsout{2}.Values.Time',-logsout{2}.Values.Data*180/pi)
plot(logsout{2}.Values.Time',-logsout{7}.Values.Data*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_1$[degrees]','Interpreter','latex')
grid
subplot(3,1,3)
hold on
plot(logsout{3}.Values.Time',-logsout{3}.Values.Data*180/pi)
plot(logsout{3}.Values.Time',-logsout{8}.Values.Data*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_2$ [degrees]','Interpreter','latex')
grid
hold off

% Plot u vs time
figure(2)
hold on
sgtitle('Inputs of the System')
subplot(3,1,1)
plot(logsout{4}.Values.Time',logsout{4}.Values.Data)
xlabel('Time (sec)')
ylabel('Force [kg*m/s^2]')
subplot(3,1,2)
plot(logsout{5}.Values.Time',-logsout{5}.Values.Data)
xlabel('Time (sec)')
ylabel('Torque_1 [kg*(m/s)^2]')
subplot(3,1,3)
plot(logsout{6}.Values.Time',-logsout{9}.Values.Data)
xlabel('Time (sec)')
ylabel('Torque_2 [kg*(m/s)^2]')
grid
hold off

%% ============ Define Local Functions ===============
function[x_dot]= nonlinear(x,x_e,u_e,K)
        u = -K * (x-x_e) + u_e;
        x2 = x(2);
        x3 = x(3);
        x4 = x(4);
        x5 = x(5);
        x6 = x(6);
        u1 = u(1);
        u2 = u(2);
        u3 = u(3);
    % Insert Equations
    x_dot = double([
                     x4;
                     x5;
                     x6;
                     -(420*u1 - (2943*sin(2*x2))/2 + 150*x5^2*sin(x2) + (135*x6^2*sin(x3))/2 - 180*u1*cos(2*x2 - 2*x3) + 400*u3*cos(2*x2 - x3) - 840*u2*cos(x2) + 840*u3*cos(x2) - 400*u3*cos(x3) + (135*x6^2*sin(2*x2 - x3))/2 + 360*u2*cos(x2 - 2*x3) - 360*u3*cos(x2 - 2*x3))/(30*(5*cos(2*x2) + 9*cos(2*x2 - 2*x3) - 26));
                     (22800*u3 - 22800*u2 - 26487*sin(x2 - 2*x3) - 91233*sin(x2) + 3600*u2*cos(2*x3) - 3600*u3*cos(2*x3) + 4725*x6^2*sin(x2 - x3) + 750*x5^2*sin(2*x2) - 4000*u3*cos(x2 + x3) + 4200*u1*cos(x2) + 1350*x5^2*sin(2*x2 - 2*x3) - 1800*u1*cos(x2 - 2*x3) + 13600*u3*cos(x2 - x3) + 675*x6^2*sin(x2 + x3))/(150*(5*cos(2*x2) + 9*cos(2*x2 - 2*x3) - 26));
                     -(2700*sin(x2 - x3)*x5^2 + 1215*sin(2*x2 - 2*x3)*x6^2 + 13600*u3 - 26487*sin(2*x2 - x3) + 26487*sin(x3) - 4000*u3*cos(2*x2) + 1800*u1*cos(2*x2 - x3) + 3600*u2*cos(x2 + x3) - 3600*u3*cos(x2 + x3) - 1800*u1*cos(x3) - 12240*u2*cos(x2 - x3) + 12240*u3*cos(x2 - x3))/(135*(5*cos(2*x2) + 9*cos(2*x2 - 2*x3) - 26))
                ]);
end

function[x_dot]= nonlinear_output(x,x_e,u_e,K,C)
        delta_y = C * (x-x_e);
        u = -K * delta_y + u_e;
        x2 = x(2);
        x3 = x(3);
        x4 = x(4);
        x5 = x(5);
        x6 = x(6);
        u1 = u(1);
        u2 = u(2);
        u3 = u(3);
    % Insert Equations
    x_dot = double([
                     x4;
                     x5;
                     x6;
                     -(420*u1 - (2943*sin(2*x2))/2 + 150*x5^2*sin(x2) + (135*x6^2*sin(x3))/2 - 180*u1*cos(2*x2 - 2*x3) + 400*u3*cos(2*x2 - x3) - 840*u2*cos(x2) + 840*u3*cos(x2) - 400*u3*cos(x3) + (135*x6^2*sin(2*x2 - x3))/2 + 360*u2*cos(x2 - 2*x3) - 360*u3*cos(x2 - 2*x3))/(30*(5*cos(2*x2) + 9*cos(2*x2 - 2*x3) - 26));
                     (22800*u3 - 22800*u2 - 26487*sin(x2 - 2*x3) - 91233*sin(x2) + 3600*u2*cos(2*x3) - 3600*u3*cos(2*x3) + 4725*x6^2*sin(x2 - x3) + 750*x5^2*sin(2*x2) - 4000*u3*cos(x2 + x3) + 4200*u1*cos(x2) + 1350*x5^2*sin(2*x2 - 2*x3) - 1800*u1*cos(x2 - 2*x3) + 13600*u3*cos(x2 - x3) + 675*x6^2*sin(x2 + x3))/(150*(5*cos(2*x2) + 9*cos(2*x2 - 2*x3) - 26));
                     -(2700*sin(x2 - x3)*x5^2 + 1215*sin(2*x2 - 2*x3)*x6^2 + 13600*u3 - 26487*sin(2*x2 - x3) + 26487*sin(x3) - 4000*u3*cos(2*x2) + 1800*u1*cos(2*x2 - x3) + 3600*u2*cos(x2 + x3) - 3600*u3*cos(x2 + x3) - 1800*u1*cos(x3) - 12240*u2*cos(x2 - x3) + 12240*u3*cos(x2 - x3))/(135*(5*cos(2*x2) + 9*cos(2*x2 - 2*x3) - 26))
                ]);
end

function[y_dot]= nonlinear_cont_obsv(y,x_e,u_e,K,L,A,C,B)
        % Set-Up
        x = y(1:6);
        x_tilda = y(7:12);
        u_delta = -K * (x_tilda-x_e);
        x_delta = x - x_e;
        y_delta = C * x_delta;
        
        % Linear Part - Observer
        x_tilda_dot = (A - L*C)*(x_tilda-x_e) + B*u_delta + L*y_delta;
        
        % Define to move forward with Nonlinear Parts
        u = -K * (x_tilda-x_e) + u_e;
        x2 = x(2);
        x3 = x(3);
        x4 = x(4);
        x5 = x(5);
        x6 = x(6);
        u1 = u(1);
        u2 = u(2);
        u3 = u(3);
        
    % Insert Nonlinear Equations
    x_dot = double([
                     x4;
                     x5;
                     x6;
                     -(420*u1 - (2943*sin(2*x2))/2 + 150*x5^2*sin(x2) + (135*x6^2*sin(x3))/2 - 180*u1*cos(2*x2 - 2*x3) + 400*u3*cos(2*x2 - x3) - 840*u2*cos(x2) + 840*u3*cos(x2) - 400*u3*cos(x3) + (135*x6^2*sin(2*x2 - x3))/2 + 360*u2*cos(x2 - 2*x3) - 360*u3*cos(x2 - 2*x3))/(30*(5*cos(2*x2) + 9*cos(2*x2 - 2*x3) - 26));
                     (22800*u3 - 22800*u2 - 26487*sin(x2 - 2*x3) - 91233*sin(x2) + 3600*u2*cos(2*x3) - 3600*u3*cos(2*x3) + 4725*x6^2*sin(x2 - x3) + 750*x5^2*sin(2*x2) - 4000*u3*cos(x2 + x3) + 4200*u1*cos(x2) + 1350*x5^2*sin(2*x2 - 2*x3) - 1800*u1*cos(x2 - 2*x3) + 13600*u3*cos(x2 - x3) + 675*x6^2*sin(x2 + x3))/(150*(5*cos(2*x2) + 9*cos(2*x2 - 2*x3) - 26));
                     -(2700*sin(x2 - x3)*x5^2 + 1215*sin(2*x2 - 2*x3)*x6^2 + 13600*u3 - 26487*sin(2*x2 - x3) + 26487*sin(x3) - 4000*u3*cos(2*x2) + 1800*u1*cos(2*x2 - x3) + 3600*u2*cos(x2 + x3) - 3600*u3*cos(x2 + x3) - 1800*u1*cos(x3) - 12240*u2*cos(x2 - x3) + 12240*u3*cos(x2 - x3))/(135*(5*cos(2*x2) + 9*cos(2*x2 - 2*x3) - 26))
                ]);
            
    % Put it Together Again
    y_dot = [x_dot;x_tilda_dot];
end
