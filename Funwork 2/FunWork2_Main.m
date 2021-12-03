% Victoria Nagorski - ECE 680
% Version 1.0 - 9/12/2021
% FunWork 2
%% Start Script
%% Load Variable From Previous Part
clear; close all; clc;
load('Values.mat');

%% Problem 1
% Solve for Lyapunov matrix for linearized open-loop
try                 % Still publish despite error
Q = eye(6);         % Define
P = lyap(A',Q);
end
    
%% Problem 2
% Design a linear state-feedback controller
p1 = -2;
p2 = -3;
p3 = -6;
p4 = -7;
p5 = -1;
p6 = -8;
K = place(A,B,[p1,p2,p3,p4,p5,p6])
A_c = A - B*K
eig(A_c)

%% Problem 3
% Transfer function of the closed-loop system
[b,a] = ss2tf(A_c,B,C,D)

%% Problem 4
% Solve for Lyapunov matrix for linearized closed-loop
P = lyap(A_c',Q)

% Check principal matrices
princ1 = det(P(1,1))
princ2 = det(P(1:2,1:2))
princ3 = det(P(1:3,1:3))
princ4 = det(P(1:4,1:4))
princ5 = det(P(1:5,1:5))
princ6 = det(P)
%% Problem 5
% Add Torque to Equations
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
syms x1 x2 x3 x4 x5 x6 u1 u2
M = [  r1        r2*cos(x2)     r3*cos(x3);
    r2*cos(x2)     r4         r5*cos(x2-x3);
    r3*cos(x3) r5*cos(x2-x3)       r6];
C = [0 -r2*x5*sin(x2)    -r3*x6*sin(x3);
     0       0            r5*x6*sin(x2-x3);
     0 -r5*x5*sin(x2-x3)       0];
G = [      0;
    -f1 * sin(x2);
    -f2 * sin(x3)];
H = [1 0;0 1;0 0];

% Solve for the Non-Linear Functions
matrix1 = [zeros(3,3)   eye(3);     % Break up equation
           zeros(3,3) -M^-1 * C];   
matrix2 = [zeros(3,1); -M^-1 * G];  % Break up equation
matrix3 = [zeros(3,2); M^-1 * H];   % Break up equation
x = [x1; x2; x3; x4; x5; x6];       % Define x vector
u = [u1; u2];                       % Define state vector
f = simplify(matrix1 * x + matrix2 + matrix3 * u);

% Linearize with the Jacobian
equil = [0;0;0;0;0;0;0;0];          % Equilibrium point (about orgin)
a = jacobian(f,x);                  % Create Jacobian matrix for A
b = jacobian(f,u);                  % Create Jacobian matrix for B
A = double(subs(a,[x;u],equil))     % Plug in equilibrium points
B = double(subs(b,[x;u],equil))     % Plug in equilibrium points
C = [1 0 0 0 0 0;                   % Define the C matrix for Sys
     0 1 0 0 0 0;
     0 0 1 0 0 0] 
D = zeros(3,2)                      % Define the D matrix for Sys

% Design a linear state-feedback controller
p1 = -1.6720+4.6295i;
p2 = -1.6720-4.6295i;
p3 = -1.1206;
p4 = -0.7333;
p5 = -0.5301 + 1.0487i;
p6 = -0.5301-1.0487i;
K = place(A,B,[p1,p2,p3,p4,p5,p6])
A_c = A - B*K
sys_s = ss(A_c,B,C,D);
damp(sys_s)

% Simulate the Controller
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
sim('Linear_Controller_Design.slx',10);

% Pull out Data
data = ans.logsout{1}.Values.Data;
time = ans.logsout{1}.Values.Time';

% Plot State vs Time
figure
hold on
sgtitle('States of the System')
subplot(6,1,1)
plot(time,data(:,1)')
xlabel('Time (sec)')
ylabel('x [m]')
grid
subplot(6,1,2)
plot(time,data(:,2)'*180/pi)
xlabel('Time (sec)')
ylabel('$\theta_1$[degrees]','Interpreter','latex')
grid
subplot(6,1,3)
plot(time,data(:,3)'*180/pi)
xlabel('Time (sec)')
ylabel('$\theta_2$ [degrees]','Interpreter','latex')
grid
subplot(6,1,4)
plot(time,data(:,4)')
xlabel('Time (sec)')
ylabel('$\dot{x}$ [m/s]','Interpreter','latex')
grid
subplot(6,1,5)
plot(time,data(:,5)'*180/pi)
xlabel('Time (sec)')
ylabel('$\dot{\theta_1}$[degrees/s]','Interpreter','latex')
grid
subplot(6,1,6)
plot(time,data(:,6)'*180/pi)
xlabel('Time (sec)')
ylabel('$\dot{\theta_2}$ [degrees/s]','Interpreter','latex')
grid
hold off

% Save Matrices for Easy Access
save('Values_New.mat','A','B','C','D','f','x','K','A_c')
%% Problem 6
% Construct the Observer
close all;
% Construct Observer Poles
l1 = real(p1) * 2;
l2 = real(p2) * 2;
l3 = real(p3) * 2;
l4 = real(p4) * 6;
l5 = real(p5) * 2;
l6 = real(p6) * 2;
L = place(A',C',[l1,l2,l3,l4,l5,l6])'

sim('Linear_Control_Observer.slx',10)
% Pull out Data
data = ans.logsout{1}.Values.Data;
data2 = ans.logsout{2}.Values.Data;
time = ans.logsout{1}.Values.Time';

% Plot State vs Time
figure
hold on
sgtitle('States of the System')
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
plot(time,data(:,2)'*180/pi)
plot(time,data2(:,2)'*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\theta_1$[degrees]','Interpreter','latex')
grid
subplot(6,1,3)
hold on
plot(time,data(:,3)'*180/pi)
plot(time,data2(:,3)'*180/pi)
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
plot(time,data(:,5)'*180/pi)
plot(time,data2(:,5)'*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\dot{\theta_1}$[degrees/s]','Interpreter','latex')
grid
subplot(6,1,6)
hold on
plot(time,data(:,6)'*180/pi)
plot(time,data2(:,6)'*180/pi)
hold off
xlabel('Time (sec)')
ylabel('$\dot{\theta_2}$ [degrees/s]','Interpreter','latex')
grid
legend('True','Observer')
hold off

close all;
% Save Matrices for Easy Access
save('Values_New.mat','A','B','C','D','f','x','K','A_c','L')
%% Problem 7
% Solve the Lyapunov Equation for closed-loop
A_co = [A    -B*K;
      L*C A-L*C-B*K]            % Closed-loop matrix
Q = eye(12);                    % Define Q matrix
P = lyap(A_co',Q)
one = det(P(1,1))
two = det(P(1:2,1:2))
three = det(P(1:3,1:3))
four = det(P(1:4,1:4))
five = det(P(1:5,1:5))
six = det(P(1:6,1:6))
seven = det(P(1:7,1:7))
eight = det(P(1:8,1:8))
nine = det(P(1:9,1:9))
ten = det(P(1:10,1:10))
eleven = det(P(1:11,1:11))
twelve = det(P)

%% Problem 8
% Transfer function of closed-loop
syms 's'
Y = C * (s*eye(6) - A + B*K)^-1 * B
% Method 2
[b1,a1] = ss2tf(A_co,[B;B],[C zeros(3,6)],D,1)      % Input 1
[b2,a2] = ss2tf(A_co,[B;B],[C zeros(3,6)],D,2)      % Input 2
%% Problem 9
% Next published code
