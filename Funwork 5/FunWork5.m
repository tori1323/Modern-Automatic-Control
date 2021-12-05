% Victoria Nagorski - ECE 680
% Version 1.0 - 11/12/2021
% FunWork 5
%% Start Script
%% Prelims
% Get scipt ready
clear; clc; close all;              % Close everything
load('Values_Rev3.mat')             % Load variables

% Initial States
x0.x = 0;
x0.theta1 = -65*pi/180;
x0.theta2 = -50*pi/180;
x0.x_dot = 0;
x0.theta1_dot = 0;
x0.theta2_dot = 0;

%% Problem 2
% Continuous UIO 
Bus_Architecture()
[n,~] = size(A);
[~,m] = size(B);
[p,~] = size(C);
B2 = [4;
      6;
      9;
      1;
      6;
      0];
D2 = [3 8;
      4 6;
      0 1];
rank([C*B2 D2;B2 zeros(6,2)]) == rank([C*B2 D2])
H0 = [1 0 0;
      0 1 0;
      0 0 1];                          % Design parameter H0
M = [B2 zeros(n,2)]*(pinv([C*B2 D2])+H0*(eye(p)-[C*B2 D2]*pinv([C*B2 D2])))
pi = (eye(n) - M*C);                   % Solve for pi
q0 = pi*[x0.x;x0.theta1;x0.theta2;x0.x_dot;x0.theta1_dot;x0.theta2_dot];

% Solve for L using LMIs
A1 = pi * A;
cvx_begin sdp quiet

% Variable definiton
variable P(n,n) symmetric
variable T(n,p) 

% LMIs
A1'*P + P*A1 - C'*T' - T*C <= -eps * eye(n)
P >= eps * eye(n)

cvx_end

L = P^-1*T;                             % Solve for L matrix

alpha = 15;                             % Define alpha

% Solve for K using LMIs
cvx_begin sdp quiet

% Variable definiton
variable S(n,n) symmetric
variable Z(m,n) 

% LMIs
A*S + S*A' - B*Z - Z'*B' + alpha*S <= -eps * eye(n)
S >= eps * eye(n)

cvx_end

K = Z*S^-1;                             % Solve for K matrix

sim('Continuous_UIO.slx',10);           % Run simulation
open_system('Continuous_UIO.slx')       % Open for publish
Graphing_Obs(ans)                       % Graph the results

%% Problem 3 & 4
% Discrete UIO
close all;
D2 = [0 0;
      0 0;
      0 0];                             % Redefine the D2 matrix

% Solve
sys_c = ss(A,B,C,D);                    % Create continous system
h = 0.001;                              % Define sampling interval
sys = c2d(sys_c,h);                     % Discritize system

M = B2*pinv(C*B2);
pi = (eye(n) - M*C);                    % Solve for pi
q0 = pi*([x0.x;x0.theta1;x0.theta2;x0.x_dot;x0.theta1_dot;x0.theta2_dot]-x_e);
A1 = pi * sys.A;

% ============== Solving Gains ===============
Q = [.0100  0    0   0   10   0;
        0  10  1000  0    0 1000;
        0 1000   1  1000  0   0;
        0   0  1000  1    0   0;
       10   0    0   0    1   0;
        0 1000   0   0    0   1];       % Design weight matrix for states
R = [1000 10 20;
     10   .1  0;
     20    0 10];
[X,~,~]= dlqr(A1',C',Q,R);              % Design weight matrix for inputs
L = X';                                 % Solve for L

alpha = .5;                             % Robust term
% LMI Solve for K
cvx_begin sdp quiet

% Variable definiton
variable S(n,n) symmetric
variable Z(m,n) 

% LMIs
[(-1+alpha)*S               S*sys.A'-Z'*sys.B';
sys.A*S-sys.B*Z         -S + alpha*eye(n)] <= -eps * eye(2*n)
S >= eps * eye(n)

cvx_end

K = Z*S^-1;                             % Solve for K matrix
eig(sys.A - sys.B*K)


sim('Discrete_UIO.slx',10);             % Run simulation
open_system('Discrete_UIO.slx')         % Open for publish
Graphing_Obs_D(ans)             