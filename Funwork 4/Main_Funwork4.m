% Victoria Nagorski - ECE 680
% Version 1.0 - 10/16/2021
% FunWork 4
%% Start Script
%% Problem 1
% Discretize the model
clear; clc; close all;              % Close everything
load('Values_Rev3.mat')             % Load variables

% Solve
sys_c = ss(A,B,C,D);                % Create continous system
h = 0.01;                           % Define sampling interval
sys = c2d(sys_c,h);                 % Discritize system

%% Problem 2
% Design an MPC for augmented model w/o contraints
Np = 23;                            % Define prediction horizon

% Initialize the problem
[matrices,constraints,r_p,m,x0,int] = Initialize_MPC(sys,Np,1);

% Define the gain matrices:
R0 = [.01 0 0;
      0 .01 0;
      0 0 .0075];
matrices.R = kron(eye(Np),R0);       % Gain Matrix - Control
Q0 = [5 0 0;
      0 30 0;
      0 0 25];
matrices.Q = kron(eye(Np),Q0);       % Gain Matrix - States

% Edit initial conditions from 0
x0.theta1 = -55*pi/180;
x0.theta2 = -40*pi/180;

% Lump together unchanging parts
K_Uncons = (matrices.R + matrices.Z'*matrices.Q*matrices.Z)^-1*matrices.Z'*matrices.Q;

sim('Aug_MPC_UnCon.slx',10);         % Run simulation
open_system('Aug_MPC_UnCon.slx')     % Open for publish
Graphing(ans)                        % Graph the results

%% Problem 3
% Impose contraints on the inputs and outputs
close all;
cnt = 2;                            % Set cnt = 2 for contraints in and out
int.mu0 = zeros(cnt*6*Np,1);        % Re-initilaize mu
Bus_Architecture(3,3,6,Np,cnt)      % Run bus-architecture for different set up
R0 = [.01 0 0;
      0 .01 0;
      0 0 .0075];
R = kron(eye(Np),R0);               % Gain Matrix - Control
Q0 = [5 0 0;
      0 30 0;
      0 0 25];
Q = kron(eye(Np),Q0);               % Gain Matrix - States

sim('Aug_MPC_Con.slx',10);          % Run simulation
open_system('Aug_MPC_Con.slx')      % Open for publish

Graphing(ans)                       % Graph the results

%% Problem 4
% Combined MPC controller-observer compensator
close all;
m = 3;
n = 6;
p = 3;
cvx_begin sdp quiet

%Variable Definitions
variable P(n,n) symmetric
variable Y(n,p) 

% LMIs
eta = 1/2;
P*A + A'*P - Y*C - C'*Y' + eta * P <= 0
P >= eps*eye(n)

cvx_end
L = P^-1*Y                         % Solve for L

sim('Aug_MPC_CO.slx',10);          % Run simulation
open_system('Aug_MPC_CO.slx')      % Open for publish
Graphing_Obs(ans)                  % Graph the results

%% Problem 5a
% Design an MPC for non-augmented model w/o contraints
% Begin Prelininaries 
clear; clc; close all;
load('Values_Rev3.mat')

% Create continuous system
sys_c = ss(A,B,C,D);
h = 0.01;

% Discritize system
sys = c2d(sys_c,h);

% Start Actual::
Np = 23;                           % Define the prediction horizon
% Initialize the problem
[matrices,constraints,r_p,m,x0,int] = Initialize_MPC_non(sys,Np,1);

% Define weight matrices
matrices.R = 0.001*eye(3*Np);      % Gain Matrix - Control
Q0 = [15 0 0;
      0 28 0;
      0 0 25];
matrices.Q = kron(eye(Np),Q0);     % Gain Matrix - Input

% Edit initial conditions from 0
x0.theta1 = -55*pi/180;
x0.theta2 = -40*pi/180;

% Lump together unchanging parts
K_Uncons = (matrices.R + matrices.Z'*matrices.Q*matrices.Z)^-1*matrices.Z'*matrices.Q;

sim('NonAug_MPC_UnCon.slx',10);    % Run simulation
open_system('NonAug_MPC_UnCon.slx')% For publishing later 
Graphing(ans)                      % Graph the results

%% Problem 5b
% Impose contraints on the inputs and outputs
close all;
Np = 43;                           % Define the prediction horizon
cnt = 2;                           % cnt = 2 for contrained in and out
[matrices,constraints,r_p,m,x0,int] = Initialize_MPC_non(sys,Np,cnt);
R0 = [.01 0 0;
      0 .001 0;
      0 0 .005];
matrices.R = kron(eye(Np),R0);     % Gain Matrix - Control
Q0 = [20 0 0;
      0 30 0;
      0 0 30];
matrices.Q = kron(eye(Np),Q0);     % Gain Matrix - Input

% Edit initial conditions from 0
x0.theta1 = -55*pi/180;
x0.theta2 = -40*pi/180;

sim('NonAug_MPC_Con.slx',10);      % Run simulation
open_system('NonAug_MPC_Con.slx')  % For publishing later 
Graphing(ans)                      % Graph the results

%% Problem 5c
% Combined MPC controller-observer compensator
close all;
m = 3;
n = 6;
p = 3;
cvx_begin sdp quiet

%Variable Definitions
variable P(n,n) symmetric
variable Y(n,p) 

% LMIs
eta = 1/2;
P*A + A'*P - Y*C - C'*Y' + eta * P <= 0
P >= eps*eye(n)

cvx_end
L = P^-1*Y                         % Solve for L

sim('NonAug_MPC_CO.slx',10);       % Run simulation
open_system('NonAug_MPC_CO.slx')   % For publishing later 
Graphing_Obs(ans)                  % Graph the results
