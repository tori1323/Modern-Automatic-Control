% Victoria Nagorski - ECE 680
% Version 1.0 - 8/24/2021
% FunWork Load Script
%% Start Script
%% Nonlinear Equations Load
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
syms x1 x2 x3 x4 x5 x6 u
M = [  r1        r2*cos(x2)     r3*cos(x3);
    r2*cos(x2)     r4         r5*cos(x2-x3);
    r3*cos(x3) r5*cos(x2-x3)       r6];
C = [0 -r2*x5*sin(x2)    -r3*x6*sin(x3);
     0       0            r5*x6*sin(x2-x3);
     0 -r5*x5*sin(x2-x3)       0];
G = [      0;
    -f1 * sin(x2);
    -f2 * sin(x3)];
H = [1;0;0];

%% Linearize Non-Linear Equations
% Solve for the Non-Linear Functions
matrix1 = [zeros(3,3)   eye(3);     % Break up equation
           zeros(3,3) -M^-1 * C];   
matrix2 = [zeros(3,1); -M^-1 * G];  % Break up equation
matrix3 = [zeros(3,1); M^-1 * H];   % Break up equation
x = [x1; x2; x3; x4; x5; x6];       % Define x vector
f = matrix1 * x + matrix2 + matrix3 * u;

% Linearize with the Jacobian
equil = [0;0;0;0;0;0;0];            % Equilibrium point (about orgin)
a = jacobian(f,x);                  % Create Jacobian matrix for A
b = jacobian(f,u);                  % Create Jacobian matrix for B
A = double(subs(a,[x;u],equil));    % Plug in equilibrium points
B = double(subs(b,[x;u],equil));    % Plug in equilibrium points
C = [1 0 0 0 0 0;                   % Define the C matrix for Sys
     0 1 0 0 0 0;
     0 0 1 0 0 0];
D = zeros(3,1);                     % Define the D matrix for Sys

% Save Matrices for Easy Access
save('Values.mat','A','B','C','D','l1','l2','f','x')

%% Controllability and Observability
% Controller Form
Co = ctrb(A,B);                     % Create controllability matrix
n = rank(Co);                       % Rank of controllability matrix
Co_inv = Co^-1;                     % Inverse
q = Co_inv(end,:);                  % Last row of inverse
T = [q;                             % Transformation matrix
    q * A;
    q * A^2;
    q * A^3;
    q * A^4;
    q * A^5];
A_c = T*A*T^-1                      % Controller form A
B_c = T*B                           % Controller form B

% Observer Form
O = obsv(A,C)                       % Observability matrix
n_O = rank(O)                       % Rank of observability matrix
A_t = A';                           % Transpose of A
B_t = C';                           % Transpose of C saved under B^T
% Calculate part of the controllability matrix of A^T and C^T
temp = [B_t(:,1) B_t(:,2) B_t(:,3)...
        A_t * B_t(:,1) A_t * B_t(:,2) A_t * B_t(:,3)...
        A_t^2 * B_t(:,1) A_t^2 * B_t(:,2) A_t^2 * B_t(:,3)];
rank(temp)                          % Calculate the rank of partial matrix
L = [B_t(:,1) A_t * B_t(:,1) ...    % Create L with independent columns
     B_t(:,2) A_t * B_t(:,2) ...
     B_t(:,3) A_t * B_t(:,3)];
L_inv = L^-1;                       % Calculate inverse of L
q1 = L_inv(end,:);                  % Pull out last row of L
T_o = [q1;                          % Calculate the transformation matrix
       q1 * A_t;
       q1 * A_t^2;
       q1 * A_t^3;
       q1 * A_t^4;
       q1 * A_t^5];
A_o = T_o'^-1*A*T_o'                % Observer form of A
C_o = C*T_o'                        % Observer form of C

%% Transfer Function
% Method 1
syms s
Y = C * (s*eye(6)-A)^-1 * B;
% Method 2
[b,a] = ss2tf(A,B,C,D)
