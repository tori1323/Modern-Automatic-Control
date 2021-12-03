function [matrices,constraints,r_p,m,x0,int] = Initialize_MPC_non(sys,Np,cnt)
    % Function builds necessary matrices to perform MPS

    % Start Code Here::
    % Determine Sizes Matrices
    [p,~] = size(sys.C);
    [~,m] = size(sys.B);
    [n,~] = size(sys.A);

        
    % Pull Out Variables from Sys
    phi = sys.A;
    gamma = sys.B;
    C = sys.C;
    
    % Solve for W and Z
    W = zeros(p*Np,n);              % Initialize W
    Z = zeros(p*Np,m*Np);           % Initialize Z
    index = 1;                      % Initialize var
    % Loop through
    for i = 1:p:p*Np
        W((i:(i+2)),:) = C * phi^index;
        index = index + 1;
    end
    index = 0;                      % Zeroize for next
    for i =1:p:p*Np
        index2 = i;
        index3 = 0;
        while index2 > 0
            Z(i:(i+2),index2:(index2+2)) = C * phi^index3 * gamma;
            index2 = index2 - p;
            index3 = index3 + 1;
        end
        index = index + 1;
    end
    
    % Initialize Bus
    Bus_Architecture_non(p,m,n,Np,cnt)
    
    % Define Reference Signal
    v_one = 0;
    v_two = -60*pi/180;
    v_three = -45*pi/180;
    r_p = zeros(3*Np,1);
    for i = 1:m:m*Np-2
        r_p(i,1) = v_one;
        r_p(i+1,1) = v_two;
        r_p(i+2,1) = v_three;
    end
    
   % Constraint Bus Initialize
    y_min = [-20;-62*pi/180;-47*pi/180];
    y_max = [20;62*pi/180;47*pi/180];
    u_min = [-50;-12;-5.9];
    u_max = [50;12;5.9];
    constraints.Y_min = kron(ones(Np,1),y_min);
    constraints.Y_max = kron(ones(Np,1),y_max);
    constraints.U_min = kron(ones(Np,1),u_min);
    constraints.U_max = kron(ones(Np,1),u_max);
    
    % Solve for S and F
    S_eye = eye(Np);
    S = kron(S_eye,[-eye(m);eye(m)]);
    F = zeros(2*m*Np,1);
    for i = 1:2*m:2*m*Np
       F((i:i+5),1) = [-u_min;u_max];   
    end
    
    % Create Matrix Bus
    matrices.W = W;
    matrices.Z = Z;
    matrices.S = S;
    matrices.F = F;
    matrices.Z_aug = [-Z;Z];
    
    % Initialize x0
    x0.x = 0;
    x0.theta1 = 0;
    x0.theta2 = 0;
    x0.x_dot = 0;
    x0.theta1_dot = 0;
    x0.theta2_dot = 0;
    
    % Initialization Bus
    int.U0 = zeros(m*Np,1);
    int.mu0 = zeros(cnt*n*Np,1);
    
end