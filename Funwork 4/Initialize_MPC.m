function [matrices,constraints,r_p,m,x0,int] = Initialize_MPC(sys,Np,cnt)
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
    
    % Solve for Augmented System
    phi_a = [phi  zeros(n,p);
            C*phi  eye(p)];
    gamma_a = [gamma;
              C*gamma];
    C_a = [zeros(p,n) eye(p)];
    
    % Solve for W and Z
    W = zeros(p*Np,p+n);            % Initialize W
    Z = zeros(p*Np,m*Np);           % Initialize Z
    index = 1;                      % Initialize var
    % Loop through
    for i = 1:p:p*Np
        W((i:(i+2)),:) = C_a * phi_a^index;
        index = index + 1;
    end
    index = 0;                      % Zeroize for next
    for i =1:p:p*Np
        index2 = i;
        index3 = 0;
        while index2 > 0
            Z(i:(i+2),index2:(index2+2)) = C_a * phi_a^index3 * gamma_a;
            index2 = index2 - p;
            index3 = index3 + 1;
        end
        index = index + 1;
    end
    
    % Solve for H and E
    H_eye = zeros(Np);
    for i = 1:Np
        j = i;
        while j > 0
            H_eye(i,j)  = 1;
            j = j - 1;
        end
    end
    H = kron(H_eye,eye(m));         % Solve H
    E = ones(Np,1);
    E = kron(E,eye(m));             % Solve E
    
    % Initialize Bus
    Bus_Architecture(p,m,n,Np,cnt)  % Call bus architecure
    
    % Define Reference Signal
    v_one = 0;
    v_two = -60*pi/180;
    v_three = -45*pi/180;
    r_p = zeros(m*Np,1);
    for i = 1:m:m*Np-2
        r_p(i,1) = v_one;
        r_p(i+1,1) = v_two;
        r_p(i+2,1) = v_three;
    end
    
    % Constraint Bus Initialize
    y_min = [-10;-65*pi/180;-50*pi/180];
    y_max = [10;65*pi/180;50*pi/180];
    u_min = [-20;-10;-5.9];
    u_max = [20;10;5.9];
    constraints.Y_min = kron(ones(Np,1),y_min);
    constraints.Y_max = kron(ones(Np,1),y_max);
    constraints.U_min = kron(ones(Np,1),u_min);
    constraints.U_max = kron(ones(Np,1),u_max);
    
    % Create Matrix Bus
    matrices.W = W;
    matrices.Z = Z;
    matrices.phi_a = phi_a;
    matrices.gamma_a = gamma_a;
    matrices.C_a = C_a;
    matrices.H = H;
    matrices.E = E;
    matrices.Z_aug = [-Z;Z];
    matrices.H_aug = [-H;H];
    
    % Initialize x0
    x0.x = 0;
    x0.theta1 = 0;
    x0.theta2 = 0;
    x0.x_dot = 0;
    x0.theta1_dot = 0;
    x0.theta2_dot = 0;
    
    % Initialization Bus
    int.delta_U0 = zeros(m*Np,1);
    int.mu0 = zeros(cnt*n*Np,1);
    
end