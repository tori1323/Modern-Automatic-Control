function Bus_Architecture_non(p,m,n,Np,cnt)
    % Give Definition to Bus of States
    state_names = {'x','theta1','theta2',...
        'x_dot','theta1_dot','theta2_dot'};
    state_types = {'double','double','double',...
        'double','double','double'};
    for i = 1:length(state_names)
        temp(i) = Simulink.BusElement;
        temp(i).Name = state_names{i};
        temp(i).SampleTime = -1;
        temp(i).Complexity = 'real';
        temp(i).Dimensions = 1;
        temp(i).DataType = state_types{i};
    end
    State_bus = Simulink.Bus;
    State_bus.Elements = temp;
    assignin('base','State_bus',State_bus)
    clear temp state_names state_types
    
    % Give Definition to Reference Bus
    ref_names = {'one','two','three'};
    ref_types = {'double','double','double'};
    for i = 1:length(ref_names)
        temp(i) = Simulink.BusElement;
        temp(i).Name = ref_names{i};
        temp(i).SampleTime = -1;
        temp(i).Complexity = 'real';
        temp(i).Dimensions = 1;
        temp(i).DataType = ref_types{i};
    end
    Reference_bus = Simulink.Bus;
    Reference_bus.Elements = temp;
    assignin('base','Reference_bus',Reference_bus)
    clear temp ref_names ref_types
    
    % Give Definition to Contrain Bus
    ref_names = {'Y_min','Y_max','U_min',...
                 'U_max'};
    ref_types = {'double','double','double',...
                 'double'};
    ref_dim = {[Np*p],[Np*p],[Np*m],...
               [Np*m]};
    for i = 1:length(ref_names)
        temp(i) = Simulink.BusElement;
        temp(i).Name = ref_names{i};
        temp(i).SampleTime = -1;
        temp(i).Complexity = 'real';
        temp(i).Dimensions = ref_dim{i};
        temp(i).DataType = ref_types{i};
    end
    Contraint_bus = Simulink.Bus;
    Contraint_bus.Elements = temp;
    assignin('base','Contraint_bus',Contraint_bus)
    clear temp ref_names ref_types
    
    % Give Definition to Matrix Bus
    ref_names = {'W','Z',...
                 'S','F','Z_aug',...
                 'R','Q'};
    ref_types = {'double','double',...
                 'double','double','double',...
                 'double','double'};
    ref_dim = {[p*Np,n],[p*Np,m*Np],...
               [2*m*Np,m*Np],[2*Np*m,1],...
               [2*p*Np,p*Np],[m*Np,m*Np],[m*Np,m*Np]};
    for i = 1:length(ref_names)
        temp(i) = Simulink.BusElement;
        temp(i).Name = ref_names{i};
        temp(i).SampleTime = -1;
        temp(i).Complexity = 'real';
        temp(i).Dimensions = ref_dim{i};
        temp(i).DataType = ref_types{i};
    end
    Matrix_bus = Simulink.Bus;
    Matrix_bus.Elements = temp;
    assignin('base','Matrix_bus',Matrix_bus)
    clear temp ref_names ref_types
    
    % Give Definition to Initialization Bus
    ref_names = {'U0','mu0'};
    ref_types = {'double','double'};
    ref_dim = {[p*Np,1],[cnt*n*Np,1]};
    for i = 1:length(ref_names)
        temp(i) = Simulink.BusElement;
        temp(i).Name = ref_names{i};
        temp(i).SampleTime = -1;
        temp(i).Complexity = 'real';
        temp(i).Dimensions = ref_dim{i};
        temp(i).DataType = ref_types{i};
    end
    Initial_bus = Simulink.Bus;
    Initial_bus.Elements = temp;
    assignin('base','Initial_bus',Initial_bus)
    clear temp ref_names ref_types
end