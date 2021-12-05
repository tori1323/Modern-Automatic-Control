function Bus_Architecture()
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
    
    % Give Definition to Bus of Refence
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
end