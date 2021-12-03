function Graphing(ans)
    % For graphing without observer
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
    plot(logsout{6}.Values.Time',-logsout{6}.Values.Data)
    xlabel('Time (sec)')
    ylabel('Torque_2 [kg*(m/s)^2]')
    grid
    hold off

end