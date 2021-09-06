%% Animate an Object Example
clear; close all; clc;

load('Values.mat');
x_vec = x;                               % Differentiate between to 'x's
clear x                                  % Clear old variable from mem
syms u                                   % Initialize u as a syms var

% Initial Values - Change these values
font_size = 10;
theta_d1 = -0.01*180/pi;                 % Theta 1 [degrees]
theta_d2 = -0.02*180/pi;                 % Theta 2 [degrees]
tfinal = 60;                             % Final time [s]
t = 0;                                   % Time [s]
nframes = 2000;                          % Number of points
%u = 0;

% Convert Values - Do not hard-code these values
theta1 = theta_d1*pi/180;               % Theta 1 convered to radians
theta2 = theta_d2*pi/180;               % Theta 2 convered to radians
dt = (tfinal-t)/nframes;                % Step size of animation

% Initialize variables
time = t:dt:tfinal;                     % Create vector of time
states = zeros(6,nframes);              % Matrix to hold states for time
states(:,1) = [0;theta1;theta2;0;0;0];  % Initial values

% Initialize Object
data_init1 = [0 l1]';                   % Pendulum length for length 1
data_init2 = [0 l2]';                   % Pendulum length for length 2
R1 = [cos(theta1) -sin(theta1);
     sin(theta1)  cos(theta1)];         % Rotation matrix 1
R2 = [cos(theta2) -sin(theta2);
     sin(theta2)  cos(theta2)];         % Rotation matrix 2
data1 = R1 * data_init1;                % Initial pendulum position
data2 = R2 * data_init2;                % Initial pendulum position

% Define Distances
d1_x = states(1,1);                     % Where M is located
d2_x = states(1,1) + data1(1);          % Where m1 is located in x
d3_x = d2_x + data2(1);                 % Where m2 is located in x
d3_y = data1(2)+data2(2);               % Where m2 is located in y

% Graphics for pendulum 1
bar1 = line('xdata',[d1_x d2_x],'ydata',[0 data1(2)]','linewidth',3);
mass1 = line('xdata',d2_x,'ydata',data1(2),'marker','o',...
    'markersize',10,'MarkerFaceColor','b');
hinge1 = line('xdata',d1_x,'ydata',0,'marker','o','markersize',7);
% Graphics for pendulum 2
bar2 = line('xdata',[d2_x d3_x],'ydata',[data1(2) d3_y]','linewidth',3);
mass2 = line('xdata',d3_x,'ydata',d3_y,'marker','o',...
    'markersize',10,'MarkerFaceColor','b');
hinge2 = line('xdata',d2_x,'ydata',data1(2),'marker','o','markersize',7);
% Graphics for mass M
mass = line('xdata',d1_x,'ydata',0,'marker','s',...
    'markersize',17,'MarkerFaceColor','b');
% Graphics for Ground
Ground = line('xdata',[-4 4],'ydata',[-.2 -.20]','linewidth',3);
% Graphics for handle
axis(4*[-1 1 -1/2 1/2]); grid on
set(gca,'Fontsize',font_size)
set(gca,'dataaspectratio',[1 1 1])
box on

% Animate Object
vidObj = VideoWriter('Double_Pendulum.avi');
open(vidObj);
Frames = moviein(nframes)
for i = 2:(nframes + 1)
    % Equations of motion
    states(:,i) = dt * (double(subs(f,[x_vec;u],[states(:,i-1);0]))) + states(:,i-1);
    % Pull out locations
    x = states(1,i);
    theta1 = states(2,i);
    theta2 = states(3,i);
   
    % Calculate new locations to update video frames
    R1 = [cos(theta1) -sin(theta1);
     sin(theta1)  cos(theta1)];         % Rotation matrix 1
    R2 = [cos(theta2) -sin(theta2);
     sin(theta2)  cos(theta2)];         % Rotation matrix 2
    data1 = R1 * data_init1;            % Initial pendulum position
    data2 = R2 * data_init2;            % Initial pendulum position
    d1_x = states(1,i);                 % Where M is located
    d2_x = states(1,i) + data1(1);      % Where m1 is located in x
    d3_x = d2_x + data2(1);             % Where m2 is located in x
    d3_y = data1(2) + data2(2);         % Where m2 is located in y
   
    % Change the property values m1
    set(bar1,'xdata',[d1_x d2_x],'ydata',[0 data1(2)]);
    set(mass1,'xdata',d2_x,'ydata',data1(2));
    set(hinge1,'xdata',d1_x,'ydata',0)
    % Change the property values m2
    set(bar2,'xdata',[d2_x d3_x],'ydata',[data1(2) d3_y]);
    set(mass2,'xdata',d3_x,'ydata',d3_y);
    set(hinge2,'xdata',d2_x,'ydata',data1(2))
    % Change the property values M
    set(mass,'xdata',d1_x,'ydata',0);

   
    % For the animation
    drawnow;
    Frames(:,i) = getframe;
    writeVideo(vidObj,Frames(:,i));
end

% Plot state vs time
hold on
sgtitle('States of the System')
subplot(3,1,1)
plot(time,states(1,:))
xlabel('Time (sec)')
ylabel('x [m]')
set(gca,'Fontsize',font_size)
subplot(3,1,2)
plot(time,states(2,:)*180/pi)
xlabel('Time (sec)')
ylabel('$\theta_1$[radians]','Interpreter','latex')
set(gca,'Fontsize',font_size)
subplot(3,1,3)
plot(time,states(3,:)*180/pi)
xlabel('Time (sec)')
ylabel('$\theta_2$ [radians]','Interpreter','latex')
grid
set(gca,'Fontsize',font_size)
hold off
close(vidObj);
