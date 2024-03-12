%% 
clc
clear
close all

%% Inputs
p_start = [0.0, 0.0, 0.0];
p_inter = [0.1, 0.01, 0.04];
p_goal = [0.15, 0.02, 0.05];
t_final = 10;

%% Sımulation
out = sim('simulation');

% Assuming you have already run the simulation and obtained the 'out' structure

% Extract numeric data from timeseries object
forceX_data = out.forceX.Data;
forceY_data = out.forceY.Data;
forceZ_data = out.forceZ.Data;
xDesired_data = out.xDesired.Data;
xSim_data = out.xSim.Data;
yDesired_data = out.yDesired.Data;
ySim_data = out.ySim.Data;
zDesired_data = out.zDesired.Data;
zSim_data = out.zSim.Data;

% Plot forceX
figure
plot(out.tout, forceX_data)
xlabel('Time')
ylabel('Force X')
title('Force X vs. Time')

% Plot forceY
figure
plot(out.tout, forceY_data)
xlabel('Time')
ylabel('Force Y')
title('Force Y vs. Time')

% Plot forceZ
figure
plot(out.tout, forceZ_data)
xlabel('Time')
ylabel('Force Z')
title('Force Z vs. Time')

% Plot xDesired and xSim
figure
plot(out.tout, xDesired_data, 'b', out.tout, xSim_data, 'g')
xlabel('Time')
ylabel('X')
title('X Desired and X Simulated vs. Time')
legend('X Desired', 'X Simulated')

% Plot yDesired and ySim
figure
plot(out.tout, yDesired_data, 'b', out.tout, ySim_data, 'g')
xlabel('Time')
ylabel('Y')
title('Y Desired and Y Simulated vs. Time')
legend('Y Desired', 'Y Simulated')

% Plot zDesired and zSim
figure
plot(out.tout, zDesired_data, 'b', out.tout, zSim_data, 'g')
xlabel('Time')
ylabel('Z')
title('Z Desired and Z Simulated vs. Time')
legend('Z Desired', 'Z Simulated')

% Extract xError, yError, and zError data
xError_data = out.xError.Data;
yError_data = out.yError.Data;
zError_data = out.zError.Data;

% Plot xError
figure
plot(out.tout, xError_data)
xlabel('Time')
ylabel('X Error')
title('X Error vs. Time')

% Plot yError
figure
plot(out.tout, yError_data)
xlabel('Time')
ylabel('Y Error')
title('Y Error vs. Time')

% Plot zError
figure
plot(out.tout, zError_data)
xlabel('Time')
ylabel('Z Error')
title('Z Error vs. Time')

% Extract theta1, theta2, and theta3 data
theta1_data = out.theta1.Data;
theta2_data = out.theta2.Data;
theta3_data = out.theta3.Data;

% Plot theta1
figure
plot(out.tout, theta1_data)
xlabel('Time')
ylabel('Theta1')
title('Theta1 vs. Time')

% Plot theta2
figure
plot(out.tout, theta2_data)
xlabel('Time')
ylabel('Theta2')
title('Theta2 vs. Time')

% Plot theta3
figure
plot(out.tout, theta3_data)
xlabel('Time')
ylabel('Theta3')
title('Theta3 vs. Time')

%% Animation 
%animateXZ(xSim_data,zSim_data,theta1_data,theta2_data,xDesired_data,zDesired_data)
animateTop(xSim_data,zSim_data,theta1_data,theta2_data,xDesired_data,zDesired_data)
%animateSide(xSim_data,ySim_data,zSim_data,theta1_data,theta2_data,xDesired_data,yDesired_data,zDesired_data)


function animateTop(xSim_data,zSim_data,theta1_data,theta2_data,xDesired_data,zDesired_data)

    baseZ = -0.3;
    baseX = -0.10;
    link1Length = 0.215;
    link2Length = 0.170;
    
   
    % Create figure and axis for animation
    figure;
    axis([-0.3 0.3 -0.3 0.3]); % Adjust the axis limits as per your requirement
    xlabel('Z');
    ylabel('X');
    title('Robot Animation TOP');
    grid on;
    hold on;
    % Create line objects for the links
    link1Line = line('XData', [], 'YData', [], 'LineWidth', 2, 'Color', 'b');
    link2Line = line('XData', [], 'YData', [], 'LineWidth', 2, 'Color', 'r');
    
    
    % Animation loop
    for i = 1:30:numel(xSim_data)
        i
        % Calculate the coordinates of the end effector at current time step
        link1Z = baseZ + link1Length * cos(theta1_data(i))*cos(theta2_data(i));
        link1X = baseX + link1Length * sin(theta1_data(i))*cos(theta2_data(i));
        link2Z = zSim_data(i);
        link2X = xSim_data(i);
 
        
        % Update the line objects with new coordinates
        set(link1Line, 'XData', [baseZ, link1Z]);
        set(link1Line, 'YData', [baseX, link1X]);
        set(link2Line, 'XData', [link1Z, link2Z]);
        set(link2Line, 'YData', [link1X, link2X]);

        plot(zDesired_data(i),xDesired_data(i),'k+')
        plot(zSim_data(i),xSim_data(i),'r*')    
        
        % Pause for a short duration to control animation speed
        pause(0.03); % Adjust the pause duration as needed
        
        % Force the figure to redraw
        drawnow;
    end
end


function animateSide(xSim_data,ySim_data,zSim_data,theta1_data,theta2_data,xDesired_data,yDesired_data,zDesired_data)

    baseY = 0;
    baseX = 0;
    link1Length = 0.215;
    link2Length = 0.170;
    
   
    % Create figure and axis for animation
    figure;
    axis([-0.3 0.3 -0.3 0.3]); % Adjust the axis limits as per your requirement
    xlabel('X');
    ylabel('Y');
    title('Robot Animation SIDE');
    grid on;
    hold on;
    % Create line objects for the links
    link1Line = line('XData', [], 'YData', [], 'LineWidth', 2, 'Color', 'b');
    link2Line = line('XData', [], 'YData', [], 'LineWidth', 2, 'Color', 'r');
    
    
    % Animation loop
    for i = 1:30:numel(xSim_data)
        i
        % Calculate the coordinates of the end effector at current time step
        link1X = baseX + link1Length * cos(theta1_data(i))*cos(theta2_data(i));
        link1Y = baseY + link1Length * sin(theta1_data(i))*cos(theta2_data(i));
        link2X = xSim_data(i);
        link2Y = ySim_data(i);
 
        
        % Update the line objects with new coordinates
        set(link1Line, 'XData', [baseX, link1X]);
        set(link1Line, 'YData', [baseY, link1Y]);
        set(link2Line, 'XData', [link1X, link2X]);
        set(link2Line, 'YData', [link1Y, link2Y]);

        plot(xDesired_data(i),yDesired_data(i),'k+')
        plot(xSim_data(i),ySim_data(i),'r*')    
        
        % Pause for a short duration to control animation speed
        pause(0.03); % Adjust the pause duration as needed
        
        % Force the figure to redraw
        drawnow;
    end
end
