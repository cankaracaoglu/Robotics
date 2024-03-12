clc
clear
close all

%% Solutions for the inverse kinematics
Px = 20;
Py = 70;

[init_sol1 init_sol2] = ik(Px,Py);

%define the theta values for solution
Px = 60;
Py = 40;

[final_sol1 final_sol2] = ik(Px,Py);




%% Plot of configuration space and the predetermined points
hold on
theta1_array = linspace(0,pi,400);
theta2_array = linspace(-pi,pi,800);
grid_panel = find_config_space(theta1_array,theta2_array);
imagesc(theta1_array,theta2_array,grid_panel);
colormap([1 1 1; 0 0 1]); % blue for 0s, white for 1s
plot(init_sol1(1),init_sol1(2),'k+')
plot(init_sol2(1),init_sol2(2),'k+')
plot(final_sol1(1),final_sol1(2),'r*')
plot(final_sol2(1),final_sol2(2),'r*')
title("Configuration Space")
ylabel("\theta_2 [°]")
xlabel("\theta_1 [°]")




%% Calculate repulsive potential field
distance_from_obstacle = bwdist(grid_panel);
rho_0 = 30;
rho = distance_from_obstacle + 1; %Added one to avoid zero division in potential calculations
eta = 1000;
repulsive_potential = 0.5 * eta * (1 ./ rho - 1 / rho_0).^2;
repulsive_potential (rho > rho_0) = 0;

%% Calculate attractive potential field

%Find the position of the goal and initial
d_theta = 0.01;
goal = zeros(length(theta2_array),length(theta1_array));
stop = false;
final_position = [0 0];
init_position = [0 0];
for i = 1:length(theta1_array)
    for j = 1:length(theta2_array)
        theta1 = theta1_array(i);
        theta2 = theta2_array(j);
        %Check the current position is close enough to desired final
        if abs(theta1 - final_sol2(1)) < d_theta && abs(theta2 - final_sol2(2)) < d_theta
            goal(j,i) = 1;
            final_position = [i j];
            break
        end
        %Check the current position is close enough to desired initial
        if abs(theta1 - init_sol1(1)) < d_theta && abs(theta2 - init_sol1(2)) < d_theta
            init_position = [i j];
            break
        end
    end
    
end

zeta = 0.001;
attractive_potential = 0.5 * zeta * (bwdist(goal)).^2;

%% Calculate the final potential field
potential_field = attractive_potential + repulsive_potential;
figure(2)
[X, Y] = meshgrid(theta1_array, theta2_array);
X_Y_values =  [X(:) Y(:)];
meshc(X,Y,potential_field)
title("Potential Field")
ylabel("\theta_2 [°]")
xlabel("\theta_1 [°]")
zlabel("Potential")



%%  Gradient Algorithm
route = gradient_algorithm(potential_field,init_position,final_position,theta1_array,theta2_array);

figure(3)
hold on
imagesc(theta1_array,theta2_array,grid_panel);
colormap([1 1 1; 0 0 1]); % blue for 0s, white for 1s
plot(route(:,1),route(:,2),'k.');
plot(init_sol1(1),init_sol1 (2),'k+')
plot(final_sol2(1),final_sol2(2),'r*')
title("Configuration Space")
ylabel("\theta_2 [°]")
xlabel("\theta_1 [°]")

%% Animate the robot
animateRobot(route(:,1) ,route(:,1) - route(:,2))


%% Auxilary functions
function [sol_1, sol_2] = ik(Px,Py)
    L1 = 38;
    L2 = 38;

    %define initial inverse kinematics of the robot 
    r1 = sqrt(Py^2 + (Px - 20) ^ 2);
    cos_theta_2 = (r1^2 - L1^2 - L2^2) / (2*L1*L2);
    theta_2_first = atan2(sqrt(1-cos_theta_2^2),cos_theta_2);
    theta_2_second = atan2(-sqrt(1-cos_theta_2^2),cos_theta_2);
    theta_1_first = atan2(Py,Px-20) + atan2(L2*sin(theta_2_first),L2*cos(theta_2_first)+L1);
    theta_1_second = atan2(Py,Px-20) + atan2(L2*sin(theta_2_second),L2*cos(theta_2_second)+L1);

%     sol_1 = [mod(theta_1_first,2*pi) mod(theta_2_first,2*pi)];
%     sol_2 = [mod(theta_1_second,2*pi) mod(theta_2_second,2*pi)];

    sol_1 = [theta_1_first theta_2_first];
    sol_2 = [theta_1_second theta_2_second];

end

function route = gradient_algorithm(potential_field, start_point, end_point,theta1_array,theta2_array)
    route = [];
    iterations = 0;
    current_point = start_point;
    tolarence = 0.1;
    step_size = 1;
    [gradient_x gradient_y] = gradient(potential_field);
    while(iterations < 1500)
        route = [route; [theta1_array(current_point(1)) theta2_array(current_point(2))]];
        [theta1_array(current_point(1)) theta2_array(current_point(2))];

        if sqrt((current_point(1) - end_point(1))^2 +(current_point(2) - end_point(2))^2 ) < tolarence
            break
        end
        
        if gradient_x(current_point(2),current_point(1)) ~= 0
            delta_x = -step_size * gradient_x(current_point(2),current_point(1)) / abs(gradient_x(current_point(2),current_point(1))) ;
        else
            delta_x = 0;
        end

        if gradient_y(current_point(2),current_point(1)) ~= 0
            delta_y = -step_size * gradient_y(current_point(2),current_point(1)) / abs(gradient_y(current_point(2),current_point(1)));
        else
            delta_y = 0;
        end

        current_point = [current_point(1) + delta_x current_point(2) + delta_y];
       
        
        iterations = iterations + 1;
    end

end

function animateRobot(angle1, angle2)
    % Robot parameters
    baseX = 20;
    baseY = 0;
    link1Length = 38;
    link2Length = 38;
    
    % Obstacle parameters
    obstacleX = 40;
    obstacleY = 60;
    obstacleRadius = 10;
    
    % Create figure and axis for animation
    figure;
    axis([0 100 0 100]); % Adjust the axis limits as per your requirement
    xlabel('X');
    ylabel('Y');
    title('Robot Animation ');
    grid on;
    hold on;
    plot(20,70,'k+')
    plot(60,40,'r*')
    % Create line objects for the links
    link1Line = line('XData', [], 'YData', [], 'LineWidth', 2, 'Color', 'b');
    link2Line = line('XData', [], 'YData', [], 'LineWidth', 2, 'Color', 'r');
    
    % Create circle object for the obstacle
    obstacleCircle = viscircles([obstacleX, obstacleY], obstacleRadius, 'Color', 'k');
    
    % Animation loop
    for i = 1:numel(angle1)
        % Calculate the coordinates of the end effector at current time step
        link1X = baseX + link1Length * cos(angle1(i));
        link1Y = baseY + link1Length * sin(angle1(i));
        link2X = link1X + link2Length * cos(angle2(i));
        link2Y = link1Y + link2Length * sin(angle2(i));
        
        % Check for collision between the robot and the obstacle
        robotObstacleDist = sqrt((link2X - obstacleX)^2 + (link2Y - obstacleY)^2);
        if robotObstacleDist <= obstacleRadius
            disp('Collision detected!');
            % You can add collision handling code here, such as stopping the animation or taking corrective actions.
        end
        
        % Update the line objects with new coordinates
        set(link1Line, 'XData', [baseX, link1X]);
        set(link1Line, 'YData', [baseY, link1Y]);
        set(link2Line, 'XData', [link1X, link2X]);
        set(link2Line, 'YData', [link1Y, link2Y]);
        
        % Pause for a short duration to control animation speed
        pause(0.03); % Adjust the pause duration as needed
        
        % Force the figure to redraw
        drawnow;
    end
end

%Collision function for the obstacle
function is_colliding = check_collision(point, center, radius)
    % Calculate the distance between the point and the center of the obstacle
    distance = sqrt((point(1) - center(1))^2 + (point(2) - center(2))^2);
    
    % Check if the distance is less than or equal to the radius of the obstacle
    if distance <= radius
        is_colliding = true;
    else
        is_colliding = false;
    end
end

function grid_panel = find_config_space(theta1_array,theta2_array)
%Define obstacle
%center and radius, collision function defined below
center = [40, 60];
radius = 10;

%Define links
L1 = 38;
L2 = 38;
%Position of the base
base = [20 0];

%Output config space
grid_panel = zeros(length(theta2_array),length(theta1_array));


%Loops go over the possible theta values
for i = 1:length(theta1_array)
    theta1 = theta1_array(i);
    for j = 1:length(theta2_array)
        theta2 = theta2_array(j);
        %If collision occured stops the loops for efficency
        collision_flag = 0;

        [theta1 theta2]

        %points that belong the links should be calculted 
        %calculate for L1
        %Position of the joint one at spesified condition
        joint_1 =  [L1*cos(theta1)+base(1) L1*sin(theta1)+base(2)];
        joint_2 =  [L2*cos(theta1 - theta2)+joint_1(1) L2*sin(theta1 - theta2)+joint_1(2)];
          
    
        for x = linspace(base(1),joint_1(1),76)%x = base(1):0.5:joint_1(1) % x values of the points on link 1
            m = (joint_1(2) - base(2)) / (joint_1(1) - base(1)); %slope of the link 1 
            y = m * (x-base(1)) + base(2);
               
            %check the points of link 1 invalidate the boundaries
            if x < 0 || x > 100 || y < 0 || y > 100 || check_collision([x y],center,radius)
                grid_panel(j,i) = 1;
                collision_flag = 1;
                %theta_pairs = [theta_pairs; [theta1 theta2]];
                break
            end
        end
           

        %do same calculation for L2
        if collision_flag == 0
            %Position of the joint two at spesified condition
            for x = linspace(joint_1(1),joint_2(1),76)
                m = (joint_2(2) - joint_1(2))/(joint_2(1) - joint_1(1)); %instantaneous slope of the link
                y = m * (x-joint_1(1)) + joint_1(2); %y value according to slope

                    %check the points of link 1 invalidate the boundaries
                    if x < 0 || x > 100 || y < 0 || y > 100 || check_collision([x y],center,radius)
                        grid_panel(j,i) = 1;
                        collision_flag = 1;
                        %theta_pairs = [theta_pairs; [theta1 theta2]];
                        break
                    end
            end
        end
    end
end

end



