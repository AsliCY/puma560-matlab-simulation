% Main simulation program
clear all; clc; close all;
% Initialize Robotics Toolbox
addpath(genpath('/MATLAB Drive/RVC2-copy/rvctools'))
startup_rvc

% Create PUMA robot model
L(1) = Link([0 0.6720 0 -pi/2]);
L(2) = Link([0 0.2290 0.4320 0]);
L(3) = Link([0 -0.0930 0.0110 pi/2]);
L(4) = Link([0 0.4330 0 -pi/2]);
L(5) = Link([0 0 0 pi/2]);
L(6) = Link([0 0.0560 0 0]);
robot = SerialLink(L, 'name', 'PUMA 560');

% Object positions
pickup_pos = [0.5; 0.3; 0.1];    % Position to pick up the object
place_pos = [-0.3; -0.2; 0.2];   % Position to place the object

% Add waypoints (for smoother motion)
pickup_approach = pickup_pos + [0; 0; 0.2]; % 20cm above pickup position
place_approach = place_pos + [0; 0; 0.2];   % 20cm above place position

% Motion parameters
t_step = 2;  % Time for each phase (seconds)
dt = 0.05;   % Time step
t = 0:dt:t_step;
n_steps = length(t);

% Initial and intermediate positions
q_home = [0 0 0 0 0 0];  % Home position

% Calculate end points using forward kinematics
T_pickup = transl(pickup_pos);
T_place = transl(place_pos);
T_pickup_approach = transl(pickup_approach);
T_place_approach = transl(place_approach);

% Inverse kinematics solutions
q_pickup = robot.ikine6s(T_pickup);
q_place = robot.ikine6s(T_place);
q_pickup_approach = robot.ikine6s(T_pickup_approach);
q_place_approach = robot.ikine6s(T_place_approach);

% Define movements (detailed motion sequence)
movements = {
    q_home, q_pickup_approach;        % From home to approach position
    q_pickup_approach, q_pickup;      % From approach to pickup position
    q_pickup, q_pickup;               % Gripping object (wait)
    q_pickup, q_pickup_approach;      % From pickup to approach position
    q_pickup_approach, q_place_approach; % Transfer between approach positions
    q_place_approach, q_place;        % From approach to place position
    q_place, q_place;                 % Releasing object (wait)
    q_place, q_place_approach;        % From place to approach position
    q_place_approach, q_home;         % From approach position to home
};

% Create main figure
h = figure('Name', 'PUMA Robot Simulation');
set(h, 'Position', [100 100 800 600]);

% Simulation for each movement
picked_object = false;
for move = 1:size(movements, 1)
    % Plan trajectory
    [q, qd, qdd] = jtraj(movements{move,1}, movements{move,2}, t);
    
    % Show movement
    for i = 1:5:n_steps
        clf; % Clear figure
        
        % Draw robot
        robot.plot(q(i,:), 'workspace', [-1 1 -1 1 -1 1]);
        view(3); % 3D view
        grid on;
        
        % Show object
        hold on;
        if ~picked_object && move <= 2
            % Object at initial position
            plot3(pickup_pos(1), pickup_pos(2), pickup_pos(3), 'ro', 'MarkerSize', 20, 'DisplayName', 'Object');
        elseif move == 3
            % Gripping moment - show object in different color
            plot3(pickup_pos(1), pickup_pos(2), pickup_pos(3), 'go', 'MarkerSize', 25, 'DisplayName', 'Gripping');
            title('PUMA 560 Robot Simulation - Gripping Object');
        elseif move >= 4 && move <= 6
            % Object moving with robot
            picked_object = true;
            T = double(robot.fkine(q(i,:)));
            pos = T(1:3,4);
            plot3(pos(1), pos(2), pos(3), 'mo', 'MarkerSize', 20, 'DisplayName', 'Moving Object');
        elseif move == 7
            % Release moment - show object in different color
            plot3(place_pos(1), place_pos(2), place_pos(3), 'go', 'MarkerSize', 25, 'DisplayName', 'Releasing');
            title('PUMA 560 Robot Simulation - Releasing Object');
        else
            % Object placed
            plot3(place_pos(1), place_pos(2), place_pos(3), 'ro', 'MarkerSize', 20, 'DisplayName', 'Object');
        end
        
        % Position indicators
        text(pickup_pos(1), pickup_pos(2), pickup_pos(3)-0.1, 'Pickup Position', 'Color', 'blue');
        text(place_pos(1), place_pos(2), place_pos(3)-0.1, 'Place Position', 'Color', 'blue');
        
        % Status message
        if move <= 2
            title('PUMA 560 Robot Simulation - Approaching Object');
        elseif move >= 4 && move <= 6
            title('PUMA 560 Robot Simulation - Transporting Object');
        elseif move > 7
            title('PUMA 560 Robot Simulation - Returning to Home Position');
        end
        
        hold off;
        drawnow;
        pause(0.01);
    end
end