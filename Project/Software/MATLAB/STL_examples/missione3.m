clear all
close all
clc

%% Add paths for solvers and related functions

% Solvers
addpath('functions');
addpath('txt_files');
addpath('maps');
addpath('print');
addpath('missions');
addpath('missions_functions');
addpath('stl_formulae_functions');
addpath('figures');
% Add CasADi paths
if ismac
    addpath('../casadi-osx');
elseif ispc
    addpath('../casadi-windows');
else isunix
    addpath('../casadi');
end

%% Simulation info

selection = 2;

enable_plot = 'n';

enable_saving_txt = 'n';

enable_traj_rot = 'n';

enable_animation = 'y';

enable_Belta_variation = 'y';

time_estimation_method = 2;

solver_choice = 1;

enable_boolean = 'n';

selection_CVX = 'n';

% To show the computation time
tic;

switch selection
          
    %%%%%%%%%%%%%%%%% CASE 2 %%%%%%%%%%%%%%%%%
    %  Scenario 2: Reach and avoid with 1 obstacle and 2 drones
    
    case 2
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% STL FORMULA %%%%%%%%%%%%%%%%%%%%% %%
        
        disp('This scenario encodes the following STL formula:');
        disp( ' ' );
        disp('\varphi_\mathrm{sep}^{i,j} = \square_{[0, T]} \left(\lVert {^W\mathbf{r}}^i - {^W\mathbf{r}}^j');
        disp('      \rVert  \geq \delta_\mathrm{min} \right), ');
        disp('\varphi_\mathrm{mra} = \wedge_{k=1}^N \varphi_\mathrm{ra}^k \bigwedge \wedge_{k=1}^N \left(');
        disp('      \wedge_{i \neq j} \varphi_\mathrm{sep}^{i,j} \right)');
        disp( ' ');
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% Parameters %%%%%%%%%%%%%%%%%%%%%% %%
        
        drones                = 2;
        map_name              = 'plot3_4.txt';
        plotting_time         = 0.05; % seconds
        scale                 = 3; % scale the quadrotor dimension
        destinationTxT        = 'txt_files'; % folder where saving txt trajectories and waypoints
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% Trajectory rotation %%%%%%%%%%%%% %%
        
        % Rotation angle between MATLAB and real-world reference frame
        angle_deg = 15; % degrees --> rad2deg(-0.4794)
        angle_rad = deg2rad(angle_deg); % radians
        
        % Translation in terms of x and y from the old starting point of
        % the reference system in MALTAB into one of the two poles (close
        % to zero in terms of y and z)
        translation_x = -25; 
        translation_y = -25;
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% Drones initial position %%%%%%%%% %%

        % Initial_position \in \mathbb{R}^{3 \times d}, where d is the
        % number of drones
        initial_position = zeros(3, drones);
        initial_position(:,1) = [15; -15; 0]; % drone1 starting point
        initial_position(:,2) = [-15;  15; 0]; % drone2 starting point
        
        % For Gazebo simulations:
        initial_position_gazebo = zeros(size(initial_position));
        rotation_gazebo = rotz(angle_deg);
        for i = 1 : drones
            initial_position_gazebo(:,i) = rotation_gazebo*( initial_position(:,i)) ...
                + [ translation_x ;  translation_y ;  0]; % translation (first) + rotation (second)
            % plot3(initial_position_gazebo(1,i),initial_position_gazebo(2,i),initial_position_gazebo(3,i),'mo');
        end

        % Initial velocity (zero) \in \mathbb{R}^{3 \times d}
        initial_velocity = zeros(3, drones);
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% Trajectory parameters %%%%%%%%%%% %%

        % The motion time. See D'Andrea paper:
        %
        % M. W. Mueller, M. Hehn and R. D'Andrea, "A Computationally Efficient 
        % Motion Primitive for Quadrocopter Trajectory Generation," in IEEE Transactions 
        % on Robotics, vol. 31, no. 6, pp. 1294-1310, Dec. 2015, doi: 10.1109/TRO.2015.2479878.
        %
        motion_time   = 1;
        sampling_time = 0.05; % Trajectory sampling time [s] - 20 Hz
        
        % minimal distance between drones
        delta_min = 5; % meter
        
        % This vector allows you to fix the constraint for the axis, that is x, y 
        % and z, which could be different for design or vehicle reasons.
        constraint_max_axis_vel = zeros(3,drones);
        constraint_max_axis_acc = zeros(3,drones);
        
        % Fixing the velocity and acceleration constraints per all drones
        for i = 1 : drones
            constraint_max_axis_vel(:,i) = [1/2.25;1/2.25;1/2.25]; % scale factor
            constraint_max_axis_acc(:,i) = ones(3,1);
        end

        % Motion constraints
        max_vel = 10; % max velocity value during the drone motion
        max_acc = 5; % max acceleration value during the drone motion
        
        % The parameters used for the min/max smooth approximation on the mutual
        % distance and STL mission requirements
        C      = 10; % STL specifications
        C_dist = 5;  % mutual distance

        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% Goals %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
       
        i = 1;
        goal{i}.stop = [10; -7; 13]; % First point to inspect - blue
        goal{i}.heading_angle = pi/2; % radians - drone orientation
        
        i = i + 1;
        goal{i}.stop = [10; -4; 13]; % Second point to inspect - blue
        goal{i}.heading_angle = pi; % radians - drone orientation
        
        i = i + 1;
        goal{i}.stop = [10; 4; 13]; % Third point to inspect - blue
        goal{i}.heading_angle = 3*pi/2; % radians - drone orientation
        
        i = i + 1;
        goal{i}.stop = [10; 7; 13]; % Fourth point to inspect - blue
        goal{i}.heading_angle = 0; % radians - drone orientation

        i = i + 1;
        goal{i}.stop = [-10; -7; 13]; % Fifth point to inspect - blue
        goal{i}.heading_angle = 0; % radians - drone orientation
        
        i = i + 1;
        goal{i}.stop = [-10; -4; 13]; % sixth point to inspect - blue
        goal{i}.heading_angle = 0; % radians - drone orientation

        i = i + 1;
        goal{i}.stop = [-10; 4; 13]; % seventh point to inspect - blue
        goal{i}.heading_angle = 0; % radians - drone orientation

        i = i + 1;
        goal{i}.stop = [-10; 7; 13]; %eighth point to inspect - blue
        goal{i}.heading_angle = 0; % radians - drone orientation

        i = i + 1;
        goal{i}.stop = [10; -5; 17.5]; % nineth point to inspect - blue
        goal{i}.heading_angle = 0; % radians - drone orientation

        i = i + 1;
        goal{i}.stop = [10; 5; 17.5]; % tenth point to inspect - blue
        goal{i}.heading_angle = 0; % radians - drone orientation

        i = i + 1;
        goal{i}.stop = [-10; 5; 17.5]; % eleventh point to inspect - blue
        goal{i}.heading_angle = 0; % radians - drone orientation

        i = i + 1;
        goal{i}.stop = [10; 0; 23]; % twelveth point to inspect - blue
        goal{i}.heading_angle = 0; % radians - drone orientation

       
        number_targets = size(goal,2); % contains the number of target regions
        for i= 1 : number_targets
            goal{i}.ds = 0.15; % Accuracy for the lower and upper bounds for the i-th target
        end
        
        % Accuracy in reaching the goal point
        for i = 1 : number_targets
            goal{i}.lb = goal{i}.stop - goal{i}.ds; % lower bound
            goal{i}.ub = goal{i}.stop + goal{i}.ds; % upper bound
        end

        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% Optimal sequence %%%%%%%%%%%%%%%% %%

        sigma_max = 15; % Maximum admissible difference between the path length of two drones [m]

        % Computation of the sequence:
        if strcmp(selection_CVX, 'n') % using intlinprog function in Matlab
                sequence = get_sequence_multidepot( initial_position , goal, drones, number_targets, sigma_max );

        elseif strcmp(selection_CVX, 'y') % using CVX
                sequence = get_sequence_multidepot_cvx( initial_position , goal, drones, number_targets, sigma_max);
        end
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% Number of WPs %%%%%%%%%%%%%%%%%%% %%
        % The number of waypoints in a non-direct way expresses the time requirements. 
        % In fact, considering that the motion time is the time taken by the drone to 
        % move from one waypoint to another, dividing the entire trajectory into a fixed 
        % number of waypoints, the time available to the drone to carry out the mission 
        % is implicitly fixed. In this case, if the number of waypoints is 20 and the 
        % movement time is 1 second, it means that the drone has 20 seconds each to perform 
        % the mission.
        
        % How long the drone should spend at that position
        inspection_time = 1;  % time required to inspect the point
        
        if time_estimation_method == 1 % maximum velocity
            margin_time = 1.8;  % The time available for the plan will be margin_time times the minimum feasible time
            
        elseif time_estimation_method == 2 % zero velocity (using splines)
            margin_time = 1.0;  % The time available for the plan will be margin_time times the minimum feasible time
            
        else
            % Stops execution of the file and gives control to the user's keyboard
            return
        end
        
        % Computation of the number of WPs:
        [ WPs_sequence , WPs_total ] = get_number_waypoints_multidepot(initial_position, ...
            goal, drones, sequence, max_vel, max_acc, constraint_max_axis_vel, ...
            constraint_max_axis_acc, margin_time, inspection_time, motion_time, time_estimation_method);
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% Timing %%%%%%%%%%%%%%%%%%%%%%%%%% %%
        % Parameters for STL formulas - interval (of time-step increments) 
        % in which formula has to be evaluated.
        
        for k = 1 : drones
            
            n_regions      = length(sequence{k})-1;
            I_installation = cell(n_regions,1);
            WPs_cumsum     = cumsum(WPs_sequence{k});
            
            for i = 1 : n_regions
                I_installation{i} = 1 + round( WPs_cumsum(2*i-1)/sampling_time) : 1 + round( WPs_cumsum(2*i)/sampling_time) ;
            end
            
            eval( [ 'parameters.I_installation.Drone_' num2str(k) '= I_installation;' ] )
        end
        
        % Message
        disp( ' ' );
        disp( 'The goal regions are represented in blue, the obstacles in red, the drone starting' );
        disp( 'points are in magenta');
        disp( ' ' );

        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% Script using STL specifications % %%

       	stl_power
end