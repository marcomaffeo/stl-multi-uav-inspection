
clear all
close all
clc

% enable_plot= 'n';
% enable_saving_txt = 'y';
% enable_traj_rot = 'y';
% enable_animation = 'n';
% enable_Belta_variation = 'y';
% time_estimation_method = '2';
% solver_choice = '1';
% enable_boolean = 'n';

%% Add paths for solvers and related functions

% Solvers
addpath('functions');
addpath('txt_files');
addpath('maps');
addpath('print');
addpath('missions');
addpath('missions_functions');
addpath('stl_formulae_functions');
% Add CasADi paths
if ismac
    addpath('../casadi-osx');
elseif ispc
    addpath('../casadi-windows');
else isunix
    addpath('../casadi');
end
% if isunix
%     addpath('../casadi');
%     addpath('../MPT3/tbxmanager');
%     addpath('../cvx');
%     tbxmanager restorepath % Run the pathdef file in the MPT3 folder
%     cd ../cvx % To setup CVX as a solver
%     cvx_setup cvx_license.dat
%     cvx_solver mosek % to use Mosek as LP solver
%     cvx_solver gurobi % to use Gurobi as LP solver
%     cvx_solver % to see if the changes apply
%     cd ../STL_examples
% elseif ispc
%     addpath('../casadi-windows');
%     addpath('../cvx-windows');
%     tbxmanager restorepath % Run the pathdef file in the MPT3 folder
%     cd ../cvx-windows % To setup CVX as a solver
%     cvx_setup cvx_license.dat
%     cvx_solver mosek % to use Mosek as LP solver
%     cvx_solver gurobi % to use Gurobi as LP solver
%     cvx_solver % to see if the changes apply
%     cd ../STL_examples
% elseif ismac
%     addpath('../casadi-osx');
% else
%     disp('Platform not supported')
% end
% 
% cvx_solver mosek

%% Simulation info

% Note: if the optimization returns errors for larger number of drones, change
% the constant C in the robustness functions to a smaller value. This can
% be done by modifying the "C" and "C_dist" variables
disp( ' ' );
disp( 'The init file allows to simulate a scenarios where four drones have to' );
disp( 'inspect a power line mock-up while avoiding obstacles placed along the way');
disp( '  1) Reach and avoid scenario (1 obstacle) considering 1 drone');
disp( '  2) Reach and avoid scenario (1 obstacle) considering 2 drone');
disp( '  3) Reach and avoid scenario (1 obstacle) considering 3 drone');
disp('---------------------------------------------------------------------------------');

disp( ' ' );
selection = str2double(input('Which scenario do you intend to simulate?\n','s'));
disp( ' ' );

enable_plot = input('Do you want to save the plots?y/n\n','s');
disp( ' ' );

enable_saving_txt = input('Do you want to save the obtained waypoints and trajectories?y/n\n','s');
disp( ' ' );

enable_traj_rot = input('Do you want to rotate and translate the obtained trajectories?y/n\n','s');
disp( ' ' );

enable_animation = input('Do you want to run the animation?y/n\n','s');
disp( ' ' );

% N. Mehdipour, C. -I. Vasile and C. Belta, "Specifying User Preferences Using Weighted Signal 
% Temporal Logic," in IEEE Control Systems Letters, vol. 5, no. 6, pp. 2006-2011, Dec. 2021, 
% doi: 10.1109/LCSYS.2020.3047362.
enable_Belta_variation = input('Do you want to use Belta''s min/max approximations?y/n\n','s');
disp( ' ' );

% Select the method to estimate the minimum feasible time
disp( ' ' );
disp('Which estimation method for the minimum feasible time do you want to use?\n');
disp('---------------------------------------------------------------------------------');
disp( ' 1 = reaching the targets with maximum velocity.  ' ); 
disp( ' 2 = reaching the targets with zero velocity (using splines). ' );
time_estimation_method = str2double(input(' \n','s'));
disp( ' ' );

% Choose the solver for computation acceleration
disp( ' ' );
disp('Which solver do you want to use?\n');
disp('---------------------------------------------------------------------------------');
disp( ' 1 = mumps.  ' ); 
disp( ' 2 = ma27. ' );
disp( ' 3 = ma57. ' );
solver_choice = str2double(input(' \n','s'));
disp( ' ' );

% Choose whether to run the optimization in boolean mode (y/n)
enable_boolean = input('Do you want to run the optimization in boolean mode?y/n\n','s');
disp( ' ' );

% To show the computation time
tic;

switch selection
     
    %%%%%%%%%%%%%%%%% CASE 1 %%%%%%%%%%%%%%%%%
    %  Scenario 1: Reach and avoid with 1 obstacle and 1 drone
    
    case 1
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% STL FORMULA %%%%%%%%%%%%%%%%%%%%% %%
        
        disp('This scenario encodes the following STL formula:');
        disp( ' ' );
        disp('\varphi_\mathrm{sep}^{i,j} = \square_{[0, T]} \left(\lVert {^W\mathbf{r}}^i - {^W\mathbf{r}}^j');
        disp('      \rVert  \geq \delta_\mathrm{min} \right), ');
        disp('\varphi_\mathrm{mra} = \wedge_{k=1}^N \varphi_\mathrm{ra}^k \bigwedge \wedge_{k=1}^N \left(');
        disp('      \wedge_{i \neq j} \varphi_\mathrm{sep}^{i,j} \right)');
        disp( ' ');
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% Parameters %%%%%%%%%%%%%%%%%%%%%% %%
        
        drones                = 1;
        map_name              = 'test_map_reach_avoid.txt';
        plotting_time         = 0.05; % seconds
        scale                 = 0.25; % scale the quadrotor dimension
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
        initial_position(:,1) = [-1.25; -1.25; 1.75]; % drone1 starting point
        
        % For Gazebo simulations:
        initial_position_gazebo = zeros(size(initial_position));
        % Rotz is part of the Phased Array toolbox
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
        delta_min = 0.1; % meter
        
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
        max_vel = 7; % max velocity value during the drone motion
        max_acc = 2; % max acceleration value during the drone motion
        
        % The parameters used for the min/max smooth approximation on the mutual
        % distance and STL mission requirements
        C      = 10; % STL specifications
        C_dist = 5;  % mutual distance

        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% Goals %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
       
        i = 1;
        goal{i}.stop = [1.75; 1.75; 0.75]; % First point to inspect - blue
        goal{i}.heading_angle = pi/2; % radians - drone orientation
        
        i = i + 1;
        goal{i}.stop = [-1.75; 1.75; 0.75]; % Second point to inspect - blue
        goal{i}.heading_angle = pi; % radians - drone orientation
        
        i = i + 1;
        goal{i}.stop = [1.75; -1.75; 0.75]; % Third point to inspect - blue
        goal{i}.heading_angle = 3*pi/2; % radians - drone orientation
        
        i = i + 1;
        goal{i}.stop = [-1.75; -1.75; 0.75]; % Fourth point to inspect - blue
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
        
        % Asking about the code execution
        disp( ' ' );
        selection_CVX = input('Do you want to use CVX to compute the optimal sequence?y/n\n','s');
        disp( ' ' );

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
        inspection_time = 5;  % time required to inspect the point
        
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

       	stl_reach_avoid
        
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
        map_name              = 'test_map_reach_avoid.txt';
        plotting_time         = 0.05; % seconds
        scale                 = 0.25; % scale the quadrotor dimension
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
        initial_position(:,1) = [-1.25; -1.25; 1.75]; % drone1 starting point
        initial_position(:,2) = [-1.25;  1.25; 1.75]; % drone2 starting point
        
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
        delta_min = 0.1; % meter
        
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
        max_vel = 7; % max velocity value during the drone motion
        max_acc = 2; % max acceleration value during the drone motion
        
        % The parameters used for the min/max smooth approximation on the mutual
        % distance and STL mission requirements
        C      = 10; % STL specifications
        C_dist = 5;  % mutual distance

        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% Goals %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
       
        i = 1;
        goal{i}.stop = [1.75; 1.75; 0.75]; % First point to inspect - blue
        goal{i}.heading_angle = pi/2; % radians - drone orientation
        
        i = i + 1;
        goal{i}.stop = [-1.75; 1.75; 0.75]; % Second point to inspect - blue
        goal{i}.heading_angle = pi; % radians - drone orientation
        
        i = i + 1;
        goal{i}.stop = [1.75; -1.75; 0.75]; % Third point to inspect - blue
        goal{i}.heading_angle = 3*pi/2; % radians - drone orientation
        
        i = i + 1;
        goal{i}.stop = [-1.75; -1.75; 0.75]; % Fourth point to inspect - blue
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
        
        % Asking about the code execution
        disp( ' ' );
        selection_CVX = input('Do you want to use CVX to compute the optimal sequence?y/n\n','s');
        disp( ' ' );

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
        inspection_time = 5;  % time required to inspect the point
        
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

       	stl_reach_avoid
        
        
    %%%%%%%%%%%%%%%%% CASE 3 %%%%%%%%%%%%%%%%%
    %  Scenario 3: Reach and avoid with 1 obstacle and 3 drones
    
    case 3
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% STL FORMULA %%%%%%%%%%%%%%%%%%%%% %%
        
        disp('This scenario encodes the following STL formula:');
        disp( ' ' );
        disp('\varphi_\mathrm{sep}^{i,j} = \square_{[0, T]} \left(\lVert {^W\mathbf{r}}^i - {^W\mathbf{r}}^j');
        disp('      \rVert  \geq \delta_\mathrm{min} \right), ');
        disp('\varphi_\mathrm{mra} = \wedge_{k=1}^N \varphi_\mathrm{ra}^k \bigwedge \wedge_{k=1}^N \left(');
        disp('      \wedge_{i \neq j} \varphi_\mathrm{sep}^{i,j} \right)');
        disp( ' ');
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% Parameters %%%%%%%%%%%%%%%%%%%%%% %%
        
        drones                = 3;
        map_name              = 'test_map_reach_avoid.txt';
        plotting_time         = 0.05; % seconds
        scale                 = 0.25; % scale the quadrotor dimension
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
        initial_position(:,1) = [-1.25; -1.25; 1.75]; % drone1 starting point
        initial_position(:,2) = [-1.25;  1.25; 1.75]; % drone2 starting point
        initial_position(:,3) = [ 1.5;   1.25; 1.75]; % drone3 starting point
        
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
        delta_min = 0.1; % meter
        
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
        max_vel = 7; % max velocity value during the drone motion
        max_acc = 2; % max acceleration value during the drone motion
        
        % The parameters used for the min/max smooth approximation on the mutual
        % distance and STL mission requirements
        C      = 10; % STL specifications
        C_dist = 5;  % mutual distance

        %% %%%%%%%%%%%%%%%%%%%%%%%%%%% Goals %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
       
        i = 1;
        goal{i}.stop = [1.75; 1.75; 0.75]; % First point to inspect - blue
        goal{i}.heading_angle = pi/2; % radians - drone orientation
        
        i = i + 1;
        goal{i}.stop = [-1.75; 1.75; 0.75]; % Second point to inspect - blue
        goal{i}.heading_angle = pi; % radians - drone orientation
        
        i = i + 1;
        goal{i}.stop = [1.75; -1.75; 0.75]; % Third point to inspect - blue
        goal{i}.heading_angle = 3*pi/2; % radians - drone orientation
        
        i = i + 1;
        goal{i}.stop = [-1.75; -1.75; 0.75]; % Fourth point to inspect - blue
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
        
        % Asking about the code execution
        disp( ' ' );
        selection_CVX = input('Do you want to use CVX to compute the optimal sequence?y/n\n','s');
        disp( ' ' );

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
        inspection_time = 5;  % time required to inspect the point
        
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

       	stl_reach_avoid
        
end
        
