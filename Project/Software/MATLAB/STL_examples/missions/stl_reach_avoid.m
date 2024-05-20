%% Import libraries

% Import CASADI library
import casadi.*

%% Main script

disp('Initialization...');

%% Reading map and getting obstacles

% Get obstacles from the map
obstacles = getObstacles(map_name);

% Load map (map_name, xy_res, z_res, margin
map = load_map(map_name, .5, .5, 0);

%% Plot the map (for the NLP solution)
NLP_results = figure();

% To plot the power line mock-up
if selection == 1 || selection == 2 || selection == 3
    plot_scenario(map, []);
end

% Number of goals within the cell
[~, goal_elements]     = size(goal);
[obstacle_elements, ~] = size(obstacles);
hold on;

% To plot the goal regions (they are in blue)
for i = 1 : goal_elements
    % The target point is represented as a polyhedron
    goal{i}.polyhedron = Polyhedron('lb', goal{i}.lb, 'ub', goal{i}.ub);
    plot(goal{i}.polyhedron, 'color', 'blue', 'alpha', 0.5); % plot the goals
end

% To plot the obstacles (they are in red)
for i = 1: obstacle_elements
    plot(obstacles{i}.shape, 'color', 'red', 'alpha', 0.5); % plot the obstacles
end

% To plot the starting point (they are in magenta)
for i = 1: drones
    initial_pos.shape = Polyhedron('lb', initial_position(:,i)-goal{i}.ds,'ub', ...
        initial_position(:,i)+goal{i}.ds);
    plot(initial_pos.shape, 'color', 'magenta', 'alpha', 0.5); % plot the obstacle
end

% To plot the map dimensions(they are in green)
data.dimension = map.map_dimension;

data.shape = Polyhedron('lb', data.dimension(1:3),'ub',data.dimension(4:6));
plot(data.shape,'EdgeColor',[0 0.5 0],'alpha', 0 ,'LineWidth', 2); % plot the map dimensions

% To improve the display
marg = 0.25;
axis ([data.dimension(1)-marg   data.dimension(4)+marg   data.dimension(2)-marg   data.dimension(5)+marg ...
    data.dimension(3)-marg   data.dimension(6)+marg]);

view(80,10)
title('NLP solution')

xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')


clear data

%% Plot the map again (for the LP solution)
LP_results = figure();

if selection == 1 || selection == 2 || selection == 3 
    plot_scenario(map, []);
end

hold on;

% To plot the goal regions (they are in blue)
for i = 1 : goal_elements
    plot(goal{i}.polyhedron, 'color', 'blue', 'alpha', 0.5); % plot the goals
end

% To plot the obstacles (they are in red)
for i = 1: obstacle_elements
    plot(obstacles{i}.shape, 'color', 'red', 'alpha', 0.5); % plot the obstacles
end

% To plot the starting point (they are in magenta)
for i = 1: drones
    initial_pos.shape = Polyhedron('lb', initial_position(:,i)-goal{i}.ds,'ub', ...
        initial_position(:,i)+goal{i}.ds);
    plot(initial_pos.shape, 'color', 'magenta', 'alpha', 0.5); % plot the obstacle
end

% To plot the map dimensions(they are in green)
data.dimension = map.map_dimension;

data.shape = Polyhedron('lb', data.dimension(1:3),'ub',data.dimension(4:6));
plot(data.shape,'EdgeColor',[0 0.5 0],'alpha', 0 ,'LineWidth', 2); % plot the map dimensions

% To improve the display
marg = 0.25;
axis ([data.dimension(1)-marg   data.dimension(4)+marg   data.dimension(2)-marg   data.dimension(5)+marg ...
    data.dimension(3)-marg   data.dimension(6)+marg]);

view(80,10)

title('LP solution')

xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

clear data

%% Spline definition

% See eq. (63) of the paper

% M. W. Mueller, M. Hehn and R. D'Andrea, "A Computationally Efficient
% Motion Primitive for Quadrocopter Trajectory Generation," in IEEE
% Transactions on Robotics, vol. 31, no. 6, pp. 1294-1310, Dec. 2015, 
% doi: 10.1109/TRO.2015.2479878.
M = 1/(2*motion_time^5) * ...
    [ 90                 0   -15*motion_time^2;
     -90*motion_time     0    15*motion_time^3;
      30*motion_time^2   0    -3*motion_time^4];
  
% The bounds on the velocity can be expressed as bounds in the WPs and between WPs.
%
% - As for the bounds in the WPs, they can be imposed by evaluating eq. (22)
% with the parameters (alpha, beta, gamma) retrieved from eq. (63). In
% other words, imposing fixed position and acceleration at the initial (p_0,v_0,a_0) and
% final (p_f,v_f,a_f) points. For the considered case scenario, Delta_a = 0
% (i.e. a_k = a_{k-1} k=[0,...,N] where N is the nummber of WPs),with a_0 = 0.
% This means: alpha=90/(2*T^5)*Delta_p, beta=-90/(2*T^4)*Delta_p and gamma=30/(2*T^3)*Delta_p.
% Now, we can replace alpha, beta and gamma in eq. (22) with t=T which means:
% 
% v_star(t=T) = \bar{v}*Delta_p+v_0   where   \bar{v} = 90/(48*T)-90/(12*T)+30/(4*T)
%
% Bounding this expression between \underline{vel_max} <= v_star(t=T) <=
% \bar{vel_max}, we assure that the velocity in the WPs will lie in the
% interval.
% As the problem is formulated with Casadi, we do not to insert this
% equation in g, i.e. the constraints of the problem because the velocity
% is declared as a symbolic variable and bounded in this interval (through
% variables lbv and ubv)
% 
% - As for the bounds between the WPs, they can be imposed by evaluating eq. (22)
% with the parameters (alpha, beta, gamma) retrieved from eq. (63). In
% other words, imposing fixed position and acceleration at the initial (p_0,v_0,a_0) and
% final (p_f,v_f,a_f) points. For the considered case scenario, Delta_a = 0
% (i.e. a_k = a_{k-1} k=[0,...,N] where N is the nummber of WPs),with a_0 = 0.
% This means: alpha=90/(2*T^5)*Delta_p, beta=-90/(2*T^4)*Delta_p and gamma=30/(2*T^3)*Delta_p.
% To evaluate the maximum of v(t) in the interval [v_{k-1} v_k] we compute
% the derivative of v over the time:
% 
% dv(t)/dt= a(t) = a_star (see eq.(22)).
% 
% Now, we impose the derivative equals to zero dv(t)/dt= a(t) =0 to
% retrieve the time value (t_v^prime) where the function reaches the maximum (or
% minimum) in a closed form (t_v^prime = T with t_v^prime in the interval [0 T]).
% As the maximum value is reached in t_v^prime = T (i.e. just in the WPs), we do not
% need to add any additional constraint in g.
% 
% Following the same approach as for the velocity, the bounds on the acceleration
% can be expressed as bounds in the WPs and between WPs.
%
% - As for the bounds in the WPs, we can express the bounds in a closed form as:
% 
% a_star(t=T) = \bar{a}*Delta_p+a_0   where   \bar{a} = 90/(12*T^2)-90/(4*T^2)+30/(2*T^2)
%
% As Delta_a = 0 for all the WPs, the maximum can never be in the WPs (it will be between them).
% 
% - As for the bounds between the WPs, they can be imposed by evaluating eq. (22)
% with the parameters (alpha, beta, gamma) retrieved from eq. (63). In
% other words, imposing fixed position and acceleration at the initial (p_0,v_0,a_0) and
% final (p_f,v_f,a_f) points. For the considered case scenario, Delta_a = 0
% (i.e. a_k = a_{k-1} k=[0,...,N] where N is the nummber of WPs),with a_0 = 0.
% This means: alpha=90/(2*T^5)*Delta_p, beta=-90/(2*T^4)*Delta_p and gamma=30/(2*T^3)*Delta_p.
% To evaluate the maximum of a_star in the interval [a_{k-1} a_k] we compute
% the derivative of a_star over the time:
% 
% da_star(t)/dt = 3*alpha/6*t^2+beta*t+gamma.
% 
% Now, we impose the derivative of a_star equals to zero i.e. da_star(t)/dt=0 to
% retrieve the time value (t_a^prime) where the function reaches the maximum (or
% minimum) in a closed form (t_a^prime = (1-sqrt(3)/3)*T with t_a^prime in the interval [0 T]).
% Therefore, the maximum value of the acceleration in the time interval [0 T]
% can be obtained as follows:
% 
% a_star(t=t_a^prime) = \tilde{a}*Delta_p+a_0   where   \tilde{a} =
% 90/(12*T^5)*(t_a^prime)^3 -90/(4*T^4)*(t_a^prime)^2 +30/(2*T^3)*(t_a^prime)
%
% This value has to be bounded between the admissible values that the
% acceleration can assume, i.e.: \underline{acc_max} <= a_star(t=t_a^prime) <=
% \bar{acc_max}

t_prime_a = (1-sqrt(3)/3)*motion_time;

acc_bar = (90/12)*t_prime_a^3/motion_time^5 - (90/4)*t_prime_a^2/motion_time^4 + ...
    (30/2)*t_prime_a/motion_time^3;

% Start and stop values assumed from/at rest
da = 0;
dv = 0;

time = sampling_time : sampling_time: motion_time;
number_steps = WPs_total*numel(time)+1;

Clen = 3*(WPs_total + 1); % for each axis

%% Optimization problem parameters
optimizationParameters.sampling_time           = sampling_time; % Samples per waypoint
optimizationParameters.motion_time             = motion_time;
optimizationParameters.M                       = M;
optimizationParameters.acc_bar                 = acc_bar;
optimizationParameters.max_vel                 = max_vel;
optimizationParameters.max_acc                 = max_acc;
optimizationParameters.da                      = da;
optimizationParameters.dv                      = dv;
optimizationParameters.obstacle_elements       = obstacle_elements;
optimizationParameters.goal_elements           = goal_elements;
optimizationParameters.WPs_total               = WPs_total;
optimizationParameters.goal                    = goal;
optimizationParameters.obstacles               = obstacles;
optimizationParameters.map                     = map;
optimizationParameters.constraint_max_axis_vel = constraint_max_axis_vel;
optimizationParameters.constraint_max_axis_acc = constraint_max_axis_acc;
optimizationParameters.initial_position        = initial_position;
optimizationParameters.drones                  = drones;
optimizationParameters.delta_min               = delta_min;
optimizationParameters.Clen                    = Clen;
optimizationParameters.C                       = C; % STL specifications
optimizationParameters.C_dist                  = C_dist; % mutual distance
optimizationParameters.enable_Belta_variation  = enable_Belta_variation;

if selection == 1 || selection == 2 || selection == 3
    optimizationParameters.sequence     = sequence;
    optimizationParameters.WPs_sequence = WPs_sequence;
end   

%% Getting the initial solution through Linear Programming

% This part of the script solves the LP optimization problem
disp('Getting initial solution...');

p0 = [];
v0 = [];

% This loop runs over the number of drones
for i = 1 : drones
    
    % Looking for the feasibility of the problem. If "temp_p" and temp_v" are
    % empty, this means the problem is unfeasible
    % [p0, v0] = get_initial_waypoints(initial_state, optimizationParameters, ...
    %   selection, iteration)
    [temp_p, temp_v] = get_initial_waypoints([initial_position(:,i); ...
        initial_velocity(:,i)], optimizationParameters, selection, i);
    
    % In case the "temp_p" and "temp_v" are empty, this means the problem
    % has not a solution. Therefore, the scripts stops and gives back the
    % control to the user
    if(sum(isnan(temp_p))>0 || sum(isnan(temp_v))>0)
        disp('Init unfeasible');
        % Stops execution of the file and gives control to the user's keyboard
        return;
    end
    
    % This part of the code allows obtaining the topic waypoint making up
    % the entire trajectory
    p0 = [p0; temp_p]; % the empty vector + the solution retrieved by the Linear Programming problem
    v0 = [v0; temp_v]; % the empty vector + the solution retrieved by the Linear Programming problem

end

var0 = [p0; v0];

%% Trajectory generation

disp('Trajectory generation...');

% Filling lower and upper bounds for the obstacles
for i = 1 : obstacle_elements
    optimizationParameters.obstacles_lb_N{i} = repmat(obstacles{i}.lb, ...
        number_steps, 1);
    optimizationParameters.obstacles_ub_N{i} = repmat(obstacles{i}.ub, ...
        number_steps, 1);
end

% Filling lower and upper bounds for the goal (target) regions
for i = 1 : goal_elements
    optimizationParameters.goal{i}.goal_lb_N = repmat(goal{i}.lb', ...
        number_steps, 1);
    optimizationParameters.goal{i}.goal_ub_N = repmat(goal{i}.ub', ...
        number_steps, 1);
end

% Filling lower and upper bounds for the home (target) regions
for i = 1 : drones
    home.lb = initial_position(:,i) - goal{1}.ds;
    home.ub = initial_position(:,i) + goal{1}.ds;
    
    optimizationParameters.home{i}.goal_lb_N = repmat(home.lb', ...
        number_steps, 1);
    optimizationParameters.home{i}.goal_ub_N = repmat(home.ub', ...
        number_steps, 1);
end

% Variable initialization
p   = []; % initialization position vector 
v   = []; % initialization velocity vector
lbp = []; % initialization lower bound position
ubp = []; % initialization upper bound position
lbv = []; % initialization lower bound velocity
ubv = []; % initialization upper bound velocity
g   = []; % initialization g function
lbg = []; % initialization lower bound g function
ubg = []; % initialization upper bound g function

% This loop runs over all the available drones
for j = 1 : drones
    
    % The symbolic variable p_j_0, with j goes from 1 to the number of
    % available drones
    % Symbolic expression of the problem. More information about the MX.sym and
    % MX.zeros commands are available here: https://web.casadi.org/docs/
    initial_position_symbolic = MX.sym(['p_' num2str(j) '_' num2str(0)], 3, 1);
    
    p = [p; initial_position_symbolic];  % p is initialized as empty vector
    
    lbp = [lbp; initial_position(:,j)];
    ubp = [ubp; initial_position(:,j)];
    
    % The symbolic variable v_j_0, with j goes from 1 to the number of
    % available drones
    initial_velocity_symbolic = MX.sym(['v_' num2str(j) '_' num2str(0)], 3, 1);
    
    v = [v; initial_velocity_symbolic];  % v is initialized as empty vector
    
    lbv = [lbv; initial_velocity(:,j)];
    ubv = [ubv; initial_velocity(:,j)];
    
    % This loop runs over all the available waypoints. The trajectory per
    % each drone is dived in N topic waypoints
    for k = 1 : WPs_total
        
        % The symbolic variabile p_j_k, with j goes from 1 to the number of
        % available drones and k goes from 1 to the M wayapoints
        position_symbolic = MX.sym(['p_' num2str(j) '_' num2str(k)], 3, 1);
        p = [p; position_symbolic];
        
        % Overall bounds on movement. In particular, bounds on the position that cannot
        % exceed the dimension of the map
        lbp = [lbp; map.map_dimension(1:3)']; % lower bounds on the map along each axis
        ubp = [ubp; map.map_dimension(4:6)']; % upper bounds on the map along each axis
        
        % The symbolic variabile v_j_k, with j goes from 1 to the number of
        % available drones and k goes from 1 to the M wayapoints
        velocity_symbolic = MX.sym(['v_' num2str(j) '_' num2str(k)], 3, 1);
        v = [v; velocity_symbolic];
        
        % There are no bounds on the acceleration because there are no
        % symbolic variables accounting for it in the problem formulation
        % This allows accounting for the bounds on the velocity at the WPs
        lbv = [lbv; -max_vel*constraint_max_axis_vel(:,j) ]; % lower bounds on the velocity
        ubv = [ubv;  max_vel*constraint_max_axis_vel(:,j) ]; % upper bounds on the velocity
        
        % distance variation for all axes, i.e., p = p{k} - p{k-1}. This
        % is possible because the problem is decoupled along each axis of
        % the inertial frame.
        % The variable Clen allows to shift the vector depending on the
        % number of drones (j accounts for it in the first loop). The
        % numbers 1, 2, and 3 refer to the x, y, and z-axis, respectively.
        difference_p_x = p(k * 3 + 1 + (j-1) * Clen) - p( (k-1) * 3 + 1 + (j-1) * Clen); % along x-axis
        difference_p_y = p(k * 3 + 2 + (j-1) * Clen) - p( (k-1) * 3 + 2 + (j-1) * Clen); % along y-axis
        difference_p_z = p(k * 3 + 3 + (j-1) * Clen) - p( (k-1) * 3 + 3 + (j-1) * Clen); % along z-axis
        
        % velocity variation for all axes, i.e.,v{k} - v{k-1}. This
        % is possible because the problem is decoupled along each axis of
        % the inertial frame. 
        % As before, the variable Clen allows shifting the vector depending
        % on the number of drones (the first loop runs over it)
        v_x_k = v( k * 3 + 1 + (j-1) * Clen); % v{k} along x-axis
        v_y_k = v( k * 3 + 2 + (j-1) * Clen); % v{k} along y-axis
        v_z_k = v( k * 3 + 3 + (j-1) * Clen); % v{k} along z-axis
        
        % km1 stands for k minus 1
        v_x_km1 = v( (k-1) * 3 + 1 + (j-1) * Clen); % v{k-1} along x-axis
        v_y_km1 = v( (k-1) * 3 + 2 + (j-1) * Clen); % v{k-1} along y-axis
        v_z_km1 = v( (k-1) * 3 + 3 + (j-1) * Clen); % v{k-1} along z-axis
        
        % Computing \alpha, \beta, and \gamma values per each axis. a_0 is
        % supposed to be equal to zero
        alfa_x  = M(1,:) * [difference_p_x - motion_time * v_x_km1; dv; da]; 
        beta_x  = M(2,:) * [difference_p_x - motion_time * v_x_km1; dv; da];
        gamma_x = M(3,:) * [difference_p_x - motion_time * v_x_km1; dv; da];

        alfa_y  = M(1,:) * [difference_p_y - motion_time * v_y_km1; dv; da];
        beta_y  = M(2,:) * [difference_p_y - motion_time * v_y_km1; dv; da];
        gamma_y = M(3,:) * [difference_p_y - motion_time * v_y_km1; dv; da];

        alfa_z  = M(1,:) * [difference_p_z - motion_time * v_z_km1; dv; da];
        beta_z  = M(2,:) * [difference_p_z - motion_time * v_z_km1; dv; da];
        gamma_z = M(3,:) * [difference_p_z - motion_time * v_z_km1; dv; da];
        
        % Velocity at the final/end point. This comes from eq. 22 of D'Andrea's
        % paper. We aim to solve the problem constraining the shape of the splines,
        % expressed in terms of position and velocity. Eventually, we want
        % to be in the final region and go there with zero velocity. v_f
        % stands for velocity final
        v_f_x = (alfa_x/24) * motion_time^4 + (beta_x/6) * motion_time^3 + ...
            (gamma_x/2) * motion_time^2 + v_x_km1;

        v_f_y = (alfa_y/24)*motion_time^4 + (beta_y/6) * motion_time^3 + ...
            (gamma_y/2) * motion_time^2 + v_y_km1;

        v_f_z = (alfa_z/24)*motion_time^4 + (beta_z/6) * motion_time^3 + ...
            (gamma_z/2) * motion_time^2 + v_z_km1;

        % "g" contains all the constraints to the optimization problem:
        % - constraint on the optimal acceleration (see eq.(24) D'Andrea's
        % paper) between WPs
        % - the final velocity is equal to the velocity star for the x, y, and
        % z-axis
        %
        g = [g; % this is empty at the beginning (for iteration)
             %
             % See equation (24) of D'Andrea's paper. a_0 = 0
             acc_bar * difference_p_x - motion_time * acc_bar * v_x_km1; % from acceleration constraints
             acc_bar * difference_p_y - motion_time * acc_bar * v_y_km1; % from acceleration constraints
             acc_bar * difference_p_z - motion_time * acc_bar * v_z_km1; % from acceleration constraints
             % This pushes the optimization problem to have next velocity
             % values as the splines in eq.(22)
             v_x_k - v_f_x; 
             v_y_k - v_f_y;
             v_z_k - v_f_z]; % from velocity dynamics
         
        % Considering the constraints on the maximum acceleration in the optimization problem
        lbg = [lbg; -max_acc * constraint_max_axis_acc(:,j); zeros(3,1)]; % lower bounds
        ubg = [ubg;  max_acc * constraint_max_axis_acc(:,j); zeros(3,1)]; % upper bounds
                
    end
        
end

%% Casadi set up

var = [p; v];
var_ub = [ubp; ubv];
var_lb = [lbp; lbv];

% The problem is setup in a way to consider the cost function as a
% constraint and the object function as zero. This minimize the computation
% time.
if strcmp(enable_boolean, 'y') % in case of boolean optimization
    g = [g; -case_inspection_Ndrones(var, optimizationParameters, parameters, selection)];
    lbg = [lbg;eps];
    ubg = [ubg;inf];
end

% Casadi settings. For better performance, you can replace "numps" solver
% with "ma27" or "ma57". If you do not have it, change this option below
if solver_choice == 1 % mumps
    options = struct('ipopt', struct('tol', 1e-6, 'acceptable_tol', 1e-4, 'max_iter', 5000, ...
        'linear_solver', 'mumps', 'hessian_approximation', 'limited-memory', ...
        'print_level',0)); % mumps, ma27, ma57, limited-memory
elseif solver_choice == 2 % ma27
    options = struct('ipopt', struct('tol', 1e-6, 'acceptable_tol', 1e-4, 'max_iter', 5000, ...
        'linear_solver', 'ma27', 'hessian_approximation', 'limited-memory', ...
        'print_level',0)); % mumps, ma27, ma57, limited-memory
elseif solver_choice == 3 % ma57
    options = struct('ipopt', struct('tol', 1e-6, 'acceptable_tol', 1e-4, 'max_iter', 5000, ...
        'linear_solver', 'ma57', 'hessian_approximation', 'limited-memory', ...
        'print_level',0)); % mumps, ma27, ma57, limited-memory
else
    % Stops execution of the file and gives control to the user's keyboard
    return;
end

options.print_time = false;
options.expand     = false;
options.verbose    = true;

% Minimize the function "case_inspection_lausanne_Ndrones" with respect to
% "p" and subject to "g" constraints
if strcmp(enable_boolean, 'y') % in case of boolean optimization
    prob = struct('f', 0, 'x', var, 'g', g);
else
    prob = struct('f', case_inspection_Ndrones(var, optimizationParameters, parameters, ...
        selection), 'x', var, 'g', g);
end

% Nonlinear programs. See https://web.casadi.org/docs/
solver = nlpsol('solver', 'ipopt', prob, options);

%% Solving the NLP

disp('Solving...');

sol = solver('x0', var0, 'lbx', var_lb, 'ubx', var_ub,'lbg', lbg, 'ubg', ubg);

%% Showing the solution

time_taken = toc % the time spent in solving the problem

p_opt = full(sol.x);

%% Plotting the solution

disp('Plotting...(press any key to start)');

% Waiting for user input
% pause;

% List of waypoints
waypoints    = cell(optimizationParameters.drones,1);
waypoints_LP = cell(optimizationParameters.drones,1);


% The maximum number of drones is fixed to four. When simulating more
% device, enter a new color and line type
% Path are indicate with dashed line
mar{1}  = 'k*'; % color and line type, i.e., black and dashed
mar{2}  = 'g*'; % color and line type, i.e., green and dashed
mar{3}  = 'r*'; % color and line type, i.e., red and dashed
mar{4}  = 'b*'; % color and line type, i.e., blue and dashed
mar{5}  = 'c*'; % color and line type, i.e., blue and dashed
mar{6}  = 'm*'; % color and line type, i.e., blue and dashed
mar{7}  = 'y*'; % color and line type, i.e., blue and dashed
mar{8}  = 'r*'; % color and line type, i.e., blue and dashed
mar{9}  = 'g*'; % color and line type, i.e., blue and dashed
mar{10} = 'k*'; % color and line type, i.e., blue and dashed
mar{11} = 'b*'; % color and line type, i.e., blue and dashed
mar{12} = 'c*'; % color and line type, i.e., blue and dashed

color_drone = [ 0.93 0.69 0.13 ;   % drone 1
                0.49 0.18 0.56 ;   % drone 2
                0.26 0.72 0.54 ;   % drone 3
                0.46 0.32 0.74 ];  % drone 4

% Plotting the waypoints
figure(NLP_results)

for n = 1 : drones

    waypoints{n} = reshape(p_opt (1 + (n - 1) * (WPs_total + 1)*3:...
        (n)*(WPs_total + 1) * 3), 3 , WPs_total + 1);
    hold all;
    plot3(waypoints{n}(1,:), waypoints{n}(2,:), waypoints{n}(3,:), mar{n});

end

% Evaluating the STL optimization problem with doubles as input
[negative_rob, pos_x, pos_y, pos_z] = case_inspection_Ndrones(p_opt, ...
        optimizationParameters, parameters, selection); 
    
% The heading vector. It is a zeros vector and has the same dimension of
% pos_x, pos_y and pos_z (the drone position in the 3D space)
heading_vector = zeros(size(pos_x,1), optimizationParameters.drones); 

% Computing the heading
for d = 1 : optimizationParameters.drones

    WPs_cumsum = cumsum(WPs_sequence{d});
    n_regions  = length(sequence{d})-1;

    for i = 1 : length(pos_x)

        % From the second point on
        if i > 1
            heading_vector(i,d) = atan2(pos_y(i,d) - pos_y(i-1,d), pos_x(i,d) - pos_x(i-1,d));    
        % i < 1     
        else % initialization step (starting point)
            heading_vector(i,d) = atan2(pos_y(i,d), pos_x(i,d));
        end

    end

    % This part of the problem imposes that the position has to be equal
    % to the target at a certain number of waypoints, and therefore time
    for i = 1 : n_regions - 1  % current_target
        heading_vector(  eval( [ 'parameters.I_installation.Drone_' num2str(d) '{' num2str(i) '}' ] )  ,d) = ...
            goal{sequence{d}(i+1)}.heading_angle;           % target region
    end

end

% Plotting the path
for d = 1 : optimizationParameters.drones

    hold all;
    plot3(pos_x(:,d), pos_y(:,d), pos_z(:,d), '-.', 'color', color_drone(d,:), 'linewidth', 1.5);

end

% --- Plotting the LP solution: ----------------------------------------- %
figure(LP_results)

for n = 1 : drones

    waypoints_LP{n} = reshape(var0 (1 + (n - 1) * (WPs_total + 1)*3:...
        (n)*(WPs_total + 1) * 3), 3 , WPs_total + 1);
    hold all;
    plot3(waypoints_LP{n}(1,:), waypoints_LP{n}(2,:), waypoints_LP{n}(3,:), mar{n});

end

% Evaluating the STL optimization problem with doubles as input
[~, pos_x_LP, pos_y_LP, pos_z_LP] = case_inspection_Ndrones(var0, ...
        optimizationParameters, parameters, selection); 

% Plotting the path
for d = 1 : optimizationParameters.drones

    hold all;
    plot3(pos_x_LP(:,d), pos_y_LP(:,d), pos_z_LP(:,d), '-.', 'color',color_drone(d,:), 'linewidth', 1.5);

end  

% ----------------------------------------------------------------------- %

figure(NLP_results)

if strcmp(enable_animation, 'y')

    % Load quadrotor structure
    load('Quadrotor_plotting_model.mat');

    % Scaling quadrotor dimension
    Quad.l = Quad.l * scale;
    Quad.t = Quad.t * scale;
    Quad.plot_arm = Quad.plot_arm * scale;
    Quad.plot_arm_t = Quad.plot_arm_t * scale;
    Quad.plot_prop = Quad.plot_prop * scale;
    Quad.X_armX = Quad.X_armX * scale;
    Quad.X_armY = Quad.X_armY * scale;
    Quad.X_armZ = Quad.X_armZ * scale;
    Quad.Y_armX = Quad.Y_armX * scale;
    Quad.Y_armY = Quad.Y_armY * scale;
    Quad.Y_armZ = Quad.Y_armZ * scale;
    Quad.Motor1X = Quad.Motor1X * scale;
    Quad.Motor1Y = Quad.Motor1Y * scale;
    Quad.Motor1Z = Quad.Motor1Z * scale;
    Quad.Motor2X = Quad.Motor2X * scale;
    Quad.Motor2Y = Quad.Motor2Y * scale;
    Quad.Motor2Z = Quad.Motor2Z * scale;
    Quad.Motor3X = Quad.Motor3X * scale;
    Quad.Motor3Y = Quad.Motor3Y * scale;
    Quad.Motor3Z = Quad.Motor3Z * scale;
    Quad.Motor4X = Quad.Motor4X * scale;
    Quad.Motor4Y = Quad.Motor4Y * scale;
    Quad.Motor4Z = Quad.Motor4Z * scale;

    % Cell containing the quadrotors position and orientations
    for i = 1 : optimizationParameters.drones

        Quad_cell{i} = Quad;

    end


    % For the animation
    for t = 1 : size(pos_x,1)

        for d = 1 : optimizationParameters.drones

            % If it is going to represent the second frame
            if t > 1

                % Make unvisible the previous drone position
                set(Quad_cell{d}.X_arm, 'xdata', Quad_cell{d}.Xtemp + Quad_cell{d}.X, 'ydata', ...
                    Quad_cell{d}.Ytemp + Quad_cell{d}.Y, 'zdata', Quad_cell{d}.Ztemp + ...
                    Quad_cell{d}.Z, 'Visible', 'off')

                set(Quad_cell{d}.Y_arm, 'xdata', Quad_cell{d}.Xtemp + Quad_cell{d}.X, 'ydata', ...
                    Quad_cell{d}.Ytemp + Quad_cell{d}.Y, 'zdata', Quad_cell{d}.Ztemp + ...
                    Quad_cell{d}.Z, 'Visible', 'off')

                set(Quad_cell{d}.Motor1, 'xdata', Quad_cell{d}.Xtemp + Quad_cell{d}.X, 'ydata', ...
                    Quad_cell{d}.Ytemp + Quad_cell{d}.Y, 'zdata', Quad_cell{d}.Ztemp + ...
                    Quad_cell{d}.Z - 2*Quad_cell{d}.t, 'Visible', 'off')

                set(Quad_cell{d}.Motor2, 'xdata', Quad_cell{d}.Xtemp + Quad_cell{d}.X, 'ydata', ...
                    Quad_cell{d}.Ytemp + Quad_cell{d}.Y, 'zdata', Quad_cell{d}.Ztemp + ...
                    Quad_cell{d}.Z - 2 * Quad_cell{d}.t, 'Visible', 'off')

                set(Quad_cell{d}.Motor3, 'xdata', Quad_cell{d}.Xtemp + Quad_cell{d}.X, 'ydata', ...
                    Quad_cell{d}.Ytemp + Quad_cell{d}.Y, 'zdata', Quad_cell{d}.Ztemp + ...
                    Quad_cell{d}.Z - 2 * Quad_cell{d}.t, 'Visible', 'off')

                set(Quad_cell{d}.Motor4, 'xdata', Quad_cell{d}.Xtemp + Quad_cell{d}.X, 'ydata', ...
                    Quad_cell{d}.Ytemp + Quad_cell{d}.Y, 'zdata', Quad_cell{d}.Ztemp + ...
                    Quad_cell{d}.Z - 2 * Quad_cell{d}.t, 'Visible', 'off')

            end

            % To represent the quadrotor in the 3D space
            Quad_cell{d}.X_arm = patch('xdata', Quad_cell{d}.X_armX, 'ydata', Quad_cell{d}.X_armY, ...
                'zdata', Quad_cell{d}.X_armZ, 'facealpha', .9, 'facecolor', 'b');
            Quad_cell{d}.Y_arm = patch('xdata', Quad_cell{d}.Y_armX, 'ydata', Quad_cell{d}.Y_armY, ...
                'zdata', Quad_cell{d}.Y_armZ, 'facealpha', .9, 'facecolor', 'b');
            Quad_cell{d}.Motor1 = patch('xdata', Quad_cell{d}.Motor1X, 'ydata', Quad_cell{d}.Motor1Y, ...
                'zdata', Quad_cell{d}.Motor1Z, 'facealpha', .3, 'facecolor', 'g');
            Quad_cell{d}.Motor2 = patch('xdata', Quad_cell{d}.Motor2X, 'ydata', Quad_cell{d}.Motor2Y, ...
                'zdata', Quad_cell{d}.Motor2Z, 'facealpha', .3, 'facecolor', 'k');
            Quad_cell{d}.Motor3 = patch('xdata', Quad_cell{d}.Motor3X, 'ydata', Quad_cell{d}.Motor3Y, ...
                'zdata', Quad_cell{d}.Motor3Z, 'facealpha', .3, 'facecolor', 'k');
            Quad_cell{d}.Motor4 = patch('xdata', Quad_cell{d}.Motor4X, 'ydata', Quad_cell{d}.Motor4Y, ...
                'zdata', Quad_cell{d}.Motor4Z, 'facealpha', .3, 'facecolor', 'k');

            % Quadrotor position and attitude
            Quad_cell{d}.X = pos_x(t,d); Quad_cell{d}.Y = pos_y(t,d); Quad_cell{d}.Z = pos_z(t,d); 
            Quad_cell{d}.phi = 0; Quad_cell{d}.theta = 0; Quad_cell{d}.psi = heading_vector(t,d);

            % Plot the quadrotor
            [Quad_cell{d}.Xtemp, Quad_cell{d}.Ytemp, Quad_cell{d}.Ztemp] = ...
                rotateBFtoGF(Quad_cell{d}.X_armX, Quad_cell{d}.X_armY, Quad_cell{d}.X_armZ, ...
                Quad_cell{d}.phi, Quad_cell{d}.theta, Quad_cell{d}.psi);
            % Update the new position within the graph
            set(Quad_cell{d}.X_arm, 'xdata', Quad_cell{d}.Xtemp + Quad_cell{d}.X, 'ydata', ...
                Quad_cell{d}.Ytemp + Quad_cell{d}.Y, 'zdata', Quad_cell{d}.Ztemp + Quad_cell{d}.Z)

            [Quad_cell{d}.Xtemp, Quad_cell{d}.Ytemp, Quad_cell{d}.Ztemp] = ...
                rotateBFtoGF(Quad_cell{d}.Y_armX, Quad_cell{d}.Y_armY, Quad_cell{d}.Y_armZ, ...
                Quad_cell{d}.phi, Quad_cell{d}.theta, Quad_cell{d}.psi);
            % Update the new position within the graph
            set(Quad_cell{d}.Y_arm, 'xdata', Quad_cell{d}.Xtemp + Quad_cell{d}.X, 'ydata', ...
                Quad_cell{d}.Ytemp + Quad_cell{d}.Y, 'zdata', Quad_cell{d}.Ztemp + Quad_cell{d}.Z)

            [Quad_cell{d}.Xtemp, Quad_cell{d}.Ytemp, Quad_cell{d}.Ztemp] = ...
                rotateBFtoGF(Quad_cell{d}.Motor1X, Quad_cell{d}.Motor1Y, Quad_cell{d}.Motor1Z, ...
                Quad_cell{d}.phi, Quad_cell{d}.theta, Quad_cell{d}.psi);
            % Update the new position within the graph
            set(Quad_cell{d}.Motor1, 'xdata', Quad_cell{d}.Xtemp + Quad_cell{d}.X, 'ydata', ...
                Quad_cell{d}.Ytemp + Quad_cell{d}.Y, 'zdata', Quad_cell{d}.Ztemp + ...
                Quad_cell{d}.Z - 2*Quad_cell{d}.t)

            [Quad_cell{d}.Xtemp, Quad_cell{d}.Ytemp, Quad_cell{d}.Ztemp] = ...
                rotateBFtoGF(Quad_cell{d}.Motor2X, Quad_cell{d}.Motor2Y, Quad_cell{d}.Motor2Z, ...
                Quad_cell{d}.phi,Quad_cell{d}.theta,Quad_cell{d}.psi);
            % Update the new position within the graph
            set(Quad_cell{d}.Motor2, 'xdata', Quad_cell{d}.Xtemp + Quad_cell{d}.X, 'ydata', ...
                Quad_cell{d}.Ytemp + Quad_cell{d}.Y, 'zdata', Quad_cell{d}.Ztemp + ...
                Quad_cell{d}.Z - 2 * Quad_cell{d}.t)

            [Quad_cell{d}.Xtemp, Quad_cell{d}.Ytemp, Quad_cell{d}.Ztemp] = ...
                rotateBFtoGF(Quad_cell{d}.Motor3X, Quad_cell{d}.Motor3Y, Quad_cell{d}.Motor3Z, ...
                Quad_cell{d}.phi, Quad_cell{d}.theta, Quad_cell{d}.psi);
            % Update the new position within the graph
            set(Quad_cell{d}.Motor3, 'xdata', Quad_cell{d}.Xtemp + Quad_cell{d}.X, 'ydata', ...
                Quad_cell{d}.Ytemp + Quad_cell{d}.Y, 'zdata', Quad_cell{d}.Ztemp + ...
                Quad_cell{d}.Z - 2 * Quad_cell{d}.t)

            [Quad_cell{d}.Xtemp, Quad_cell{d}.Ytemp, Quad_cell{d}.Ztemp] = ...
                rotateBFtoGF(Quad_cell{d}.Motor4X, Quad_cell{d}.Motor4Y, Quad_cell{d}.Motor4Z, ...
                Quad_cell{d}.phi, Quad_cell{d}.theta, Quad_cell{d}.psi);
            % Update the new position within the graph
            set(Quad_cell{d}.Motor4, 'xdata', Quad_cell{d}.Xtemp + Quad_cell{d}.X, 'ydata', ...
                Quad_cell{d}.Ytemp + Quad_cell{d}.Y, 'zdata', Quad_cell{d}.Ztemp + ...
                Quad_cell{d}.Z - 2 * Quad_cell{d}.t)

        end

        pause(plotting_time); 

    end

end

%% Trajectory rotation

if strcmp(enable_traj_rot, 'y')
   
    disp('Rotating obtained trajectories...');
    
    angle_deg = rad2deg(angle_rad); % degrees
    
    % Rotation angle specified as a real-valued scalar. The rotation angle is 
    % positive if the rotation is in the counter-clockwise direction when 
    % viewed by an observer looking along the z-axis towards the origin. 
    % Angle units are in degrees.
    Rz = rotz(angle_deg); % rotation matrix along z-axis
    
    % Vector initialization
    pos_x_rot   = zeros(length(pos_x), optimizationParameters.drones);
    pos_y_rot   = zeros(length(pos_x), optimizationParameters.drones);
    pos_z_rot   = zeros(length(pos_x), optimizationParameters.drones);
    heading_rot = zeros(length(pos_x), optimizationParameters.drones);
    
    for d = 1 : optimizationParameters.drones
           
        for i = 1 : length(pos_x)

            temp = Rz * [pos_x(i,d); pos_y(i,d); pos_z(i,d)];
            pos_x_rot(i,d) = temp(1); 
            pos_y_rot(i,d) = temp(2);
            pos_z_rot(i,d) = temp(3);

        end
            
        WPs_cumsum = cumsum(WPs_sequence{d});
        n_regions  = length(sequence{d})-1;
        
        for i = 2 : length(pos_x_rot)
            
            % From the second point on
            if i > 1
                heading_rot(i,d) = atan2(pos_y_rot(i,d) - pos_y_rot(i-1,d), pos_x_rot(i,d) - pos_x_rot(i-1,d));    
            % i < 1     
            else % initialization step (starting point)
                heading_rot(i,d) = atan2(pos_y_rot(i,d), pos_x_rot(i,d));
            end

        end
        
        % This part of the problem imposes that the position has to be equal
        % to the target at a certain number of waypoints, and therefore time
        for i = 1 : n_regions - 1  % current_target
            heading_rot(  eval( [ 'parameters.I_installation.Drone_' num2str(d) '{' num2str(i) '}' ] )  ,d) = ...
                goal{sequence{d}(i+1)}.heading_angle + angle_rad;  % target region
        end
        
    end   
    
    heading_rot(1,:) = heading_rot(2,:);
    
    % Translating the trajectories
    pos_x_rot   = pos_x_rot + translation_x;
    pos_y_rot   = pos_y_rot + translation_y;
  
end  

%% Data saving in a txt file

if strcmp(enable_saving_txt, 'y')
    
    enable_LP_trajectories = 'n';

    disp('Data saving...');
    data_saving

end

%% Making the video animation

disp( ' ' );
enable_video_animation = input('Do you record a video animation?y/n\n','s');
disp( ' ' );

if strcmp(enable_video_animation, 'y')
    
    % Set up 3D plot to record. Figure full screen mode
    NLP_video = figure('units','normalized','outerposition',[0 0 1 1]); clf;
    
    % To plot the power line mock-up
    if selection == 1 || selection == 2 || selection == 3
        plot_scenario(map, []);
    end

    % Number of goals within the cell
    [~, goal_elements]     = size(goal);
    [obstacle_elements, ~] = size(obstacles);
    hold on;

    % To plot the goal regions (they are in blue)
    for i = 1 : goal_elements
        % The target point is represented as a polyhedron
        goal{i}.polyhedron = Polyhedron('lb', goal{i}.lb, 'ub', goal{i}.ub);
        plot(goal{i}.polyhedron, 'color', 'blue', 'alpha', 0.5); % plot the goals
    end

    % To plot the starting point (they are in magenta)
    for i = 1: drones
        initial_pos.shape = Polyhedron('lb', initial_position(:,i)-goal{i}.ds,'ub', ...
            initial_position(:,i)+goal{i}.ds);
        plot(initial_pos.shape, 'color', 'magenta', 'alpha', 0.5); % plot the obstacle
    end

    % To plot the map dimensions(they are in green)
    data.dimension = map.map_dimension;
    
    % To improve the display
    marg = 0.25;
    axis ([data.dimension(1)-marg   data.dimension(4)+marg   data.dimension(2)-marg   data.dimension(5)+marg ...
        data.dimension(3)-marg   data.dimension(6)+marg]);
    if selection ~= 4 % axis does not work with case 4
        axis equal
    end

    view(80,10)

    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
        
    % Plotting the path
    for d = 1 : optimizationParameters.drones

        hold all;
        plot3(pos_x(:,d), pos_y(:,d), pos_z(:,d), '-.', 'color',color_drone(d,:), 'linewidth', 2.0);

    end  
    
    % Prepare the new file.
    vidObj = VideoWriter('animation.avi');
    vidObj.Quality = 95;
    vidObj.FrameRate = 15;
    open(vidObj);
 
    % Create an animation.
    set(gca,'nextplot','replacechildren');
 
    for k = 1:360
       view(80+k,10)
       
       % Write each frame to the file.
       currFrame = getframe(NLP_video);
       writeVideo(vidObj,currFrame);
       pause(0.05)
    end
  
    % Close the file.
    close(vidObj);

end
