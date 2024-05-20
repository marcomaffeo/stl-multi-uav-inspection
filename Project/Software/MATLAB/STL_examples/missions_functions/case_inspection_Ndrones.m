function [negative_rob, pos_x, pos_y, pos_z] = case_inspection_Ndrones(var, ...
    optimizationParameters, parameters, selection)
% The "case_inspection_Ndrones" MATLAB function computes the cost 
% objective function for the reach and avoid task with N vehicles (N can be 
% equal to one).
%
% Inputs:
% - var, contains the symbolic variables used for the problem
% - optimizationParameters, all parameters for the cost function
% - parameters, contains all the parameters for the STL formulas
% - selection, it allows to know the scenario is going to be simulated
%
% Outputs:
% - negative_rob, the minimum shoot between rho unsafe and rho goal, where
% rho is the STL formulas in the optimization problem
% - pos_x, position equations for the solver \in \mathbb{R}^{waypoints}
% - pos_y, position equations for the solver \in \mathbb{R}^{waypoints}
% - pos_z, position equation for the solver \in \mathbb{R}^{waypoints}

import casadi.*

sampling_time          = optimizationParameters.sampling_time;
motion_time            = optimizationParameters.motion_time;
dv                     = optimizationParameters.dv;
da                     = optimizationParameters.da;
M                      = optimizationParameters.M;
Clen                   = optimizationParameters.Clen;
drones                 = optimizationParameters.drones;
WPs_total              = optimizationParameters.WPs_total;
C                      = optimizationParameters.C;      % Used for the min/max smooth approximation on the STL mission requirements
C_dist                 = optimizationParameters.C_dist; % Used for the min/max smooth approximation on the mutual distance
enable_Belta_variation = optimizationParameters.enable_Belta_variation;
delta_min              = optimizationParameters.delta_min;


type_of = isfloat(var); % 0 for casadi
optimizationParameters.type_of = type_of;

if selection == 1 || selection == 2 || selection == 3 
    sequence = optimizationParameters.sequence;
    
end

% Half of the "var" vector contains the position, the other half contains
% the velocity
p = var( 1 : numel(var)/2);
v = var( numel(var)/2 + 1 : end);

% Creating the time vector to sample the trajectory between two waypoints
time = sampling_time : sampling_time: motion_time;

% It depends on the value assumed by the cost function. The MATLAB function
% works both with double and symbolic variables provided as inputs. This is
% the case when symbolic variables are in the loop
if type_of  % if input is a double (i.e., numbers)
    
    % Just temporany vector used while computing position
    temp_x = zeros(drones, numel(time)); % \in \mathbb{R}^{drones \times size time vector}
    temp_y = zeros(drones, numel(time)); % \in \mathbb{R}^{drones \times size time vector}
    temp_z = zeros(drones, numel(time)); % \in \mathbb{R}^{drones \times size time vector}
    
    % The position vectors. The term "+1" accounts for the initial drone position
    pos_x = zeros(numel(time) * WPs_total + 1, drones); % \in \mathbb{R}^{size time vector + 1 \times drones}
    pos_y = zeros(numel(time) * WPs_total + 1, drones); % \in \mathbb{R}^{size time vector + 1 \times drones}
    pos_z = zeros(numel(time) * WPs_total + 1, drones); % \in \mathbb{R}^{size time vector + 1 \times drones}
    
    % The STL specifications:
    % - rho_unsafe codifies the avoiding obstacles specification
    % - rho_goal codifies the reach the target and stay there specification
    rho_unsafe = zeros(drones, 1); % \in \mathbb{R}^{drones}. It's a double per drone
    rho_goal     = zeros(drones, 1); % \in \mathbb{R}^{drones}. It's a double per drone

    % In case there is more than one drone, the mutual distance is also
    % accounted in the STL logic
    if drones > 1
        % Binomial coefficient or all combinations
        % see https://www.mathworks.com/help/matlab/ref/nchoosek.html
        rho_dists = zeros(nchoosek(drones, 2), 1);
        % To account for the mutual distance requirements for all the
        % points. As said, the trajectory is divided in topic waypoints,
        % and then, the trajectory between two consecutive waypoints, is
        % sampled considering the sampling time value (i.e., 0.05 seconds).
        % The "+1" term accounts for the initial drone position
        mutual_distances = zeros(numel(time)*WPs_total + 1, 1);
        
    end
    
else
    
    % Just temporany vector used while computing position with symbolic variables
    temp_x = MX.zeros(drones, numel(time)); % \in \mathbb{R}^{drones \times size time vector}
    temp_y = MX.zeros(drones, numel(time)); % \in \mathbb{R}^{drones \times size time vector}
    temp_z = MX.zeros(drones, numel(time)); % \in \mathbb{R}^{drones \times size time vector}
    
    % The position vectors. The term "+1" accounts for the initial drone position
    pos_x = MX.zeros(numel(time) * WPs_total + 1, drones);    
    pos_y = MX.zeros(numel(time) * WPs_total + 1, drones);    
    pos_z = MX.zeros(numel(time) * WPs_total + 1, drones);
    
    % The STL specifications in case of symbolic variables: 
    % - rho_unsafe codifies the avoiding obstacles specification
    % - rho_goal codifies the reach the target specification
    rho_unsafe   = MX.zeros(drones, 1); % \in \mathbb{R}^{drones}. It's a double per drone
    rho_goal     = MX.zeros(drones, 1); % \in \mathbb{R}^{drones}. It's a double per drone
    
    % In case there is more than one drone, the mutual distance is also
    % accounted in the STL logic. This holds in case of symbolic variables
    if drones > 1
        % Binomial coefficient or all combinations
        % see https://www.mathworks.com/help/matlab/ref/nchoosek.html
        rho_dists = MX.zeros(nchoosek(drones,2),1);
        % To account for the mutual distance requirements for all the
        % points. As said, the trajectory is divided in topic waypoints,
        % and then, the trajectory between two consecutive waypoints, is
        % sampled considering the sampling time value (i.e., 0.05 seconds).
        % The "+1" term accounts for the initial drone position
        mutual_distances = MX.zeros(numel(time) * WPs_total+1, 1);
        
    end
    
end

%% Here start computing the objective function
for i = 1 : drones
    
    % Initial drone position. The optimization problem start from this initial value
    pos_x(1,i) = p(1 + (i-1) * Clen);
    pos_y(1,i) = p(2 + (i-1) * Clen);
    pos_z(1,i) = p(3 + (i-1) * Clen);
    
    % Get all sampled splines
    for k = 1 : WPs_total
        
        % distance for for all axes. considering initial velocity and
        % acceleration equal to zero
        difference_p_x = p(k * 3 + 1 + (i-1) * Clen) - p((k-1) * 3 + 1 + (i-1) * Clen); % p_k - p_{k-1} along x-axis
        difference_p_y = p(k * 3 + 2 + (i-1) * Clen) - p((k-1) * 3 + 2 + (i-1) * Clen); % p_k - p_{k-1} along y-axis
        difference_p_z = p(k * 3 + 3 + (i-1) * Clen) - p((k-1) * 3 + 3 + (i-1) * Clen); % p_k - p_{k-1} along z-axis
        
        v_x_km1 = v( (k-1) * 3 + 1 + (i-1) * Clen); % v{k-1} along x-axis
        v_y_km1 = v( (k-1) * 3 + 2 + (i-1) * Clen); % v{k-1} along y-axis
        v_z_km1 = v( (k-1) * 3 + 3 + (i-1) * Clen); % v{k-1} along z-axis
        
        % constants for all 3 axes
        alfa_x  = M(1,:) * [difference_p_x - motion_time * v_x_km1; dv; da]; 
        beta_x  = M(2,:) * [difference_p_x - motion_time * v_x_km1; dv; da];
        gamma_x = M(3,:) * [difference_p_x - motion_time * v_x_km1; dv; da];

        alfa_y  = M(1,:) * [difference_p_y - motion_time * v_y_km1; dv; da];
        beta_y  = M(2,:) * [difference_p_y - motion_time * v_y_km1; dv; da];
        gamma_y = M(3,:) * [difference_p_y - motion_time * v_y_km1; dv; da];

        alfa_z  = M(1,:) * [difference_p_z - motion_time * v_z_km1; dv; da];
        beta_z  = M(2,:) * [difference_p_z - motion_time * v_z_km1; dv; da];
        gamma_z = M(3,:) * [difference_p_z - motion_time * v_z_km1; dv; da];

        % See W. Mueller, M. Hehn and R. D'Andrea, "A Computationally  Efficient 
        % Motion Primitive for Quadrocopter Trajectory Generation," in IEEE 
        % Transactions on Robotics, vol. 31, no. 6, pp. 1294-1310, Dec. 2015, 
        % doi: 10.1109/TRO.2015.2479878.
        % For more detail see eqs. (23--24)
        temp_x(i,:) = (alfa_x/120)*time.^5 + (beta_x/24)*time.^4 + ...
            (gamma_x/6)*time.^3 + time * v( (k-1) * 3 + 1 + (i-1) * Clen) + ...
            p( (k-1) * 3 + 1 + (i-1) * Clen); %fix w points
        
        % This allows creating a position vector
        interval = 2+(k-1)*numel(time) : k*numel(time)+1;
        pos_x(interval, i) = temp_x(i,:)';

        temp_y(i,:) = (alfa_y/120)*time.^5 + (beta_y/24)*time.^4 + ...
            (gamma_y/6)*time.^3 + time * v( (k-1) * 3 + 2 + (i-1) * Clen) + ...
            p((k-1) * 3 + 2 +(i-1) * Clen) ; % fix w points
        pos_y(interval, i) = temp_y(i,:)';

        temp_z(i,:) = (alfa_z/120)*time.^5 + (beta_z/24)*time.^4 + ...
            (gamma_z/6)*time.^3 + time * v( (k-1) * 3 + 3 + (i-1) * Clen) + ...
            p((k-1) * 3 + 3 + (i-1) * Clen); %fix w points
        pos_z(interval, i) = temp_z(i,:)'; 
        
    end
    
    %% Accounting for the avoid obstacle specification
    
    % Unsafe set. This allows to avoid obstacle along the path
    if strcmp(enable_Belta_variation, 'y') && (selection == 1 || selection ==2 || selection == 3)
        
        % robustness_unsafe(pos_x, pos_y, pos_z, drone_ref, optimizationParameters)
        rho_unsafe(i) = robustness_unsafe_Belta(pos_x, pos_y, pos_z, i, optimizationParameters);

    elseif selection == 1 || selection == 2 || selection == 3
    
        % robustness_unsafe_Belta(pos_x, pos_y, pos_z, drone_ref, optimizationParameters)
        % This uses the Log-Sum-Exponential smooth approximation
        rho_unsafe(i) = robustness_unsafe(pos_x, pos_y, pos_z, i, optimizationParameters);
    
    else
        % In case none of the option above mentioned. This returns the command to the user
        return
    end
    
    %% Accounting for reach the target.
    
    % In this simple example, the STL specification requires to reach the targets
    % and the home following the optimal sequence computed previously (staying in
    % all these regions during the time interval specified with the vectors
    % "parameters.I_installation.Drone_k{i}"). These time intervals implicitly
    % include the amount of time needed to reach the targets.
    if selection == 1 || selection == 2 || selection == 3 
        
        n_regions = length(sequence{i})-1;
        
        if type_of  % if input is a double (i.e., numbers)
            rho_stay = zeros( n_regions , 1 );
        else
            rho_stay = MX.zeros( n_regions, 1 );
        end
        
        %%% STAY IN THE TARGET REGION (OR AT HOME)
        for j=1:n_regions
            
            goal_ref = sequence{i}(1+j);
            
            interval = eval( [ 'parameters.I_installation.Drone_' num2str(i) '{' num2str(j) '}' ] );

            if strcmp(enable_Belta_variation, 'y') 

                % robustness_stay_Belta(pos_x, pos_y, pos_z, drone_ref, goal_ref, I, optimizationParameters)
                rho_stay(j) = robustness_stay_Belta(pos_x, pos_y, pos_z, i, goal_ref, ...
                    interval, optimizationParameters);
                
            elseif strcmp(enable_Belta_variation, 'n')

                % robustness_stay(pos_x, pos_y, pos_z, drone_ref, goal_ref, I, optimizationParameters)
                % This uses the Log-Sum-Exponential smooth approximation
                rho_stay(j) = robustness_stay(pos_x, pos_y, pos_z, i, goal_ref, ...
                    interval, optimizationParameters);
                
            end
            
        end
        
        %%% THIS SMOOTHMIN FUNCTION SIMPLY IMPLEMENTS THE "AND" OPERATION.
        %%% FOR MORE INFORMATION ON HOW THE OPERATOR ARE CODIFIED, REFER TO
        %%% THIS PAPER:
        %%%
        %%% G. Silano, T. Baca, R. Penicka, D. Liuzza and M. Saska, "Power Line 
        %%% Inspection Tasks With Multi-Aerial Robot Systems Via Signal Temporal 
        %%% Logic Specifications," in IEEE Robotics and Automation Letters, vol. 6, 
        %%% no. 2, pp. 4169-4176, April 2021, doi: 10.1109/LRA.2021.3068114.
        if strcmp(enable_Belta_variation, 'y')
            
            % x_min = SmoothMin_Belta(vec_x, C) 
            % i accounts for the drone (i.e., the first, the second, etc.)
            rho_goal(i) = SmoothMin_Belta(rho_stay,C); % This is an AND operation
        
        elseif strcmp(enable_Belta_variation, 'n')
            
            % x_min = SmoothMin(vec_x, C) 
            % This uses the Log-Sum-Exponential smooth approximation
            % i accounts for the drone (i.e., the first, the second, etc.)
            rho_goal(i) = SmoothMin(rho_stay,C); % This is an AND operation
            
        end
        
    end

end

%% Pairwise distances

% Only if there is more than one drone. This saves in computation time
if (drones > 1) && (selection == 1 || selection == 2 || selection == 3)
    
    % Binomial coefficient or all combinations.
    combos = nchoosek(1 : drones, 2);
    for p = 1 : size(combos, 1)

        %%% Over the entire trajectory of the drone1 and drone, in general
        %%% for all the possibile combinations of drones, the algorithm
        %%% computes the norm of the difference between the position and
        %%% checker whether this is greater than the threshold
        for uuu = 1 : size(pos_x,1) %for all time steps
            
            % Position drone A and drone B
            pa = [pos_x(uuu, combos(p, 1)); pos_y(uuu, combos(p, 1)); pos_z(uuu, combos(p, 1))];
            pb = [pos_x(uuu, combos(p, 2)); pos_y(uuu, combos(p, 2)); pos_z(uuu, combos(p, 2))];
            
            % Mutual distance between drone A and drone B. The mutual
            % distance is a vector, which dimension depends on the size of
            % the binomial coefficient
            mutual_distances(uuu) = norm(pa-pb,2) - delta_min;
            
        end

        %%% The distance between the drone A and the drone B has to be
        %%% greater than a threshold. In other words, the difference
        %%% between the norm and the threshold has to be greater than zero.
        %%% This has to be valid "ALWAYS". This operator is codified with a
        %%% min operation.
        if strcmp(enable_Belta_variation, 'y')
               
            % x_min = SmoothMin_Belta(vec_x, C) 
            rho_dists(p) = SmoothMin_Belta(mutual_distances, C_dist);  % This is an ALWAYS
        
        elseif strcmp(enable_Belta_variation, 'n')
            
            % x_min = SmoothMin(vec_x, C) 
            % This uses the Log-Sum-Exponential smooth approximation
            rho_dists(p) = SmoothMin(mutual_distances, C_dist); % This is an ALWAYS
            
        end

    end
    
else
    rho_dists = []; % in case the number of drones is lower than 1
end

%%% The minus before "SmoothMin" is due the standard form of the problem in
%%% CASADI. It always minimize, but we want to maximize the robustness.
%%% Therefore, we put minus before the equation.
%%% In the end, we want that all the specifications have to be satisfied.
%%% Hence, in the following there is nothing but an "AND" operation
if strcmp(enable_Belta_variation, 'y') && (selection == 1 || selection ==2 || selection == 3)
    
    % x_min = SmoothMin_Belta(vec_x, C) 
    negative_rob = -SmoothMin_Belta([rho_unsafe; rho_goal; rho_dists], C); % This is an AND

elseif strcmp(enable_Belta_variation, 'n') && (selection == 1 || selection == 2 || selection == 3)
    
    % x_min = SmoothMin(vec_x, C) 
    % This uses the Log-Sum-Exponential smooth approximation
    negative_rob = -SmoothMin([rho_unsafe; rho_goal; rho_dists], C); % This is an AND
    
else
    % In case none of the option above mentioned. This returns the command to the user
    return
    
end

end
