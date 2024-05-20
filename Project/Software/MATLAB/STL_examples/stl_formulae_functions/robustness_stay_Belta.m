function rho_stay = robustness_stay_Belta(pos_x, pos_y, pos_z, drone_ref, goal_ref, ...
    I, optimizationParameters)
% The "robustness_stay_Belta" MATLAB function produces as output the set of 
% waypoints to stay in the goal. Eventually alwaysThe function uses the
% approximation of the min and max functions defined in [1]
%
% [1] N. Mehdipour, C. -I. Vasile and C. Belta, "Specifying User Preferences 
% Using Weighted Signal Temporal Logic," in IEEE Control Systems Letters, 
% vol. 5, no. 6, pp. 2006-2011, Dec. 2021, doi: 10.1109/LCSYS.2020.3047362.
%
% Inputs:
% - pos_x, drone position along x-axis \in \mathbb{R}^{elements \times drones}
% - pos_y, drone position along y-axis \in \mathbb{R}^{elements \times drones}
% - pos_z, drone position along z-axis \in \mathbb{R}^{elements \times drones}
% - goal_ref, the goal point the drone has to reach
% - I, contains the interval time in which the task has to be
% achieved
% - drone_ref, drones, \in \{1, 2, 3, \dots, N\}
% - optimizationParameters, the set of optimization parameters
%
% Outputs:
% - rho_stay, smooth version of the rho_stay
%

import casadi.*

type_of = optimizationParameters.type_of;  % In case of symbolic or numerical evaluation

if(type_of)
    temp = zeros(numel(I),1);
else
    temp = MX.zeros(numel(I),1);
end

C = optimizationParameters.C;

if goal_ref == 0  % 0 encodes the home regions
    goal = optimizationParameters.home{drone_ref};
elseif goal_ref < 0  % this encodes the refilling stations
    goal = optimizationParameters.refilling_station{-goal_ref,drone_ref};
else
    goal = optimizationParameters.goal{goal_ref};
end

% always goal in x y z
rho_lb_pos_x = pos_x(I, drone_ref) - goal.goal_lb_N(I, 1);
rho_ub_pos_x = goal.goal_ub_N(I, 1) - pos_x(I, drone_ref);

rho_lb_pos_y = pos_y(I, drone_ref) - goal.goal_lb_N(I, 2);
rho_ub_pos_y = goal.goal_ub_N(I, 2) - pos_y(I, drone_ref);

rho_lb_pos_z = pos_z(I, drone_ref) - goal.goal_lb_N(I, 3);
rho_ub_pos_z = goal.goal_ub_N(I, 3) - pos_z(I, drone_ref);
    
for i = 1 : numel(rho_lb_pos_x)

    temp_vec = [rho_lb_pos_x(i) rho_ub_pos_x(i) rho_lb_pos_y(i) rho_ub_pos_y(i) ...
        rho_lb_pos_z(i) rho_ub_pos_z(i)];
     % The AND operation is missing because the MIN of a MIN is also a MIN.
     % The MIN applied twice does not change the result of the expression.
    temp(i) = SmoothMin_Belta(temp_vec,C); % Implements ALWAYS

end

rho_stay = SmoothMin_Belta(temp,C); % Implements ALWAYS

end

