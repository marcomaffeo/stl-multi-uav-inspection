function rho_unsafe = robustness_unsafe(pos_x, pos_y, pos_z, drone_ref, ...
    optimizationParameters)
% The "robustness_unsafe" MATLAB function produces as output the set of 
% waypoints not safe. Always not unsafe in x y z
%
% Inputs:
% - pos_x, drone position along x-axis \in \mathbb{R}^{elements \times drones}
% - pos_y, drone position along y-axis \in \mathbb{R}^{elements \times drones}
% - pos_z, drone position along z-axis \in \mathbb{R}^{elements \times drones}
% - drone_ref, drones, \in \{1, 2, 3, \dots, N\}
% - optimizationParameters, the set of optimization parameters
%
% Outputs:
% - rho_unsafe, smooth version of the rho_unsafe
%

import casadi.*
type_of = optimizationParameters.type_of; % In case of symbolic or numerical evaluation

if(type_of)
    temp = zeros(size(pos_x,1), size(optimizationParameters.obstacles, 1));
    temp_unsafe = zeros(size(optimizationParameters.obstacles, 1), 1);
else
    temp = MX.zeros(size(pos_x, 1), size(optimizationParameters.obstacles, 1));
    temp_unsafe = MX.zeros(size(optimizationParameters.obstacles, 1), 1);
end

% There is not time in this formula because it has to be ALWAYS satisfied
C = optimizationParameters.C;
for j = 1 : optimizationParameters.obstacle_elements
    
    % always not unsafe in x y z
    rho_lb_pos_x = pos_x(:,drone_ref) - optimizationParameters.obstacles_lb_N{j}(:,1);
    rho_ub_pos_x = optimizationParameters.obstacles_ub_N{j}(:,1) - pos_x(:,drone_ref);
    
    rho_lb_pos_y = pos_y(:,drone_ref) - optimizationParameters.obstacles_lb_N{j}(:,2);
    rho_ub_pos_y = optimizationParameters.obstacles_ub_N{j}(:,2) - pos_y(:,drone_ref);
    
    rho_lb_pos_z = pos_z(:,drone_ref) - optimizationParameters.obstacles_lb_N{j}(:,3);
    rho_ub_pos_z = optimizationParameters.obstacles_ub_N{j}(:,3) - pos_z(:,drone_ref);

    for i = 1 : numel(rho_lb_pos_x)
        
        temp_vec = [rho_lb_pos_x(i) rho_ub_pos_x(i) rho_lb_pos_y(i) rho_ub_pos_y(i) ...
            rho_lb_pos_z(i) rho_ub_pos_z(i)];
        temp(i, j) = SmoothMin(temp_vec,C); % Unsafe specification, in the obstacle
        
    end
    
    temp_unsafe(j) = SmoothMin(-temp(:,j),C); % Implements NOT (NOT in the obstacles)

end

rho_unsafe = SmoothMin(temp_unsafe,C); % Implements ALWAYS

end


