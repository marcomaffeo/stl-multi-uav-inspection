function [p0, v0] = get_initial_waypoints(initial_state, optimizationParameters, selection, iteration)
% The "get_initial_waypoints" MATLAB sets a linear programming 
% problem for getting the initial waypoints values.
%
% Inputs:
% - initial_state, drones initial positions and velocity \in \mathbb{R}^{N \times 3}
% - optizationParameters, option parameters for the optimization problem
% - selection, it allows to know the scenario is going to be simulated
% - iteration, the current iteration (drone)
%
% Outputs:
% - p0, initial position of drone trajectories
% - v0, initial velocity of drone trajectories
%

WPs_total               = optimizationParameters.WPs_total;
motion_time             = optimizationParameters.motion_time;
M                       = optimizationParameters.M;
acc_bar                 = optimizationParameters.acc_bar;
max_vel                 = optimizationParameters.max_vel;
max_acc                 = optimizationParameters.max_acc;
constraint_max_axis_vel = optimizationParameters.constraint_max_axis_vel;
constraint_max_axis_acc = optimizationParameters.constraint_max_axis_acc;
map                     = optimizationParameters.map;
goal                    = optimizationParameters.goal;


if selection == 1 || selection == 2 || selection == 3 
    sequence     = optimizationParameters.sequence;
    WPs_sequence = optimizationParameters.WPs_sequence;
end

dv = 0;
da = 0; 

cvx_begin quiet

variable pp(WPs_total+1,3)
variable vv(WPs_total+1,3)

if selection == 1 || selection == 2 || selection == 3
    minimise sum(abs( pp(2:end,1)-pp(1:end-1,1) )) + sum(abs( pp(2:end,2)-pp(1:end-1,2) )) + sum(abs( pp(2:end,3)-pp(1:end-1,3) ))
end

pp(1,:) == initial_state(1:3)'; % initial position
vv(1,:) == initial_state(4:6)'; % initial velocity

for i = 2 : WPs_total+1
    
    % Along each of the inertial frame
    for j = 1:3
        
        % Bounds on the workspace
        pp(i,j)>= map.map_dimension(j);
        pp(i,j)<= map.map_dimension(j+3);

        alfa  = M(1,:)*[(pp(i,j) - pp(i-1,j) - vv(i-1,j) * motion_time ); dv; da];
        beta  = M(2,:)*[(pp(i,j) - pp(i-1,j) - vv(i-1,j) * motion_time ); dv; da];
        gamma = M(3,:)*[(pp(i,j) - pp(i-1,j) - vv(i-1,j) * motion_time ); dv; da];
        
        % Bounds on the acceleration
        % See W. Mueller, M. Hehn and R. D'Andrea, "A Computationally  Efficient 
        % Motion Primitive for Quadrocopter Trajectory Generation," in IEEE 
        % Transactions on Robotics, vol. 31, no. 6, pp. 1294-1310, Dec. 2015, 
        % doi: 10.1109/TRO.2015.2479878.
        acc_bar * ( pp(i,j) - pp(i-1,j)) - motion_time * acc_bar * vv(i-1,j)<= ...
             max_acc * constraint_max_axis_acc(j, iteration);
        acc_bar * ( pp(i,j) - pp(i-1,j)) - motion_time * acc_bar * vv(i-1,j)>= ...
            -max_acc * constraint_max_axis_acc(j, iteration);
        
        % Bounds on the velocity
        % See W. Mueller, M. Hehn and R. D'Andrea, "A Computationally  Efficient 
        % Motion Primitive for Quadrocopter Trajectory Generation," in IEEE 
        % Transactions on Robotics, vol. 31, no. 6, pp. 1294-1310, Dec. 2015, 
        % doi: 10.1109/TRO.2015.2479878.
        % For more detail see eqs. (23--24)
        vv(i,j) == (alfa/24)*motion_time^4 + (beta/6)*motion_time^3 + ...
            (gamma/2)*motion_time^2 + vv(i-1,j);
        
            vv(i,j) <=  max_vel * constraint_max_axis_vel(j, iteration);
            vv(i,j) >= -max_vel * constraint_max_axis_vel(j, iteration);
            
    end
    
end

%%% The part reported in the following strongly depends on the mission.
%%% Therefore, on the target regions to inspect.

% In this simple example the STL specification requires first to reach
% the target1, then the target2,then the target 3 and finally the home
% (staying in all these regions the amount of time specified with the vector
% "parameters.I_installationi"). Each target has to be reached with the same
% amount of time specified with the vector "parameters.I_navigationi". Of
% course, the time per each drone can be different but it has been codified
% with the same amount for the sake of simplicity
if selection == 1 || selection == 2 || selection == 3 
    
    WPs_cumsum = cumsum(WPs_sequence{iteration});
    n_regions  = length(sequence{iteration})-1;

    % This part of the problem imposes that the position has to be equal
    % to the target at a certain number of waypoints, and therefore time
    for i = 1 : n_regions  % current_target
        for j = WPs_cumsum(2*i-1):WPs_cumsum(2*i)
            if sequence{iteration}(1+i) == 0
                pp( 1+j ,:) == initial_state(1:3)';                  % home
            else
                pp( 1+j ,:) == goal{sequence{iteration}(1+i)}.stop'; % target region
            end
            
        end
    end
    
end

% End the Linear Programming problem
cvx_end

% Reshape the solution
p0 = reshape(pp', (WPs_total + 1)*3, 1);
v0 = reshape(vv', (WPs_total + 1)*3, 1);

end
