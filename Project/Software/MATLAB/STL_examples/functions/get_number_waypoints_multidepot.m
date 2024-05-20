function [ WPs_sequence , WPs_total ] = get_number_waypoints_multidepot(initial_position, goal, n_drones, ...
    sequence, max_vel, max_acc, constraint_max_axis_vel, constraint_max_axis_acc, ...
    margin_time, installation_time, motion_time, time_estimation_method)
% The "get_number_waypoints_multidepot" MATLAB function computes the number
% of waypoints necessary to move the drone the quad-rotor from a point A to
% a point B approximating the drone drone as:
%
% 1) It will arrive at the final point with maximum velocity (no
% constraints on the velocity, dv=0, eq. (63) in Raffaele D'Andrea's paper).
% 2) It will arrive at the final point with zero velocity (constraints also
% on the velocity, dv \neq 0, eq. (61) in Raffaele D'Andrea's paper).
%
% Inputs:
% - initial_position, it contains the initial drone position \in
% \mathbb{R}^{3 \times number of drones}
% - goal, it contains the goal targets \in \mathbb{R}^{3 \times number of
% targets}
% - n_drones, the number of drones \in \mathbb{R}
% - sequence, the optimal sequence obtained by solving the LP problem 
% - max_vel, drone maximum velocity \in \mathbb{R}
% - max_acc, drone maximum acceleration \in \mathbb{R}
% - constraints_max_axis_vel, linear velocity constraints per axis \in 
% \mathbb{R}^3
% constraints_max_axis_acc, linear acceleration constraints per axis \in
% \mathbb{R}^3
% - safety margin time, the LP problem does not account for the obstacles
% and the power tower. Hence, this values gives a margin of manuvering to
% the NLP problem \in \mathbb{R}
% - installation_time, the bird diverter installation time. In other words,
% the time the drone should remain in the target region \in \mathbb{R}
% - motion_time, the time spent by the drone to move between two waypoints
% \in \mathbb{R}
% - time_estimation_method, the method used to estimation the drone's
% motion (i.e., 1) or 2) option)
%
% Outputs:
% - WPs_squence, the number of waypoints per each drone. Take into
% consideration that if a drone is faster than other this will remain the
% home position until the other reach the home again.
% - WPs_total, the total number of waypoints. This is necessary to generate
% the constraints over the entire optimization problem
%


WPs_sequence = cell(n_drones,1);
n_WPs_sequence = zeros(n_drones,1);
for k = 1 : n_drones
    
    % Target positions:
    goal_vector = zeros(length(sequence{k})-2,3);
    for i = 2 : length(sequence{k})-1
        goal_vector(i-1,:) = goal{sequence{k}(i)}.stop;
    end

    % Positions following the sequence:
    positions = [ initial_position(:,k)' ;   % drone position
                  goal_vector            ;   % target positions
                  initial_position(:,k)' ];  % drone position (home)

    difference_position = diff(positions,1,1);

    % Computation of the minimum feasible time:
    vel_max_axis = max_vel*constraint_max_axis_vel(:,k);
    acc_max_axis = max_acc*constraint_max_axis_acc(:,k);

    Constant = (sqrt(3)-1)/(2*sqrt(3)); % To compute the minimum time ssociated to the maximum acceleration (when time_estimation_method = 2)

    t_min_sequence_navigation = zeros(1,size(difference_position,1));
    for i=1:length(t_min_sequence_navigation) % Sequence loop
        t_min_axis = zeros(1,3);
        for j=1:3 % Axis loop

            if time_estimation_method == 1 % maximum velocity

                t_min_1 = sqrt(2*abs(difference_position(i,j))/acc_max_axis(j));
                t_min_2 = abs(difference_position(i,j))/vel_max_axis(j) + vel_max_axis(j)/(2*acc_max_axis(j));

                if t_min_1 >=0 && t_min_1<=vel_max_axis(j)/acc_max_axis(j)
                    t_min_axis(j) = t_min_1;
                elseif t_min_2 > vel_max_axis(j)/acc_max_axis(j)
                    t_min_axis(j) = t_min_2;
                else
                    % Stops execution of the file and gives control to the user's keyboard
                    return
                end

            elseif time_estimation_method == 2 % zero velocity (using splines)

                t_min_vel = 15/8*abs(difference_position(i,j))/vel_max_axis(j);                                          % Associated to the velocity
                t_min_acc = sqrt(60*Constant*(2*Constant^2-3*Constant+1)*abs(difference_position(i,j))/acc_max_axis(j)); % Associated to the acceleration

                t_min_axis(j) = max(t_min_vel,t_min_acc);

            else
                % Stops execution of the file and gives control to the user's keyboard
                return
            end

        end
        t_min_sequence_navigation(i) = max(t_min_axis);
    end
    
    % Adding the installation time:
    t_min_sequence = zeros(1 , 2*length(t_min_sequence_navigation) );
    for i = 1 : length(t_min_sequence_navigation)
        t_min_sequence(2*i-1) = margin_time*t_min_sequence_navigation(i);
        t_min_sequence(2*i) = installation_time;
    end
    
    WPs_sequence{k} = ceil(t_min_sequence/motion_time);
    n_WPs_sequence(k) = sum(WPs_sequence{k});
end

% Balance in the total number of WPs (the drone finishing firstly should wait at home):
WPs_total = max(n_WPs_sequence); % Total number of WPs

for k = 1 : n_drones
    diff_WPs = WPs_total-n_WPs_sequence(k);
    WPs_sequence{k}(end) = WPs_sequence{k}(end) + diff_WPs;
end

end
