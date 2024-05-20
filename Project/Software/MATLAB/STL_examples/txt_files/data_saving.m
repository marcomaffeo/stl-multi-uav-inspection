%% Write data to text file - WAYPOINTS

for i = 1 : optimizationParameters.drones

    number_drone = sprintf('%d', i);
    fid_txt = fopen(strcat('waypoints_', number_drone, '.txt'), 'w');
    fid_csv = fopen(strcat('waypoints_', number_drone, '.csv'), 'w');

    for j = 1 : WPs_total + 1
        
        fprintf(fid_txt, '%f %f %f\n', [waypoints{i}(1,j) waypoints{i}(2,j) ...
            waypoints{i}(3,j)]');
        fprintf(fid_csv, '%f, %f, %f %f\n', [waypoints{i}(1,j) waypoints{i}(2,j) ...
            waypoints{i}(3,j)]');

    end

    % Close the txt and csv files
    fclose(fid_txt);
    fclose(fid_csv);

    % Move the obtained files
    movefile(strcat('waypoints_', number_drone, '.txt'), destinationTxT);
    movefile(strcat('waypoints_', number_drone, '.csv'), destinationTxT);

end 


%% Write data to text and csv files - TRAJECTORIES

for i = 1 : optimizationParameters.drones

    number_drone = sprintf('%d', i);
    fid_txt = fopen(strcat('trajectory_', number_drone, '.txt'), 'w');
    fid_csv = fopen(strcat('trajectory_', number_drone, '.csv'), 'w');
    
    % If the rotate option is enabled
    if strcmp(enable_traj_rot, 'y')
        
        fid_txt_rot_txt = fopen(strcat('trajectory_rotated_', number_drone, '.txt'), 'w');
        fid_txt_rot_csv = fopen(strcat('trajectory_rotated_', number_drone, '.csv'), 'w');
     
    end
    
    % If the LP trajectories
    if strcmp(enable_LP_trajectories, 'y')
        
        fid_txt_LP_txt = fopen(strcat('trajectory_LP_', number_drone, '.txt'), 'w');
        fid_txt_LP_csv = fopen(strcat('trajectory_LP_', number_drone, '.csv'), 'w');
     
    end
    
    % If the LP trajectories and rotated
    if strcmp(enable_LP_trajectories, 'y') && strcmp(enable_traj_rot, 'y')
        
        fid_txt_rot_LP_txt = fopen(strcat('trajectory_rotated_LP_', number_drone, '.txt'), 'w');
        fid_txt_rot_LP_csv = fopen(strcat('trajectory_rotated_LP_', number_drone, '.csv'), 'w');
     
    end

    for j = 1 : size(pos_x,1)

        fprintf(fid_txt, '%f %f %f %f\n', [pos_x(j,i) pos_y(j,i) pos_z(j,i) ...
            heading_vector(j,i)]');
        fprintf(fid_csv, '%f, %f, %f, %f\n', [pos_x(j,i) pos_y(j,i) pos_z(j,i) ...
            heading_vector(j,i)]');
        
        % If the rotate option is enabled
        if strcmp(enable_traj_rot, 'y')
        
            fprintf(fid_txt_rot_txt, '%f %f %f %f\n', [pos_x_rot(j,i) pos_y_rot(j,i) pos_z_rot(j,i) ...
                heading_rot(j,i)]');
            fprintf(fid_txt_rot_csv, '%f, %f, %f, %f\n', [pos_x_rot(j,i) pos_y_rot(j,i) pos_z_rot(j,i) ...
                heading_rot(j,i)]');
        
        end
        
        % If the rotate and LP options are enabled
        if strcmp(enable_LP_trajectories, 'y')
        
            fprintf(fid_txt_LP_txt, '%f %f %f %f\n', [pos_x_LP(j,i) pos_y_LP(j,i) pos_z_LP(j,i) ...
                heading_vector_LP(j,i)]');
            fprintf(fid_txt_LP_csv, '%f, %f, %f, %f\n', [pos_x_LP(j,i) pos_y_LP(j,i) pos_z_LP(j,i) ...
                heading_vector_LP(j,i)]');
        
        end
        
        % If the rotate and LP options are enabled
        if strcmp(enable_LP_trajectories, 'y') && strcmp(enable_traj_rot, 'y')
        
            fprintf(fid_txt_rot_LP_txt, '%f %f %f %f\n', [pos_x_LP_rot(j,i) pos_y_LP_rot(j,i) pos_z_LP_rot(j,i) ...
                heading_LP_rot(j,i)]');
            fprintf(fid_txt_rot_LP_csv, '%f, %f, %f, %f\n', [pos_x_LP_rot(j,i) pos_y_LP_rot(j,i) pos_z_LP_rot(j,i) ...
                heading_LP_rot(j,i)]');
        
        end

    end

    % Close the txt and csv files
    fclose(fid_txt);
    fclose(fid_csv);
    
    % If the rotation option is enabled
    if strcmp(enable_traj_rot, 'y')
        
        fclose(fid_txt_rot_txt);
        fclose(fid_txt_rot_csv);
        
    end
    
    % If the LP trajectories
    if strcmp(enable_LP_trajectories, 'y')
        
        fclose(fid_txt_LP_txt);
        fclose(fid_txt_LP_csv);
     
    end
    
    % If the LP trajectories and rotated
    if strcmp(enable_LP_trajectories, 'y') && strcmp(enable_traj_rot, 'y')
        
        fclose(fid_txt_rot_LP_txt);
        fclose(fid_txt_rot_LP_csv);
     
    end

    % Move the obtained files
    movefile(strcat('trajectory_', number_drone, '.txt'), destinationTxT);
    movefile(strcat('trajectory_', number_drone, '.csv'), destinationTxT);
    
    % If the rotation option is enabled
    if strcmp(enable_traj_rot, 'y')

        movefile(strcat('trajectory_rotated_', number_drone, '.txt'), destinationTxT);
        movefile(strcat('trajectory_rotated_', number_drone, '.csv'), destinationTxT);

    end
    
    % If the LP trajectories
    if strcmp(enable_LP_trajectories, 'y')
        
        movefile(strcat('trajectory_LP_', number_drone, '.txt'), destinationTxT);
        movefile(strcat('trajectory_LP_', number_drone, '.csv'), destinationTxT);
     
    end
    
    % If the LP trajectories and rotated
    if strcmp(enable_LP_trajectories, 'y') && strcmp(enable_traj_rot, 'y')
        
        movefile(strcat('trajectory_rotated_LP_', number_drone, '.txt'), destinationTxT);
        movefile(strcat('trajectory_rotated_LP_', number_drone, '.csv'), destinationTxT);
     
    end

end 

%% To split trajectories in multiple files

disp( ' ' );
enable_state_machine = input('Do you want to use the state machine?y/n\n','s');
disp( ' ' );

% This part of the code split the trajectories contained in the pos_x,
% pos_y, and pos_z variables (this holds also for pos_x_rot, pos_y_rot,
% pos_z_rot) in multiple piecies. In other words, the trajectory is breaken
% as soon as the drone reach the refilling station, and new file is created
% for the new piece

if strcmp(enable_state_machine, 'y')
    
    % To look for how many times home is visited
    for i = 1 : optimizationParameters.drones 
        max_home(i) = sum(sequence{i}==0); 
    end
    n_max_sub_trajectories = max(max_home) - 1; % drone starts from home position
    
    % This vector will contain the subtrajectories per each drone, i.e.,
    % from home to the home will be visited.
    % 4 because x, y, z, heading
    new_pos_vector = cell(n_max_sub_trajectories, 4, optimizationParameters.drones);
    new_pos_vector_rot = cell(n_max_sub_trajectories, 4, optimizationParameters.drones); % rotated

    for d = 1 : optimizationParameters.drones
        
        I_installation_temp = eval( [ 'parameters.I_installation.Drone_' num2str(d) ';' ] );

        % To navigate all points in pos_x, pos_y and pos_z. The vector
        % dimension changes every loop cycle
        t = size(pos_x,1);
        tt_prev = 0; % set the starting point per each trajectory
        
        offset = 1; % offset for navigating the refilling stations per drone
        index_trajectory = 1; % index for the new cell containing the subtrajectories

        for tt = I_installation_temp{1}  : t

            % If the point is contained within the bounds
            % optimizationParameters.refilling_station{ n_refilling_station, drones }
            % refilling_station_pos( drones , 3 , n_refilling_station )
            if(pos_x(tt,d) > initial_position(1,d)-goal{1}.ds && pos_x(tt,d) < initial_position(1,d)+goal{1}.ds && ...
                pos_y(tt,d) > initial_position(2,d)-goal{1}.ds && pos_y(tt,d) < initial_position(2,d)+goal{1}.ds && ...
                pos_z(tt,d) > initial_position(3,d)-goal{1}.ds && pos_z(tt,d) < initial_position(3,d)+goal{1}.ds)

               % Saving the trajectory up to the refilling station
               % new_pos_vector = cell(n_max_sub_trajectories, 4, optimizationParameters.drones);
               % In this case the refilling station represents the home point
               new_pos_vector{index_trajectory, 1, d} = pos_x(1+tt_prev:tt, d); 
               new_pos_vector{index_trajectory, 2, d} = pos_y(1+tt_prev:tt, d); 
               new_pos_vector{index_trajectory, 3, d} = pos_z(1+tt_prev:tt, d);
               new_pos_vector{index_trajectory, 4, d} = heading_vector(1+tt_prev:tt, d);
               
               new_pos_vector_rot{index_trajectory, 1, d} = pos_x_rot(1+tt_prev:tt, d); 
               new_pos_vector_rot{index_trajectory, 2, d} = pos_y_rot(1+tt_prev:tt, d); 
               new_pos_vector_rot{index_trajectory, 3, d} = pos_z_rot(1+tt_prev:tt, d);
               new_pos_vector_rot{index_trajectory, 4, d} = heading_rot(1+tt_prev:tt, d);
                
               % Move to the next drone  if the offset is bigger than the
               % feasible maximum value
               break
               
            end

        end
        
        % To save the obtained trajectories
        number_drone = sprintf('%d', d); % to add the drone number in the name of the file
        ll_text = sprintf('%d', 1);  % to add the subtrajectory value in the name of the file
        fid_txt = fopen(strcat('trajectory_', number_drone, '_', ll_text, '.txt'), 'w');
        fid_csv = fopen(strcat('trajectory_', number_drone, '_', ll_text, '.csv'), 'w');
        
        % If the rotate option is enabled
        if strcmp(enable_traj_rot, 'y')
            ll_text = sprintf('%d', 1);  % to add the subtrajectory value in the name of the file
            fid_txt_rot_txt = fopen(strcat('trajectory_rotated_', number_drone, '_', ll_text, '.txt'), 'w');
            fid_txt_rot_csv = fopen(strcat('trajectory_rotated_', number_drone, '_', ll_text, '.csv'), 'w');
            
        end


        % new_pos_vector = cell(n_max_sub_trajectories, 4, optimizationParameters.drones);
        for ll = 1 : n_max_sub_trajectories

            pos_x_temp = new_pos_vector{ll, 1, d};
            pos_y_temp = new_pos_vector{ll, 2, d};
            pos_z_temp = new_pos_vector{ll, 3, d};
            heading_vector_temp = new_pos_vector{ll, 4, d};
            
            pos_x_rot_temp = new_pos_vector_rot{ll, 1, d};
            pos_y_rot_temp = new_pos_vector_rot{ll, 2, d};
            pos_z_rot_temp = new_pos_vector_rot{ll, 3, d};
            heading_vector_rot_temp = new_pos_vector_rot{ll, 4, d};
            
            for j = 1 : size(pos_x_temp,1)

                fprintf(fid_txt, '%f %f %f %f\n', [pos_x_temp(j) pos_y_temp(j) pos_z_temp(j) ...
                    heading_vector_temp(j)]');
                fprintf(fid_csv, '%f, %f, %f, %f\n', [pos_x_temp(j) pos_y_temp(j) pos_z_temp(j) ...
                    heading_vector_temp(j)]');

                % If the rotate option is enabled
                if strcmp(enable_traj_rot, 'y')

                    fprintf(fid_txt_rot_txt, '%f %f %f %f\n', [pos_x_rot_temp(j) pos_y_rot_temp(j) ...
                        pos_z_rot_temp(j) heading_vector_rot_temp(j)]');
                    fprintf(fid_txt_rot_csv, '%f, %f, %f, %f\n', [pos_x_rot_temp(j) pos_y_rot_temp(j) ...
                        pos_z_rot_temp(j) heading_vector_rot_temp(j)]');

                end

            end
            
            % Close the txt and csv files
            fclose(fid_txt(ll));
            fclose(fid_csv(ll));

            % If the rotation option is enabled
            if strcmp(enable_traj_rot, 'y')

                fclose(fid_txt_rot_txt);
                fclose(fid_txt_rot_csv);

            end
            
            % Move the obtained files
            ll_text = sprintf('%d', 1);  % to add the subtrajectory value in the name of the file
            movefile(strcat('trajectory_', number_drone, '_', ll_text, '.txt'), destinationTxT);
            movefile(strcat('trajectory_', number_drone, '_', ll_text, '.csv'), destinationTxT);


            % If the rotation option is enabled
            if strcmp(enable_traj_rot, 'y')
                ll_text = sprintf('%d', 1);  % to add the subtrajectory value in the name of the file
                movefile(strcat('trajectory_rotated_', number_drone, '_', ll_text, '.txt'), destinationTxT);
                movefile(strcat('trajectory_rotated_', number_drone, '_', ll_text, '.csv'), destinationTxT);
            end
            
        end

    end

end
