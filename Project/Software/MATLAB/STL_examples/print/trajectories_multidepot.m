% Starting from pos_x, pos_y, and pos_z, the script computes the velocity
% acceleration and mutual distance signals

%% --- Initialization: ------------------------------------------------- %%
% Position:
position_x = pos_x;
position_y = pos_y;
position_z = pos_z;

% Velocity:
velocity_x = zeros(size(pos_x));
velocity_y = zeros(size(pos_y));
velocity_z = zeros(size(pos_z));

% Acceleration:
acceleration_x = zeros(size(pos_x));
acceleration_y = zeros(size(pos_y));
acceleration_z = zeros(size(pos_z));

% Time:
total_time = 0 : sampling_time : motion_time * WPs_total;
time = sampling_time : sampling_time : motion_time;

% WPs (cumulative sum):
WPs_cumsum = cell(drones,1);
for i=1:drones
    WPs_cumsum{i} = cumsum(WPs_sequence{i});
end

%% --- Computation: ---------------------------------------------------- %%

% Velocity and acceleration:
for i = 1 : drones
    
    velocity_x(1,i) = initial_velocity(1,i);
    velocity_y(1,i) = initial_velocity(2,i);
    velocity_z(1,i) = initial_velocity(3,i);
    
    acceleration_x(1,i) = 0;
    acceleration_y(1,i) = 0;
    acceleration_z(1,i) = 0;
    
    for k = 1 : WPs_total
        
        % Interval associated to each WP:
        interval = 2+(k-1)*numel(time) : k*numel(time)+1;
        
        % The difference between p_k and p_{k-1}
        distance_variation_x = pos_x(interval(end),i) - pos_x(interval(1)-1,i); % difference in position along x
        distance_variation_y = pos_y(interval(end),i) - pos_y(interval(1)-1,i); % difference in position along y
        distance_variation_z = pos_z(interval(end),i) - pos_z(interval(1)-1,i); % difference in position along z
        
        % Alpha, beta and gamma parameters:
        alpha_x = M(1,:) * [distance_variation_x - motion_time * velocity_x( interval(1)-1 ,i ); dv; da]; 
        beta_x  = M(2,:) * [distance_variation_x - motion_time * velocity_x( interval(1)-1 ,i ); dv; da];
        gamma_x = M(3,:) * [distance_variation_x - motion_time * velocity_x( interval(1)-1 ,i ); dv; da];

        alpha_y = M(1,:) * [distance_variation_y - motion_time * velocity_y( interval(1)-1 ,i ); dv; da];
        beta_y  = M(2,:) * [distance_variation_y - motion_time * velocity_y( interval(1)-1 ,i ); dv; da];
        gamma_y = M(3,:) * [distance_variation_y - motion_time * velocity_y( interval(1)-1 ,i ); dv; da];

        alpha_z = M(1,:) * [distance_variation_z - motion_time * velocity_z( interval(1)-1 ,i ); dv; da];
        beta_z  = M(2,:) * [distance_variation_z - motion_time * velocity_z( interval(1)-1 ,i ); dv; da];
        gamma_z = M(3,:) * [distance_variation_z - motion_time * velocity_z( interval(1)-1 ,i ); dv; da];
        
        % vel_star, i.e., the velocity value computed using the splines(eq.(22)):
        velocity_x(interval,i) = (alpha_x/24) * time.^4 + (beta_x/6) * time.^3 + (gamma_x/2) * time.^2 + velocity_x( interval(1)-1 , i );
        velocity_y(interval,i) = (alpha_y/24) * time.^4 + (beta_y/6) * time.^3 + (gamma_y/2) * time.^2 + velocity_y( interval(1)-1 , i );
        velocity_z(interval,i) = (alpha_z/24) * time.^4 + (beta_z/6) * time.^3 + (gamma_z/2) * time.^2 + velocity_z( interval(1)-1 , i );
        
        % acc_star, i.e., the acceleration value computed using the splines(eq.(22)):
        acceleration_x(interval,i) = (alpha_x/6) * time.^3 + (beta_x/2) * time.^2 + gamma_x * time;
        acceleration_y(interval,i) = (alpha_y/6) * time.^3 + (beta_y/2) * time.^2 + gamma_y * time;
        acceleration_z(interval,i) = (alpha_z/6) * time.^3 + (beta_z/2) * time.^2 + gamma_z * time;
        
    end

end
% Horizontal velocity:
velocity_xy = zeros( size(pos_x,1) , drones );
for i = 1 : drones
    velocity_xy(:,i) = sqrt( ( velocity_x(:,i) ).^2 + ( velocity_y(:,i) ).^2 );
end

% Mutual distance between drones:
if drones >1
    combos   = nchoosek(1 : drones, 2);
    n_combos = size(combos,1);

    mutual_distance = zeros( size(pos_x,1) , n_combos );
    for i = 1 : n_combos
        for j = 1 : size(pos_x,1)
            mutual_distance(j,i) = norm( [ pos_x(j,combos(i,2))-pos_x(j,combos(i,1)) , pos_y(j,combos(i,2))-pos_y(j,combos(i,1)) , pos_z(j,combos(i,2))-pos_z(j,combos(i,1)) ] );
        end
    end

end
%% --- Figures: -------------------------------------------------------- %%

if selection == 10
    color_drone = [ 0.26 0.72 0.54  ]; % drone 3
    
else
    color_drone = [ 0.93 0.69 0.13 ;   % drone 1
                    0.49 0.18 0.56 ;   % drone 2
                    0.26 0.72 0.54 ;   % drone 3
                    0.46 0.32 0.74 ];  % drone 4
    
end
            
for k = 1 : drones
    figure

    %%% --- POSITION -------------------------------------------------- %%%
    subplot(3,3,1) % p_x
        hold on
        
        % Plots:
        plot(0,0,'color',color_drone(k,:),'linewidth',2); % No representation. It keeps space for the legend (drone k)
        
        for i=1:length(WPs_cumsum{k})/2
            if sequence{k}(1+i) <= 0
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Refilling intervals
                      [ min(position_x(:,k))-1  min(position_x(:,k))-1 max(position_x(:,k))+1 max(position_x(:,k))+1  min(position_x(:,k))-1 ],...
                      [0 1 0],'EdgeColor','none','FaceAlpha',0.1)
            else
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                      [ min(position_x(:,k))-1  min(position_x(:,k))-1 max(position_x(:,k))+1 max(position_x(:,k))+1  min(position_x(:,k))-1 ],...
                      [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
            end
        end
        
        plot([0 total_time(end)],[0 0],'--','color',[0 0 0],'linewidth',0.5) % p = 0
        
        plot(total_time, position_x(:,k),'color',color_drone(k,:),'linewidth',2); % p - evolution (drone k)
        
        plot(0:motion_time:motion_time*WPs_total , position_x(1:length(time):end,k),'.','color',color_drone(k,:),'markersize',10); % p - WPs (drone k)
        
        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) min(position_x(:,k))-1  max(position_x(:,k))+1 ])
        
        % Visualization:
        title('Position')
        xlabel('t [s]')
        ylabel('p_x [m]')
        % legend([ 'Drone ' num2str(k) ],'location','best')
        set(gca,'fontsize',10,'fontweight','bold')

    subplot(3,3,4) % p_y
        hold on
        
        % Plots:
        plot(0,0,'color',color_drone(k,:),'linewidth',2); % No representation. It keeps space for the legend (drone k)
        
        for i=1:length(WPs_cumsum{k})/2
            if sequence{k}(1+i) <= 0
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Refilling intervals
                  [ min(position_y(:,k))-1  min(position_y(:,k))-1 max(position_y(:,k))+1 max(position_y(:,k))+1  min(position_y(:,k))-1 ],...
                  [0 1 0],'EdgeColor','none','FaceAlpha',0.1)
            else
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                  [ min(position_y(:,k))-1  min(position_y(:,k))-1 max(position_y(:,k))+1 max(position_y(:,k))+1  min(position_y(:,k))-1 ],...
                  [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
            end
        end
        
        plot([0 total_time(end)],[0 0],'--','color',[0 0 0],'linewidth',0.5) % p = 0
        
        plot(total_time, position_y(:,k),'color',color_drone(k,:),'linewidth',2); % p - evolution (drone k)
        
        plot(0:motion_time:motion_time*WPs_total , position_y(1:length(time):end,k),'.','color',color_drone(k,:),'markersize',10); % p - WPs (drone k)
        
        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) min(position_y(:,k))-1  max(position_y(:,k))+1 ])
        
        % Visualization:
        xlabel('t [s]')
        ylabel('p_y [m]')
        % legend([ 'Drone ' num2str(k) ],'location','best')
        set(gca,'fontsize',10,'fontweight','bold')
        
    subplot(3,3,7) % p_z
        hold on
        
        % Plots:
        plot(0,0,'color',color_drone(k,:),'linewidth',2); % No representation. It keeps space for the legend (drone k)
        
        for i=1:length(WPs_cumsum{k})/2
            if sequence{k}(1+i) <= 0
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Refilling intervals
                  [ min(position_z(:,k))-1  min(position_z(:,k))-1 max(position_z(:,k))+1 max(position_z(:,k))+1  min(position_z(:,k))-1 ],...
                  [0 1 0],'EdgeColor','none','FaceAlpha',0.1)
            else
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                  [ min(position_z(:,k))-1  min(position_z(:,k))-1 max(position_z(:,k))+1 max(position_z(:,k))+1  min(position_z(:,k))-1 ],...
                  [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
            end
        end
        
        plot([0 total_time(end)],[0 0],'--','color',[0 0 0],'linewidth',0.5) % p = 0
        
        plot(total_time, position_z(:,k),'color',color_drone(k,:),'linewidth',2); % p - evolution (drone k)
        
        plot(0:motion_time:motion_time*WPs_total , position_z(1:length(time):end,k),'.','color',color_drone(k,:),'markersize',10); % p - WPs (drone k)
        
        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) min(position_z(:,k))-1  max(position_z(:,k))+1 ])
        
        % Visualization:
        xlabel('t [s]')
        ylabel('p_z [m]')
        % legend([ 'Drone ' num2str(k) ],'location','best')
        set(gca,'fontsize',10,'fontweight','bold')
        
    
    %%% --- VELOCITY -------------------------------------------------- %%%
    subplot(3,3,2) % v_x
        hold on
        
        % Plots:
        plot(0,0,'color',color_drone(k,:),'linewidth',2); % No representation. It keeps space for the legend (drone k)
        
        for i=1:length(WPs_cumsum{k})/2
            if sequence{k}(1+i) <= 0
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Refilling intervals
                  1.1*max_vel*constraint_max_axis_vel(1,k)*[ -1 -1 1 1 -1 ],...
                  [0 1 0],'EdgeColor','none','FaceAlpha',0.1)
            else
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                  1.1*max_vel*constraint_max_axis_vel(1,k)*[ -1 -1 1 1 -1 ],...
                  [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
            end
        end
        
        plot([0 total_time(end)],[0 0],'--','color',[0 0 0],'linewidth',0.5) % v = 0
        
        plot(total_time, velocity_x(:,k),'color',color_drone(k,:),'linewidth',2); % v - evolution (drone k)
        
        plot(0:motion_time:motion_time*WPs_total , velocity_x(1:length(time):end,k),'.','color',color_drone(k,:),'markersize',10); % v - WPs (drone k)
        
        plot([0 total_time(end)],-max_vel*constraint_max_axis_vel(1,k)*[1 1],'--','color',[1 0 0],'linewidth',1) % min v
        plot([0 total_time(end)], max_vel*constraint_max_axis_vel(1,k)*[1 1],'--','color',[1 0 0],'linewidth',1) % max v

        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) 1.1*max_vel*constraint_max_axis_vel(1,k)*[-1 1] ])
        
        % Visualization:
        title('Velocity')
        xlabel('t [s]')
        ylabel('v_x [m/s]')
        
        if selection ==10
            legend('Drone 3','location','best')
        else
            legend([ 'Drone ' num2str(k) ],'location','best')
        end
        
        set(gca,'fontsize',10,'fontweight','bold')

    subplot(3,3,5) % v_y
        hold on
        
        % Plots:
        plot(0,0,'color',color_drone(k,:),'linewidth',2); % No representation. It keeps space for the legend (drone k)
        
        for i=1:length(WPs_cumsum{k})/2
            if sequence{k}(1+i) <= 0
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Refilling intervals
                  1.1*max_vel*constraint_max_axis_vel(2,k)*[ -1 -1 1 1 -1 ],...
                  [0 1 0],'EdgeColor','none','FaceAlpha',0.1)
            else
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                  1.1*max_vel*constraint_max_axis_vel(2,k)*[ -1 -1 1 1 -1 ],...
                  [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
            end
        end
        
        plot([0 total_time(end)],[0 0],'--','color',[0 0 0],'linewidth',0.5) % v = 0
        
        plot(total_time, velocity_y(:,k),'color',color_drone(k,:),'linewidth',2); % v - evolution (drone k)
        
        plot(0:motion_time:motion_time*WPs_total , velocity_y(1:length(time):end,k),'.','color',color_drone(k,:),'markersize',10); % v - WPs (drone k)
        
        plot([0 total_time(end)],-max_vel*constraint_max_axis_vel(2,k)*[1 1],'--','color',[1 0 0],'linewidth',1) % min v
        plot([0 total_time(end)], max_vel*constraint_max_axis_vel(2,k)*[1 1],'--','color',[1 0 0],'linewidth',1) % max v

        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) 1.1*max_vel*constraint_max_axis_vel(2,k)*[-1 1] ])
        
        % Visualization:
        xlabel('t [s]')
        ylabel('v_y [m/s]')
        % legend([ 'Drone ' num2str(k) ],'location','best')
        set(gca,'fontsize',10,'fontweight','bold')
        
    subplot(3,3,8) % v_z
        hold on
        
        % Plots:
        plot(0,0,'color',color_drone(k,:),'linewidth',2); % No representation. It keeps space for the legend (drone k)
        
        for i=1:length(WPs_cumsum{k})/2
            if sequence{k}(1+i) <= 0
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Refilling intervals
                  1.1*max_vel*constraint_max_axis_vel(3,k)*[ -1 -1 1 1 -1 ],...
                  [0 1 0],'EdgeColor','none','FaceAlpha',0.1)
            else
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                  1.1*max_vel*constraint_max_axis_vel(3,k)*[ -1 -1 1 1 -1 ],...
                  [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
            end
        end
        
        plot([0 total_time(end)],[0 0],'--','color',[0 0 0],'linewidth',0.5) % v = 0
        
        plot(total_time, velocity_z(:,k),'color',color_drone(k,:),'linewidth',2); % v - evolution (drone k)
        
        plot(0:motion_time:motion_time*WPs_total , velocity_z(1:length(time):end,k),'.','color',color_drone(k,:),'markersize',10); % v - WPs (drone k)
        
        plot([0 total_time(end)],-max_vel*constraint_max_axis_vel(3,k)*[1 1],'--','color',[1 0 0],'linewidth',1) % min v
        plot([0 total_time(end)], max_vel*constraint_max_axis_vel(3,k)*[1 1],'--','color',[1 0 0],'linewidth',1) % max v

        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) 1.1*max_vel*constraint_max_axis_vel(3,k)*[-1 1] ])
        
        % Visualization:
        xlabel('t [s]')
        ylabel('v_z [m/s]')
        % legend([ 'Drone ' num2str(k) ],'location','best')
        set(gca,'fontsize',10,'fontweight','bold')
        
        
    %%% --- ACCELERATION ---------------------------------------------- %%%
    subplot(3,3,3) % a_x
        hold on
        
        % Plots:
        plot(0,0,'color',color_drone(k,:),'linewidth',2); % No representation. It keeps space for the legend (drone k)
        
        for i=1:length(WPs_cumsum{k})/2
            if sequence{k}(1+i) <= 0
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Refilling intervals
                  1.1*max_acc*constraint_max_axis_acc(1,k)*[ -1 -1 1 1 -1 ],...
                  [0 1 0],'EdgeColor','none','FaceAlpha',0.1)
            else
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                  1.1*max_acc*constraint_max_axis_acc(1,k)*[ -1 -1 1 1 -1 ],...
                  [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
            end
        end
        
        plot([0 total_time(end)],[0 0],'--','color',[0 0 0],'linewidth',0.5) % a = 0
        
        plot(total_time, acceleration_x(:,k),'color',color_drone(k,:),'linewidth',2); % a - evolution (drone k)
        
        plot(0:motion_time:motion_time*WPs_total , acceleration_x(1:length(time):end,k),'.','color',color_drone(k,:),'markersize',10); % a - WPs (drone k)
        
        plot([0 total_time(end)],-max_acc*constraint_max_axis_acc(1,k)*[1 1],'--','color',[1 0 0],'linewidth',1) % min a
        plot([0 total_time(end)], max_acc*constraint_max_axis_acc(1,k)*[1 1],'--','color',[1 0 0],'linewidth',1) % max a

        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) 1.1*max_acc*constraint_max_axis_acc(1,k)*[-1 1] ])
        
        % Visualization:
        title('Acceleration')
        xlabel('t [s]')
        ylabel('a_x [m/s^2]')
        % legend([ 'Drone ' num2str(k) ],'location','best')
        set(gca,'fontsize',10,'fontweight','bold')

    subplot(3,3,6) % a_y
        hold on
        
        % Plots:
        plot(0,0,'color',color_drone(k,:),'linewidth',2); % No representation. It keeps space for the legend (drone k)
        
        for i=1:length(WPs_cumsum{k})/2
            if sequence{k}(1+i) <= 0
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Refilling intervals
                  1.1*max_acc*constraint_max_axis_acc(2,k)*[ -1 -1 1 1 -1 ],...
                  [0 1 0],'EdgeColor','none','FaceAlpha',0.1)
            else
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                  1.1*max_acc*constraint_max_axis_acc(2,k)*[ -1 -1 1 1 -1 ],...
                  [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
            end
        end
        
        plot([0 total_time(end)],[0 0],'--','color',[0 0 0],'linewidth',0.5) % a = 0
        
        plot(total_time, acceleration_y(:,k),'color',color_drone(k,:),'linewidth',2); % a - evolution (drone k)
        
        plot(0:motion_time:motion_time*WPs_total , acceleration_y(1:length(time):end,k),'.','color',color_drone(k,:),'markersize',10); % a - WPs (drone k)
        
        plot([0 total_time(end)],-max_acc*constraint_max_axis_acc(2,k)*[1 1],'--','color',[1 0 0],'linewidth',1) % min a
        plot([0 total_time(end)], max_acc*constraint_max_axis_acc(2,k)*[1 1],'--','color',[1 0 0],'linewidth',1) % max a

        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) 1.1*max_acc*constraint_max_axis_acc(2,k)*[-1 1] ])
        
        % Visualization:
        xlabel('t [s]')
        ylabel('a_y [m/s^2]')
        % legend([ 'Drone ' num2str(k) ],'location','best')
        set(gca,'fontsize',10,'fontweight','bold')
        
    subplot(3,3,9) % a_z
        hold on
        
        % Plots:
        plot(0,0,'color',color_drone(k,:),'linewidth',2); % No representation. It keeps space for the legend (drone k)
        
        for i=1:length(WPs_cumsum{k})/2
            if sequence{k}(1+i) <= 0
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Refilling intervals
                  1.1*max_acc*constraint_max_axis_acc(3,k)*[ -1 -1 1 1 -1 ],...
                  [0 1 0],'EdgeColor','none','FaceAlpha',0.1)
            else
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                  1.1*max_acc*constraint_max_axis_acc(3,k)*[ -1 -1 1 1 -1 ],...
                  [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
            end
        end
        
        plot([0 total_time(end)],[0 0],'--','color',[0 0 0],'linewidth',0.5) % a = 0
        
        plot(total_time, acceleration_z(:,k),'color',color_drone(k,:),'linewidth',2); % a - evolution (drone k)
        
        plot(0:motion_time:motion_time*WPs_total , acceleration_z(1:length(time):end,k),'.','color',color_drone(k,:),'markersize',10); % a - WPs (drone k)
        
        plot([0 total_time(end)],-max_acc*constraint_max_axis_acc(3,k)*[1 1],'--','color',[1 0 0],'linewidth',1) % min a
        plot([0 total_time(end)], max_acc*constraint_max_axis_acc(3,k)*[1 1],'--','color',[1 0 0],'linewidth',1) % max a

        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) 1.1*max_acc*constraint_max_axis_acc(3,k)*[-1 1] ])
        
        % Visualization:
        xlabel('t [s]')
        ylabel('a_z [m/s^2]')
        % legend([ 'Drone ' num2str(k) ],'location','best')
        set(gca,'fontsize',10,'fontweight','bold')
        
end

if selection == 11
    %%% --- HORIZONTAL VELOCITIES ------------------------------------- %%%
    figure

    for k = 1 : drones
        subplot(drones,1,k)
        hold on

        % Plots:
        plot(0,0,'color',color_drone(k,:),'linewidth',2); % No representation. It keeps space for the legend (drone k)

        for i=1:length(WPs_cumsum{k})/2
            if sequence{k}(1+i) <= 0
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Refilling intervals
                  [ -0.1  -0.1 max( [ velocity_max_range(k) ; velocity_xy(:,k) ] )+0.5 max( [ velocity_max_range(k) ; velocity_xy(:,k) ] )+0.5  -0.1 ],...
                  [0 1 0],'EdgeColor','none','FaceAlpha',0.1)
            else
                patch( motion_time*WPs_cumsum{k}([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                  [ -0.1  -0.1 max( [ velocity_max_range(k) ; velocity_xy(:,k) ] )+0.5 max( [ velocity_max_range(k) ; velocity_xy(:,k) ] )+0.5  -0.1 ],...
                  [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
            end
        end

        plot([0 total_time(end)],[0 0],'--','color',[0 0 0],'linewidth',0.5) % v = 0

        plot(total_time, velocity_xy(:,k),'color',color_drone(k,:),'linewidth',2); % v - evolution (drone k)

        plot(0:motion_time:motion_time*WPs_total , velocity_xy(1:length(time):end,k),'.','color',color_drone(k,:),'markersize',10); % v - WPs (drone k)

        plot([0 total_time(end)], velocity_max_range(k)*[1 1],'--','color',[1 0 0],'linewidth',1) % v maximum range

        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) -0.1 max( [ velocity_max_range(k) ; velocity_xy(:,k) ] )+0.5 ])

        % Visualization:
        if k == 1
            title('Horizontal velocity')
        end
        xlabel('t [s]')
        ylabel('|v_{xy}| [m/s]')

        if selection ==10
            legend('Drone 3','location','best')
        else
            legend([ 'Drone ' num2str(k) ],'location','best')
        end

        set(gca,'fontsize',10,'fontweight','bold')

    end

end

%%% --- MUTUAL DISTANCES ---------------------------------------------- %%%
if drones >1
    
    figure

    for k = 1 : n_combos
        subplot(1,n_combos,k)
        hold on

        % Plots:
        plot(0,0,'color',[0 0.5 0],'linewidth',2); % No representation. It keeps space for the legend

        % for i=1:length(WPs_cumsum)/2
        %    patch( motion_time*WPs_cumsum([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
        %          [ -0.5 -0.5 1.1*max(mutual_distance) 1.1*max(mutual_distance) -0.5 ],...
        %          [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
        % end

        plot(total_time, mutual_distance(:,k),'color',[0 0.5 0],'linewidth',2); % d - evolution

        plot(0:motion_time:motion_time*WPs_total , mutual_distance(1:length(time):end,k),'.','color',[0 0.5 0],'markersize',10); % d - WPs

        plot([0 total_time(end)],delta_min*[1 1],'--','color',[1 0 0],'linewidth',1) % min d

        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) 0 1.1*max(mutual_distance(:,k)) ])

        % Visualization:
        title('Mutual distance')
        xlabel('t [s]')
        ylabel('d [m]')
        legend([ 'Drone ' num2str(combos(k,1)) ' - Drone ' num2str(combos(k,2)) ],'location','best')
        set(gca,'fontsize',10,'fontweight','bold')
    end
end
    