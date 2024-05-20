% Starting from pos_x, pos_y, and pos_z, the script computes the velocity
% and acceleration signals

%% --- Initialization: ------------------------------------------------- %%

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
WPs_cumsum = cumsum(WPs_sequence);

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

% Mutual distance between drones:
mutual_distance = zeros( size(pos_x,1) , 1 );

for j = 1:length(mutual_distance)
    mutual_distance(j) = norm( [ pos_x(j,2)-pos_x(j,1) , pos_y(j,2)-pos_y(j,1) , pos_z(j,2)-pos_z(j,1) ] );
end

%% --- Figures: -------------------------------------------------------- %%

figure

    %%% --- VELOCITY -------------------------------------------------- %%%
    subplot(4,2,1) % v_x
        hold on
        
        % Plots:
        plot(0,0,'color',[0.93,0.69,0.13],'linewidth',2); % No representation. It keeps space for the legend (drone 1)
        plot(0,0,'color',[0.49,0.18,0.56],'linewidth',2); % No representation. It keeps space for the legend (drone 2)
        
        for i=1:length(WPs_cumsum)/2
            patch( motion_time*WPs_cumsum([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                  1.1*max_vel*constraint_max_axis_vel(1,1)*[ -1 -1 1 1 -1 ],...
                  [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
        end
        
        plot([0 total_time(end)],[0 0],'--','color',[0 0 0],'linewidth',0.5) % v = 0
        
        plot(total_time, velocity_x(:,1),'color',[0.93,0.69,0.13],'linewidth',2); % v - evolution (drone 1)
        plot(total_time, velocity_x(:,2),'color',[0.49,0.18,0.56],'linewidth',2); % v - evolution (drone 2)
        
        plot(0:motion_time:motion_time*WPs_total , velocity_x(1:length(time):end,1),'.','color',[0.93,0.69,0.13],'markersize',10); % v - WPs (drone 1)
        plot(0:motion_time:motion_time*WPs_total , velocity_x(1:length(time):end,2),'.','color',[0.49,0.18,0.56],'markersize',10); % v - WPs (drone 2)
        
        plot([0 total_time(end)],-max_vel*constraint_max_axis_vel(1,1)*[1 1],'--','color',[1 0 0],'linewidth',1) % min v
        plot([0 total_time(end)], max_vel*constraint_max_axis_vel(1,1)*[1 1],'--','color',[1 0 0],'linewidth',1) % max v

        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) 1.1*max_vel*constraint_max_axis_vel(1,1)*[-1 1] ])
        
        % Visualization:
        title('Velocity')
        xlabel('t [s]')
        ylabel('v_x [m/s]')
        legend('Drone 1','Drone 2','location','best')
        set(gca,'fontsize',10,'fontweight','bold')

    subplot(4,2,3) % v_y
        hold on
        
        % Plots:
        plot(0,0,'color',[0.93,0.69,0.13],'linewidth',2); % No representation. It keeps space for the legend (drone 1)
        plot(0,0,'color',[0.49,0.18,0.56],'linewidth',2); % No representation. It keeps space for the legend (drone 2)
        
        for i=1:length(WPs_cumsum)/2
            patch( motion_time*WPs_cumsum([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                  1.1*max_vel*constraint_max_axis_vel(2,1)*[ -1 -1 1 1 -1 ],...
                  [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
        end
        
        plot([0 total_time(end)],[0 0],'--','color',[0 0 0],'linewidth',0.5) % v = 0
        
        plot(total_time, velocity_y(:,1),'color',[0.93,0.69,0.13],'linewidth',2); % v - evolution (drone 1)
        plot(total_time, velocity_y(:,2),'color',[0.49,0.18,0.56],'linewidth',2); % v - evolution (drone 2)
        
        plot(0:motion_time:motion_time*WPs_total , velocity_y(1:length(time):end,1),'.','color',[0.93,0.69,0.13],'markersize',10); % v - WPs (drone 1)
        plot(0:motion_time:motion_time*WPs_total , velocity_y(1:length(time):end,2),'.','color',[0.49,0.18,0.56],'markersize',10); % v - WPs (drone 2)
        
        plot([0 total_time(end)],-max_vel*constraint_max_axis_vel(2,1)*[1 1],'--','color',[1 0 0],'linewidth',1) % min v
        plot([0 total_time(end)], max_vel*constraint_max_axis_vel(2,1)*[1 1],'--','color',[1 0 0],'linewidth',1) % max v

        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) 1.1*max_vel*constraint_max_axis_vel(2,1)*[-1 1] ])
        
        % Visualization:
        xlabel('t [s]')
        ylabel('v_y [m/s]')
        % legend('Drone 1','Drone 2','location','best')
        set(gca,'fontsize',10,'fontweight','bold')
        
    subplot(4,2,5) % v_z
        hold on
        
        % Plots:
        plot(0,0,'color',[0.93,0.69,0.13],'linewidth',2); % No representation. It keeps space for the legend (drone 1)
        plot(0,0,'color',[0.49,0.18,0.56],'linewidth',2); % No representation. It keeps space for the legend (drone 2)
        
        for i=1:length(WPs_cumsum)/2
            patch( motion_time*WPs_cumsum([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                  1.1*max_vel*constraint_max_axis_vel(3,1)*[ -1 -1 1 1 -1 ],...
                  [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
        end
        
        plot([0 total_time(end)],[0 0],'--','color',[0 0 0],'linewidth',0.5) % v = 0
        
        plot(total_time, velocity_z(:,1),'color',[0.93,0.69,0.13],'linewidth',2); % v - evolution (drone 1)
        plot(total_time, velocity_z(:,2),'color',[0.49,0.18,0.56],'linewidth',2); % v - evolution (drone 2)
        
        plot(0:motion_time:motion_time*WPs_total , velocity_z(1:length(time):end,1),'.','color',[0.93,0.69,0.13],'markersize',10); % v - WPs (drone 1)
        plot(0:motion_time:motion_time*WPs_total , velocity_z(1:length(time):end,2),'.','color',[0.49,0.18,0.56],'markersize',10); % v - WPs (drone 2)
        
        plot([0 total_time(end)],-max_vel*constraint_max_axis_vel(3,1)*[1 1],'--','color',[1 0 0],'linewidth',1) % min v
        plot([0 total_time(end)], max_vel*constraint_max_axis_vel(3,1)*[1 1],'--','color',[1 0 0],'linewidth',1) % max v

        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) 1.1*max_vel*constraint_max_axis_vel(3,1)*[-1 1] ])
        
        % Visualization:
        xlabel('t [s]')
        ylabel('v_z [m/s]')
        % legend('Drone 1','Drone 2','location','best')
        set(gca,'fontsize',10,'fontweight','bold')
        
        
    %%% --- ACCELERATION ---------------------------------------------- %%%
    subplot(4,2,2) % a_x
        hold on
        
        % Plots:
        plot(0,0,'color',[0.93,0.69,0.13],'linewidth',2); % No representation. It keeps space for the legend (drone 1)
        plot(0,0,'color',[0.49,0.18,0.56],'linewidth',2); % No representation. It keeps space for the legend (drone 2)
        
        for i=1:length(WPs_cumsum)/2
            patch( motion_time*WPs_cumsum([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                  1.1*max_acc*constraint_max_axis_acc(1,1)*[ -1 -1 1 1 -1 ],...
                  [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
        end
        
        plot([0 total_time(end)],[0 0],'--','color',[0 0 0],'linewidth',0.5) % a = 0
        
        plot(total_time, acceleration_x(:,1),'color',[0.93,0.69,0.13],'linewidth',2); % a - evolution (drone 1)
        plot(total_time, acceleration_x(:,2),'color',[0.49,0.18,0.56],'linewidth',2); % a - evolution (drone 2)
        
        plot(0:motion_time:motion_time*WPs_total , acceleration_x(1:length(time):end,1),'.','color',[0.93,0.69,0.13],'markersize',10); % a - WPs (drone 1)
        plot(0:motion_time:motion_time*WPs_total , acceleration_x(1:length(time):end,2),'.','color',[0.49,0.18,0.56],'markersize',10); % a - WPs (drone 2)
        
        plot([0 total_time(end)],-max_acc*constraint_max_axis_acc(1,1)*[1 1],'--','color',[1 0 0],'linewidth',1) % min a
        plot([0 total_time(end)], max_acc*constraint_max_axis_acc(1,1)*[1 1],'--','color',[1 0 0],'linewidth',1) % max a

        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) 1.1*max_acc*constraint_max_axis_acc(1,1)*[-1 1] ])
        
        % Visualization:
        title('Acceleration')
        xlabel('t [s]')
        ylabel('a_x [m/s^2]')
        % legend('Drone 1','Drone 2','location','best')
        set(gca,'fontsize',10,'fontweight','bold')

    subplot(4,2,4) % a_y
        hold on
        
        % Plots:
        plot(0,0,'color',[0.93,0.69,0.13],'linewidth',2); % No representation. It keeps space for the legend (drone 1)
        plot(0,0,'color',[0.49,0.18,0.56],'linewidth',2); % No representation. It keeps space for the legend (drone 2)
        
        for i=1:length(WPs_cumsum)/2
            patch( motion_time*WPs_cumsum([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                  1.1*max_acc*constraint_max_axis_acc(2,1)*[ -1 -1 1 1 -1 ],...
                  [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
        end
        
        plot([0 total_time(end)],[0 0],'--','color',[0 0 0],'linewidth',0.5) % a = 0
        
        plot(total_time, acceleration_y(:,1),'color',[0.93,0.69,0.13],'linewidth',2); % a - evolution (drone 1)
        plot(total_time, acceleration_y(:,2),'color',[0.49,0.18,0.56],'linewidth',2); % a - evolution (drone 2)
        
        plot(0:motion_time:motion_time*WPs_total , acceleration_y(1:length(time):end,1),'.','color',[0.93,0.69,0.13],'markersize',10); % a - WPs (drone 1)
        plot(0:motion_time:motion_time*WPs_total , acceleration_y(1:length(time):end,2),'.','color',[0.49,0.18,0.56],'markersize',10); % a - WPs (drone 2)
        
        plot([0 total_time(end)],-max_acc*constraint_max_axis_acc(2,1)*[1 1],'--','color',[1 0 0],'linewidth',1) % min a
        plot([0 total_time(end)], max_acc*constraint_max_axis_acc(2,1)*[1 1],'--','color',[1 0 0],'linewidth',1) % max a

        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) 1.1*max_acc*constraint_max_axis_acc(2,1)*[-1 1] ])
        
        % Visualization:
        xlabel('t [s]')
        ylabel('a_y [m/s^2]')
        % legend('Drone 1','Drone 2','location','best')
        set(gca,'fontsize',10,'fontweight','bold')
        
    subplot(4,2,6) % a_z
        hold on
        
        % Plots:
        plot(0,0,'color',[0.93,0.69,0.13],'linewidth',2); % No representation. It keeps space for the legend (drone 1)
        plot(0,0,'color',[0.49,0.18,0.56],'linewidth',2); % No representation. It keeps space for the legend (drone 2)
        
        for i=1:length(WPs_cumsum)/2
            patch( motion_time*WPs_cumsum([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                  1.1*max_acc*constraint_max_axis_acc(3,1)*[ -1 -1 1 1 -1 ],...
                  [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
        end
        
        plot([0 total_time(end)],[0 0],'--','color',[0 0 0],'linewidth',0.5) % a = 0
        
        plot(total_time, acceleration_z(:,1),'color',[0.93,0.69,0.13],'linewidth',2); % a - evolution (drone 1)
        plot(total_time, acceleration_z(:,2),'color',[0.49,0.18,0.56],'linewidth',2); % a - evolution (drone 2)
        
        plot(0:motion_time:motion_time*WPs_total , acceleration_z(1:length(time):end,1),'.','color',[0.93,0.69,0.13],'markersize',10); % a - WPs (drone 1)
        plot(0:motion_time:motion_time*WPs_total , acceleration_z(1:length(time):end,2),'.','color',[0.49,0.18,0.56],'markersize',10); % a - WPs (drone 2)
        
        plot([0 total_time(end)],-max_acc*constraint_max_axis_acc(3,1)*[1 1],'--','color',[1 0 0],'linewidth',1) % min a
        plot([0 total_time(end)], max_acc*constraint_max_axis_acc(3,1)*[1 1],'--','color',[1 0 0],'linewidth',1) % max a

        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) 1.1*max_acc*constraint_max_axis_acc(3,1)*[-1 1] ])
        
        % Visualization:
        xlabel('t [s]')
        ylabel('a_z [m/s^2]')
        % legend('Drone 1','Drone 2','location','best')
        set(gca,'fontsize',10,'fontweight','bold')
        

    %%% --- MUTUAL DISTANCE ------------------------------------------- %%%
    subplot(4,2,7:8)
        hold on
        
        % Plots:
        plot(0,0,'color',[0 0.5 0],'linewidth',2); % No representation. It keeps space for the legend
        
        for i=1:length(WPs_cumsum)/2
            patch( motion_time*WPs_cumsum([ 2*i-1 2*i 2*i 2*i-1 2*i-1 ]),... % Installation intervals
                  [ -0.5 -0.5 1.1*max(mutual_distance) 1.1*max(mutual_distance) -0.5 ],...
                  [0 0 1],'EdgeColor','none','FaceAlpha',0.1)
        end
                
        plot(total_time, mutual_distance,'color',[0 0.5 0],'linewidth',2); % d - evolution
        
        plot(0:motion_time:motion_time*WPs_total , mutual_distance(1:length(time):end,1),'.','color',[0 0.5 0],'markersize',10); % d - WPs
        
        plot([0 total_time(end)],delta_min*[1 1],'--','color',[1 0 0],'linewidth',1) % min d

        % Axis:
        grid on; grid minor; box on
        axis([ total_time([1 end]) 0 1.1*max(mutual_distance) ])
        
        % Visualization:
        title('Mutual distance')
        xlabel('t [s]')
        ylabel('d [m]')
        legend('Drone 1 - Drone 2','location','best')
        set(gca,'fontsize',10,'fontweight','bold')        
