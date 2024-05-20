function sequence = get_sequence_multidepot(initial_position, goal, n_drones, n_targets, delta_max)
% The "get_sequence_multidepot" MATLAB function provides the inspection
% sequence for all the drones and target regions provided as input.
%
% Inputs:
% - initial_position, it contains the initial positions of the drones.
% - goal, it contains the goals need to be inspected by the drones.
% - n_drones, it containts the number of drones.
% - n_targets, it containts the number of target regions.
% - delta_max, it containts the maximum admissible difference between the path length of two drones
% 
% Outputs:
% - sequence, it contains the inspection sequence for all the drones. In
% other words, the order (1, 3, 4, 6) of the targets need to be inspected.
% The sequence is obtained minimizing the distance between the drone and
% the target (i.e., L2 norm of the distance). It has one row per drone.

%% --- Nodes: ---------------------------------------------------------- %%
edges_drone   = nchoosek(0:n_targets,2); % edges per drone [node_i node_j]
n_edges_drone = size(edges_drone,1);     % number of edges per drone

n_edges = n_edges_drone*n_drones; % total number of edges

%% --- Combination between drones: ------------------------------------- %%
if n_drones>1
    combos_drones   = nchoosek(1:n_drones, 2);  % combinations between drones
    n_combos_drones = size(combos_drones,1);    % number of combinations between drones
else
    n_combos_drones = 0;
end

%% --- Goal positions: ------------------------------------------------- %%
goal_matrix = zeros(n_targets,3);
for i = 1 : n_targets
    goal_matrix(i,:) = goal{i}.stop;
end

%%  --- Edges and Cost (euclidean distance): --------------------------- %%
edges = zeros( n_edges , 3 );
Cost  = zeros( n_edges , 1 );

for i = 1 : n_drones
    
    % Nodes [x,y,z]:
    nodes = [ initial_position(:,i)' ;    % drone position
              goal_matrix             ];  % target positions
          
    % Edges:
    edges(1+n_edges_drone*(i-1):n_edges_drone*i,:) = [edges_drone i*ones(n_edges_drone,1)];

    %%% Calculate the distance for each trip.
    %%% The cost function to minimize is the sum of the trip distances for each trip in the tour.
    % Computing the cost function as the sum of the L2 norm between all edges
    % in the graph.
    for j = 1+n_edges_drone*(i-1) : n_edges_drone*i
        Cost(j) = norm(nodes(edges(j,2)+1,:) - nodes(edges(j,1)+1,:),2);
    end

end

%% --- Sequence optimization: ------------------------------------------ %%

% coefficient vector of the objective function:
objective_fcn = [ Cost' ones(1,n_combos_drones) zeros(1,n_targets*n_drones) ];

% All the target regions are visited only once (one input + one output):
Aeq_targets = zeros( n_targets , n_edges+n_combos_drones+n_targets*n_drones );

for i = 1 : n_targets
    for j = 1 : n_edges
        if any(edges(j,1:2)==i)
           Aeq_targets(i,j)=1; 
        end
    end
end

beq_targets = 2*ones(n_targets,1);

% Each target region is visited by only one drone:
Aeq_targets_drone = zeros( n_targets*n_drones , n_edges+n_combos_drones+n_targets*n_drones );

for k= 1 : n_drones
    for i = 1 : n_targets
        for j = 1 : n_edges
            if any(edges(j,1:2)==i) && edges(j,3)==k
               Aeq_targets_drone( (k-1)*n_targets + i , j) = 1; 
            end
        end
        Aeq_targets_drone( (k-1)*n_targets + i , n_edges+n_combos_drones+(k-1)*n_targets + i) = -2; 
    end
end

beq_targets_drone = zeros(n_targets*n_drones,1);

% The depot node is visited twice per drone (one input + one output per drone)
Aeq_depot = zeros( n_drones , n_edges+n_combos_drones+n_targets*n_drones );

for i = 1 : n_drones
    for j = 1 : n_edges
        if any(edges(j,1:2)==0) && edges(j,3)==i
           Aeq_depot(i,j)=1;
        end
    end
end

beq_depot = 2*ones(n_drones,1);

% All the equality constraints together:
Aeq = [ Aeq_targets ; Aeq_targets_drone ; Aeq_depot ];
beq = [ beq_targets ; beq_targets_drone ; beq_depot ];


% The distance covered by all the drones should be as balanced as possible;
A = zeros( 2*n_combos_drones , n_edges+n_combos_drones+n_targets*n_drones );

for i = 1 : n_combos_drones
    A( 2*i-1 , 1+n_edges_drone*(combos_drones(i,1)-1) : n_edges_drone*combos_drones(i,1) ) =  Cost(1+n_edges_drone*(combos_drones(i,1)-1) : n_edges_drone*combos_drones(i,1));
    A( 2*i-1 , 1+n_edges_drone*(combos_drones(i,2)-1) : n_edges_drone*combos_drones(i,2) ) = -Cost(1+n_edges_drone*(combos_drones(i,2)-1) : n_edges_drone*combos_drones(i,2));
    A( 2*i-1 , n_edges+i ) = -1;
    
    A( 2*i , 1+n_edges_drone*(combos_drones(i,2)-1) : n_edges_drone*combos_drones(i,2) ) =  Cost(1+n_edges_drone*(combos_drones(i,2)-1) : n_edges_drone*combos_drones(i,2));
    A( 2*i , 1+n_edges_drone*(combos_drones(i,1)-1) : n_edges_drone*combos_drones(i,1) ) = -Cost(1+n_edges_drone*(combos_drones(i,1)-1) : n_edges_drone*combos_drones(i,1));
    A( 2*i , n_edges+i ) = -1;
end

b = zeros(2*n_combos_drones,1);

% Binary solution:
intcon = [1 : n_edges   n_edges+n_combos_drones+1 : n_edges+n_combos_drones+n_targets*n_drones ];  % intcon stands for integer variables

lb = zeros(n_edges+n_combos_drones+n_targets*n_drones,1);                                  % setting up lower bounds
ub = [ ones(n_edges,1) ; delta_max*ones(n_combos_drones,1) ; ones(n_targets*n_drones,1) ]; % setting up upper bounds
ub( edges(:,1)==0 ) = 2; % This allows back and forth routes between the depot and a single target

% To suppress iterative output, turn off the default display: opts = optimoptions('intlinprog','Display','off');

% Result:
% x = intlinprog(f    ,intcon,A ,b ,Aeq,beq,lb,ub);
x = intlinprog(objective_fcn,intcon,A,b,Aeq,beq,lb,ub);
x([1:n_edges, n_edges+n_combos_drones+1 : n_edges+n_combos_drones+n_targets*n_drones]) = round(x([1:n_edges, n_edges+n_combos_drones+1 : n_edges+n_combos_drones+n_targets*n_drones]));

%% --- Subtour detection: ---------------------------------------------- %%
[edges_tour,tours,n_subtour] = subtours_detection_multidepot(x,edges);

% %% --- Figures: -------------------------------------------------------- %%
% plot_tour_multidepot(initial_position,goal_matrix,n_drones,edges_tour)

%% --- Breaking subtours: ---------------------------------------------- %%
%%%% Because you can't add all of the subtour constraints, take an iterative approach. 
%%%% Detect the subtours in the current solution, then add inequality constraints to prevent 
%%%% those particular subtours from happening. By doing this, you find a suitable tour in a few iterations.

% while the number of subtours is greater than zero, there exist
% at least one subtour
while n_subtour>0

    % pause % for visualization purposes

    A_p = zeros(n_subtour,n_edges+n_combos_drones+n_targets*n_drones);
    b_p = zeros(n_subtour,1);
    
    for i = 1 : n_subtour
        
        edges_sub = nchoosek(tours{1+i}(1:end-1),2);
        for j=1:size(edges_sub,1)
            if edges_sub(j,1)>edges_sub(j,2)
                edges_sub(j,[1 2])=edges_sub(j,[2 1]);
            end
        end

        for j = 1:n_edges
            if any( all( edges(j,[1 2]) == edges_sub , 2 ) )
                A_p(i,j) = 1;
            end
        end

        b_p(i) = size(tours{1+i},1)-2;
    end

    A = [ A ; A_p ];
    b = [ b ; b_p ];

    % Result:
    % x = intlinprog(f    ,intcon,A ,b ,Aeq,beq,lb,ub);
    x = intlinprog(objective_fcn,intcon,A,b,Aeq,beq,lb,ub);
    x([1:n_edges, n_edges+n_combos_drones+1 : n_edges+n_combos_drones+n_targets*n_drones]) = round(x([1:n_edges, n_edges+n_combos_drones+1 : n_edges+n_combos_drones+n_targets*n_drones]));

    %% --- Subtour detection: -------------------------------------- %%
    [edges_tour,tours,n_subtour] = subtours_detection_multidepot(x,edges);

end

%% --- Figures: -------------------------------------------------------- %%
plot_tour_multidepot(initial_position,goal_matrix,n_drones,edges_tour)

%% --- Optimal sequence: ----------------------------------------------- %%
% distance_1 = Cost(1:n_edges_drone)'*x(1:n_edges_drone);
% distance_2 = Cost(1+n_edges_drone:2*n_edges_drone)'*x(1+n_edges_drone:2*n_edges_drone);
% delta =  x(n_edges+1:n_edges+n_combos_drones);

sequence_ini = find(tours{1}==0);

sequence = cell(n_drones,1);
for i = 1 : n_drones
    sequence{i} = tours{1}( sequence_ini(i):sequence_ini(i+1) )';
end

end

%% ===================================================================== %%
%                            Subtour detection                            %
% ======================================================================= %
function [edges_tour,tours,n_subtour] = subtours_detection_multidepot(x,edges)
    
    n_edges      = size(edges,1);
    edges_true   = [ edges( x(1:n_edges)~=0 , : ) ; edges( x(1:n_edges)==2 , : ) ];
    n_edges_true = size(edges_true,1);
    
    edges_tour   = zeros(n_edges_true,3);
    node_current = 0;
    n_subtour    = 0;

    for i=1:n_edges_true
        edge_current_id = find( edges_true(:,1:2) == node_current ,1 );
        
        if isempty(edge_current_id)
            edge_current_id = find(edges_true>0,1);
            n_subtour = n_subtour + 1;
        end

        if edge_current_id>n_edges_true
            edge_current_id=edge_current_id-n_edges_true;
            edges_true(edge_current_id,[1 2])=edges_true(edge_current_id,[2 1]);
        end
        
        node_current = edges_true(edge_current_id,2);
        edges_tour(i,:) = edges_true(edge_current_id,:);
        edges_true(edge_current_id,:) = [-1 -1 -1];
    end
    
    subtour_ini = [ 1     find( edges_tour( 1:n_edges_true-1 , 2 ) ~= edges_tour( 2:n_edges_true , 1 ) )' + 1     n_edges_true+1 ];
    
    tours = cell(1+n_subtour,1);
    for i=1:1+n_subtour
        tours{i} = [ edges_tour( subtour_ini(i):subtour_ini(i+1)-1,1 )  ;  edges_tour( subtour_ini(i),1 ) ];
    end

    disp('=============================')
    disp(['Number of subtours: ', num2str(n_subtour)])
    for i=1:n_subtour
        disp(['-Subtour ',num2str(i), ' (nodes): ', num2str(tours{1+i}')])
    end
    disp('=============================')
    disp(' ')
    disp(' ')

end

%% ===================================================================== %%
%                          Route representation                           %
% ======================================================================= %

function plot_tour_multidepot(initial_position,goal_matrix,n_drones,edges_tour)
% --- Figures: ---------------------------------------------------------- %
figure
hold on

% Initial positions:
for i = 1:n_drones
    plot3(initial_position(1,i),initial_position(2,i),initial_position(3,i),'ro','linewidth',4,'markersize',4)
end

% Targets:
for i = 1:size(goal_matrix,1)
    plot3(goal_matrix(1:end,1),goal_matrix(1:end,2),goal_matrix(1:end,3),'ko','linewidth',4,'markersize',4)
end


% Node id:
for i=1:1:size(initial_position,2)
    text( initial_position(1,i) , initial_position(2,i) , initial_position(3,i) , [ '  0 (Drone ' num2str(i) ')'] , 'fontsize' , 12 , 'fontweight','bold')
end

for i=1:size(goal_matrix,1)
    text( goal_matrix(i,1) , goal_matrix(i,2) , goal_matrix(i,3) , ['  ' num2str(i)] , 'fontsize' , 12 , 'fontweight','bold')
end

% Computed route:
color_drone = [ 0.93 0.69 0.13 ;   % drone 1
                0.49 0.18 0.56 ;   % drone 2
                0.26 0.72 0.54 ;   % drone 3
                0.46 0.32 0.74  ]; % drone 4

for i = 1 : n_drones
    
    nodes = [ initial_position(:,i)' ; goal_matrix ];
    
    edges_tour_drone = edges_tour( edges_tour(:,3)==i , : );
    
    for j = 1:size(edges_tour_drone,1)
        P1 = nodes(1+edges_tour_drone(j,1),:);
        P2 = nodes(1+edges_tour_drone(j,2),:);
        D  = P2-P1;
        quiver3(P1(1),P1(2),P1(3),D(1),D(2),D(3),0,'MaxHeadSize',5/vecnorm(D,2,2),'color',color_drone(i,:),'linestyle','-','linewidth',2)
    end
end

% Some improvements in the figure:
nodes = [ initial_position' ; goal_matrix ];

axis equal
axis([ min(nodes(:,1))-3 max(nodes(:,1))+3 min(nodes(:,2))-3 max(nodes(:,2))+3 min(nodes(:,3))-3 max(nodes(:,3))+3])
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

grid on
grid minor
box on
set(gca,'fontsize',12,'fontweight','bold');
view(70,40)
% set(gcf,'position',[-1365  323  1366  651])

end

% ======================================================================= %