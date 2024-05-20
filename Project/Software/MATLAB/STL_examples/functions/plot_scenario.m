function plot_scenario(map, path)
% The "plot_scenario" MATLAB function creates a figure showing a path within a
% scenario. The path is a N-by-3 matrix where each row corresponds to the
% (x, y, z) coordinates of one point along the path.
%
% Inputs:
% - map, map file name expressed a string. It is a txt file.
% - path, a path. It can be an empty value
%

% [v_lowx v_lowy v_lowz v_upperx v_uppery v_upperz]
vertex_x = zeros(4, 6 * map.number_obstacles); 
vertex_y = zeros(4, 6 * map.number_obstacles); 
vertex_z = zeros(4, 6 * map.number_obstacles);


% Representing the obstacles into the map
for i = 1 : map.number_obstacles-4
% for i = 1 : map.number_obstacles
    
    % [lowx lowy lowz]
    lowerLimit = map.obstacles(i,1:3);
    
    % [upperx uppery upperz]
    upperLimit = map.obstacles(i,4:6);
    
    vertices = [ lowerLimit(1:3); ...
                 upperLimit(1) lowerLimit(2) lowerLimit(3);...
                 upperLimit(1) lowerLimit(2) upperLimit(3);...
                 lowerLimit(1) lowerLimit(2) upperLimit(3);...
                 lowerLimit(1) upperLimit(2) upperLimit(3);...
                 upperLimit(1) upperLimit(2) upperLimit(3);...
                 upperLimit(1) upperLimit(2) lowerLimit(3);...
                 lowerLimit(1) upperLimit(2) lowerLimit(3)];
 
    start_y = (i-1)*6+1;
    
    vertex_x(:,start_y:start_y+5) = [
        vertices(1,1) vertices(2,1) vertices(3,1) vertices(1,1) vertices(1,1) vertices(5,1); ...
        vertices(2,1) vertices(3,1) vertices(6,1) vertices(8,1) vertices(2,1) vertices(8,1);...
        vertices(3,1) vertices(6,1) vertices(5,1) vertices(5,1) vertices(7,1) vertices(7,1);...
        vertices(4,1) vertices(7,1) vertices(4,1) vertices(4,1) vertices(8,1) vertices(6,1)];
    
    vertex_y(:,start_y:start_y+5) = [
        vertices(1,2) vertices(2,2) vertices(3,2) vertices(1,2) vertices(1,2) vertices(5,2); ...
        vertices(2,2) vertices(3,2) vertices(6,2) vertices(8,2) vertices(2,2) vertices(8,2);...
        vertices(3,2) vertices(6,2) vertices(5,2) vertices(5,2) vertices(7,2) vertices(7,2);...
        vertices(4,2) vertices(7,2) vertices(4,2) vertices(4,2) vertices(8,2) vertices(6,2)];
    
    vertex_z(:,start_y:start_y+5) = [
        vertices(1,3) vertices(2,3) vertices(3,3) vertices(1,3) vertices(1,3) vertices(5,3); ...
        vertices(2,3) vertices(3,3) vertices(6,3) vertices(8,3) vertices(2,3) vertices(8,3);...
        vertices(3,3) vertices(6,3) vertices(5,3) vertices(5,3) vertices(7,3) vertices(7,3);...
        vertices(4,3) vertices(7,3) vertices(4,3) vertices(4,3) vertices(8,3) vertices(6,3)];
end

% figure object is created in the caller function
hold on; % for displaying multiple obstacles on the same plot
grid on; % enables grids
grid minor;
xlabel('x'); % labels (rewritten by the followin xlabel command. Kept for further changes)
ylabel('y'); % labels (rewritten by the followin xlabel command. Kept for further changes)
zlabel('z'); % labels (rewritten by the followin xlabel command. Kept for further changes)

% Filled 3-D polygons. 
% fill3(X,Y,Z,C) fills the 3-D polygon defined by vectors X, Y and Z
% with the color specified by C. If C is an RGB row vector triple, [r g b]
% [1, 1, 1] = black
fill3(vertex_x, vertex_y, vertex_z, [1 1 1]); 
alpha(.24); % for the grids

% Plotting the path. Skipped if the path is empty. 
% The path can be a vector. It will be displayed dashed
for i = 1 : length(path)
    if (size(path{i}))
        plot3(path{i}(:,1), path{i}(:,2), path{i}(:,3), '-', 'linewidth', 3); 
    end
end

% disable holding figure
hold off;

% For the axes
data = map.map_dimension;
axis ([data(1) data(4) data(2) data(5) data(3) data(6)]);

end


