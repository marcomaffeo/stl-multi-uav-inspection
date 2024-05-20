function map = load_map(filename, xy_res, z_res, margin)
% The "load_map" MATLAB function load a map from a txt file. The function
% creates an occupancy grid map where a node is considered fill if it lies 
% within 'margin' distance of an obstacle.
% Occupancy grids are used to represent a robot workspace as a discrete grid. 
%
% Inputs:
% - filename, the name of the file contaning the map. It is a txt file
% - xy_res, the sampling space value. It is a double \in \mathbb{R}
% - z_res, the sampling space value. It is a double \in \mathbb{R}
% - margin, margin \in \mathbb{R}
%
% Outputs:
% - map, an occupancy grid map

% Define containers
fid = fopen(filename);

map.xy_res = xy_res; % for partitioning the xy space, sampling space value
map.z_res = z_res; % for partitioning th z space, sampling space value
map.margin = margin;
map.m_grid = zeros(0, 0, 0, 0);

m = margin;
map_dimension = [];
obstacles_from_file = zeros(100, 9);
v = 1;

% Start reading the map
while ~feof(fid) % loop over the following until the end of the file is reached.
    line = fgets(fid); % read in one line
    if strfind(line,'#') % skip comments
        continue
    elseif strfind(line,'map_dimension') % for map dimension
        % [lowx lowy lowz upperx uppery upperz]
        B = str2num(erase(line,'map_dimension'));
        map_dimension = B;
        
        % partitioning xy space - x vector
        x = B(1) : xy_res : (B(4));
        
        % partitioning xy space - y vector
        y = B(2) : xy_res : (B(5));
        
        % partitioning z space - z vector
        z = B(3) : z_res : (B(6));
        
        % Map length along each axis, i.e., x, y and z
        map.xlen = length(x);
        map.ylen = length(y);
        map.zlen = length(z);
        % Map dimension. It is mainly the area of the cuboid
        map.dimension = map.xlen * map.ylen * map.zlen;
        
        % map_grid initialization as a nan vector (a cell to be precise)
        m_grid = nan(map.xlen, map.ylen, map.zlen, 4);
        
        % Filling the grid map
        for i = 1 : map.xlen
            for j = 1 : map.ylen
                for k = 1 : map.zlen
                    m_grid(i, j, k, 1:3) = [x(i) y(j) z(k)];
                end
            end
        end
        
    elseif strfind(line,'obstacles') % for obstacles
        % [lowx lowy lowz upperx uppery upperz R G B]
        b = str2num(erase(line,'obstacles'));
        obstacles_from_file(v,:) = b;
        v = v + 1;
        
    else
        continue
    end
end

fclose(fid); % close the file

obstacles_from_file(v:end,:) = [];
map.map_dimension = map_dimension;
map.obstacles = obstacles_from_file;
map.number_obstacles = v-1;

for p = 1 : v-1
    
    % Add obstacles to map
    B = map_dimension;
    b = obstacles_from_file(p,:);
    cb = 1000000*b(7) + 1000*b(8) + b(9); % color bar

    % Converts the vector X to single precision. X can be any
    % numeric object (such as a DOUBLE).
    % Obstacle - Map_dimension (coordinates, i.e., x, y, and z) /
    % partitioning space along axes
    % m = margin
    x_start = 1 + floor(single( (b(1) - B(1) - m )/ xy_res) );
    y_start = 1 + floor(single( (b(2) - B(2) - m )/ xy_res) );
    z_start = 1 + floor(single( (b(3) - B(3) - m )/ z_res) );
    
    x_end = 1 + floor(single( (b(4) - B(1) + m )/ xy_res));
    y_end = 1 + floor(single( (b(5) - B(2) + m )/ xy_res));
    z_end = 1 + floor(single( (b(6) - B(3) + m )/ z_res));
    
    for a = x_start : x_end
        for b = y_start : y_end
            for c = z_start : z_end
                if ((a > 0) && (b > 0) && (c > 0)...
                        && (a <= map.xlen) && (b <= map.ylen)...
                        && (c <= map.zlen))
                    m_grid(a, b, c, 4) = cb; % color bar
                end
            end
        end
    end
end

% Mapping grid
map.m_grid = m_grid;

end
