function obstacles = getObstacles(map, varargin)
% The "getObstacles" MATLAB function allows of getting obstacles from the 
% map considering also its dimension. The function can be used for reading 
% obstacles from multiple files (txt) in one single shot
%
% Inputs:
% - map, map file name expressed a string. It is a txt file.
%
% Outputs:
% - obstacles contains the lower and upper bounds delimiting the object in
% a 3D space and the color (R G B) will be used for plotting, obstacles in
% \mathbb{R}^{100 \times 9}
%


% Flag used to exit from obstacles matrix. The matrix can be used for
% reading till 100 obstacles in a scenario
ext_obstacles_flag = 0;
ext_obstacles = [];

% varargin allows any number of arguments to a function.
% this is used to skip if on line 56
if size(varargin)
    ext_obstacles = varargin{1};
    ext_obstacles_flag = 1;
end

% Define containers for reading the txt file
fid = fopen(map);

% Matrix of obstacles = [lowx lowy lowz upperx uppery upperz R G B]
% An obstacles is modeled as a polyhedron
% RGB indicate the color of the obstacle in the scenario
obstacles_from_file = zeros(100, 9); 
i = 1; % index for navigating the matrix

% Matrix of piece of power tower = [lowx lowy lowz upperx uppery upperz R G B]
% An a piece of the power tower is modeled as a polyhedron
% RGB indicate the color of the obstacle in the scenario
tower_from_file = zeros(100, 9); 
j = 1; % index for navigating the matrix

% Start reading the map
while ~feof(fid) % loop over the following until the end of the file is reached.
    line = fgets(fid); % read in one line
    if strfind(line,'#') % skip comments
        continue
    elseif strfind(line,'map_dimension') % for map_dimension
        % [lowx lowy lowz upperx uppery upperz]
        B = str2num(erase(line,'map_dimension'));
        continue
    elseif strfind(line,'obstacles') % for obstacles
        % [lowx lowy lowz upperx uppery upperz R G B]
        b = str2num(erase(line,'obstacles'));
        obstacles_from_file(i,:) = b;
        i = i + 1;    
    else
        continue
    end
end

fclose(fid); % close the file
obstacles_from_file(i:end,:) = [];
obstacles_from_file = obstacles_from_file(:,1:6); % [lowx lowy lowz upperx uppery upperz]

[N, ~] = size(obstacles_from_file(:,1)); % rows

% Define Obstacles as polyhedrons
obstacles = cell(N+size(ext_obstacles,1),1); % for plotting 

% for simple map types
A = [-eye(3);eye(3)];

% Creating a cell with all data - obstacles
for i = 1 : N
   obstacles{i}.lb = obstacles_from_file(i,1:3); % [lowx lowy lowz]
   obstacles{i}.ub = obstacles_from_file(i,4:6); % [upperx uppery upperz]
   obstacles{i}.A = A; % simple map type
   obstacles{i}.b = [-obstacles{i}.lb obstacles{i}.ub]'; % [lowx lowy lowz upperx uppery upperz]'
   obstacles{i}.shape = Polyhedron('lb', obstacles{i}.lb,'ub', obstacles{i}.ub);
end

if (ext_obstacles_flag && size(ext_obstacles,1))
    
    for i = N+1:size(ext_obstacles(:,1))+N
        
        obstacles{i}.lb = ext_obstacles(i-N,1:3); 
        obstacles{i}.ub = ext_obstacles(i-N,4:6);
        obstacles{i}.A = A;
        obstacles{i}.b = [-obstacles{i}.lb obstacles{i}.ub]';
        % Create a polyhedron object.
        % P = {x | H*[x;-1] &lt;= 0} cap {x | He*[x;-1] = 0}
        % or
        % P = {V'lam | lam &gt;= 0, sum(lam) = 1} + {R'gam | gam &gt;= 0}
        obstacles{i}.shape = Polyhedron('lb', obstacles{i}.lb,'ub', obstacles{i}.ub);
        
    end
    
end

% for plotting - axis size of the map [lowx lowy lowz upperx uppery upperz]
axis([B(1) B(4) B(2) B(5) B(3) B(6)]);

% axis EQUAL  sets the aspect ratio so that equal tick mark
% increments on the x-,y- and z-axis are equal in size. This
% makes SPHERE(25) look like a sphere, instead of an ellipsoid.
axis equal

end
