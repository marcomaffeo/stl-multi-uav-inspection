clear all
close all
clc

%% Plot two towers plus cables with a single stl file

data_stl_2towers_cables = stlread('power_tower_danube_2tower_and_wires.stl');

figure()
trimesh(data_stl_2towers_cables,'FaceColor','none','EdgeColor','k');
xlabel('x')
ylabel('y');
zlabel('z');
axis equal

%% Plot two towers plus cables

data_stl_cables = stlread('power_tower_danube_wires.stl');
data_stl_tower1 = stlread('power_tower_danube.stl');
data_stl_tower2 = stlread('power_tower_danube_tower2.stl');

figure()
xlabel('x')
ylabel('y');
zlabel('z');
axis equal
trimesh(data_stl_tower1,'FaceColor','none','EdgeColor','k');
hold on
trimesh(data_stl_cables,'FaceColor','none','EdgeColor','k');
trimesh(data_stl_tower2,'FaceColor','none','EdgeColor','k');
hold off


%% power_tower_danube_wires

% To read the STL file
data = stlread('power_tower_danube_wires.stl');

% Triangular surface plot. It displays the triangles defined in the M-by-3
% face matrix TRI as a surface.
figure()
trisurf(data);
title('Triangular surface plot');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-3 3 -10 10 0 25]);

% Plots a 2D triangulation. It displays the triangles defined in the
% M-by-3 matrix TRI.
figure()
triplot(data);
title('Plots a 2D triangulation');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-3 3 -10 10]);

% Triangular mesh plot. It displays the triangles defined in the M-by-3
% face matrix TRI as a mesh
figure()
trimesh(data,'FaceColor','none','EdgeColor','k')
title('Triangular mesh plot');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-3 3 -10 10 0 25]);

%% power_tower_danube

% To read the STL file
data = stlread('power_tower_danube.stl');

% Triangular surface plot. It displays the triangles defined in the M-by-3
% face matrix TRI as a surface.
figure()
trisurf(data);
title('Triangular surface plot');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-3 3 -10 10 0 25]);

% Plots a 2D triangulation. It displays the triangles defined in the
% M-by-3 matrix TRI.
figure()
triplot(data);
title('Plots a 2D triangulation');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-3 3 -10 10]);

% Triangular mesh plot. It displays the triangles defined in the M-by-3
% face matrix TRI as a mesh
figure()
trimesh(data,'FaceColor','none','EdgeColor','k')
title('Triangular mesh plot');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-3 3 -10 10 0 25]);


%% power_tower_danube_lowpoly

% To read the STL file
data = stlread('power_tower_danube_lowpoly.stl');

% Triangular surface plot. It displays the triangles defined in the M-by-3
% face matrix TRI as a surface.
figure(67)
trisurf(data);
title('Triangular surface plot');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-3 3 -10 10 0 25]);


% Plots a 2D triangulation. It displays the triangles defined in the
% M-by-3 matrix TRI.
figure()
triplot(data);
title('Plots a 2D triangulation');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-3 3 -10 10]);

% Triangular mesh plot. It displays the triangles defined in the M-by-3
% face matrix TRI as a mesh
figure()
trimesh(data,'FaceColor','none','EdgeColor','k')
title('Triangular mesh plot');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-3 3 -10 10 0 25]);

%% Leica model 2towers

% To read the STL file
data = stlread('2towers.stl');

% Triangular surface plot. It displays the triangles defined in the M-by-3
% face matrix TRI as a surface.
figure()
trisurf(data);
title('Triangular surface plot');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-10 60 -13 8 -1.0 5]);

% Plots a 2D triangulation. It displays the triangles defined in the
% M-by-3 matrix TRI.
figure()
triplot(data);
title('Plots a 2D triangulation');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-10 60 -13 8 -1.0 5]);

% Triangular mesh plot. It displays the triangles defined in the M-by-3
% face matrix TRI as a mesh
figure()
trimesh(data,'FaceColor','none','EdgeColor','k')
title('Triangular mesh plot');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-10 60 -13 8 -1.0 5]);

%% Leica model power tower

% To read the STL file
data = stlread('tower.stl');

% Triangular surface plot. It displays the triangles defined in the M-by-3
% face matrix TRI as a surface.
figure()
trisurf(data);
title('Triangular surface plot');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-4.5 4.5 -6 6 0 20]);

% Plots a 2D triangulation. It displays the triangles defined in the
% M-by-3 matrix TRI.
figure()
triplot(data);
title('Plots a 2D triangulation');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-4.5 4.5 -6 6 0 20]);

% Triangular mesh plot. It displays the triangles defined in the M-by-3
% face matrix TRI as a mesh
figure()
trimesh(data,'FaceColor','none','EdgeColor','k')
title('Triangular mesh plot');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-4.5 4.5 -6 6 0 20]);

%% Plot power line mock-up

% To read the STL file
data = stlread('power_line_mock-up.stl');

% Triangular surface plot. It displays the triangles defined in the M-by-3
% face matrix TRI as a surface.
figure()
trisurf(data);
title('Triangular surface plot');
xlabel('x');
ylabel('y');
zlabel('z');

% Plots a 2D triangulation. It displays the triangles defined in the
% M-by-3 matrix TRI.
figure()
triplot(data);
title('Plots a 2D triangulation');
xlabel('x');
ylabel('y');
zlabel('z');

% Triangular mesh plot. It displays the triangles defined in the M-by-3
% face matrix TRI as a mesh
figure()
trimesh(data,'FaceColor','none','EdgeColor','k')
title('Triangular mesh plot');
xlabel('x');
ylabel('y');
zlabel('z');

%% Plot power line mock-up

% To read the STL file
data = stlread('power_line_mock-up_scaled.stl');

% Triangular surface plot. It displays the triangles defined in the M-by-3
% face matrix TRI as a surface.
figure()
trisurf(data);
title('Triangular surface plot');
xlabel('x');
ylabel('y');
zlabel('z');

% Plots a 2D triangulation. It displays the triangles defined in the
% M-by-3 matrix TRI.
figure()
triplot(data);
title('Plots a 2D triangulation');
xlabel('x');
ylabel('y');
zlabel('z');

% Triangular mesh plot. It displays the triangles defined in the M-by-3
% face matrix TRI as a mesh
figure()
trimesh(data,'FaceColor','none','EdgeColor','k')
title('Triangular mesh plot');
xlabel('x');
ylabel('y');
zlabel('z');
