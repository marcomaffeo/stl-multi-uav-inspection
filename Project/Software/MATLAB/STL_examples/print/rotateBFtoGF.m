function [x_out, y_out, z_out] = rotateBFtoGF(x, y, z, phi, theta, psi)
% The "rotateBFtoGF" MATLAB function rotates a point or matrix of points 
% from the Body Frame to the Global Frame based on the quadrotor's Euler 
% angles (orientation)
%
% Inputs:
% - x, y and z are the drone position coordinates
% - phi, theta, and yaw are the drone attitude
%
% Outputs:
% - x_out, y_out and z_out are the new drone position coordinates

% This function 
  % define rotation matrix
  R_roll = [1,    0,         0;
            0, cos(phi), -sin(phi);
            0, sin(phi),  cos(phi)];
        
  R_pitch = [ cos(theta),  0,   sin(theta);
                  0,       1,       0;
             -sin(theta),  0,   cos(theta)];
         
  R_yaw = [cos(psi), -sin(psi),  0;
           sin(psi),  cos(psi),  0;
              0,        0,       1];
          
  R = R_roll' * R_pitch' * R_yaw';

  % rotate vertices
  B = size(x);
  
  for i = 1 : B(2) * B(1)
      
      pts = [x(i), y(i), z(i)]*R;

      x_out(i) = pts(:,1);
      y_out(i) = pts(:,2);
      z_out(i) = pts(:,3);
      
  end
  
end