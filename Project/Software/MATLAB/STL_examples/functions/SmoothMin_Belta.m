function x_min = SmoothMin_Belta(vec_x, C) 
% The "SmoothMin_Belta" MATLAB function computes the minimum smooth value
% assumed by the input vector "vec_x" by using the constant value C. The 
% function implements the smooth approximation of min function described in
% [1, eq.(3)]
%
% [1] N. Mehdipour, C. -I. Vasile and C. Belta, "Specifying User Preferences 
% Using Weighted Signal Temporal Logic," in IEEE Control Systems Letters, 
% vol. 5, no. 6, pp. 2006-2011, Dec. 2021, doi: 10.1109/LCSYS.2020.3047362.
%
% Inputs:
% - vec_x, a generic vector \in \mathbb{R}
% - C, a constant value
% 
% Output:
% - x_min, minimum smooth valued assumed by the vector vec_x \in \mathbb{R}

x_min = (-1/C)*log(sum(exp(-C*vec_x)));

end