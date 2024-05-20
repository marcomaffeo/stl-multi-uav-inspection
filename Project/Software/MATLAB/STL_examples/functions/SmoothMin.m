function x_min = SmoothMin(vec_x, C) 
% The "SmoothMin" MATLAB function computes the minimum smooth value assumed
% by the input vector "vec_x" by using the constant value C. Larger is C, 
% highest is the probability that MATLAB approximate log(sum(exp(C * vec_x))) 
% with Inf
%
% Inputs:
% - vec_x, a generic vector \in \mathbb{R}
% - C, a constant value
% 
% Output:
% - x_min, minimum smooth valued assumed by the vector vec_x \in \mathbb{R}

x_min = (-1/C)*log(sum(exp(-C*vec_x)));

end