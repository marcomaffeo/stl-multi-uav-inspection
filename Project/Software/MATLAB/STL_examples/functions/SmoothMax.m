function x_max = SmoothMax(vec_x, C) 
% The "SmoothMax" MATLAB function computes the maximum smooth value assumed
% by the input vector "vec_x" by using the constant value C. Larger is C, 
% highest is the probability that MATLAB approximate log(sum(exp(C * vec_x))) 
% with Inf
%
% Inputs:
% - vec_x, a generic vector \in \mathbb{R}
% - C, a constant value
% 
% Output:
% - x_max, maximum smooth valued assumed by the vector vec_x \in \mathbb{R}

x_max = (1/C)*log(sum(exp(C*vec_x)));

end