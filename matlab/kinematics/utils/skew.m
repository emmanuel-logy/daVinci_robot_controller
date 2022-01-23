function S = skew(v)
% SKEW  Returns the skew-symmetric matrix associated with the 3D vector.
% 
% [v] = skew(v) , where [v] is the bucket matrix of vector v.
% 
% Input: v - a 3x1 vector.
%
% Output: [v] - a 3x3 skew-symmetric matrix.

if length(v) == 3
    S = [  0   -v(3)  v(2)
        v(3)  0    -v(1)
        -v(2) v(1)   0];
else
    error('argument must be a 3-vector');
end

end

