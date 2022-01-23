function R = axisangle2rot(omega,theta)
% AXISANGLE2ROT Calculates the 3x3 rotation matrix corresponding to a
% rotation theta about an axis defined by omega.
%
% This function effectively implents Rodrigues' formula.
%
% Inputs: omega - 3D vector representing the axis of rotation
%         theta - scalar representing the rotation angle
%
% Output: R - rotation matrix (SO(3))
%
% See also SKEW

    omega_ss = skew(omega); 
    R = eye(3) + sin(theta) * omega_ss + (1 - cos(theta)) * omega_ss^2;
end