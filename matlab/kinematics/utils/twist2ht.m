function T = twist2ht(S,theta)
% TWIST2HT Calculate the 4x4 homogenours transformation matrix from given
% screw axes and rotation angles.
%
% T = twist2ht(S,theta) Calculate the 4x4 homogenours transformation matrix
% from given screw axis S and joint angle theta.
% 
% Inputs: S - screw axis, 6x1 vector.
%         theta - joint angle.
% 
% Output: T - a 4x4 homogenours transformation matrix.
% 
% ** also include the situation for prismatic joints.
%
% See also SKEW, AXISANGLE2ROT. 

omega = S(1:3);
omega_ss = skew(omega); 
v = S(4:6);
if (norm(omega)<1e-8)
    T = [eye(3),v*theta;0,0,0,1];
    % fprintf('Prismatic Joint!\n')
else
    R = axisangle2rot(omega,theta);
    v_bucket = (eye(3)*theta + (1 - cos(theta))*omega_ss + (theta-sin(theta))*omega_ss^2)*v;
    T = [R,v_bucket;0,0,0,1];
end
end