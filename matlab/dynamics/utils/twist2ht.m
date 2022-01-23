function T = twist2ht(S,theta)
% twist2ht Calculates the homogeneous transform given screw axis representation
%
% Inputs: S - Screw axis matrix
%         theta - angle
%		  
% Output: T - Homogeneous transformation matrix
    omega = S(1:3);
    v = S(4:6);
    R = axisangle2rot(omega,theta);
    V = translation(omega,v,theta);
    T = [R V; 0 0 0 1];
end

function V = translation(omega,v,theta)
    omega_ss = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
    V = (eye(3)*theta + (1-cos(theta))*omega_ss + (theta-sin(theta))*omega_ss^2)*v;
end