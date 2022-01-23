function V_b = twistspace2body(V_s,T)
% twistspace2body calculates the body twist given twist in the space frame.
%
% Inputs: V_s - Spatial velocity expressed in space frame.
%         T - Homogeneous tranformation matrix from the space frame to body frame
%
% Output: V_b - Twist expressed in the body frame

    R = T(1:3,1:3); % extract rotation matrix
    d = T(1:3,4);	% extract displacement vector
    p = [0 -d(3) d(2); d(3) 0 -d(1); -d(2) d(1) 0]; % d skew
    zero = zeros(3,3);
	
    V_b = [inv(R) zero; -inv(R)*p inv(R)]*V_s;
    
end