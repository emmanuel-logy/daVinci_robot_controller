function omega_b = angvelocityspace2body(omega_s,R)
% angvelocityspace2body calculates the angular velocity in the body frame
%given the angular velocity in the space frame.
%
% Inputs: omega_s - The angular velocity expressed in space frame.
%         R - Rotation matrix
%
% Output: omega_b - angular velocity in the body frame
    omega_b = R'*omega_s
end