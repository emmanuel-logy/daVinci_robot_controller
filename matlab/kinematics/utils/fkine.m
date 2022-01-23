function T = fkine(S,M,q)
% FKINE   Calculate the forward kinematics of robots using the product of
% exponentials formula.
% 
% T = fkine(S,M,q) calculate the forward kinematics given the screw axis S,
% home configuration M and joint variables q.
% 
% Inputs: S - screw axes for the robot; 6xn matrix, where n is the number
%         of joints.
%         M - home configurate; 4x4 matrix homogenous transformation matrix
%         from the base frame to the end effector frame.
%         q - joint variables; nx1 vector, where n is the number of joints.
%
% Output: T - forward kinematics of the robot, 4x4 matrix homogenous
%             transformation matrix
%
% See also TWIST2HT.

T = eye(4);
N_joint = size(S,2);
for i = 1:N_joint
    T_i = twist2ht(S(:,i),q(i));
    T = T*T_i;
end
T = T*M;
end