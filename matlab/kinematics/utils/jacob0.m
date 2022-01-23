function J = jacob0(S,q)
% JACOB0 Calculate the space Jacobian matrix of the robot.
% 
% J = jacob0(S,q) calculate the space Jacobian matrix the robot with screw
% axes S and joint variables q.
% 
% Inputs: S - screw axes of the robot; 6xn matrix, where n is the number of
%             joints.
%         q - joint variables of the robot; nx1 vector, where n is the
%             number of joints.
% 
% Output: J - the space Jacobian matrix of the robot; 6xn matrix, where n
%             is the number of joints.
% 
% See also TWIST2HT, ADJOINT.

N_joint = size(S,2);
J = [];
if N_joint == 1
    J = [J,S];
else
    T = eye(4);
    J = [J,S(:,1)];
    for i = 2:N_joint
        T_i = twist2ht(S(:,i-1),q(i-1));
        T = T*T_i;
        J_i = adjoint(S(:,i),T);
        J = [J,J_i];
    end
end
end