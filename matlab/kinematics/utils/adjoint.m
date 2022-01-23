function twist_inB = adjoint(twist_inA,T_AB)
% ADJOINT Calculate the adjoint transformation of 4x4 homogenous
% transformation matrix and implement on twists.
% 
% twist_B = adjoint(twist_A, T) calculate twist_B = Ad_T(twist_A).
% 
% Inputs: twist_A - screw axis of A; 6x1 vector.
%         T - 4x4 homogenous transformation matrix from A to B.
%
% Output: twist_B - screw axis of B; 6x1 vector.
% 
% See also SKEW

R = T_AB(1:3,1:3);
p = T_AB(1:3,4);
p_ss = skew(p);
Adj_mtx = [R,zeros(3,3);p_ss*R,R];
twist_inB = Adj_mtx*twist_inA;

end

