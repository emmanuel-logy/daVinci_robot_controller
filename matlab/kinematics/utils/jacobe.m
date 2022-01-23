function J_b = jacobe(S,M,q)
% jacoba Calculates the analytical Jacobian of the given screw matrix
%
% Inputs: S - Screw matrix
%         q - pose
%         M - home matrix
%		  
% Output: J_b - Jacobian in body frame
    [~,n] = size(q);
    if n == 1
        [n,~] = size(q);
    end
    J_s = [];
    J_v = [];
    T = eye(4,4);
    for i = [1:1:n]
        T = T * (twist2ht(S(:,i),q(i)));
        Vtrans = adjoint(S(:,i),T);
        J_s = [J_s Vtrans];
    end
    J_b = twistspace2body(J_s, T * M);
end
