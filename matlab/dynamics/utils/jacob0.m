function J = jacob0(S,q) 
% jacob0 Calculates the Jacobian of the given screw matrix
%
% Inputs: S - Screw matrix
%         q - pose
%		  
% Output: J - Jacobian
	[~,n] = size(q);
    J = [];
    T = eye(4,4);
    for i = [1:1:n]
        T = T *(twist2ht(S(:,i),q(i)));
        Vtrans = adjoint(S(:,i),T);
        J = [J Vtrans];
    end
end