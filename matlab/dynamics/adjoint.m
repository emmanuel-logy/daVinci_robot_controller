function Vtrans = adjoint(V,T)
% adjoint Calculates the adjoint transformation
%
% Inputs: V - Screw vector for the joint
%         T - Twist in homogeneous form 
%		  
% Output: Vtrans - Adjoint transformation matrix

    R = T(1:3,1:3);
    d = T(1:3,4);
    p = [0 -d(3) d(2); d(3) 0 -d(1); -d(2) d(1) 0]; %skew of D
    zero = zeros(3,3);
    Vtrans = [R zero; p*R R]*V;
end
