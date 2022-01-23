function J_a = jacoba(S,M,q)
% jacoba Calculates the analytical Jacobian of the given screw matrix
%
% Inputs: S - Screw matrix
%         q - pose
%         M - home matrix
%		  
% Output: J_a - analytical Jacobian

    J_b = jacobe(S,M,q);
    [~,n] = size(q);
    if n == 1
        [n,~] = size(q);
    end
    T = eye(4,4);
    for i = [1:1:n]
        T = T *(twist2ht(S(:,i),q(i)));
    end
    T = T * M; 
    R = T(1:3,1:3); % Extracting the Rotation Matrix
    J_a = [];
    for i = 1:1:size(J_b,2)
        J_b(4:6,i); % Taking the last three elements
        J_a = [J_a R*J_b(4:6,i)];
    end
end