function T = fkine(S,M,q,frame)
% fkine Calculates the forward kinematics of a system
%
%
% Inputs: S - Screw axis matrix
%         M - home position
%		  q - current pose
%         frame - body or space frame
% Output: T - Forward kinematic matrix (4x4)

    T = eye(4,4);
	[~,numCols] = size(S);
    for i = [1:1:numCols]
        T = T * (twist2ht(S(:,i),q(i)));
    end
    if frame == "space"
        T = T*M;
    else
        T = M*T;
    end
end