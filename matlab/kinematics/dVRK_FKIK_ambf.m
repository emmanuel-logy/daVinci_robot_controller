%% RBE501 Mid Report

close all
clear all
clc

addpath('utils');


%% PSMFK.py model

l1 = 4.389;
l2 = 4.16;
l3 = 0.09;
l4 = 0;
% l4 = 0.106

robot_psmfk = SerialLink([RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2, 'offset', pi/2), ...             %q(1)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...           %q(2)
                    PrismaticMDH('a', 0, 'theta', 0, 'alpha', pi/2, 'offset', -l1), ...     %q(3)
                    RevoluteMDH('a', 0, 'd', l2, 'alpha', 0, 'offset', 0), ...               %q(4)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...           %q(5)
                    RevoluteMDH('a', l3, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2)], ...       %q(6)
                    'name', '6 DoF dVRK');

qlim_PSMFKpy = [-pi/2  pi/2;  % q(1)
        -pi/3  pi/3;  % q(2)
         0.0 2.4;     % q(3)
        -pi/2  pi/2;  % q(4)
      -2*pi/5  2*pi/5;  % q(5)
        -pi/2  pi/2];  % q(6)

robot_psmfk.qlim = qlim_PSMFKpy;

close(figure(1))
figure(1)
q = zeros(1,6);
robot_psmfk.teach(q);
hold on

M = [-1,0,0,0;...
    0,0,-1,0;...
    0,-1,0,l1-l2-l3-l4;...
    0,0,0,1]

S = [0,-1,0,0,0,0;... % q(1)
    -1,0,0,0,0,0;... % q(2)
    0,0,0,0,0,-1;... % q(3)
    0,0,-1,0,0,0;... % q(4)
    -1,0,0,0,l2-l1,0;... % q(5)
    0,-1,0,l1-l2-l3,0,0]'; % q(6)

S_body = adjoint(S,inv(M));

%% FK test

fkOn = true;
% Plot option
plotOn = false;%true; %false; %% If you want to plot, please change this option to true.

%%% Evaluate the forward kinematics.
%%%%
% Define the number of tests:
nTests = 100;
%
% Test start
if fkOn
    fprintf('---------------------Forward Kinematics Test---------------------\n');
    fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
    fprintf('Progress: ');
    nbytes = fprintf('0%%');
    
    % Test the forward kinematics for 100 random sets of joint variables
    tic
    for ii = 1 : nTests
        fprintf(repmat('\b',1,nbytes));
        nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
        
        % Generate a random configuration
        qlim = robot_psmfk.qlim;
        q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
            qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
            qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(),...
            qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
            qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
            qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
        
        % Calculate the forward kinematics
        T = fkine(S,M,q);%...
        
        if plotOn
            figure(1)
            robot_psmfk.teach(q);
            title('Forward Kinematics Test');
        end
        
        assert(all(all(abs(double(robot_psmfk.fkine(q)) - T) < 1e-10)));
    end
    total_t = toc;
% test finish
    fprintf('\nTest passed successfully.\n');
    fprintf(['Total runtime is: ' num2str(total_t) ' s.\n']);
end

%% Jacobian test

% %%% Set the test options
% %%%%
% % IK test option
jacobOn = true;
% Plot option
plotOn = false; %true; %false; %% If you want to plot, please change this option to true.

%%% Evaluate the forward kinematics.
%%%%
% Define the number of tests:
nTests = 100;
% test start
if jacobOn
    
    fprintf('-------------------Space Jacobian Test------------------\n');
    fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
    fprintf('Progress: ');
    nbytes = fprintf('0%%');
    
    % Test the correctness of the Jacobian for 100 random sets of joint
    % variables
    tic
    for ii = 1 : nTests
        fprintf(repmat('\b',1,nbytes));
        nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
        
        % Generate a random configuration
        qlim = robot_psmfk.qlim;
        q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
            qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
            qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(),...
            qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
            qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
            qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
        
        % Calculate the Forward Kinematics
        T = fkine(S,M,q);
        
        % Calculate the Jacobian
        J = jacob0(S,q); %...
        
        if plotOn
            figure(1)
            robot_psmfk.teach(q);
            title('Space Jacobian Test');
        end
        
        % Test the correctness of the Jacobian
        Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
        assert(all(all(abs(double(robot_psmfk.jacob0(q)) - Jcoords) < 1e-10)));
    end
    total_t = toc;
% test finished
    fprintf('\nTest passed successfully.\n');
    fprintf(['Total runtime is: ' num2str(total_t) ' s.\n']);
end

%% IK test
NPts = 500;
t = linspace(0, 2*pi, NPts);
% x = 0.5 * cos(t) + 0.8 ;
% y = 0.5 * sin(t) + 0.8 ;
% z = -0.8 * ones(1, NPts);
x = 0.2 * ones(1, NPts);
y = linspace(-0.1,0.5, NPts);
z = -0.8 * ones(1, NPts) ;
path = [x; y; z];
csvwrite('Path.csv',path')

figure(1)
set(gcf,'Position',[50 50 800 600])
scatter3(path(1,:), path(2,:), path(3,:), 'filled');
title('home configuration')
 % each column of this matrix is a target pose represented by a twist
targetQ = zeros(6,NPts);

% Set the current joint variables
currentQ = zeros(1,6);
fprintf('Calculating the Inverse Kinematics... ');
robot_psmfk.plot(currentQ);
hold on
scatter3(path(1,:), path(2,:), path(3,:), 'filled');

% Iterate over the target points
for ii = 1 : NPts
    % Select the next target point
    targetPose = path(:,ii);
    T = fkine_new(S_body,M,currentQ,'body');
    currentPose = T(1:3,4);
    
    while norm(targetPose - currentPose) > 1e-3
        J_a = jacoba(S,M,currentQ);
        
        % Use the Levenberg-Marquadt algorithm (Damped Least Squares)
%         deltaQ = pinv(J_a)*(targetPose - currentPose);
%         lambda = 0.5;
%         deltaQ = J_a' * pinv(J_a*J_a' + lambda^2 * eye(3)) * (targetPose - currentPose);
        alpha = 0.1;
        deltaQ = alpha * J_a'* (targetPose - currentPose);

                    
        currentQ = currentQ + deltaQ';       
        T = fkine_new(S_body,M,currentQ,'body');
        currentPose = T(1:3,4);
    end
    targetQ(:,ii) = currentQ;
end

figure(3)
set(gcf,'Position',[900 50 800 600])
title('plot crown')
robot_psmfk.plot(targetQ(:,1:10:end)', 'trail', {'r', 'LineWidth', 5});

csvwrite('JointPos.csv',targetQ)


