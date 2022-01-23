%% RBE501 Mid Report

close all
clear all
clc

addpath('utils');


%% WPI model

L2l1 = 0.096;
L2l2 = 0.516;
L2l3 = 0.04009;
L2h1 = 0.14454;
L3 = 0.04009;
LRCC = 0.4318;
Ltool = 0.4162;
Lpitch2yaw = 0.0091;

%% WPI simplified model

robot_wpisimplified = SerialLink([RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2, 'offset', pi/2), ...             %q(1)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...           %q(2)
                    PrismaticMDH('a', 0, 'theta', 0, 'alpha', pi/2, 'offset', -LRCC), ...     %q(3)
                    RevoluteMDH('a', 0, 'd', Ltool, 'alpha', 0, 'offset', 0), ...               %q(4)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...           %q(5)
                    RevoluteMDH('a', Lpitch2yaw, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...       %q(6)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', 0, 'offset', pi)], ...      %q(7)
                    'name', '6 DoF dVRK');

qlim_wpisimplified = [-pi/3  pi/3;  % q(1)
        -pi/3  pi/3;  % q(2)
          0     0.24;    % q(3)
         -pi    pi;  % q(4)
        -pi/2  pi/2;  % q(5)
        -pi/3  pi/3;  % q(6)
           0     0];    % q(7)
       
robot_wpisimplified.qlim = qlim_wpisimplified;
close(figure(1))
figure(1)
q = zeros(1,7);
% q(2) = pi/3
% q(2) = -pi/3;
robot_wpisimplified.teach(q);
hold on
% robot_wpisimplified.teach(q,'zoom',0.4);

M = [1,0,0,0;...
    0,0,-1,0;...
    0,1,0,LRCC-Ltool-Lpitch2yaw;...
    0,0,0,1]

S = [0,-1,0,0,0,0;... % q(1)
    -1,0,0,0,0,0;... % q(2)
    0,0,0,0,0,-1;... % q(3)
    0,0,-1,0,0,0;... % q(4)
    -1,0,0,0,Ltool-LRCC,0;... % q(5)
    0,-1,0,LRCC-Ltool-Lpitch2yaw,0,0;... % q(6)
    0,-1,0,LRCC-Ltool-Lpitch2yaw,0,0]' % q(7)

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
        qlim = robot_wpisimplified.qlim;
        q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
            qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
            qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(),...
            qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
            qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
            qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand(),...
            qlim(7,1) + (qlim(7,2) - qlim(7,1)) * rand()];
        
        % Calculate the forward kinematics
        T = fkine(S,M,q);%...
        
        if plotOn
            figure(1)
            robot_wpisimplified.teach(q);
            title('Forward Kinematics Test');
        end
        
        assert(all(all(abs(double(robot_wpisimplified.fkine(q)) - T) < 1e-10)));
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
        qlim = robot_wpisimplified.qlim;
        q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
            qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
            qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(),...
            qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
            qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
            qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand(),...
            qlim(7,1) + (qlim(7,2) - qlim(7,1)) * rand()];
        
        % Calculate the Forward Kinematics
        T = fkine(S,M,q);
        
        % Calculate the Jacobian
        J = jacob0(S,q); %...
        
        if plotOn
            figure(1)
            robot_wpisimplified.teach(q);
            title('Space Jacobian Test');
        end
        
        % Test the correctness of the Jacobian
        Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
        assert(all(all(abs(double(robot_wpisimplified.jacob0(q)) - Jcoords) < 1e-10)));
    end
    total_t = toc;
% test finished
    fprintf('\nTest passed successfully.\n');
    fprintf(['Total runtime is: ' num2str(total_t) ' s.\n']);
end

%% IK test

%%%% create desired path --> crown!

NPts = 500;

t = linspace(-pi, pi, NPts);
% x = 0.05*cos(t);
% y = 0.05*sin(t);
% z = -0.05*ones(1,NPts);
x = -0.04*ones(1,NPts);
y = linspace(-0.01,0.06, NPts);
z = -0.08*ones(1,NPts);
path = [x; y; z];
csvwrite('Path_full.csv',path')

figure(1)
set(gcf,'Position',[50 50 800 600])
scatter3(path(1,:), path(2,:), path(3,:), 'filled');
title('home configuration')

targetPose = zeros(6,size(path,2)); % each column of this matrix is a target pose represented by a twist

for ii = 1 : size(path,2)
    % First calculate the homogeneous transformation matrix representing
    % each target pose
    R = [0 0 -1; 0 1 0; 1 0 0]';
    T = [R path(:,ii); 
         0 0 0 1];
     
    % Then perform the matrix logarithm operation to convert transformation
    % matrices into 4x4 elements of se(3)
    t = MatrixLog6(T);
    
    % Finally, "unpack" this matrix (i.e., perform the inverse of the
    % bracket operator)
    targetPose(:,ii) = [t(3,2) t(1,3) t(2,1) t(1:3,4)']';
end


% IK test option
ikOn = true;
% Plot option
plotOn = false;%true; %false; %% If you want to plot, please change this option to true.
% Figure option 
figOn = true;%true; %false; %% If you don't want to see the norm figures, please change this option to false.

%%% Set the number of test

nTests = size(targetPose,2);

qList = [];

%%% Evaluation of the test
if ikOn 
    fprintf('----------------------Inverse Kinematics Test--------------------\n');
    fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
    fprintf('Progress: ');
    nbytes = fprintf('0%%');

    % Calculate the twist representing the robot's home pose
    % current_Pose = MatrixLog6(M);
    % current_Pose = [current_Pose(3,2) current_Pose(1,3) current_Pose(2,1) current_Pose(1:3,4)']';
    
% Set the initial current joint variables
    currentQ = [0,0,0,0,0,0,0];
% Set up figure options    
    if figOn
        t = 0;
        figure(2)
        set(gcf,'Position',[50 700 1700 350])
        h1_axis = gca;
        h_r1 = animatedline(h1_axis,'Color','r','LineWidth',2);
        h_c1 = animatedline(h1_axis,'Color','b','LineWidth',2);
        xlabel('time(s)')
        ylabel('norm of pose difference')
        legend('target','current','Location','best')
        title('norm of pose differenc v.s. time [time step: 0.01s]')
        grid on
    end
% Test start
    tic
    for ii = 1 : nTests
        fprintf(repmat('\b',1,nbytes));
        nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
        
        
        % Calculate the twist representing the robot's current pose
        T = fkine(S,M,currentQ);
        current_Pose = MatrixLog6(T);
        current_Pose = [current_Pose(3,2) current_Pose(1,3) current_Pose(2,1) current_Pose(1:3,4)']';
        % disp(current_Pose)
        % Generate the robot's target pose
        target_Pose = targetPose(:,ii);
        
        
        
        % Inverse Kinematics
        while norm(target_Pose - current_Pose) > 1e-3
            J = jacob0(S,currentQ);
            
            %%%% pseudo-inveerse
            % deltaQ = J'*pinv(J*J')*(target_Pose - current_Pose);
            
            %%%% Damped least square
            % lambda = 7;
            % deltaQ = J'*pinv(J*J'+lambda^2*eye(6))*(target_Pose - current_Pose);
            
            if norm(target_Pose - current_Pose) > 0.1
                %%%% Jacobian transpose
                alpha = 0.01;
                deltaQ = alpha*J'*(target_Pose - current_Pose);
            else
                %%%% Newton Raphson
                deltaQ = pinv(J)*(target_Pose - current_Pose);
            end
            
            currentQ = currentQ + deltaQ';
            
            T = fkine(S,M,currentQ);
            current_Pose = MatrixLog6(T);
            current_Pose = [current_Pose(3,2) ...
                current_Pose(1,3) ...
                current_Pose(2,1) ...
                current_Pose(1:3,4)']';
            
            if figOn
                
                addpoints(h_c1,t,norm(target_Pose - current_Pose))
                addpoints(h_r1,t,1e-3)
                drawnow limitrate
                t = t + 0.01;
            end
            
            if plotOn
                try
                    figure(1)
                    robot_wpisimplified.teach(currentQ);
                    drawnow;
                catch e
                    continue;
                end
            end
        end
        
        if norm(target_Pose - current_Pose) <= 1e-3
            qList = [qList;currentQ];
        end
    end
% test finished
    total_t = toc;
    fprintf('\nTest passed successfully.\n');
    fprintf(['Total runtime is: ' num2str(total_t) ' s.\n']);
    
%%% Plot heart!
%     figure(3)
%     set(gcf,'Position',[900 50 800 600])
%     title('plot crown')
%     robot_wpisimplified.plot(qList, 'trail', {'r', 'LineWidth', 5});
end

csvwrite('JointPos_full.csv',qList');


%% WPI full model [for visualization onlys]


robot_wpi = SerialLink([RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2, 'offset', pi/2), ...             %q(1)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...           %q(2)
                    RevoluteMDH('a', L2l3, 'd', 0, 'alpha', 0, 'offset', pi/2), ...           %q(2')
                    RevoluteMDH('a', L2h1, 'd', 0, 'alpha', 0, 'offset', pi/2), ...           %q(2'')
                    RevoluteMDH('a', L2l2, 'd', 0, 'alpha', 0, 'offset', 0), ...           %q(2'''')
                    PrismaticMDH('a', L3, 'theta', 0, 'alpha', -pi/2, 'offset', -(LRCC-L2h1)), ...     %q(3)
                    RevoluteMDH('a', 0, 'd', Ltool, 'alpha', 0, 'offset', 0), ...               %q(4)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2, 'offset', pi/2), ...           %q(5)
                    RevoluteMDH('a', Lpitch2yaw, 'd', 0, 'alpha', -pi/2, 'offset', pi/2), ...       %q(6)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', 0, 'offset', 0)], ...      %q(7)
                    'name', '6 DoF dVRK');

qlim_wpi = [ -pi/3  pi/3;  % q(1)
        -pi/3  pi/3;  % q(2)
            0  0;       % q(2')
         -pi/3 pi/3;     % q(2'')
         -pi/3 pi/3;     % q(2'''')
           0    0.24;    % q(3)
         -pi   pi;  % q(4)
        -pi/2  pi/2;  % q(5)
        -pi/3  pi/3;  % q(6)
           0     0];    % q(7)


robot_wpi.qlim = qlim_wpi;

q = zeros(1,10);

%%% plot
qList_full = zeros(size(qList,1),10);

qList_full(:,1) = qList(:,1);
qList_full(:,2) = qList(:,2);
qList_full(:,4) = -qList(:,2);
qList_full(:,5) = qList(:,2);
qList_full(:,6) = qList(:,3);
qList_full(:,7) = qList(:,4);
qList_full(:,8) = qList(:,5);
qList_full(:,9) = qList(:,6);
close(figure(1))
close(figure(2))
close(figure(3))

% t = linspace(-pi, pi, 50);
% x = 0.1*sin(t);
% y = 0.1*cos(t)+L2l2;
% z = 0.05*sin(3*t) -0.1;
y = y + L2l2;
path = [x; y; z];


figure(4)
robot_wpi.teach(q)
set(gcf,'Position',[50 50 900 900])
hold on
scatter3(path(1,:), path(2,:), path(3,:), 'filled','MarkerFaceColor','#ffb300');
title('home configuration')

figure(5)
set(gcf,'Position',[1000 50 900 900])
title('plot crown')
robot_wpi.plot(qList_full(1:10:end,:), 'trail', {'LineWidth',5, 'Color','#FF6700'});
