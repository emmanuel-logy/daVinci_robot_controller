% Ritwik Pandey
clear, clc, close all
addpath('utils');

plotOn = false;
%% Robot Parametres

n = 6;
format short
l_RCC = 4.389;
l_tool = 4.16;
l_Pitch2Yaw = 0.09;
l_Yaw2CtrlPnt = 0.106;

q = zeros(1,n);
d3 = q(3);

m1 = 5;
m2 = 5;
m3 = 2;
m4 = 1;
m5 = 1;
m6 = 1;
m7 = 2;

g = [0 0 -9.81]; % Gravity Vector [m/s^2]

robot = SerialLink([RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2, 'offset', pi/2), ...             %q(1)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...           %q(2)
                    PrismaticMDH('a', 0, 'theta', 0, 'alpha', pi/2, 'offset', -l_RCC), ...      %q(3)
                    RevoluteMDH('a', 0, 'd', l_tool, 'alpha', 0, 'offset', 0), ...                %q(4)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...           %q(5)
                    RevoluteMDH('a', l_Pitch2Yaw, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2)], ... %q(6)
                    'name', '6 DoF dVRK');

qlim = [-pi/2  pi/2;  % q(1)
        -pi/3  pi/3;  % q(2)
         0.0 2.4;     % q(3)
        -pi/2  pi/2;  % q(4)
      -2*pi/5  2*pi/5;  % q(5)
        -pi/2  pi/2];  % q(6)
       
for i = 1:size(qlim,1)
    robot.links(i).qlim = qlim(i,:);
end

close(figure(1))
figure(1)
% Display the manipulator in the home configuration

robot.plot(q);

%% Part 1 Screw axes (space frame) and home configuration

% Calculating the space frame screw axes for the robot
w1 = [0 -1 0];
p1 = [0 0 0]';
v1 = -skew(w1)*p1;
S1 = [w1'; v1];

w2 = [-1 0 0];
p2 = [0 0 0]';
v2 = -skew(w2)*p2;
S2 = [w2'; v2];

%This is a prismatic Joint
w3 = [0 0 0];
v3 = [0 0 -1]';
S3 = [w3'; v3];

w4 = [0 0 -1];
p4 = [0 0 (l_RCC - d3 - l_tool)]';
v4 = -skew(w4)*p4;
S4 = [w4'; v4];

w5 = [-1 0 0];
p5 = [0 0 (l_RCC - d3 - l_tool)]';
v5 = -skew(w5)*p5;
S5 = [w5'; v5];

w6 = [0 -1 0];
p6 = [0 0 (l_RCC - d3 - l_tool - l_Pitch2Yaw)]';
v6 = -skew(w6)*p6;
S6 = [w6'; v6];

w7 = [0 0 -1];
p7 = [0 0 (l_RCC - d3 - l_tool - l_Pitch2Yaw-l_Yaw2CtrlPnt)]';
v7 = -skew(w7)*p7;
S7 = [w7'; v7];

S = [S1 S2 S3 S4 S5 S6 S7]

% Calculating the homogeneous transformation matrix M for the home configuration
R = [0 1 0; 1 0 0; 0 0 -1];
p = [0; 0; (l_RCC - d3 - l_tool - l_Pitch2Yaw - l_Yaw2CtrlPnt)];
M = [R p; zeros(1,3) 1]


% Link frames in the home configuration
M01 = [rotx(90) [0;0;0]; [0 0 0 1]]; % Pose of link {1} in frame {0}
M12 = [roty(-90) [0;0;0]; [0 0 0 1]]; % Pose of link {2} in frame {1}
M23 = [rotx(90) [0; (l_RCC/2); 0]; [ 0 0 0 1]]; % Pose of link {3} in frame {4}
M34 = [eye(3) [0; 0; (-l_RCC/2)+((l_tool)/2)]; [ 0 0 0 1]]; % Pose of link {4} in frame {3}
M45 = [rotx(-90)  [0;0;0]; [0 0 0 1]]; % Pose of link {5} in frame {4}
M56 = [roty(90) [0; -(((l_tool)/2) + (l_Pitch2Yaw)/2); 0]; [0 0 0 1]]; % Pose of link {6} in frame {5}
M67 = [[0 1 0; 0 0 -1; -1 0 0] [0; -((l_Pitch2Yaw/2)+l_Yaw2CtrlPnt); 0]; [0 0 0 1]]; % Pose of link {7} in frame {6}

M1 = M01;            % Pose of link {1} in frame {0}
M2 = M1 * M12;  % Pose of link {2} in frame {0}
M3 = M2 * M23;  % Pose of link {3} in frame {0}
M4 = M3 * M34;  % Pose of the body frame {4} in frame {0}
M5 = M4 * M45;  % Pose of the body frame {5} in frame {0}
M6 = M5 * M56;  % Pose of the body frame {6} in frame {0}
M7 = M6 * M67  % Pose of the body frame {7} in frame {0}

M = M01*M12*M23*M34*M45*M56*M67;

Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67);

q = zeros(1,n);
robot.teach(q);

%% Part 2 Spatial Inertia Matrices
I_b = zeros(3,3);
z = zeros(3,3);

% Rotational Inertia Matrices
Ib1 = I_b;%diag([0.24577 0.24577 0.015]);      % Rotational Inertia Matrix of Link 1
Ib2 = I_b;   % Rotational Inertia Matrix of Link 2
Ib3 = I_b;                                                 % Rotational Inertia Matrix of Link 3
Ib4 = diag([1.192 1.192 0.00089]);          % Rotational Inertia Matrix of Link 4
Ib5 = diag([0.00076 0.00278 0.00278]);  % Rotational Inertia Matrix of Link 5
Ib6 = diag([0.00024 0.00024 0.00024]);  % Rotational Inertia Matrix of Link 6
Ib7= diag([0.00087 0.00052 0.00082]);   % Rotational Inertia Matrix of Link 7


% Spatial Inertia Matrices
G1 = [Ib1 z; z m1*eye(3)]; % Spatial Inertia Matrix for link 1
G2 = [Ib2 z; z m2*eye(3)]; % Spatial Inertia Matrix for link 2
G3 = [Ib3 z; z m3*eye(3)]; % Spatial Inertia Matrix for link 3
G4 = [Ib4 z; z m4*eye(3)]; % Spatial Inertia Matrix for link 4
G5 = [Ib5 z; z m5*eye(3)]; % Spatial Inertia Matrix for link 5
G6 = [Ib6 z; z m6*eye(3)]; % Spatial Inertia Matrix for link 6
G7 = [Ib7 z; z m7*eye(3)]; % Spatial Inertia Matrix for link 7

Glist = cat(3, G1, G2, G3, G4, G5, G6, G7);


%% Forward Dynamics -- robot falls under gravity
fprintf('---------------Simulation of Robot Falling Under Gravity----------------\n');

q0 = zeros(n,1);      % initial joint variables
qd0 = zeros(n,1);     % initial joint velocities
taumat = zeros(40, n); % joint torques (no torque being applied)
Ftipmat = zeros(size(taumat, 1), 6); % end effector wrench (no force/moment being applied)
dt = 0.05;            % time step
intRes = 8;           % Euler integration step

% We will now use the `ForwardDynamicsTrajectory` to simulate the robot
% motion.
[qt,~] = ForwardDynamicsTrajectory(q0, qd0, taumat, g, ...
                                   Ftipmat, Mlist, Glist, S, dt, ...
                                   intRes);
                               
for i = 1:size(qt,1)
    if qt(i,3) > qlim(3,2)
        qt(i,3) = qlim(3,2);
    end
end

title('Robot Falling Under Gravity');                               
robot.plot(qt);

% input('Simulation complete. Press Enter to continue.');


%% Inverse Dynamics: Gravity Compensation
% We will now calculate the joint torques necessary for the robot to stay
% in place, i.e., not fall under its own weight.
fprintf('---------------------------Gravity Compensation-------------------------\n');

q0 = zeros(n,1);  % initial configuration 
qd0 = zeros(n,1); % initial velocities
% q0(1) = 0.2;
% q0(2) = 0.2;
% q0(3) = 1;
% q0(4) = 0.5;

% Invoke the `GravityForces` function. This function calculates the inverse
% dynamics of the robot when the robot is not moving and the only force
% acting on it is gravity.
grav = GravityForces(q0, g, Mlist, Glist, S);

fprintf('Joint Torques: ');
fprintf('[%f %f %f %f %f %f %f] Nm\n', grav(1), grav(2), grav(3), grav(4), grav(5), grav(6));

% Simulate for 5 seconds
tf = 5;           % total simulation time
dt = 0.05;        % time step
taumat = repmat(grav', [tf/dt 1]);   % apply only the torque required for gravity comp
Ftipmat = zeros(size(taumat, 1), 6); % end effector wrench
intRes = 8;       % Euler integration step

% Invoke the `ForwardDynamicsTrajectory` to simulate the robot motion.
[qt,~] = ForwardDynamicsTrajectory(q0, qd0, taumat, g, ...
                                   Ftipmat, Mlist, Glist, S, dt, ...
                                   intRes);

title('Gravity Compensation');
robot.plot(qt);

% 
% input('Simulation complete. Press Enter to continue.');


%% Dynamics III - Control the motion between set points
fprintf('----------------------Dynamic Control of a 3-DoF Arm--------------------\n');

% We will first generate a path in task space and solve the inverse
% kinematics for each of these points. 
n=7;
% Initialize the matrix to store the IK result
q = zeros(1,n);
nPts = 500;
targetQ = zeros(n,nPts);

% Set the current joint variables
currentQ = zeros(1,n);

% Calculate the Analytical Jacobian at the current configuration
J_a = jacoba(S, M, currentQ);

% Generate a path to follow
fprintf('Generating task space path... ');
t = linspace(0, 2*pi, nPts);
% x = 0.5 * cos(t) + 0.8 ;
% y = 0.5 * sin(t) + 0.8 ;
% z = -0.8 * ones(1, nPts);
x = 0.5 * ones(1, nPts);
y = 0.2 * t - 0.3;
z = -1.0 * ones(1, nPts) ;
path = [x; y; z];
fprintf('Done.\n');
% csvwrite('data/Path.csv',path');

fprintf('Calculating the Inverse Kinematics... ');
robot.plot(currentQ(1:6));
hold on
scatter3(path(1,:), path(2,:), path(3,:), 'filled');

% Iterate over the target points
for ii = 1 : nPts
    % Select the next target point
    targetPose = path(:,ii);
    T = fkine(S,M,currentQ,'space');
    currentPose = T(1:3,4);
    
    while norm(targetPose - currentPose) > 1e-3
        J_a = jacoba(S,M,currentQ);
        
        % Use the Levenberg-Marquadt algorithm (Damped Least Squares)
        lambda = 0.5;
        deltaQ = J_a' * pinv(J_a*J_a' + lambda^2 * eye(3)) * (targetPose - currentPose);
                    
        currentQ = currentQ + deltaQ';       
        T = fkine(S,M,currentQ,'space');
        currentPose = T(1:3,4);
    end
    currentQ_store(:,ii) = currentQ;
    targetQ(:,ii) = currentQ;
end

fprintf('Done.\n');

%printing data for export to AMBF
fprintf('\n\n%d Joint positions (7 joint angles for each position) \n', nPts)
% disp(currentQ_store);    % 20 Joint positions displayed (7 joint angles for each position)
% csvwrite('data/JointPos.csv',currentQ_store);

%% Dynamics

% Now, for each pair of consecutive set points, we will first calculate a
% trajectory between these two points, and then calculate the torque
% profile necessary to move from one point to the next.
fprintf('Generating the Joint Torque Profiles... ');
n=6;
% Initialize the arrays where we will accumulate the output of the robot
% dynamics, so that we can display it later
qtt = []; % Joint Variables
tau = [];

for jj = 1 : nPts - 1
    t0 = 0; tf = 2; % Starting and ending time of each trajectory
    N = 50;          % Number of intermediate setpoints
    t = linspace(t0, tf, N); % time vector
    
    q = zeros(n,N);   % joint variables
    qd = zeros(n,N);  % joint velocities
    qdd = zeros(n,N); % joint accelerations
    
    for ii = 1 : n
        % Calculate the coefficients of the quintic polynomial
        a = quinticpoly(t0, tf, ...
            targetQ(ii,jj), targetQ(ii,jj+1), ...
            0, 0, 0, 0);
        
        % Generate the joint profiles (position, velocity, and
        % acceleration)
        q(ii,:) = a(1) + a(2) * t + a(3) * t.^2 + a(4) * t.^3 + a(5) * t.^4 + a(6) * t.^5;
        qd(ii,:) = a(2) + 2*a(3)*t + 3*a(4)*t.^2 + 4*a(5)*t.^3 + 5*a(6)*t.^4;
        qdd(ii,:) = 2*a(3) + 6*a(4)*t + 12*a(5)*t.^2 + 20*a(6)*t.^3;
    end
    
    % Use the equations of motion to calculate the necessary torques to trace
    % the trajectory
    Ftipmat = zeros(N,6); % no end effector force
    % 1 kg payload added
	for i = 1 : N
        Ftipmat(i,:) = [0 0 0 -9.8*1 0 0];
    end
	taumat = InverseDynamicsTrajectory(q', qd', qdd', ...
        g, Ftipmat, Mlist, Glist, S);
    
    % Use the Forward Dynamics to simulate the robot behavior
    dt = tf/N;  % time step
    intRes = 1; % Euler integration constant
    [qt, qdt] = ForwardDynamicsTrajectory(q(:,1), qd(:,1), taumat, g, ...
        Ftipmat, Mlist, Glist, S, dt, ...
        intRes);
    
    qtt = [qtt; qt]; % Accumulate the results
    tau = [tau; taumat];
end

fprintf('Done.\n');

fprintf('Simulating the robot...');
title('Inverse Dynamics Control');
robot.plot(qtt(1:50:end,:));
fprintf('Done.\n');

% Display the Joint Torques
figure, hold on, grid on
plot((1:length(tau))*dt, tau(:,1), 'Linewidth', 2); 
plot((1:length(tau))*dt, tau(:,2), 'Linewidth', 2); 
plot((1:length(tau))*dt, tau(:,3), 'Linewidth', 2);
plot((1:length(tau))*dt, tau(:,4), 'Linewidth', 2);
plot((1:length(tau))*dt, tau(:,5), 'Linewidth', 2);
plot((1:length(tau))*dt, tau(:,6), 'Linewidth', 2);

xlim([0 max((1:length(tau))*dt)]);
xlabel('Time [s]'), ylabel('Torque [Nm]');
legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'});
set(gca, 'FontSize', 14);
% csvwrite('data/Tau.csv',taumat');
fprintf('Program completed successfully.\n');
