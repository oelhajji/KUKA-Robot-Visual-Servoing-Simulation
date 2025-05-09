clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deg2rad = pi/180;

%% Kuka Initial Configuration
q = [ 50*deg2rad; 45*deg2rad; 30*deg2rad; 20*deg2rad; 10*deg2rad; 25*deg2rad; 15*deg2rad ];
q_init = q;

%% Camera Intrinsic Matrix
fx = 800; fy = 800; uo = 512; vo = 512; % Camera parameters
K = [ fx,  0, uo; 0, fy, vo; 0,  0,  1 ];

%% Desired End-Effector Pose for the Robot
u_BR = [ 1; 0; 0]; theta_BR = -100 * deg2rad; t_BR = [ 0.2; 0.2; 0.8 ]; % Desired pose
dq_BR = uthetat2dq( u_BR, theta_BR, t_BR ); % Convert to dual quaternion
[ u_BR, theta_BR, R_BR, t_BR ] = dualq2uthetaRt( dq_BR );
X_BR = [ R_BR, t_BR ; 0 0 0 1]; % Cartesian pose

%% Pattern in Space
u_CR = [ 1; 0; 0]; theta_CR = 90 * deg2rad; t_CR = [ 0.2; 0.9; 0.7 ];
dq_CR = uthetat2dq( u_CR, theta_CR, t_CR );
[ u_CR, theta_CR, R_CR, t_CR ] = dualq2uthetaRt( dq_CR );
X_CR = [ R_CR, t_CR; 0 0 0 1];
pattern_points_CR = put_pattern( X_CR ); % 3D pattern points

%% Desired Image of the Pattern
image_pattern_B = take_image(inv(X_BR), K, pattern_points_CR); % Image from desired pose
image_pattern_B_met = pattern_pix2metric(image_pattern_B, fx, fy, uo, vo);
s_B = image_pattern_B_met(:);

%% Initialize Variables
time = 0; tf = 10; dt = 0.1; % Simulation time
error_log = []; time_log = []; % Logging
traces = []; % To store camera trajectory

figure;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main Loop
while (time < tf)
    %% Forward Kinematics
    dq_AR = fkm(q); % End-effector pose in dual quaternion
    [u_AR, theta_AR, R_AR, t_AR] = dualq2uthetaRt(dq_AR);
    X_AR = [R_AR, t_AR; 0 0 0 1]; % Current Cartesian pose of end-effector

    %% Camera Pose
    % Camera is fixed to the end-effector, so X_AR is also the camera pose
    image_pattern_A = take_image(inv(X_AR), K, pattern_points_CR); % Image from current pose
    image_pattern_A_met = pattern_pix2metric(image_pattern_A, fx, fy, uo, vo);
    s_A = image_pattern_A_met(:);
    
    %% Error and Interaction Matrix
    e = s_A - s_B; % Image-based error
    Z = 0.8; % Depth approximation
    L = image_jacobian_4points(s_A, Z); % Interaction matrix

    %% Control Law
    k = 2;
    V_AA = -k * pinv(L) * e; % Visual servoing control law in camera frame
    T_AR = [R_AR, skew(t_AR) * R_AR; zeros(3,3), R_AR]; % Transformation to world frame
    V_AR = T_AR * V_AA;

    %% Convert Control Law to Robot Joint Space
    q_dot = jacobian(q)' * V_AR; % Compute joint velocities
    q = q_dot * dt + q; % Update joint configuration
    
    %% Log Error and Time
    error_log = [error_log, norm(e)];
    time_log = [time_log, time];
    time = time + dt;

    %% Plot 3D Scene
    traces = [traces, t_AR]; % Camera trajectory
    hold on; clf;

    subplot(1, 2, 1); hold on;
    plot_kuka_robot(q_init, 0); % Initial robot pose
    plot_kuka_robot(q, 0); % Current robot pose
    plot_pattern(pattern_points_CR);
    plot_camera(X_AR, 'b'); % Camera fixed to end-effector
    plot_camera(X_BR, 'r'); % Desired end-effector pose
    axis([-1.2 1.2 -1.2 1.2 0 1.4]);
    view(50, 40);

    %% Plot Image
    subplot(1, 2, 2); hold on; box on;
    p1 = image_pattern_B(:, 1); p2 = image_pattern_B(:, 2);
    p3 = image_pattern_B(:, 3); p4 = image_pattern_B(:, 4);
    plot(p1(1), p1(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot(p2(1), p2(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(p3(1), p3(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    plot(p4(1), p4(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    p1A = image_pattern_A(:, 1); p2A = image_pattern_A(:, 2);
    p3A = image_pattern_A(:, 3); p4A = image_pattern_A(:, 4);
    plot(p1A(1), p1A(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    plot(p2A(1), p2A(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot(p3A(1), p3A(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    plot(p4A(1), p4A(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    axis ij; axis image; xlabel('i'); ylabel('j');
    axis([0 1024 0 1024]);
    title('Camera View');
    drawnow;
end

%% Plot Error Over Time
figure;
plot(time_log, error_log, 'LineWidth', 2);
xlabel('Time (seconds)');
ylabel('Error Norm');
title('Error vs Time');
grid on;
