clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deg2rad = pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Camera Intrinsic Matrix
fx = 800; % focal length (mm) / pixel_width_x (mm) = pixels 
fy = 800; % focal length (mm) / pixel_height_y (mm) = pixels
uo = 512; % image center x-coordinate
vo = 512; % image center y-coordinate

K = [ fx,  0, uo;
       0, fy, vo;
       0,  0,  1 ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Current Camera location at A wrt to the fixed reference frame at R
u_AR = [ 1; 0; 0];         % u (orientation axis)
theta_AR = -110*deg2rad;   % theta (orientation angle)
t_AR = [ 0.0; 0.0; 0.9 ];  % position

dq_AR = uthetat2dq( u_AR, theta_AR, t_AR ); % Dual quaternion pose
[ u_AR, theta_AR, R_AR, t_AR ] = dualq2uthetaRt( dq_AR );

X_AR = [ R_AR, t_AR; 0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired Camera location at B wrt to the fixed reference frame at R
u_BR = [ 1; 0; 0];         % u (orientation axis)
theta_BR = -90*deg2rad;    % theta (orientation angle)
t_BR = [ 0.3; 0.2; 0.8 ]; % position

dq_BR = uthetat2dq( u_BR, theta_BR, t_BR ); % Dual quaternion pose
[ u_BR, theta_BR, R_BR, t_BR ] = dualq2uthetaRt( dq_BR );

X_BR = [ R_BR, t_BR; 0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Pattern location at C wrt to the fixed reference frame at R
u_CR = [ 1; 0; 0];        % u (orientation axis)
theta_CR = 90*deg2rad;    % theta (orientation angle)
t_CR = [ 0.2; 0.9; 0.7 ]; % position

dq_CR = uthetat2dq( u_CR, theta_CR, t_CR ); % Dual quaternion pose 
[ u_CR, theta_CR, R_CR, t_CR ] = dualq2uthetaRt( dq_CR );

X_CR = [ R_CR, t_CR; 0 0 0 1];

pattern_points_CR = put_pattern( X_CR ); % 3D pattern points

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired Image of the Pattern from location B ?
X_RB=inv(X_BR);
image_pattern_B=take_image(X_RB,K,pattern_points_CR);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired image point features vector s_B from location B in metric units ?
image_pattern_B_met=pattern_pix2metric(image_pattern_B,fx,fy,uo,vo);
s_B=image_pattern_B_met(:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure; 
traces=[];
time=0;
tf=3;
Hz=10;
dt=1/10;
%% LOOP SHOULD START FROM HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
while (time<tf)
  

%% Take an image from location A ?
X_RA=inv(X_AR);
image_pattern_A=take_image(X_RA,K,pattern_points_CR);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Build the point features vector s_A in metric units ?
image_pattern_A_met=pattern_pix2metric(image_pattern_A,fx,fy,uo,vo);
s_A=image_pattern_A_met(:);
%% Calculate interaction matrix (Image Jacobian) ?
Z=0.8;
L=image_jacobian_4points(s_A,Z);
%% Error ?
e=s_A-s_B
%% Compute Control Law ?
k=2;
V_AA=-k*pinv(L)*e;

%% Convert Control Law ?
[u_AR,theta_AR,R_AR,t_AR]=dualq2uthetaRt(dq_AR);
T_AR=[R_AR,skew(t_AR)*R_AR;
zeros(3,3),R_AR];
V_AR=T_AR*V_AA;

%% Move Camera ?
v=V_AR(1:3);
w=V_AR(4:6);
theta=norm(w);
if (theta==0) u=[0;0;1] ; else u=w/norm(w); end

dq_update=uthetat2dq(u,dt*theta,dt*v);
dq_AR = muldualpq(dq_update,dq_AR ); 
[ u_AR, theta_AR, R_AR, t_AR ] = dualq2uthetaRt( dq_AR );
X_AR = [ R_AR, t_AR; 0 0 0 1];


%% Plot the 3D scene wrt to the reference frame at R 
time=time+dt;
traces=[traces, t_AR];
hold on; clf;
plot3( traces(1,:), traces(2,:), traces(3,:), 'k' );
subplot(1,2,1); hold on; 
plot_pattern( pattern_points_CR );
plot_camera( X_AR, 'b' );
plot_camera( X_BR, 'r' );
axis([-1.2 1.2 -1.2 1.2 0 1.4]);
view( 50, 40 ); 

%% Plot the image of the camera from locations A and B  
subplot(1,2,2); hold on; box on;
p1 = image_pattern_B(:,1);
p2 = image_pattern_B(:,2);
p3 = image_pattern_B(:,3);
p4 = image_pattern_B(:,4);


plot( p1(1),  p1(2),  'ro',  'MarkerSize', 8, 'MarkerFaceColor', 'r' );
plot( p2(1),  p2(2),  'go',  'MarkerSize', 8, 'MarkerFaceColor', 'g' );
plot( p3(1),  p3(2),  'bo',  'MarkerSize', 8, 'MarkerFaceColor', 'b' );
plot( p4(1),  p4(2),  'ko',  'MarkerSize', 8, 'MarkerFaceColor', 'k' );
p1A = image_pattern_A(:,1);
p2A = image_pattern_A(:,2);
p3A = image_pattern_A(:,3);
p4A = image_pattern_A(:,4);


plot( p1A(1),  p1A(2),  'ro',  'MarkerSize', 8, 'MarkerFaceColor', 'r' );
plot( p2A(1),  p2A(2),  'go',  'MarkerSize', 8, 'MarkerFaceColor', 'g' );
plot( p3A(1),  p3A(2),  'bo',  'MarkerSize', 8, 'MarkerFaceColor', 'b' );
plot( p4A(1),  p4A(2),  'ko',  'MarkerSize', 8, 'MarkerFaceColor', 'k' );
axis ij; axis image; xlabel('i'); ylabel('j');
axis([ 0 1024 0 1024 ]);
title('Camera View');
drawnow;
endwhile