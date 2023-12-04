clear;
close all;
%% PID using LQR
[m,g,Ix,Iy,Iz,l,k,b]=get_model();
omega_eq = sqrt((m*g/4)/k);     % By solving for u when sdot = 0 = f(s,u)
omega1 = omega_eq;
omega2 = -omega_eq;
omega3 = omega_eq;
omega4 = -omega_eq;

u_nom = [omega1;omega2;omega3;omega4];
s_nom = [0;0;0;0;0;0;0;0;0;0;0;0];

[A,B]=eval_jacobian(s_nom,u_nom);
[nx,nu] = size(B);
ny = nx; C = eye(ny);

P_ol=ss(A,B,C,0);

disp("Uncontrollable states: ")
Ctrb = ctrb(A,B);
unco = length(A) - rank(Ctrb);
disp(unco)

%% LQI Control

% Augmenting the system with integrator states
Aaug = [A zeros(nx,ny);C zeros(ny,ny)];
Baug = [B; zeros(ny,nu)];

stabilizable = [A B;C zeros(ny,nu)];
expectedrank = nx + ny;
rankDeficiency = (expectedrank - rank(stabilizable));
disp("Unstabilizable integral control states")
disp(rankDeficiency)

% The system is not stabilizable, 
% now let's assume the output is
% y=[dx dy dz dpsi].T

% Augmenting the system with integrator states
C = [1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 0 0];
ny = 4;
Aaug = [A zeros(nx,ny);C zeros(ny,ny)];
Baug = [B; zeros(ny,nu)];

stabilizable = [A B;C zeros(ny,nu)];
expectedrank = nx + ny;
rankDeficiency = (expectedrank - rank(stabilizable));
disp("Unstabilizable integral control states")
disp(rankDeficiency)

% This system is stabilizable


%% LQI Weighing Matrices

Qx=diag([100 100 120 10 10 10 100 100 200 40 40 10]); % shape ny
QI=diag([0.1 0.1 0.1 0.1])*0.01; % shape ny
Qaug = blkdiag(Qx, QI);
R=diag([1/30 1/30 1/30 1/30])*0.1; % shape nu

[Kaug,S,P] = lqr(Aaug, Baug, Qaug, R);
Kx = Kaug(:,1:nx);
KI = Kaug(:,nx+1:nx+ny);

% Feedforward term
G = inv(C/(-A+B*Kx)*B);

disp("Closed-loop LQI poles")
disp(P)
disp("LQI Gain: ")
disp(Kaug)
disp("Precompensator Gain: ")
disp(G)

% Form loop for state-feedback
Lsf = ss(Aaug, Baug, Kaug,0);

% State-feedback gain/phase/S-based disk margins
AMsf = allmargin(Lsf);
DMsf = diskmargin(Lsf);

disp("Disk Margins (>0.4): ")
DMsf(1).DiskMargin
DMsf(2).DiskMargin
DMsf(3).DiskMargin
DMsf(4).DiskMargin

%% Closed Loop Transfer functions

Acl = Aaug-Baug*Kaug;
Bcl = [B*G; -eye(ny)];
Ccl = [C zeros(ny,ny)];

Pcl = ss(Acl, Bcl, Ccl, 0);
Pu = ss(Acl, Bcl, -Kaug, 0);

%% Step response plots
figure(1)
step(Pcl)
figure(2)
step(Pu)


%% Penalize the error

Qy = C*Qx*C';

([0.1;0.1;0.1;0.4] - [0;0;0;0])'*Qy*([0.1;0.1;0.1;0.4] - [0;0;0;0])


nyquistplot(Lsf)