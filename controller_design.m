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
%Q=diag([80 80 80 10 10 10 12 12 12 8 8 8]); 
Q=diag([50 50 50 5 5 5 2 2 2 1 1 1]);
R=diag([1/225 1/225 1/225 1/225]); 
[K,S,P]=lqr(A,B,Q,R);
disp(P)
disp(K)

P_cl=ss(A-B*K,B,eye(12),0);

% Form loop for state-feedback
Lsf = K*ss(A,B,eye(12),0);
allmargin(Lsf)
MMIO = diskmargin(ss(A,B,eye(12),0),K)
[np, wp] = norm(P_cl,inf)
