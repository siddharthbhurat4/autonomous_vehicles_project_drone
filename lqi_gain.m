function [K,G] = lqi_gain()
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
    ny = 4;

    C = [1 0 0 0 0 0 0 0 0 0 0 0;
         0 1 0 0 0 0 0 0 0 0 0 0;
         0 0 1 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 1 0 0 0];

    Aaug = [A zeros(nx,ny);C zeros(ny,ny)];
    Baug = [B; zeros(ny,nu)];

    Qx=diag([100 100 120 10 10 10 100 100 10 40 40 20]); % shape ny
    QI=diag([0.1 0.1 0.1 0.1]); % shape ny
    Qaug = blkdiag(Qx, QI);
    R=diag([1/30 1/30 1/30 1/30]); % shape nu
    
    [K,S,P] = lqr(Aaug, Baug, Qaug, R);

    Kx = K(:,1:nx);
    KI = K(:,nx+1:nx+ny);
    
    G = inv(C/(-A+B*Kx)*B);
end