function [K_pre,G_pre] = gain_scheduling()
    [m,g,~,~,~,~,k,~]=get_model();
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

    Qx=diag([100 100 120 10 10 10 100 100 200 40 40 10]); % shape ny
    QI=diag([0.1 0.1 0.1 0.1]); % shape ny
    Qaug = blkdiag(Qx, QI);
    R=diag([1/30 1/30 1/30 1/30]); % shape nu
    
    [K,~,~] = lqr(Aaug, Baug, Qaug, R);

    Kx = K(:,1:nx);
    KI = K(:,nx+1:nx+ny);

    max1=Kx(3,1);
    max2=KI(3,1);
    
    K_pre = K;
    K_pre(:,1:2) = max1;
    K_pre(:,13:14) = max2;
    
    G = inv(C/(-A+B*Kx)*B);

    G_pre = G;
    G_pre(:,1:2) = max1;
end