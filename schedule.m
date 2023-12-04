function [K,G] = schedule(state,K_pre,G_pre)
    psi = state(9);
    R = [-cos(psi) -sin(psi);
            sin(psi) -cos(psi);
            cos(psi)  sin(psi);
           -sin(psi)  cos(psi);];
    K = K_pre;
    K(:,1:2) = K(:,1:2).*R;
    K(:,13:14) = K(:,13:14).*R;
    G = G_pre;
    G(:,1:2) = G(:,1:2).*R;
end