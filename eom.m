function sdot = eom(t, s, K_pre, G_pre, s_nom, u_nom, s_ref)
% Computes the continuous-time nonlinear state equation 
%   sdot = f(s,u)  
    
    delta_s = get_delta_s(s, s_nom);
    
    if size(s, 1) == 16
        C = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
             0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
             0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
             0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0];
        [K,G] = schedule(s,K_pre,G_pre);
        u = -K*delta_s + G*s_ref + u_nom;
        s_xdot = f_nonlinear(s(1:12),u);
        s_edot = C*s - s_ref;
        sdot = [s_xdot;s_edot];
    else
        u = -K*delta_s + u_nom;
        sdot = f_nonlinear(s,u);
    end
end
