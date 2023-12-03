function sdot = eom(t, s)
% Computes the continuous-time nonlinear state equation 
%   sdot = f(s,u)
    [m,g,Ix,Iy,Iz,l,k,b]=get_model();
    
    s_nom = [0;0;0;0;0;0;0;0;0;0;0;0];
    u_nom = get_u_nom(0,0); % Hover

    delta_s = s-s_nom;
    
    K = get_K_pid(s_nom,u_nom);
    
    u = -K*delta_s+u_nom;
    sdot = f_nonlinear(s,u);
end
