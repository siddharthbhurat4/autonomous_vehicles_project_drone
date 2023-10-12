function sdot = eom(t, s)
% Computes the continuous-time nonlinear state equation 
%   sdot = f(s,u)
    [m,g,Ix,Iy,Iz,l,k,b]=get_model();
    
    s_nom = [0;0;0;0;0;0;0;0;0;0;0;0];
    u_nom = get_u_nom(0,0); % Hover

    delta_s = s-s_nom;
    if delta_s(7)>pi
        delta_s(7)=delta_s(7)-2*pi;
    end
    if delta_s(8)>pi
        delta_s(8)=delta_s(8)-2*pi;
    end
    if delta_s(9)>pi
        delta_s(9)=delta_s(9)-2*pi;
    end
    if delta_s(7)<-pi
        delta_s(7)=delta_s(7)+2*pi;
    end
    if delta_s(8)<-pi
        delta_s(8)=delta_s(8)+2*pi;
    end
    if delta_s(9)<-pi
        delta_s(9)=delta_s(9)+2*pi;
    end
    K = get_K_pid(s_nom,u_nom);
    
    u = -K*delta_s+u_nom;
    sdot = f_nonlinear(s,u);
end
