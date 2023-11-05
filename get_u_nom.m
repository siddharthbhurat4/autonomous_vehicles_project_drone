function u = get_u_nom(wdot_des,rdot_des)
% Get feedforward term u when linearizing over 0=f(0..w_des..rdot_des..0,u)
% |rdot_des|< (b*m*(g+w_des))/(Iz*k) : 1.775 when w_des=0
    [m,g,Ix,Iy,Iz,l,k,b]=get_model();
    omega1 = sqrt((b*m*(g+wdot_des)-Iz*k*rdot_des)/(4*b*k));
    omega2 = -sqrt((b*m*(g+wdot_des)+Iz*k*rdot_des)/(4*b*k));
    omega3 = sqrt((b*m*(g+wdot_des)-Iz*k*rdot_des)/(4*b*k));
    omega4 = -sqrt((b*m*(g+wdot_des)+Iz*k*rdot_des)/(4*b*k));
    u = [omega1;omega2;omega3;omega4];
end