function sdot = f_linear(s, u, s_nom, u_nom)
% Propagate the linear dynamics around the nominal state and input s_nom,
% u_nom
    [A,B]=eval_jacobian(s_nom,u_nom);
    delta_s = s-s_nom;
    delta_u = u-u_nom;
    sdot=A*delta_s+B*delta_u;
end
