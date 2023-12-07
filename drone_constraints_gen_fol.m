function [c_ineq,c_eq] = drone_constraints(u,current_pos_feedback,horizon_length,T_MPC)
    
    N = horizon_length; % horizon length
    delta_T = T_MPC;    % time step
    c1 = [];
    c2 = [];
    c3 = [];
    c_ineq = [];
    c_eq = [];
    buffer = 0.8;
    x = current_pos_feedback(1)*ones(N,1);
    y = current_pos_feedback(2)*ones(N,1);
    z = current_pos_feedback(3)*ones(N,1);
    x_vel = current_pos_feedback(4)*ones(N,1);
    v = current_pos_feedback(5)*ones(N,1);
    w = current_pos_feedback(6)*ones(N,1);
    phi = current_pos_feedback(7)*ones(N,1);
    theta = current_pos_feedback(8)*ones(N,1);
    psi = current_pos_feedback(9)*ones(N,1);
    p = current_pos_feedback(10)*ones(N,1);
    q = current_pos_feedback(11)*ones(N,1);
    r = current_pos_feedback(12)*ones(N,1);

    state = [x(1) y(1) z(1) x_vel(1) v(1) w(1) phi(1) theta(1) psi(1) p(1) q(1) r(1)]';
    % Step through the horizon to calculate the final position
    for i=2:N
        K1 = drone_dynamics(state,u(i-1,:));
        K2 = drone_dynamics(state+K1*delta_T/2,u(i-1,:));
        K3 = drone_dynamics(state+K2*delta_T/2,u(i-1,:));
        K4 = drone_dynamics(state+K3*delta_T,u(i-1,:));
        K_weighted = 1/6*(K1 + 2*K2 + 2*K3 + K4);
        
        x(i) = x(i-1) + K_weighted(1)*delta_T;
        y(i) = y(i-1) + K_weighted(2)*delta_T;
        z(i) = z(i-1) + K_weighted(3)*delta_T;
        x_vel(i) = x_vel(i-1) + K_weighted(4)*delta_T;
        v(i) = v(i-1) + K_weighted(5)*delta_T;
        w(i) = w(i-1) + K_weighted(6)*delta_T;
        phi(i) = phi(i-1) + K_weighted(7)*delta_T;
        theta(i) = theta(i-1) + K_weighted(8)*delta_T;
        psi(i) = psi(i-1) + K_weighted(9)*delta_T;
        p(i) = p(i-1) + K_weighted(10)*delta_T;
        q(i) = q(i-1) + K_weighted(11)*delta_T;
        r(i) = r(i-1) + K_weighted(12)*delta_T;
        
        state = [x(i) y(i) z(i) x_vel(i) v(i) w(i) phi(i) theta(i) psi(i) p(i) q(i) r(i)]';
        buffer = 0.1;
        radius = 1.4142/2;
        c_ineq = [c_ineq; (radius+buffer)^2 - (x(i)-1.0)^2 - (y(i)-1.0)^2];
        % % c_eq = [c_eq state(1)-current_waypoint_targ(1), state(2)-current_waypoint_targ(2), state(3)-current_waypoint_targ(3), state(9)-current_waypoint_targ(4)];
        % c1 = [c1, state(1)-targ_waypoint_window(i,1)-buffer -state(1)+targ_waypoint_window(i,1)+buffer];
        % c2 = [c2, state(2)-targ_waypoint_window(i,2)-buffer -state(2)+targ_waypoint_window(i,2)+buffer];
        % c3 = [c3, state(3)-targ_waypoint_window(i,3)-buffer -state(3)+targ_waypoint_window(i,3)+buffer];
        % c4 = [c4; x_vel(N)-4.0 -x_vel(N)+4.0 v(N)-4.0 -v(N)+4.0 w(N)-4.0 -w(N)+4.0];
    end
    % 
    % c_ineq = [c1' ;c2'; c3'];%; c4'];
    % c_ineq = c4;
    % % c_eq = [c_eq state(1)-current_waypoint_targ(1), state(2)-current_waypoint_targ(2), state(3)-current_waypoint_targ(3), state(9)-current_waypoint_targ(4)];
    % % c_eq = c_eq';
    % c1 = [state(1)-current_waypoint_targ(1)-buffer -state(1)+current_waypoint_targ(1)+buffer];
    % c2 = [state(2)-current_waypoint_targ(2)-buffer -state(2)+current_waypoint_targ(2)+buffer];
    % c3 = [state(3)-current_waypoint_targ(3)-buffer -state(3)+current_waypoint_targ(3)+buffer];
    % c_ineq = [c1'; c2'; c3'; c4'];

return
