function J = drone_objective(u,current_pos_feedback,horizon_length,T_MPC,current_waypoint_targ,targ_waypoint_window)
    % u,x_pos,y_pos,psi,N,T_MPC
    N = horizon_length;        % horizon length
    delta_T = T_MPC;   % time step
    
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

    Q = eye(6);
    R = eye(6);
    
    %   Step through the horizon to calculate the objective function value
    J = 0;
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
        % cost_state = [state(1) state(2) state(3) state(9) state(7) state(8) state(10) state(11) state(12)]'; %%Can also adjust this using Q later
        cost_state = [state(1) state(2) state(3) state(9) state(7) state(8)]'; %%Can also adjust this using Q later
        cost_vel = [state(4) state(5) state(6) state(10) state(11) state(12)]';
        cost_accel = [K_weighted(4) K_weighted(5) K_weighted(6) K_weighted(10) K_weighted(11) K_weighted(12)]';
        % disp("Orientation")
        % state(7)
        % state(8)
        % state(9)
        % J = J + 100*(cost_state-[targ_waypoint_window(i,:)';0;0;0;0;0])'*Q*(cost_state-[targ_waypoint_window(i,:)';0;0;0;0;0]) + 0.01*((u(i-1,:)'-[225; -225; 225; -225])'*(u(i-1,:)'-[225; -225; 225; -225]));
        % J = J + 100*(cost_state-[targ_waypoint_window(i,:)';0;0])'*Q*(cost_state-[targ_waypoint_window(i,:)';0;0]) + 0.1*cost_vel'*R*cost_vel + 0.01*((u(i-1,:)'-[225; -225; 225; -225])'*(u(i-1,:)'-[225; -225; 225; -225]));
        J = J + 100*(cost_state-[targ_waypoint_window(i,:)';deg2rad(0);deg2rad(0)])'*Q*(cost_state-[targ_waypoint_window(i,:)';0;0]) + 0.0*cost_vel'*R*cost_vel + 0.01*((u(i-1,:)'-[225; -225; 225; -225])'*(u(i-1,:)'-[225; -225; 225; -225])) + 0.1*cost_accel'*R*cost_accel;

        
        % J = J + (u(i,:)'-[225; -225; 225; -225])'*(u(i,:)'-[225; -225; 225; -225]);
    end
    % J = (cost_state-targ_waypoint_window(i,:))'*Q*(cost_state-targ_waypoint_window(targ_waypoint_window(i,:)));
return
