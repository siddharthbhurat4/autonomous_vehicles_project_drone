clear,clc

% Simulation parameters
T_MPC = 0.1;
delta_T = T_MPC;
%Define the horizon length and time step
N = 20;

%Specify an initial guess (control input)
% u0 = []*ones(N,4);
u_hover = [225 -225 225 -225];
u0 = repmat(u_hover, N, 1);

%   Initial position and orientation
current_pos = 0*ones(12,1);
dist_pos = [current_pos(1),current_pos(2),current_pos(3),current_pos(9)]';

% MPC RUNS HERE
i = 1;

goal_pos = [2.0,2.0,2.0,0]';

%Setting the threshold for condition to check if waypoint is reached
reach_cond_thresh = 0.5;
end_window = N;
pose = [];

%MPC ?
while norm(dist_pos - goal_pos) > reach_cond_thresh  
    
    options = optimoptions('fmincon','Algorithm','sqp', 'MaxFunctionEvaluations',6*1200000);   %   Last entry sets the algorithm to SQP
    u_opt = fmincon(@(u)drone_objective_gen_fol(u,current_pos,N,T_MPC,goal_pos),u0,[],[],[],[],[],[],@(u)drone_constraints_gen_fol(u,current_pos,N,T_MPC),options);

    disp("Current Control Input")
    disp(u_opt(1,:))
    disp("Current Position")
    disp(current_pos(1:3))

    K1 = drone_dynamics(current_pos,u_opt(1,:));
    K2 = drone_dynamics(current_pos+K1*delta_T/2,u_opt(1,:));
    K3 = drone_dynamics(current_pos+K2*delta_T/2,u_opt(1,:));
    K4 = drone_dynamics(current_pos+K3*delta_T,u_opt(1,:));
    K_weighted = 1/6*(K1 + 2*K2 + 2*K3 + K4);

    current_pos(1) = current_pos(1) + K_weighted(1)*delta_T;
    current_pos(2) = current_pos(2) + K_weighted(2)*delta_T;
    current_pos(3) = current_pos(3) + K_weighted(3)*delta_T;
    current_pos(4) = current_pos(4) + K_weighted(4)*delta_T;
    current_pos(5) = current_pos(5) + K_weighted(5)*delta_T;
    current_pos(6) = current_pos(6) + K_weighted(6)*delta_T;
    current_pos(7) = current_pos(7) + K_weighted(7)*delta_T;
    current_pos(8) = current_pos(8) + K_weighted(8)*delta_T;
    current_pos(9) = current_pos(9) + K_weighted(9)*delta_T;
    current_pos(10) = current_pos(10) + K_weighted(10)*delta_T;
    current_pos(11) = current_pos(11) + K_weighted(11)*delta_T;
    current_pos(12) = current_pos(12) + K_weighted(12)*delta_T;
    dist_pos = [current_pos(1),current_pos(2),current_pos(3),current_pos(9)]';
    pose = [pose;current_pos'];
    % state = [x(i) y(i) z(i) x_vel(i) v(i) w(i) phi(i) theta(i) psi(i) p(i) q(i) r(i)]';
end
%%
% load("drone_statesx_mpc_circle.mat")
positions = pose(:,1:3);
angles = pose(:,7:9);
animate_cuboid(positions,angles,positions,positions);
%%
% figure(1)
% subplot(3,1,1)
% hold on
% stairs(ans.time_vec,ans.x);
% grid
% xlabel('Time (s)','fontsize',12);
% ylabel('x position (m)','fontsize',12);
% subplot(3,1,2)
% hold on
% stairs(ans.time_vec,ans.v);
% grid
% xlabel('Time (s)','fontsize',12);
% ylabel('Velocity Profile','fontsize',12);
% subplot(3,1,3)
% hold on
% stairs(ans.time_vec,ans.u.Data);
% grid
% xlabel('Time (s)','fontsize',12);
% ylabel('Control Input','fontsize',12);
% 
% print ex4b_hw4_me599_u_control_MPC -dpng;