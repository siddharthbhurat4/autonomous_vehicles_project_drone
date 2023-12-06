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

%% Trajectory Generation
%Setting the first waypoint target
% load("waypoints_square.mat");
% waypoints = waypoint_gen(waypoints_square);
delta_theta = 1.3;
radius = 1.0;
z = 1;
waypoints = [];

% Circle shaped 
total_height = 5;
increment = 0;%total_height/401;
% z = 0;
for i = 0:delta_theta:360
    x = radius*cos(deg2rad(i));
    y = radius*sin(deg2rad(i));
    z = z + increment;
    waypoints = [waypoints; x y z 0];
end

% testing landing
% waypoints = [waypoints; 0 0 0 0];

% infinity shaped
% total_height = 5;
% increment = total_height/401;
% z = 0;
% for t = 0:delta_theta:360
%     t = deg2rad(t);
%     x = (radius * sqrt(2) * cos(t)) ./ (1 + sin(t).^2);
%     y = (radius * sqrt(2) * cos(t) .* sin(t)) ./ (1 + sin(t).^2);
%     z = z + increment;
%     waypoints = [waypoints; x y z 0];
% end
% 
% distance = norm(current_pos(1:3) - waypoints(1,:));
% step_incre = distance/50;
% start = 0.0;
% extra_points = [];
% while start < distance
%     start = start + step_incre;
%     extra_points = [extra_points;waypoints(1,:) + start];
% end
% waypoints = [extra_points; waypoints];

%% MPC RUNS HERE
i = 1;
current_waypoint_targ = waypoints(i,:);
targ_waypoint_window = waypoints(i:N,:);
dist_pos = [current_pos(1),current_pos(2),current_pos(3),current_pos(9)]';

%Setting the threshold for condition to check if waypoint is reached
reach_cond_thresh = 0.5;
end_window = N;
pose = [];
length = size(waypoints,1);
disp("Total Waypoints: ")
disp(length)
%MPC ?
while i < size(waypoints,1)  
    if i == length
        break;
    end
    if norm(current_waypoint_targ - dist_pos) < reach_cond_thresh
        if i < size(waypoints,1)
            i = i + 1;
            end_window = N + i -1;
            if end_window > size(waypoints,1)
                waypoints = [waypoints;waypoints(end,:)];
            end
        end
    end

    targ_waypoint_window = waypoints(i:end_window,:);
    current_waypoint_targ = waypoints(i,:)';
    
    options = optimoptions('fmincon','Algorithm','sqp', 'MaxFunctionEvaluations',6*1200000);   %   Last entry sets the algorithm to SQP
    u_opt = fmincon(@(u)drone_objective(u,current_pos,N,T_MPC,current_waypoint_targ,targ_waypoint_window,waypoints),u0,[],[],[],[],[],[],@(u)drone_constraints(u,current_pos,N,T_MPC,current_waypoint_targ,targ_waypoint_window),options);
    
    disp("Current Control Input")
    disp(u_opt(1,:))
    disp("Current Position")
    disp(current_pos(1:3))
    disp("Current Waypoint Target")
    disp(current_waypoint_targ(1:3))

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
    disp("Waypoint Index: ")
    disp(i)
    pose = [pose;current_pos'];
    % state = [x(i) y(i) z(i) x_vel(i) v(i) w(i) phi(i) theta(i) psi(i) p(i) q(i) r(i)]';
end
%%
% load("drone_states_mpc_circle.mat")
positions = pose(:,1:3);
angles = pose(:,7:9);
animate(positions,angles, positions,waypoints(:,1:3));
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