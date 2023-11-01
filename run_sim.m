clear;
close all;
%s0 = ones(12,1);   % state initial conditions
s0 = [0;0;0;0;0;0;0;0;0;0;0;0];
%s0 = [0;2;0;0;0;0;0;0;0;0;0;-2];
%s0 = [0;0;0;0;0;0;0;0;0;0;0;0];


max_time = 20;             % time horizon (sec)
tstep    = 0.01; % this determines the time step at which the solution is given
cstep    = 0.05; % image capture time interval
max_iter = max_time/cstep; % max iteration
nstep    = cstep/tstep;
time     = 0; % current time
curr_state = s0;

s_save = [];
t_save = [];
traj_save = [];


load('waypoints_file.mat');
trajectory = waypoint_gen(waypoints_checkpoint2);
current_traj_idx = 1;

for iter = 1:max_iter
    tspan = time:tstep:time+cstep;
    s_nom = [trajectory(current_traj_idx,:)'; zeros(6,1)];
    [t s] = ode45(@(t,s) eom(t,s, s_nom),tspan,curr_state);
    curr_state    = s(end, :)';
    time = time + cstep;
    s_save = [s_save; s];
    t_save = [t_save; t];
    traj_save = [traj_save,s_nom];
    if norm(s_nom(1:3) - curr_state(1:3))<0.4
        if current_traj_idx < size(trajectory,1)
            current_traj_idx = current_traj_idx +1;
        end
    end
end

%tspan = [0 T];

%[t s] = ode45(@eom,tspan,s0);

figure(2)
positions = s_save(:,1:3);
angles = s_save(:,7:9);
animate(positions,angles,trajectory);

%plot(t_save,s_save(:,1:3),t_save(1:6:end),traj_save(1:3,:))
%legend('x','y','z')