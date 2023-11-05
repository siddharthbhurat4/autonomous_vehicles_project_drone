clear;
close all;

[K,G] = lqi_gain();
s_nom = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
s0=s_nom;
u_nom = get_u_nom(0,0); % Hover

max_time = 20;             % time horizon (sec)
tstep    = 0.025; % this determines the time step at which the solution is given
cstep    = 0.05; % image capture time interval
max_iter = max_time/cstep; % max iteration
nstep    = cstep/tstep;
time     = 0; % current time
curr_state = s0;

s_save = [s0.'];
t_save = [0];
traj_save = [];

load('waypoints_file.mat');
trajectory = waypoint_gen(waypoints_checkpoint2);
%trajectory = (minjerkpolytraj(waypoints_checkpoint2',[0 1 2 3],20))';
current_traj_idx = 1;

for iter = 1:max_iter
    tspan = time:tstep:time+cstep;
    s_ref = [trajectory(current_traj_idx,:)].';
    [t, s] = ode78(@(t,s) eom(t, s, K, G, s_nom, u_nom, s_ref),tspan,curr_state);
    curr_state  = s(end, :)';
    time = time + cstep;
    s_save = [s_save; s(2:end,:)];
    t_save = [t_save; t(2:end)];
    traj_save = [traj_save,curr_state];
    if norm(s_ref(1:4) - curr_state(1:4))<0.4
        if current_traj_idx < size(trajectory,1)
            current_traj_idx = current_traj_idx +1;
        end
    end
end

figure(1)
positions = s_save(:,1:3);
angles = s_save(:,7:9);
animate(positions,angles,traj_save.',trajectory);

figure(2)
p=plot(t_save,s_save,'LineWidth',1);
p(1).LineStyle = '--'; 
p(2).LineStyle = '--';
p(3).LineStyle = '--';
p(7).LineStyle = '-.';
p(8).LineStyle = '-.';
p(9).LineStyle = '-.';
p(13).LineWidth = 2;
p(14).LineWidth = 2;
p(15).LineWidth = 2;
p(16).LineWidth = 2;
hold on
yline(s_ref,LineStyle=":")
legend('x','y','z', ...
    '$\phi$','$\theta$','$\psi$', ...
    '$\dot{x}$','$\dot{y}$','$\dot{z}$', ...
    '$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$', ...
    'exdot','eydot','ezdot','epsidot', ...
    'Interpreter','latex','Fontsize',12)


%plot(trajectory)
%legend('x','y','z')