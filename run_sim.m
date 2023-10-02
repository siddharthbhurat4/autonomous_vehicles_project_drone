clear;
close all;
s0 = zeros(12,1);   % state initial conditions
T = 5;             % time horizon (sec)

tspan = [0 T];

[t s] = ode45(@eom_nonlinear,tspan,s0);
l = 0.225;


figure(1)
plot(t,s);
legend('x','y','z','$\phi$','$\theta$','$\psi$','$\dot{x}$','$\dot{y}$','$\dot{z}$','$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$','Interpreter','latex','Fontsize',12)

figure(2)
positions = s(:,1:3);
angles = s(:,4:6);
animate(positions,angles,l);