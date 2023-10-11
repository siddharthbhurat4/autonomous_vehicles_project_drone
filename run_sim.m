clear;
close all;
%s0 = ones(12,1);   % state initial conditions
s0 = [0;3;0;0;0;0;0;0;0;0;0;0];
%s0 = [0;2;0;0;0;0;0;0;0;0;0;-2];
%s0 = [0;0;0;0;0;0;0;0;0;0;0;0];
T = 10;             % time horizon (sec)

tspan = [0 T];

[t s] = ode45(@eom,tspan,s0);

figure(1)
p=plot(t,s,'LineWidth',1);
p(1).LineStyle = '--'; 
p(2).LineStyle = '--';
p(3).LineStyle = '--';
p(7).LineStyle = '-.';
p(8).LineStyle = '-.';
p(9).LineStyle = '-.';
legend('x','y','z','$\phi$','$\theta$','$\psi$','$\dot{x}$','$\dot{y}$','$\dot{z}$','$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$','Interpreter','latex','Fontsize',12)


figure(2)
pause(0.05)
positions = s(:,1:3);
angles = s(:,7:9);
animate(positions,angles);