clear;
close all;
Z0 = zeros(12,1);
tspan = 1:0.01:2;
[T Z] = ode45(@eom,tspan,Z0);
disp(Z)
l = 0.23;

    
% figure(1)
% plot(T,Z);
% legend('x','y','z','$\phi$','$\theta$','$\psi$','$\dot{x}$','$\dot{y}$','$\dot{z}$','$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$','Interpreter','latex','Fontsize',12)

figure(1)
positions = Z(:,1:3);
angles = Z(:,7:9);
animate(positions,angles,l);