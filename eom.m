function sdot = eom(t, s)

m = 0.468;
g = 9.81;
Ix = 4.856*1e-3;
Iy = 4.856*1e-3;
Iz = 8.801*1e-3;
l = 0.225;
K = 2.980*1e-6;
b = 1.14*1e-7;
% params.Ax = 0.25*0;
% params.Ay = 0.25*0;
% params.Az = 0.25*0;


%state vars
x = s(1);
y = s(2);
z = s(3);
u = s(4);
v = s(5);
w = s(6);
phi = s(7);
theta = s(8);
psi = s(9);
p = s(10);
q = s(11);
r = s(12);

rotm = eul2rotm([phi, theta, psi],"ZYX");
lin_vel = rotm*[u v w]';

omega = ones(4,1)*m*g/4;
disp(omega);
% omega = omega + [0;1;0;-1]; %roll
% omega = omega - [-1;0;1;0]; %pitch
% omega = omega - [-1;1;-1;1]; %yaw

% params.omega1 = speed-0.5*dspeed;
% params.omega2 = speed+0.5*dspeed;
% params.omega3 = speed-0.5*dspeed;
% params.omega4 = speed+0.5*dspeed;

F = sum(omega);
tau_phi = K*l*(omega(4)^2-omega(2)^2);
tau_theta = K*l*(omega(3)^2-omega(1)^2);
tau_psi = b*(omega(1)^2-omega(2)^2+omega(3)^2-omega(4)^2);


udot = r*v-q*w + -g*sin(theta);
vdot = p*w-r*u + g*cos(theta)*sin(phi);
wdot = q*u-p*v + g*cos(theta)*cos(phi) + 1/m*-F;

rot_rotm = [1 sin(phi)*tan(theta) cos(phi)*tan(theta)
           0 cos(phi) -sin(phi)
           0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
angular_vel = rot_rotm*[p q r]';

pdot = ((Iy - Iz)/Ix)*q*r + tau_phi/Ix;
qdot = ((Iz - Ix)/Iy)*p*r + tau_theta/Iy;
rdot = ((Ix - Iy)/Iz)*p*q + tau_psi/Iz;

sdot = zeros(12,1);
sdot(1) = lin_vel(1);
sdot(2) = lin_vel(2);
sdot(3) = lin_vel(3);
sdot(4) = udot;
sdot(5) = vdot;
sdot(6) = wdot;
sdot(7) = angular_vel(1);
sdot(8) = angular_vel(2);
sdot(9) = angular_vel(3);
sdot(10) = pdot;
sdot(11) = qdot;
sdot(12) = rdot;
end