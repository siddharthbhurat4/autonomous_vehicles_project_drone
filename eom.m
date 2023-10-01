function sdot = eom(t, s)

syms x y z real
syms u v w real
syms phi theta phsi real %euler angles
syms pdot qdot rdot real
syms udot vdot wdot real 
syms p q r real
syms m g Ix Iy Iz real 
syms K l b Ax Ay Az real %k-lift constant, b- drag constant
syms omega1 omega2 omega3 omega4 real

m = 0.6461009174; %0.468;
g = 9.81;
Ix = 7.5*(10^-3);%4.856*1e-3;
Iy = 7.5*(10^-3);%4.856*1e-3;
Iz = 1.3^(10^-2);%8.801*1e-3;
l = 0.23; %0.225;
K = 3.13*(10^-5);%2.980*1e-6;
b = 1.14*1e-7;
% params.Ax = 0.25*0;
% params.Ay = 0.25*0;
% params.Az = 0.25*0;

omega = sqrt((m*g/4)/K);
dspeed = 0.05*omega;
omega1 = omega+(1.5*dspeed);
omega2 = omega-(1.5*dspeed);
omega3 = omega+(1.5*dspeed);
omega4 = omega-(1.5*dspeed);

%state vars
x = s(1);
y = s(2);
z = s(3);
u = s(4);
v = s(5);
w = s(6);
phi = s(7);
theta = s(8);
phsi = s(9);
p = s(10);
q = s(11);
r = s(12);

F = K*(omega1^2 + omega2^2 + omega3^2 + omega4^2);
tau_phi = K*l*(omega4^2-omega2^2);
tau_theta = K*l*(omega3^2-omega1^2);
tau_psi = b*(omega1^2+omega2^2+omega3^2+omega4^2);

%Translational Kinematics
rotm_transkin = [cos(phsi)*cos(theta), cos(phsi)*sin(theta)*sin(phi)-sin(phsi)*cos(phi), cos(phsi)*sin(theta)*cos(phi)+sin(phsi)*sin(phi);
                 sin(phsi)*cos(theta), sin(phsi)*sin(theta)*sin(phi) + cos(phsi)*cos(phi), sin(phsi)*sin(theta)*cos(phi)-cos(phsi)*sin(phi);
                 -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];
lin_vel = rotm_transkin*[u v w]';

% translational dynamics
udot = r*v-q*w + g*sin(theta);
vdot = p*w-r*u - g*cos(theta)*sin(phi);
wdot = q*u-p*v - g*cos(theta)*cos(phi) + F/m;

%Rotational Kinematics
rotm_rotkin = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
               0 cos(phi) -sin(phi);
               0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
angular_vel = rotm_rotkin*[p q r]';
disp(size(angular_vel))

% rotational dynamics
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
