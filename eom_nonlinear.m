function sdot = eom_nonlinear(t, s)
% Computes the continuous-time nonlinear state equation 
%   sdot = f(s,u)

    % Model Parameters 
    m = 0.6461009174; %0.468;       % total mass (kg)
    g = 9.81;                       % gravity (m/s^2)
    Ix = 7.5*(10^-3);%4.856*1e-3;   % moment of inertia around x body frame (kg m^2)
    Iy = 7.5*(10^-3);%4.856*1e-3;   % moment of inertia around y body frame (kg m^2)
    Iz = 1.3*(10^-2);%8.801*1e-3;   % moment of inertia around z body frame (kg m^2)
    l = 0.23; %0.225;               % distance from z body frame to rotor axis (m)
    k = 3.13*(10^-5);%2.980*1e-6;   % lift constant (thrust)
    b = 1.14*1e-7;                  % drag constant (torque)
    % params.Ax = 0.25*0;
    % params.Ay = 0.25*0;
    % params.Az = 0.25*0;
    
    % Defining input
    omega_eq = sqrt((m*g/4)/k);     % By solving for u when sdot = 0 = f(s,u)
    dspeed = 0.001*omega_eq;         % Creating a small variation around equilibrium
    
    % Hover
    % omega1 = omega_eq;
    % omega2 = omega_eq;
    % omega3 = omega_eq;
    % omega4 = omega_eq;
    
    % Test pitch and roll
    omega1 = omega_eq + dspeed;
    omega2 = omega_eq;% - dspeed;
    omega3 = omega_eq;% + dspeed;
    omega4 = omega_eq;% - dspeed;
    
    % Hover and rotate
    % alpha : desired yaw angular acceleration 
    % |alpha|< (b*g*m)/(Iz*k) : 1.775

    % alpha = 0.5;
    % omega1 = sqrt((b*g*m-Iz*k*alpha)/(4*b*k));
    % omega2 = sqrt((b*g*m+Iz*k*alpha)/(4*b*k));
    % omega3 = sqrt((b*g*m-Iz*k*alpha)/(4*b*k));
    % omega4 = sqrt((b*g*m+Iz*k*alpha)/(4*b*k));
    
    % State Vars
    % x, y, z           linear position in world frame
    % u, v, w           linear velocity in body frame
    % phi, theta, psi   angular position in world frame
    % p, q, r           angular velocity in body frame
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
    
    F = k*(omega1^2 + omega2^2 + omega3^2 + omega4^2);  % Total thrust (force in z body)
    tau_phi = k*l*(-omega2^2+omega4^2);                  % Torque (x body) 
    tau_theta = k*l*(omega3^2-omega1^2);                % Torque (y body)
    tau_psi = b*(-omega1^2+omega2^2-omega3^2+omega4^2);  % Torque (z body)
    
    % Translational Kinematics
    Rb2w = get_rotation_zyx(phi,theta,psi);     % Compute rotation matrix from body
    lin_vel_world = Rb2w*[u v w]';              % frame to world frame
    
    % Translational Dynamics (body frame)
    udot = r*v-q*w + g*sin(theta);
    vdot = p*w-r*u - g*cos(theta)*sin(phi);
    wdot = q*u-p*v - g*cos(theta)*cos(phi) + F/m; % account for non-inertial frame, gravity and thrust
    
    %Rotational Kinematics
    Tb2w = [1  sin(phi)*tan(theta)  cos(phi)*tan(theta);    % Compute transformation matrix from 
            0     cos(phi)               -sin(phi);         % euler angle derivatives to angular 
            0  sin(phi)*sec(theta)  cos(phi)*sec(theta)];   % velocity in world frame
    angular_vel = Tb2w*[p q r]';
    
    % Rotational Dynamics (body frame)
    pdot = ((Iy - Iz)/Ix)*q*r + tau_phi/Ix;
    qdot = ((Iz - Ix)/Iy)*p*r + tau_theta/Iy;
    rdot = ((Ix - Iy)/Iz)*p*q + tau_psi/Iz;
    
    sdot = zeros(12,1);
    sdot(1) = lin_vel_world(1);
    sdot(2) = lin_vel_world(2);
    sdot(3) = lin_vel_world(3);
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
