function sdot = f_nonlinear(s,omega)
% Computes the continuous-time nonlinear state equation 
%   sdot = f(s,u)
    
    % Model Parameters 
    [m,g,Ix,Iy,Iz,l,k,b]=get_model();
    
    % State Vars
    % x, y, z           linear position in world frame
    % u, v, w           linear velocity in body frame
    % phi, theta, psi   euler angles
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
    
    omega1 = omega(1);
    omega2 = omega(2);
    omega3 = omega(3);
    omega4 = omega(4);
    
    % System inputs
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
    Tb2e = [1  sin(phi)*tan(theta)  cos(phi)*tan(theta);    % Compute transformation matrix from 
            0     cos(phi)               -sin(phi);         % body frame to euler angle derivatives 
            0  sin(phi)*sec(theta)  cos(phi)*sec(theta)];  
    angular_vel = Tb2e*[p q r]';
    
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