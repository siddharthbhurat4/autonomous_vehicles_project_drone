function [A,B] = eval_jacobian(s,u)
% Return linearized model at given state and input

    [m,g,Ix,Iy,Iz,l,k,b]=get_model();
    
    % System inputs
    omega1 = u(1);
    omega2 = u(2);
    omega3 = u(3);
    omega4 = u(4);

    % State Vars
    % x, y, z           linear position in world frame
    % u, v, w           linear velocity in body frame
    % phi, theta, psi   euler-angles
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

    % Taking the jacobian
    A = zeros(12,12);

    A(1, 4) = cos(theta) * cos(phi);
    A(1, 5) = cos(phi) * sin(theta) * sin(phi) - cos(phi) * sin(psi);
    A(1, 6) = cos(phi) * cos(psi) * sin(theta) + sin(phi) * sin(psi);
    A(1, 7) = (cos(phi) * cos(psi) * sin(theta) + sin(phi) * sin(psi)) * v + (-cos(psi) * sin(theta) * sin(phi) + cos(phi) * sin(psi)) * w;
    A(1, 8) = -cos(psi) * sin(theta) * u + cos(theta) * cos(psi) * sin(phi) * v + cos(theta) * cos(phi) * cos(psi) * w;
    A(1, 9) = -cos(theta) * sin(psi) * u + (-cos(phi) * cos(psi) - sin(theta) * sin(phi) * sin(psi)) * v + (cos(psi) * sin(phi) - cos(phi) * sin(theta) * sin(psi)) * w;
    
    A(2, 4) = cos(theta) * sin(phi);
    A(2, 5) = cos(phi) * cos(psi) + sin(theta) * sin(phi) * sin(psi);
    A(2, 6) = -cos(psi) * sin(phi) + cos(phi) * sin(theta) * sin(psi);
    A(2, 7) = cos(psi) * (-sin(phi) * v - cos(phi) * w) + sin(theta) * sin(psi) * (cos(phi) * v - sin(phi) * w);
    A(2, 8) = -sin(theta) * sin(psi) * u + cos(theta) * sin(psi) * (sin(phi) * v + cos(phi) * w);
    A(2, 9) = cos(theta) * cos(psi) * u + cos(psi) * sin(theta) * (sin(phi) * v + cos(phi) * w) - sin(psi) * (cos(phi) * v - sin(phi) * w);
    
    A(3, 4) = -sin(theta);
    A(3, 5) = cos(theta) * sin(phi);
    A(3, 6) = cos(theta) * cos(phi);
    A(3, 7) = cos(theta) * (cos(phi) * v - sin(phi) * w);
    A(3, 8) = -cos(theta) * u - sin(theta) * (sin(phi) * v + cos(phi) * w);
    
    A(4, 5) = r;
    A(4, 6) = -q;
    A(4, 8) = g * cos(theta);
    A(4, 11) = -w;
    A(4, 12) = v;
    
    A(5, 4) = -r;
    A(5, 6) = p;
    A(5, 7) = -g * cos(theta) * cos(phi);
    A(5, 8) = g * sin(theta) * sin(phi);
    A(5, 10) = w;
    A(5, 12) = -u;
    
    A(6, 4) = q;
    A(6, 5) = -p;
    A(6, 7) = g * cos(theta) * sin(phi);
    A(6, 8) = g * cos(phi) * sin(theta);
    A(6, 10) = -v;
    A(6, 11) = u;
    
    A(7, 7) = (cos(phi) * q - r * sin(phi)) * tan(theta);
    A(7, 8) = sec(theta)^2 * (cos(phi) * r + q * sin(phi));
    A(7, 10) = 1;
    A(7, 11) = sin(phi) * tan(theta);
    A(7, 12) = cos(phi) * tan(theta);
    
    A(8, 7) = -cos(phi) * r - q * sin(phi);
    A(8, 11) = cos(phi);
    A(8, 12) = -sin(phi);
    
    A(9, 7) = sec(theta) * (cos(phi) * q - r * sin(phi));
    A(9, 8) = sec(theta) * (cos(phi) * r + q * sin(phi)) * tan(theta);
    A(9, 11) = sin(phi) * sec(theta);
    A(9, 12) = cos(phi) * sec(theta);
    
    A(10, 11) = ((Iy - Iz) * r) / Ix;
    A(10, 12) = ((Iy - Iz) * q) / Ix;
    
    A(11, 10) = ((-Ix + Iz) * r) / Iy;
    A(11, 12) = ((-Ix + Iz) * p) / Iy;
    
    A(12, 10) = ((Ix - Iy) * q) / Iz;
    A(12, 11) = ((Ix - Iy) * p) / Iz;

    B=zeros(12,4);

    B(6,1)=2*k*omega1/m;
    B(6,2)=2*k*omega2/m;
    B(6,3)=2*k*omega3/m;
    B(6,4)=2*k*omega4/m;

    B(10,2)=-2*k*l*omega2/Ix;
    B(10,4)=2*k*l*omega4/Ix;

    B(11,1)=-2*k*l*omega1/Iy;
    B(11,3)=2*k*l*omega3/Iy;

    B(12,1)=-2*b*omega1/Iz;
    B(12,2)=2*b*omega2/Iz;
    B(12,3)=-2*b*omega3/Iz;
    B(12,4)=2*b*omega4/Iz;
    
end