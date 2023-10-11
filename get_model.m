function [m,g,Ix,Iy,Iz,l,k,b]=get_model()
    % Model Parameters 
    m = 0.6461009174;           % total mass (kg)
    g = 9.81;                   % gravity (m/s^2)
    Ix = 7.5*(10^-3);           % moment of inertia around x body frame (kg m^2)
    Iy = 7.5*(10^-3);           % moment of inertia around y body frame (kg m^2)
    Iz = 1.3*(10^-2);           % moment of inertia around z body frame (kg m^2)
    l = 0.23;                   % distance from z body frame to rotor axis (m)
    k = 3.13*(10^-5);           % lift constant (thrust)
    b = 1.14*1e-7;              % drag constant (torque)
end