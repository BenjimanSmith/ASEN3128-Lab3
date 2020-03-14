function dfdt = nonlin_forces(t, variables, engine_forces, pertibutions, constants)

%% constants
g = constants(1); % gravity (m/s^2)
m = constants(2); % mass (kg)
r = constants(3); % radius from cg to center of rotor (m)
k = constants(4); % force/moment constant (Nm/N)
eta = constants(5); % equal to 0.5*rho*A*cd (N/(m/s)^2)
alpha = constants(6); % [N/(m/s)^2]
q = constants(7); % (m)
Ix = constants(8); % Moment of inertia about the x-axis
Iy = constants(9); % Moment of inertia about the y-axis
Iz = constants(10); % Moment of inertia about the z-axis

%% Variables

del_p = variables(4); % roll rate (rad/s)
del_q = variables(5); % pitch rate (rad/s)
del_r = variables(6); % yaw rate (rad/s)

del_psi = variables(7); % initial bank angle, rotation about the x-axis (rad/s)
del_theta = variables(8); % initial elevation angle, rotation about the x-axis (rad/s)
del_phi = variables(9); % initial azimuth angle, rotation about the x-axis (rad/s)

del_u = variables(1); % initial velocity in the body x-direction (m/s)
del_v = variables(2); % initial velocity in the body y-direction (m/s)
del_w = variables(3); % initial velocity in the body z-direction (m/s)

del_x = variables(10);
del_y = variables(11);
del_z = variables(12);


%% Engine Forces

engine_1 = engine_forces(1); % #1 engine
engine_2 = engine_forces(2); % #2 engine
engine_3 = engine_forces(3); % #3 engine
engine_4 = engine_forces(4); % #4 engine

%% Aerodynamics Moments

L_a = -alpha * sqrt(del_p^2+del_q^2+del_r^2) * sign(del_p); % aerodynamic moment about the x-axis
M_a = -alpha * sqrt(del_p^2+del_q^2+del_r^2) * sign(del_q); % aerodynamic moment about the y-axis
N_a = -alpha * sqrt(del_p^2+del_q^2+del_r^2) * sign(del_r); % aerodynamic moment about the z-axis

%% Control Moments

L_c = q*(engine_1 + engine_2 -engine_3 - engine_4); % control moment about the x-axis
M_c = q*(engine_2 + engine_3 -engine_1 - engine_4); % control moment about the y-axis
N_c = q*(engine_2 + engine_3 -engine_1 - engine_4); % control moment about the z-axis

%% Total Aircrraft Moments 

L = L_a + L_c; % total moment about the x-axis
M = M_a + M_c; % total moment about the y-axis
N = N_a + N_c; % total moment about the z-axis

if t > 1 && t < 1.5 % loop for moment perturbation
    L = L + pertibutions(1);
    M = M + pertibutions(2);
    N = N + pertibutions(3);
end

%% Aerodynamic Forces

X_a = -eta * sqrt(del_u^2 + del_v^2 + del_w^2) * sign(del_u); % aerodynamic forces in the x-direction (N)
Y_a = -eta * sqrt(del_u^2 + del_v^2 + del_w^2) * sign(del_v); % aerodynamic forces in the y-direction (N)
Z_a = -eta * sqrt(del_u^2 + del_v^2 + del_w^2) * sign(del_w); % aerodynamic forces in the z-direction (N)

%% Control Forces 

X_c = 0; % control forces in the x-direction (N)
Y_c = 0; % control forces in the y-direction (N)
Z_c = engine_1 + engine_2 + engine_3 + engine_4; % control forces in the z-direction (N)

%% Total Aircraft Forces

X = X_a + X_c; % total forces in the x-direction (N)
Y = Y_a + Y_c; % total forces in the y-direction (N)
Z = Z_a + Z_c; % total forces in the z-direction (N)

%% Derivates of pitch rate, yaw rate, and roll rate 

p_dot = (1/Ix) * (L - del_q * del_r * (Iz-Iy)); % derivative of roll rate 
q_dot = (1/Iy) * (M - del_r * del_p * (Ix-Iz)); % derivative of pitch rate 
r_dot = (1/Iz) * (N - del_p * del_q * (Iy-Iz)); % derivative of yaw rate 
omega_dot = [p_dot, q_dot, r_dot]'; % putting the rates into a column vector

%% Translational Rates

u_dot = (X/m) - g*sin(del_theta) - del_q*del_w - del_r*del_v;
v_dot = (Y/m) + g*cos(del_theta)*sin(del_phi) - del_r*del_u + del_p*del_w;
w_dot = (Z/m) + g*cos(del_theta)*cos(del_phi) - del_p*del_v + del_q*del_u;
vel_dot = [u_dot, v_dot, w_dot]';

%% Angular Momentum to Euler Angles

phi_dot = del_p + (del_q * sin(del_phi) + del_r * cos(del_phi) * tan(del_theta));
theta_dot = del_q * cos(del_phi) - del_r * cos(del_phi);
psi_dot = ((del_q * sin(del_phi)) + del_r * cos(del_phi))*sec(del_theta);
euler_dot = [phi_dot, theta_dot, psi_dot]';

%% Body Cordinates to Inertial Coordinates

% x_dot =  del_u * cos(del_theta)*cos(del_psi) + ...
%          del_v * (sin(del_phi)*sin(del_theta)*cos(del_psi) - cos(del_phi)*sin(del_psi)) + ...
%          del_w * (cos(del_phi)*sin(del_theta)*cos(del_psi) + sin(del_phi)*sin(del_psi));
% y_dot =  del_u * cos(del_theta)*sin(del_psi) + ...
%          del_v * (sin(del_phi)*sin(del_theta)*sin(del_psi) + cos(del_phi)*cos(del_psi)) + ...
%          del_w * (cos(del_phi)*sin(del_theta)*sin(del_psi) - sin(del_phi)*cos(del_psi));
% z_dot = -del_u * sin(del_theta) + ...
%          del_v * sin(del_phi)*cos(del_theta) + ...
%          del_w * cos(del_phi)*cos(del_theta);
x_dot =  del_u * cos(del_theta)*cos(del_psi) +del_v * (sin(del_phi)*sin(del_theta)*cos(del_psi) - cos(del_phi)*sin(del_psi)) + del_w * (cos(del_phi)*sin(del_theta)*cos(del_psi) + sin(del_phi)*sin(del_psi));
    y_dot =  del_u * cos(del_theta)*sin(del_psi) + del_v * (sin(del_phi)*sin(del_theta)*sin(del_psi) + cos(del_phi)*cos(del_psi)) + del_w * (cos(del_phi)*sin(del_theta)*sin(del_psi) - sin(del_phi)*cos(del_psi));
    z_dot = -del_u * sin(del_theta) + del_v * sin(del_phi)*cos(del_theta) + del_w * cos(del_phi)*cos(del_theta);
frame_dot = [x_dot, y_dot, z_dot]';


%% output

dfdt = [omega_dot; vel_dot; euler_dot; frame_dot];

end




