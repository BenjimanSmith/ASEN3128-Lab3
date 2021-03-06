% This script initializes initial conditions for the trajectory of a drone
% in steady flight conditions (hovering) then calls the ODE45 function to
% numerically integrate the position of the drone with respect to the
% initial conditions, along with plotting the results answering Question
% 7
%
%   Author: Benjiman Smith
%   Collaborators: E. Owen, I. Quezada
%   Date: 1/25/2020
%
clc;
clear all;
close all;
m = 0.068; % mass of the drone [kg]
r = 0.06;  % body to motor distance [m]
k = 0.0024;  % [Nm/N]
rad = r/sqrt(2);  % [m]
g = 9.81; % gravity [m/s^2]
alpha = 2e-6;   % [N/(m/s)^2]
eta = 1e-3;    % [N/(rad/s)^2]
Ix = 6.8e-5;   % moment of inertia in the x direction [kg m^2]
Iy = 9.2e-5;   % moment of inertia in the y direction [kg m^2]
Iz = 1.35e-4;  % moment of inertia in the z direction [kg m^2]
givens = [alpha eta Ix Iy Iz m r k rad g]; % givens vector
F = m*g;
%% .01 roll rate
tspan = linspace(0,5); % time vector
Pertubations = zeros(1, 3); % No perturbation
TrimForces = ones(1, 4) * m * g / 4; % forces required by each motor to maintain hover
conditions = zeros(1, 12); % initialize conditions vector (very large)
conditions(4) = 0.1;
conditions(12) = -1; % set down direction to 1 to make signs correct for plotting
t1 = 0;
t2 = 0;
X = 0;
X2 = 0;
options = odeset('Events', @StopFnct, 'RelTol', 1e-8); % stop function that ends ODE when a tolerance of 1e-8 is met
[t1, X] = ode45(@(t, F)Specs2LB3NLC(t, F, TrimForces, Pertubations, givens), tspan, conditions, options);


figure()
sgtitle('roll deviation with feedback control');
 subplot(4,2,1);
plot(t1, X(:,7));

grid on
xlabel('time (s)')
ylabel('\phi (rad)')
title('change in bank over time')
hold off
%
 subplot(4,2,2);
plot(t1, X(:,8));

grid on
xlabel('time (s)')
ylabel('\theta (rad)')
title('change in elevation over time')
hold off
%
 subplot(4,2,3);
plot(t1, X(:,4));

grid on
xlabel('time (s)')
ylabel('p (rad/s)')
title('change in roll rate over time')
hold off
%
 subplot(4,2,4);
plot(t1, X(:,5));
 
grid on
xlabel('time (s)')
ylabel('q (rad/s)')
title('change in pitch rate over time')
hold off
%
 subplot(4,2,5);
plot(t1, X(:,6));

grid on
xlabel('time (s)')
ylabel('r (rad/s)')
title('change in yaw rate over time')
hold off
%
 subplot(4,2,6);
plot(t1, X(:,1));

grid on
xlabel('time (s)')
ylabel('u (m/s)')
title('change in u over time')
hold off
%
 subplot(4,2,7);
plot(t1, X(:,2));

grid on
xlabel('time (s)')
ylabel('v (m/s)')
title('change in v over time')
hold off
%
 subplot(4,2,8);
plot(t1, X(:,3));

grid on
xlabel('time (s)')
ylabel('w (m/s)')
title('change in w over time')
hold off

%% .01 pitch rate
tspan = linspace(0,5); % time vector
Pertubations = zeros(1, 3); % No perturbation
TrimForces = ones(1, 4) * m * g / 4; % forces required by each motor to maintain hover
conditions = zeros(1, 12); % initialize conditions vector (very large)
conditions(5) = 0.1;
conditions(12) = -1; % set down direction to 1 to make signs correct for plotting
t1 = 0;
t2 = 0;
X = 0;
X2 = 0;
options = odeset('Events', @StopFnct, 'RelTol', 1e-8); % stop function that ends ODE when a tolerance of 1e-8 is met
[t1, X] = ode45(@(t, F)Specs2LB3NLC(t, F, TrimForces, Pertubations, givens), tspan, conditions, options);


figure ()
sgtitle('pitch deviation with feedback control');
 subplot(4,2,1);
plot(t1, X(:,7)); 
grid on
xlabel('time (s)')
ylabel('\phi (rad)')
title('change in bank over time')
hold off
%
 subplot(4,2,2);
plot(t1, X(:,8));

grid on
xlabel('time (s)')
ylabel('\theta (rad)')
title('change in elevation over time')
hold off
%
 subplot(4,2,3);
plot(t1, X(:,4)); 
grid on
xlabel('time (s)')
ylabel('p (rad/s)')
title('change in roll rate over time')
hold off
%
 subplot(4,2,4);
plot(t1, X(:,5));

grid on
xlabel('time (s)')
ylabel('q (rad/s)')
title('change in pitch rate over time')
hold off
%
 subplot(4,2,5);
plot(t1, X(:,6));
hold on

xlabel('time (s)')
ylabel('r (rad/s)')
title('change in yaw rate over time')
hold off
%
 subplot(4,2,6);
plot(t1, X(:,1));
 
grid on
xlabel('time (s)')
ylabel('u (m/s)')
title('change in u over time')
hold off
%
 subplot(4,2,7);
plot(t1, X(:,2));

grid on
xlabel('time (s)')
ylabel('v (m/s)')
title('change in v over time')
hold off
%
 subplot(4,2,8);
plot(t1, X(:,3));

grid on
xlabel('time (s)')
ylabel('w (m/s)')
title('change in w over time')
hold off

%% .01 yaw rate
tspan = linspace(0,5); % time vector
Pertubations = zeros(1, 3); % No perturbation
TrimForces = ones(1, 4) * m * g / 4; % forces required by each motor to maintain hover
conditions = zeros(1, 12); % initialize conditions vector (very large)
conditions(6) = 0.1;
conditions(12) = -1; % set down direction to 1 to make signs correct for plotting
t1 = 0;
t2 = 0;
X = 0;
X2 = 0;
options = odeset('Events', @StopFnct, 'RelTol', 1e-8); % stop function that ends ODE when a tolerance of 1e-8 is met
[t1, X] = ode45(@(t, F)Specs2LB3NLC(t, F, TrimForces, Pertubations, givens), tspan, conditions, options);


figure()
sgtitle('yaw deviation with feedback control');
 subplot(4,2,1);
plot(t1, X(:,7));
 
grid on
xlabel('time (s)')
ylabel('\phi (rad)')
title('change in bank over time')
hold off
%
 subplot(4,2,2);
plot(t1, X(:,8));
 
grid on
xlabel('time (s)')
ylabel('\theta (rad)')
title('change in elevation over time')
hold off
%
 subplot(4,2,3);
plot(t1, X(:,4));

grid on
xlabel('time (s)')
ylabel('p (rad/s)')
title('change in roll rate over time')
hold off
%
 subplot(4,2,4);
plot(t1, X(:,5));
 
grid on
xlabel('time (s)')
ylabel('q (rad/s)')
title('change in pitch rate over time')
hold off
%
 subplot(4,2,5);
plot(t1, X(:,6));

grid on
xlabel('time (s)')
ylabel('r (rad/s)')
title('change in yaw rate over time')
hold off
%
 subplot(4,2,6);
plot(t1, X(:,1));
 
grid on
xlabel('time (s)')
ylabel('u (m/s)')
title('change in u over time')
hold off
%
 subplot(4,2,7);
plot(t1, X(:,2));

grid on
xlabel('time (s)')
ylabel('v (m/s)')
title('change in v over time')


%
 subplot(4,2,8);
plot(t1, X(:,3));
 
grid on
xlabel('time (s)')
ylabel('w (m/s)')
title('change in w over time')


