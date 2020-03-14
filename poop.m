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
%% Hover Trim
tspan = linspace(0,5); % time vector
Pertubations = zeros(1, 3); % No perturbation
TrimForces = ones(1, 4) *F / 4; % forces required by each motor to maintain hover
conditions = zeros(1, 12); % initialize conditions vector (very large)
conditions(12) = 1; % set down direction to 1 to make signs correct for plotting
conditions(1) = .005;
conditions(2) = .455;
conditions(3) = .334;
conditions(4) = 5;
conditions(5) = 3;
conditions(6) = 21;
conditions(7) = .5;
conditions(8) = .345;
conditions(9) = .4545;

options = odeset('Events', @StopFnct, 'RelTol', 1e-8); % stop function that ends ODE when a tolerance of 1e-8 is met
[t, X] = ode45(@(t, F)Specs2LB3NL(t, F, TrimForces, Pertubations, givens), tspan, conditions, options);

figure(1)
plot3(0, 0, -1,'.', 'Markersize',35)

hold on
plot3(X(:, 11), X(:, 10), -X(:, 12), 'linewidth', 2)
grid on
set(gca, 'zdir','reverse')
zlabel('Down Position, [m]')
xlabel('North Position, [m]')
ylabel('East Position, [m]')
title('Steady Hover Trim (5 second flight)')