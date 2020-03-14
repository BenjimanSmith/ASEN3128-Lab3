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
close all
clear all
load('RSdata_0958.mat');

xdata1=rt_estimatedStates.signals.values(:,1);
ydata1=rt_estimatedStates.signals.values(:,2);
zdata1=rt_estimatedStates.signals.values(:,3);
figure(5)
plot3(xdata1, ydata1, zdata1);
zlabel('Down Position, [m]')
xlabel('East Position, [m]')
ylabel('North Position, [m]')
title('Experimental Data (Data 0958)');
figure(6)
psi1 = rt_estimatedStates.signals.values(:,4);
theta1 = rt_estimatedStates.signals.values(:,5);
phi1 =  rt_estimatedStates.signals.values(:,6);
rt_estimatedStates.signals.values(:,4);
times1 =rt_estimatedStates.time(:);
plot(times1, psi1,'linewidth', 2);
hold on
plot(times1, theta1, 'linewidth', 2);
plot(times1, phi1, 'linewidth', 2);
legend('\psi', '\theta', '\phi');
xlabel('time [s]')
ylabel('Radians')
title('Experimental bank, elevation, azimuth during time of flight');

load('RSdata_1137.mat');

xdata2=rt_estim.signals.values(:,1);
ydata2=rt_estim.signals.values(:,2);
zdata2=rt_estim.signals.values(:,3);
figure(5)
plot3(xdata2, ydata2, zdata2);
zlabel('Down Position, [m]')
xlabel('East Position, [m]')
ylabel('North Position, [m]')
title('Experimental Data (Data 0958)');
figure(6)
psi2 = rt_estim.signals.values(:,4);
theta2 = rt_estim.signals.values(:,5);
phi2 =  rt_estim.signals.values(:,6);
rt_estim.signals.values(:,4);
times2 =rt_estim.time(:);
plot(times2, psi2,'linewidth', 2);
hold on
plot(times2, theta2, 'linewidth', 2);
plot(times2, phi2, 'linewidth', 2);
legend('\psi', '\theta', '\phi');
xlabel('time [s]')
ylabel('Radians')
title('Experimental bank, elevation, azimuth during time of flight');

figure(5);
plot3(xdata1, ydata1, zdata1,'linewidth', 2);
hold on
plot3(xdata2, ydata2, zdata2, 'linewidth', 2);
zlabel('Down Position, [m]')
xlabel('East Position, [m]')
ylabel('North Position, [m]')
title('Experimental Data for controlled and uncontrolled flight (Data 0958, 1137)');
legend('not controlled', 'controlled');
hold off
figure(6)
plot(times1, psi1,'linewidth', 2);
hold on
plot(times1, theta1, 'linewidth', 2);
plot(times1, phi1, 'linewidth', 2);
plot(times2, psi2,'linewidth', 2);
hold on
plot(times2, theta2, 'linewidth', 2);
plot(times2, phi2, 'linewidth', 2);
legend('\psi uncontrolled', '\theta uncontrolled', '\phi controlled', '\psi controlled', '\theta controlled', '\phi controlled');
xlabel('time [s]')
ylabel('Radians')
title('Experimental bank, elevation, azimuth during time of flight');




