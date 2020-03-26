%% Principe de guidage des systèmes autonomes (Au 422)
% TP2 : Application of Kalman Filter Data Fusion

%% Preparing workspace
clear vars; clc; 
%Clear vars is used instead of clear all as it is more optimized with newer
%versions
close all;

%% Parameters of the system
m = 1; % Mass of the vehicle in kg

%% Initial conditions (states)

zp0 = 0; % Initial position in m
zv0 = 0; % Initial velocity in m/s

%% Desired position
zd = 2; % Desired position in m

%% PD control gains
Kp = 50; % Proportional gain
Kd = 5; % Derivative gain

%% Simulation parameters
t_sim = 30; % Simulation time in s
dt = 0.001; % Simulation step time in s
t_dis1 = 20; % Time when the first disturbance occurs
F_dis1 = 150; % Magnetude of the first disturbance in N
t_dis2 = 40; % Time when the second disturbance occurs
F_dis2 = -107.5; % Magnetude of the second disturbance in N

%% Sensor characterization
Min = -1.3352; % Minimum value that the Bias can take
Max = -1.1666; % Maximum value that the Bias can take
Bias_Sample_time = 30 * dt; % Sampling time for the Bias
Variance = 0.75; % Variance of the Sensor Noise
%% Simulation run
sim('TP3Au422', t_sim); % Runs the Simulink file

%% Plots
figure(1) % Creates Fig1
subplot(2, 2, 1) % Row 1 Column 1
plot(t, zp, '-b', 'Linewidth', 3) % Position
xlabel('time [s]');
ylabel('position (z) [m]');
grid on;
subplot(2, 2, 2) % Row 1 Column 2
plot(t, zv, '-r', 'Linewidth', 3) % Velocity
xlabel('time [s]');
ylabel('velocity (v) [m/s]');
grid on;
subplot(2, 2, 3) % Row 2 Column 1
plot(t, za, '-k', 'Linewidth', 3) % Acceleration
xlabel('time [s]');
ylabel('acceleration (a) [m/s^2]');
grid on;
subplot(2, 2, 4) % Row 2 Column 2
plot(zp, zv, '-m', 'Linewidth', 3) % States
xlabel('position (z) [m]');
ylabel('velocity (v) [m/s]');
grid on;

figure(2) % Creates Fig1
subplot(2, 2, 1)
plot(t, zp, '-b', 'Linewidth', 3)
hold on;
plot(t, zv, 'r', 'Linewidth', 3)
xlabel('time [s]');
ylabel('states (z, v) [m, m/s]');
legend('position (x)', 'velocity (v)', 'Location', 'East')
grid on;
subplot(2, 2, 2)
plot(t, uz, '-g', 'Linewidth', 3)
xlabel('time [s]');
ylabel('Control [N]');
grid on;
subplot(2, 2, 3)
plot(t, dist1, '-c', 'Linewidth', 3)
xlabel('time [s]');
ylabel('disturbance_1 [N]');
grid on;
subplot(2, 2, 4)
plot(t, dist2, '-c', 'Linewidth', 3)
xlabel('time [s]');
ylabel('disturbance_2 [N]');
grid on;