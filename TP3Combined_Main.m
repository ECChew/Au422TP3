%% Principe de guidage des systèmes autonomes (Au 422)
% TP3 : Taking gravity into account

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
Kp = 6; % Proportional gain
Kd = 3; % Derivative gain

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
Mean_Offset = (Max + Min) / 2; % Computing Bias as the mean of the minimum  
%and maximum values previously defined
VarianceA = 0.075; % Variance of the Sensor Noise
%% Simulate the Ultrasonic sensor
VarianceU = 0.0000075;
%% Initial states
xp0 = 0;
xv0 = 0;
%%
A = [1 dt;0 1];
H = [1 0;0 1];
B = [dt*dt/2;dt];
Q = [0.0001 0;0 2];
R = [VarianceU^2 0;0 0.075];
P0 = [VarianceU^2 0;0 VarianceA^2];
%% Simulation run
sim('TP3combine', t_sim); % Runs the Simulink file

%% Plots
figure(1)
subplot(2, 1, 1)
plot(t, pos_est, '-r', 'Linewidth', 3)
hold on;
plot(t, realpos, '-b', 'Linewidth', 3)
xlabel('time [s]')
ylabel('position(z) [m]')
hold off;
legend('Data fusion','Real')
grid on;
subplot(2, 1, 2)
plot(t, vel_est, '-r', 'Linewidth', 3)
hold on;
plot(t, realvel, '-b', 'Linewidth', 3)
xlabel('time [s]');
ylabel('velocity (v) [m/s]');
hold off;
legend('Real','Data Fusion')
grid on;
figure(2)
subplot(2, 1, 1)
plot(t, zp, '-r', 'Linewidth', 3)
xlabel('time [s]')
ylabel('position(z) [m]')
grid on;
subplot(2, 1, 2)
plot(t, zv, '-b', 'Linewidth', 3)
xlabel('time [s]');
ylabel('velocity (v) [m/s]');
grid on;