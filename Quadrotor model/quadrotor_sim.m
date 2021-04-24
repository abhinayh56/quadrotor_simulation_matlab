format compact; clear; close all; clc;
%%


%% Initial conditions
% time variables
ti = 0; % initial time
tf = 10; % final time
dt = 0.01; % time step for simulation
t_span = ti:dt:tf; % time span

% initial states in NWU coordinates in Inertial coordinates
% linear position
xi = 0;
yi = 0;
zi = 0;

% linear velocity
dxi = 0;
dyi = 0;
dzi = 0;

% angular position
phi = 0;
th  = 0;
psi = 0;

% angular velocity
pi = 0;
qi  = 0;
ri = 0;

Xi = [xi yi zi dxi dyi dzi phi th psi pi qi ri]';

[t_sol, X_sol] = ode45(@quadrotor_dyn,t_span,Xi);

t = t_sol;
x = X_sol(:,1);
y = X_sol(:,2);
z = X_sol(:,3);

plot(t,x)
hold on
plot(t,y)
plot(t,z)
xlabel('t sec)')
ylabel('xyz (m)')
title('position')
grid on
grid minor
legend('x','y','z')












