clear; clc

% Vehicle parameters
m  = 300;      % mass (kg)
Iz = 120;      % yaw inertia
a  = 0.8;      % CG to front axle (m)
b  = 0.8;      % CG to rear axle (m)
U  = 15;       % speed (m/s)

% Tire parameters (Pacejka)
B = 10;
C = 1.3;
D = 4000;

% Simulation
dt = 0.01;
t  = 0:dt:5;

beta = zeros(size(t));
r    = zeros(size(t));

delta = deg2rad(5);   % steering angle

for k = 1:length(t)-1

    % Slip angles
    alpha_f = delta - beta(k) - a*r(k)/U;
    alpha_r = -beta(k) + b*r(k)/U;

    % Tire forces
    Fy_f = D*sin(C*atan(B*alpha_f));
    Fy_r = D*sin(C*atan(B*alpha_r));

    % Equations of motion
    beta_dot = (Fy_f + Fy_r)/(m*U) - r(k);
    r_dot    = (a*Fy_f - b*Fy_r)/Iz;

    % Integrate
    beta(k+1) = beta(k) + beta_dot*dt;
    r(k+1)    = r(k) + r_dot*dt;
end

% Plots
figure
plot(t,r)
xlabel('Time (s)')
ylabel('Yaw Rate (rad/s)')
grid on

figure
plot(t,beta)
xlabel('Time (s)')
ylabel('Sideslip (rad)')
grid on