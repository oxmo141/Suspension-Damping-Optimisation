clear; clc
%% Parameters
param;
lat_g = 1;

% moments about each axle
Moment.frontunsprung = lat_g*g*frontunsprung.m*(car.r_wheel-front.RC);
Moment.rearunsprung = lat_g*g*rearunsprung.m*(car.r_wheel-rear.RC);
Moment.sprung = lat_g*g*car.m_sprung*car.h_roll;

% stiffness matrix
A = [front.k_roll, 0, -car.TR;
    0, rear.k_roll, car.TR;
    1, -1, 1];

% load matrix
M = [Moment.frontunsprung + car.b/(car.a+car.b)*Moment.sprung;
    Moment.rearunsprung + car.a/(car.a+car.b)*Moment.sprung;
    0];

%% Calculations
% find roll angles
R = A\ M;
fprintf('Front roll angle: %.3f deg\nRear roll angle: %.3f deg\nChassis roll angle: %.3f deg\n', ...
    R(1)*180/pi, R(2)*180/pi, R(3)*180/pi);