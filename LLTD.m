clear; clc
%% Parameters
param;
lat_g = 1;
car.TR = 1000;
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
fprintf('Front roll angle: %.3f deg/g\nRear roll angle: %.3f deg/g\nChassis roll angle: %.3f deg/g\n', ...
    R(1)*180/pi, R(2)*180/pi, R(3)*180/pi);

% LLTD
LDT.front_lat = 1/car.track * (car.TR*R(3) + ...
    car.b/(car.a+car.b)*Moment.sprung + Moment.frontunsprung);
LDT.rear_lat = 1/car.track * (-car.TR*R(3) + ...
    car.a/(car.a+car.b)*Moment.sprung + Moment.rearunsprung);
LDT.Dist_lat = LDT.front_lat / (LDT.front_lat+LDT.rear_lat);

%% Iterations ARB Stiffness
front.ARB = linspace(192,709,100);
rear.ARB = linspace(730,158,100);

front.k_roll = zeros(size(front.ARB));
rear.k_roll = zeros(size(front.ARB));
LDT_val = zeros(size(front.ARB));
car.TR = [car.TR-500, car.TR, car.TR+500];

for j = 1:length(car.TR)
    for i = 1:length(front.k_roll)
        front.k_roll(i) = front.ARB(i) + front.ks_roll;
        rear.k_roll(i) = rear.ARB(i) + rear.ks_roll;
        
        A = [front.k_roll(i), 0, -car.TR(j);
            0, rear.k_roll(i), car.TR(j);
            1, -1, 1];
        R = A\ M;
    
        LDT.front_lat = 1/car.track * (car.TR(j)*R(3) + ...
            car.b/(car.a+car.b)*Moment.sprung + Moment.frontunsprung);
        LDT.rear_lat = 1/car.track * (-car.TR(j)*R(3) + ...
            car.a/(car.a+car.b)*Moment.sprung + Moment.rearunsprung);
        LDT_val(i) = LDT.front_lat / (LDT.front_lat+LDT.rear_lat);
    
    end
    
    figure (j)
    plot(1:100, LDT_val);
    xlabel('iteration');
    ylabel('Lateral Load Transfer Distribution');
    title('Lateral Load Transfer Distribution vs iteration');
    grid on;
end