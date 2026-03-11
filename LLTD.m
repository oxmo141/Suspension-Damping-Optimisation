clear; clc
% kf = linspace(10000,50000,50);
% kr = linspace(10000,50000,50);
% 
% [KF,KR] = meshgrid(kf,kr);
% 
% LLTD = KF ./ (KF + KR);   % simple distribution example
% 
% contourf(KF,KR,LLTD,20)
% xlabel('Front Roll Stiffness')
% ylabel('Rear Roll Stiffness')
% title('LLTD Distribution')
% colorbar
%% Sample Code
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

% Calculations
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

%% Lateral Load Transfer Distribution, Sweep Stiffness Distribution and Weight Distribution
front.ARB = linspace(192,709,100);
rear.ARB = linspace(730,158,100);
weight_distribution = linspace(0,1,100);

front.k_roll = zeros(size(front.ARB));
rear.k_roll = zeros(size(front.ARB));
front.k_roll = front.ARB + front.ks_roll;
rear.k_roll = rear.ARB + rear.ks_roll;
LDT_val = zeros(size(front.ARB));
%%
for j = 1:length(front.k_roll)
    for i = 1:length(front.k_roll)
        
        A = [front.k_roll(i), 0, -car.TR;
            0, rear.k_roll(i), car.TR;
            1, -1, 1];
        R = A\ M;
    
        LDT.front_lat = 1/car.track * (car.TR*R(3) + ...
            (1-weight_distribution(j))*Moment.sprung + Moment.frontunsprung);
        LDT.rear_lat = 1/car.track * (-car.TR*R(3) + ...
            weight_distribution(j)*Moment.sprung + Moment.rearunsprung);
        LDT_val(j, i) = LDT.front_lat / (LDT.front_lat+LDT.rear_lat);
    
    end
end

LSD = front.k_roll ./ (front.k_roll+rear.k_roll);

[WD_mesh, LSD_mesh] = meshgrid(weight_distribution*100, LSD*100);
contourf(WD_mesh,LSD_mesh,LDT_val*100,10)
xlabel('WD% front bias')
ylabel('RD% front bias')
title('LLTD Distribution% front bias')
colorbar

% figure (1)
% plot(1:100, LDT_val);
% xlabel('iteration');
% ylabel('Lateral Load Transfer Distribution');
% title('Lateral Load Transfer Distribution vs iteration');
% grid on;
