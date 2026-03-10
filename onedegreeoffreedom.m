%% 1/8 car model, simplest model of body motion: Appendix C
% assumption that the unsprung mass is insignificant compared to the sprung
% mass of each corner.
clear; clc

%% Parameters
param;
front.cs = 600; % [Ns/m]
rear.cs = 1600;

%% Damping Coeff. and Natural Frequency
sus.front.axle      = front;
sus.front.unsprung  = frontunsprung;
sus.front.sprung    = frontsprung;

sus.rear.axle       = rear;
sus.rear.unsprung   = rearunsprung;
sus.rear.sprung     = rearsprung;

susp = fieldnames(sus);
results = struct();

for j = 1:length(susp)
    
    axl = sus.(susp{j}).axle;
    us  = sus.(susp{j}).unsprung;
    s   = sus.(susp{j}).sprung;

    results.(susp{j}).damping_ratio = ...
        axl.cs / (2*sqrt(axl.kw/s.m));
    results.(susp{j}).omega_natural = sqrt(axl.kw/s.m);
    
end
%