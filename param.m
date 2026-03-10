%% All Parameters of the FSAE Car
%% Distributions
weight_distribution = 50.95; %front bias

%% Unsprung mass of 1 corner
frontunsprung.m = 12.15;
rearunsprung.m = 13.35;

%% Car/ Sprung (Appendix A: diagram of car parameters)
car.wheelbase = 1.558;
car.track = 1.21;
car.cgh = 0.256;
car.m = 270;

car.b = (car.wheelbase * car.m * weight_distribution/100) / car.m;
car.a = car.wheelbase - car.b;

%% Sprung mass of 1 corner
frontsprung.m = (0.5 * (weight_distribution/100) * car.m) - frontunsprung.m;
rearsprung.m = (0.5 * ((100-weight_distribution)/100) * car.m) - rearunsprung.m;

%% Chassis and Suspension Stiffness 
car.TR = (3047.62 + 2912.586)/2; 

front.ks = 225 * 175.12684; % spring rate
rear.ks = 200 * 175.12684;
car.Kt = (10^3)/0.0122; % tyre radial rate
front.MR = 1.0; % motion ratio of coil over
rear.MR = 1.0;

front.kw = front.ks*front.MR^2; % wheel rate
rear.kw = rear.ks*rear.MR^2;
front.Kr = (front.kw*car.Kt) / (front.kw+car.Kt); %ride rate
rear.Kr = (rear.kw*car.Kt) / (rear.kw+car.Kt);

%% Natural Frequency and Critical Damping
% frontsprung.omega = (1/(2*pi)) * sqrt(front.Kr/car.m_sprung);
% rearsprung.omega =  (1/(2*pi)) * sqrt(rear.Kr/car.m_sprung);

