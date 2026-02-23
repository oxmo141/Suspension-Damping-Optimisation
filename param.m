%All Parameters of the FSAE Car
%% Distributions
weight_distribution = 50.95; %front bias

%% Unsprung mass
frontunsprung.m = 12.15;
rearunsprung.m = 13.35;

%% Car/ Sprung (Appendix A: diagram of car parameters)
car.wheelbase = 1.558;
car.track = 1.21;
car.cgh = 0.256;
car.m = 270;

car.a = car.wheelbase/2 - car.wheelbase*(weight_distribution/100 - 0.5);
car.b = car.wheelbase-car.a;

% Assume sprung corner weight is equally distributed
car.m_sprung = (car.m - 2*frontunsprung.m - 2*rearunsprung.m)/4;

%% Suspension Stiffness 
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
frontsprung.omega = (1/(2*pi)) * sqrt(front.Kr/car.m_sprung);
rearsprung.omega =  (1/(2*pi)) * sqrt(rear.Kr/car.m_sprung);