%% 1/8 car model, simplest model of body motion: Appendix C
% assumption that the unsprung mass is insignificant compared to the sprung
% mass of each corner.
clear; clc

% Parameters

param;

% Masses
mf = frontunsprung.m;
mr = rearunsprung.m;


% Suspension stiffness
kf = front.kw + car.Kt;
kr = rear.kw + car.Kt;

% Undamped

front.omega_n = sqrt(kf/mf);
rear.omega_n = sqrt(kr/mr);

fprintf(['front natural frequency: %3f rad/s\n rear natural frequency: %3f rad/s\n'], ...
        front.omega_n, rear.omega_n);

% Damped 

function dxdt = eom(~, x)

param;

% Masses
mf = frontunsprung.m;
mr = rearunsprung.m;

% Damping
cf = 4000;
cr = 2300;

% Suspension stiffness
kf = front.kw + car.Kt;
kr = rear.kw + car.Kt;

% Tire stiffness
k_t = car.Kt;

% Road input
%r = 0.05*sin(2*pi*1*t); % sinusoidal
r= 0.03; % step

% Tire forces
Ff = k_t*(r - x(1));
Fr = k_t*(r - x(3));

dxdt = zeros(4,1);

% Front
dxdt(1) = x(2);
dxdt(2) = (Ff - cf*x(2) - kf*x(1))/mf;

% Rear
dxdt(3) = x(4);
dxdt(4) = (Fr - cr*x(4) - kr*x(3))/mr;

end

tspan = [0 10];

% initial conditions
x0 = [0 0 0 0];

[t,x] = ode45(@eom,tspan,x0);

figure
plot(t,x(:,1),'b',t,x(:,3),'r')
xlabel('Time')
ylabel('Displacement')
legend('Front','Rear')
title('Front and Rear Corner Mass Response')
grid on

% function dxdt = eom(t,x,cf,cr)
% 
% param;
% 
% mf = frontunsprung.m;
% mr = rearunsprung.m;
% 
% kf = front.Kr;
% kr = rear.Kr;
% 
% k_t = car.Kt;
% 
% r = 0.05*sin(2*pi*1*t);
% 
% Ff = k_t*(r - x(1));
% Fr = k_t*(r - x(3));
% 
% dxdt = zeros(4,1);
% 
% % Front
% dxdt(1) = x(2);
% dxdt(2) = (Ff - cf*x(2) - kf*x(1))/mf;
% 
% % Rear
% dxdt(3) = x(4);
% dxdt(4) = (Fr - cr*x(4) - kr*x(3))/mr;
% 
% end
% 
% function J = objective(p)
% 
% cf = p(1);
% cr = p(2);
% 
% tspan = [0 10];
% x0 = [0 0 0 0];
% 
% [t,x] = ode45(@(t,x) eom(t,x,cf,cr),tspan,x0);
% 
% % overshoot calculation
% front_os = max(x(:,1));
% rear_os  = max(x(:,3));
% 
% J = front_os^2 + rear_os^2;
% 
% end
% 
% % p0 = [2000 1600];   % initial guess
% % 
% % p_opt = fminsearch(@objective,p0);
% % 
% % cf_opt = p_opt(1)
% % cr_opt = p_opt(2)
% % 
% % [t,x] = ode45(@(t,x) eom(t,x,cf_opt,cr_opt),[0 10],[0 0 0 0]);
% % 
% % figure
% % plot(t,x(:,1),'b',t,x(:,3),'r')
% % legend('Front','Rear')
% % xlabel('Time')
% % ylabel('Displacement')
% % title('Optimized Damping Response')
% % grid on
% 
% cf_range = linspace(500,5000,50);
% cr_range = linspace(500,5000,50);
% 
% for i = 1:length(cf_range)
%     for j = 1:length(cr_range)
% 
%         [t,x] = ode45(@(t,x) eom(t,x,cf_range(i),cr_range(j)),[0 10],[0 0 0 0]);
% 
%         overshoot(i,j) = max(x(:,1));
% 
%     end
% end
% 
% contour(cf_range,cr_range,overshoot')
% xlabel('Front damping')
% ylabel('Rear damping')
% title('Overshoot map')