% Quarter Car Vibration Model: Appendix B assume no damping from tyre
%% Parameters
param;
front.cs = 3000; % [Ns/m]
rear.cs = 1600;

% %% Test Critical Damping
% front.c_critical = (front.cs) / (2*sqrt(car.m_sprung*front.Kr));
% rear.c_critical = (rear.cs) / (2*car.m_sprung*rearsprung.omega);

%% Function
susp = {'front','rear'};  % cell array of suspension names
unsprung = {'frontunsprung','rearunsprung'};
results = struct();
for j = 1:length(susp)
    
    % Dynamically get struct
    s = eval(susp{j});  % s = front or rear
    us = eval(unsprung{j});

    % Mass matrix
    M = [car.m_sprung, 0;
         0, us.m];
    
    % Damping matrix
    C = [s.cs, -s.cs;
        -s.cs, s.cs];
    
    % Stiffness matrix
    K = [s.ks, -s.ks;
        -s.ks, s.ks + car.Kt];
    
    % State matrix
    A = [zeros(2) eye(2);
        -M\K       -M\C];
    
    % Eigenvalues and eigenvectors
    [V,D] = eig(A);
    lambda = diag(D);

    % Save eigenvalues and eigenvectors
    results.(susp{j}).lambda = lambda;
    % results.(fields{j}).eigenvectors = V;

    fprintf('=== %s Suspension ===\n', susp{j});
    
    % Loop over complex conjugate pairs
    for k = 1:2:length(lambda)
        sigma = real(lambda(k));
        wd = imag(lambda(k));
    
        wn = sqrt(sigma^2 + wd^2);
        zeta = -sigma/wn;
    
        fprintf('Mode:\n')
        fprintf('wn = %.2f rad/s\n', wn)
        fprintf('zeta = %.3f\n\n', zeta)
    end
end
% cs_range = linspace(500,5000,100);
% omega = linspace(0.1,50,500);   % rad/s sweep
% 
% peak_acc = zeros(size(cs_range));
% 
% for i = 1:length(cs_range)
% 
%     C = [cs_range(i), -cs_range(i);
%         -cs_range(i),  cs_range(i)];
% 
%     for j = 1:length(omega)
% 
%         D = K - omega(j)^2*M + 1i*omega(j)*C;
% 
%         % Road force input (through tire stiffness)
%         F = [0; car.Kt];
% 
%         X = D \ F;
% 
%         body_disp = X(1);
% 
%         body_acc(j) = abs(-omega(j)^2 * body_disp);
%     end
% 
%     peak_acc(i) = max(body_acc);
% end
% 
% plot(cs_range, peak_acc)
% xlabel('Damping coefficient c_s')
% ylabel('Peak body acceleration')