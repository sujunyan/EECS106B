function [tau] = find_tau(u, t, alpha, gamma, tau0, dtau0)
%find_tau integrates tau given u
%   u is a vector of pwm inputs (with the same dimension as t)
%   t is a vector of timesteps when data was collected
%   alpha is the input gain
%   gamma is the input inertia
%   tau0 is the initial torque (probably zero)
%   dtau0 is the initial jerk (probably zero)

tau = zeros(size(t));
tau_ode_0 = [tau0; dtau0];
tau(1) = tau_ode_0(1);
tStart = tic;
for index = 2:numel(t)
    tau_ode_dynamics = @(time, tau_ode) tau_dynamics(tau_ode(1), ...
                                            tau_ode(2), u(index-1), ...
                                            gamma, alpha);

    [~, tau_out] = ode45(tau_ode_dynamics, [t(index-1), t(index)], tau_ode_0);
    
    tau(index) = tau_out(end, 1);
    tau_ode_0 = tau_out(end, :);
end
origin_used_t = toc(tStart);
tStart = tic;
%% other way to find tau
%n = size(t,1);
% tau_ode_0 = [tau0; dtau0];
% [~,tautau] = ode45(@tau_ode_dynamics0, t, tau_ode_0);
% now_used_t = toc(tStart);
% fprintf("In find tau: Difference between two methods %e origin method uses %f now uses %f\n"...
%             ,norm(tautau(:,1)-tau),origin_used_t,now_used_t);
%     function uu = tau_ode_dynamics0(time, tau_ode)
%         persistent index0;
%         if isempty(index0)
%             index0 = 2;
%         end
%         now_t = t(index0);
%         while(now_t < time && index0 <= n)
%             index0 = index0 + 1;
%             now_t = t(index0);
%         end
%         % map from t to u
%         u0 = u(index0-1);
%         uu = tau_dynamics(tau_ode(1),tau_ode(2),u0,gamma,alpha);
%     end
% 


end