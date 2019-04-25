function [q] = find_q(tau, t, K, D, q0, dq0)
%find_tau integrates tau given u
%   tau is a vector of torque inputs (with the same dimension as t)
%   t is a vector of timesteps when data was collected
%   K is the stiffness gain
%   D is the damping
%   q0 is the initial angle (probably zero)
%   dq0 is the initial angular velocity (probably zero)

q = zeros(size(t));
q_ode_0 = [q0; dq0];
q(1) = q_ode_0(1);

%% original method
tStart = tic;
for index = 2:numel(t)
    q_ode_dynamics = @(time, q_ode) dynamics_gen(q_ode(1),q_ode(2),K,D,tau(index));

    [~, q_out] = ode45(q_ode_dynamics, [t(index-1), t(index)], q_ode_0);
    
    q(index) = q_out(end, 1);
    q_ode_0 = q_out(end, :);
end
origin_used_t = toc(tStart);
tStart = tic;
%% a faster way to find q
% n = size(t,1);
% q_ode_0 = [q0; dq0];
% [~,qq] = ode45(@q_ode_dynamics0, t, q_ode_0);
% now_used_t = toc(tStart);
% fprintf("Difference between two methods %e origin method uses %f now uses %f\n"...
%             ,norm(qq(:,1)-q),origin_used_t,now_used_t);
%     function u = q_ode_dynamics0(time, q_ode)
%         persistent index0;
%         if isempty(index0)
%             index0 = 2;
%         end
%         now_t = t(index0);
%         while(now_t < time && index0 <= n)
%             index0 = index0 + 1;
%             now_t = t(index0);
%         end
%         % map from t to tau
%         tau0 = tau(index0-1);
%         u = dynamics_gen(q_ode(1),q_ode(2),K,D,tau0);
%     end
end






