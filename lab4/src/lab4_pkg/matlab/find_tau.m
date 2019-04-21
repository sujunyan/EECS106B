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

for index = 2:numel(t)
    tau_ode_dynamics = @(time, tau_ode) tau_dynamics(tau_ode(1), ...
                                            tau_ode(2), u(index-1), ...
                                            gamma, alpha);

    [~, tau_out] = ode45(tau_ode_dynamics, [t(index-1), t(index)], tau_ode_0);
    
    tau(index) = tau_out(end, 1);
    tau_ode_0 = tau_out(end, :);
end





