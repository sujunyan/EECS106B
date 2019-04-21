function [error] = cost_function(K, D, alpha, gamma, t, u, q, q0, dq0, tau0, dtau0)
%cost_function Cost function to be minimized in the linear regression
%   K is the stiffness (linear elastic model)
%   D is the damping
%   alpha is the input gain
%   gamma is the input inertia
%   t is a vector of timesteps when data was collected
%   u is a vector of pwm inputs (with the same dimension as t)
%   q is a vector of angle measurements (with the same dimenstion as t)
%   q0 is the initial angle (probably zero)
%   dq0 is the initial angular velocity (probably zero)
%   tau0 is the initial torque (probably zero)
%   dtau0 is the initial jerk (probably zero)

% Integrate to find tau as a function of time
tau = find_tau(u, t, alpha, gamma, tau0, dtau0);

% Integrate to find q as a function of time
qhat = find_q(tau, t, K, D, q0, dq0);

error = qhat - q;

end

