function [tau_out] = tau_dynamics(tau, dtau, u, gamma, alpha)
%tau_dynamics dynamics of the pump
%   tau is the current torque
%   dtau the current jerk
%   gamma is the system inertia
%   alpha is the system gain
    ddtau = (alpha*u - 2*gamma*dtau - tau)/gamma^2;
    tau_out = [dtau; ddtau];
end