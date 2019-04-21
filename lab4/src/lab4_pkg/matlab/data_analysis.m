clear all
close all
clc

%% Package paths

cur = pwd;
addpath( genpath( [cur, '/gen/' ] ));

%% testing with generated data
% Note the values used are totally made up and shouldn't be used as
% starting points for your actual analysis

alpha = 0.1;
gamma = 0.5;
t = 0:0.01:2;
u = 150*ones(size(t));
tau0 = 0;
dtau0 = 0;
tau = find_tau(u, t, alpha, gamma, tau0, dtau0);

K = 3;
D = 0.4;
q0 = 0;
dq0 = 0;
q = find_q(tau, t, K, D, q0, dq0);


% x = K, D, alpha, gamma

cost = @(x) cost_function(x(1), x(2), x(3), x(4), t, u, q, q0, dq0, tau0, dtau0);


[X, resnorm] = lsqnonlin(cost, [2.6, 0.5, 0.31, 0.6])
% [X, resnorm] = lsqnonlin(cost, [K, D, alpha, gamma])


%% 

clear all
close all
clc

%% Load CSV




%% Generate q(t)




%% do your regression





