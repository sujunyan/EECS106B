% clear all
% close all
% clc

%% Package paths

cur = pwd;
addpath( genpath( [cur, '/gen/' ] ));


%% Load CSV

% q0 = 0;
% dq0 = 0;
data = importdata('data/data_0.csv');
t = data.time; u = data.right_pwm;
q = flex2angle(data.right_flex);
q0 = 0; dq0 = 0; tau0 = 0; dtau0 = 0;



%% Generate q(t)
K = 48;
D = 10;
alpha = 0.07;
gamma = 0.58;

cost = @(x) cost_function(x(1), x(2), x(3), x(4), t, u, q, q0, dq0, tau0, dtau0);

%% do your regression
options = optimoptions(@lsqnonlin,'Display','iter-detailed','OptimalityTolerance',1e-6);
x0 = [K, D, alpha, gamma];
[X, resnorm] = lsqnonlin(cost, x0,[],[],options)

