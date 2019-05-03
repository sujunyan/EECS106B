% clear all
% close all
% clc

%% Package paths

cur = pwd;
addpath( genpath( [cur, '/gen/' ] ));


%% Load CSV

data = importdata('data/data_0.csv');
t = data.time; u = data.right_pwm;
q = flex2angle(data.right_flex);
q0 = q(1); dq0 = 0; tau0 = 0; dtau0 = 0;




%% Generate q(t)

cost = @(x) cost_function(x(1), x(2), x(3), x(4), t, u, q, q0, dq0, tau0, dtau0);


%% do your regression
x0 = [2020.799291556778, 778.459224920680, 3.077191660625, 0.034022577055]; % near-optimal solution
% options = optimoptions(@lsqnonlin,'Display','iter-detailed','OptimalityTolerance',1e-6);
% [X, resnorm] = lsqnonlin(cost, x0,[],[],options)
K=x0(1); D=x0(2); alpha=x0(3); gamma=x0(4); 
tau = find_tau(u, t, alpha, gamma, tau0, dtau0);
qhat = find_q(tau, t, K, D, q0, dq0);
plot(t,q)
hold on
plot(t,qhat)
legend("measured q","estimated q")
%% validation for other data sets
% fprintf("Data set \t cost \n");
% file_name = sprintf("data/calibration_data.csv");
% c = calculate_cost(file_name,x0);
% fprintf("%2d \t\t %.6f\n",0,norm(c,inf));
% for i = 1:10
%     file_name = sprintf("data/data_%d.csv",i);
%     c = calculate_cost(file_name,x0);
%     fprintf("%2d \t\t %.6f\n",i,norm(c,inf));
%     c_list(i) = norm(c,inf);
% end
% function c = calculate_cost(file_name,x0)
%     data = importdata(file_name);
%     t = data.time; u = data.right_pwm; 
%     q = flex2angle(data.right_flex);
%     q0 = q(1); dq0 = 0; tau0 = 0; dtau0 = 0;
%     cost = @(x) cost_function(x(1), x(2), x(3), x(4), t, u, q, q0, dq0, tau0, dtau0);
%     c = cost(x0);
% end
