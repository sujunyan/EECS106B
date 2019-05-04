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

%cost = @(x) cost_function2(x(1), x(2), x(3), x(4), t, u, q, q0, dq0, tau0, dtau0);
cost = @(x) cost_function2(x(1), x(2), x(3), x(4), x(5), t, u, q, q0, dq0, tau0, dtau0);
%% do your regression

x0 = [28047.662639397073  0.973171960805  1.019462351266  1.727925441558 0.185597355241];
options = optimoptions(@lsqnonlin,'Display','iter-detailed','OptimalityTolerance',1e-6);



[X, resnorm] = lsqnonlin(cost, x0,[],[],options);
x0 = X;
%% validation for other data sets
% fprintf("Data set \t cost \n");
% file_name = sprintf("data/calibration_data.csv");
% c = calculate_cost(file_name,x0);
% fprintf("%2d \t\t %.6f\n",0,norm(c,inf));
for i = 1:10
    file_name = sprintf("data/data_%d.csv",i);
    c = calculate_cost(file_name,x0);
    fprintf("%2d \t\t %.6f\n",i,norm(c,inf));
    c_list(i) = norm(c,inf);
end
function c = calculate_cost(file_name,x0)
    data = importdata(file_name);
    t = data.time; u = data.right_pwm; 
    q = flex2angle(data.right_flex);
    q0 = q(1); dq0 = 0; tau0 = 0; dtau0 = 0;
    cost = @(x) cost_function2(x(1), x(2), x(3), x(4), x(5), t, u, q, q0, dq0, tau0, dtau0);
    c = cost(x0);
end
