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
%x0 = [2020.799291556778, 778.459224920680, 3.077191660625, 0.034022577055]; % near-optimal solution
% 4.61
%x0 = [2.801818877269 	 -1110.578618870909 	 -0.105775564549 	 9.435703659073 	 1322.188538153578] ;
x0 = [9.125583714058 	 -1110.627346825599 	 0.000715435708 	 12.815304225003 	 1322.139801653447];
%x0 = [9.125583714058 	 -1110.627346825599 	 0.000715435708 	 3.077191660625        0.034022577055];
%x0=[6.859304576305 	 -1110.627358027546 	 0.000309261520
%13.870559872646 	 1322.119356989924];
options = optimoptions(@lsqnonlin,'Display','iter-detailed','OptimalityTolerance',1e-6);
%cost_function used 0.516728 s
% cost_function used 0.111822 s
%9.125583714058 	 -1110.627346825599 	 0.000715435708 	 12.815304225003 	 1322.139801653447
% 1 		 0.249418
%cost_function used 0.109316 s
%9.125583714058 	 -1110.627346825599 	 0.000715435708 	 12.815304225003 	 1322.139801653447
% 2 		 0.313700
%cost_function used 0.110577 s
%9.125583714058 	 -1110.627346825599 	 0.000715435708 	 12.815304225003 	 1322.139801653447
% 3 		 0.311426
%cost_function used 0.112023 s
%9.125583714058 	 -1110.627346825599 	 0.000715435708 	 12.815304225003 	 1322.139801653447
% 4 		 0.364456
%cost_function used 0.110353 s
%9.125583714058 	 -1110.627346825599 	 0.000715435708 	 12.815304225003 	 1322.139801653447
% 5 		 0.247496
%cost_function used 0.113004 s
%9.125583714058 	 -1110.627346825599 	 0.000715435708 	 12.815304225003 	 1322.139801653447
% 6 		 0.412186
%cost_function used 0.113957 s
%9.125583714058 	 -1110.627346825599 	 0.000715435708 	 12.815304225003 	 1322.139801653447
% 7 		 0.287926
%cost_function used 0.114834 s
%9.125583714058 	 -1110.627346825599 	 0.000715435708 	 12.815304225003 	 1322.139801653447
% 8 		 0.329005
%cost_function used 0.110855 s
%9.125583714058 	 -1110.627346825599 	 0.000715435708 	 12.815304225003 	 1322.139801653447
% 9 		 0.192403
%cost_function used 0.112137 s
%9.125583714058 	 -1110.627346825599 	 0.000715435708 	 12.815304225003 	 1322.139801653447
%10 		 0.4189981

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
