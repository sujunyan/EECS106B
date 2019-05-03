
% C1 C2 D alpha gamma 
x0 = [2020.799291556778, 778.459224920680, 3.077191660625, 0.034022577055];
%x0 = [9.125583714058 	 -1110.627346825599 	 0.000715435708 	 12.815304225003 	 1322.139801653447];
length = 50; % time duration for one step response file 0 
%C1 = x0(1);
%C2 = x0(2);
%D = x0(3);
%alpha = x0(4);
%gamma = x0(5);

K = x0(1);
D = x0(2);
alpha = x0(3);
gamma = x0(4);

%%
file_name = sprintf("data/data_%d.csv",0);
data = importdata(file_name);
t = data.time(1:length);
u = data.right_pwm;
tan_angle = abs(data.tip_pos_x - data.base_pos_x) ./ ...
            abs(data.tip_pos_y - data.base_pos_y);
q = atan(tan_angle);
q = q(1:length);

%%

q0 = q(1); dq0 = 0; tau0 = 0; dtau0 = 0;
tau = find_tau(u, t, alpha, gamma, tau0, dtau0);
%qhat = find_q2(tau, t, C1, C2, D, q0, dq0);
qhat = find_q(tau, t, K, D, q0, dq0);
%%
hold on;
figure(1)
plot(t,q);
plot(t,qhat);
