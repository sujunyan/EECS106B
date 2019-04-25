%% load the calibration data and generate the function flex2angle
cali_data = importdata('data/calibration_data.csv',232,inf);
tan_angle = abs(cali_data.tip_pos_x - cali_data.base_pos_x) ./ ...
            abs(cali_data.tip_pos_y - cali_data.base_pos_y);
angle = atan(tan_angle);
flex = cali_data.right_flex;

n = size(flex,1);
R2 = 1; % should be R2 = 47k in real, but doesnot matter
R_flex = (1024*ones(n,1)./flex - ones(n,1) ) * R2;

f = fit(R_flex,angle,'poly1');
% plot(f,R_flex,angle);

%% Generate the matlab function flex2angle
syms flex;
Rf = ((1024/flex)-1)*R2;
matlabFunction(f.p1 * Rf + f.p2, 'File', 'gen/flex2angle');
