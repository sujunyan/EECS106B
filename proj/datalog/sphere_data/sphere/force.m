clc;
clear;
%%
psi_list = {1.0, 1.5, 2.0};
color_list = {'b','r','g'};
para_list = ["max_dis1","mean_dis1"];
slope = [];
shape = "sphere";

figure(1);
   
psi = psi_list{1};
[force1, max_dis1, mean_dis1,full_area1] = import_data(psi,shape);

y = psi_sphere_1(max_dis1);
slope = (psi_sphere_1(0.02)-psi_sphere_1(0.015))/(0.02-0.015);
disp(slope);
scatter(max_dis1,y,color_list{1});

hold on;
psi = psi_list{2};
[force2, max_dis2, mean_dis2,full_area2] = import_data(psi,shape);
y = psi_sphere_2(max_dis2);
scatter(max_dis2,y,color_list{2});

psi = psi_list{3};
[force3, max_dis3, mean_dis3,full_area3] = import_data(psi,shape);
y = psi_sphere_3(max_dis3);
scatter(max_dis3,y,color_list{3});

y_std = standard_model(max_dis1);
scatter(max_dis1, y_std);


legend('1.0PSI','1.5PSI','2.0PSI','Hertz Model');

xlim([0.01 0.05]);
ylim([0 16]);
title_name = sprintf("Force-indentation results upon contact with spheres of diameter 0.004m");
ylabel('F_z [N]'); xlabel("\delta_{max} [m]");
title(title_name);
grid;


%%
function [y] = standard_model(x)
    y = 1100 * x.^(3/2);
end




   

%%

function [force1, max_dis1, mean_dis1,full_area] = import_data(psi,shape)
    file_name = sprintf("%0.1fpsi_%s.csv",psi,shape);
    fileID = fopen(file_name,'r');
    formatSpec = '%f, %f, %f, %f, %f%[^\n\r]';
    dataArray = textscan(fileID,formatSpec);
    force_0 = dataArray{1}; 
    max_dis_0 = dataArray{5};
    contact_area = dataArray{3};
    full_area_0 = dataArray{2};
    mean_dis_0 = dataArray{4};
    force_limit = 0; contact_area_limit = 1e-5;
    force1 = []; max_dis1 = []; mean_dis1 = [];full_area = [];
    for i = 1:size(force_0,1)
         if max_dis_0(i) < 0.04
             force1 = [force1; force_0(i)];
             max_dis1 = [max_dis1 ;max_dis_0(i)];
             mean_dis1 = [mean_dis1 ; mean_dis_0(i)];
             full_area = [full_area;full_area_0(i)];
         end
             
    end
    fclose(fileID);
end