%%
clc;
clear;
%%
psi_list = {1.0, 1.5, 2.0};
color_list = {'b','r','g'};

figure(2);
[force1, max_dis1] = import_data(1.0);
[force2, max_dis2] = import_data(1.5);
[force3, max_dis3] = import_data(2.0);


y = psi_plane_1(max_dis1);
scatter(max_dis1,y,color_list{1});

hold on;

y = psi_plane_2(max_dis2);
scatter(max_dis2,y,color_list{2});


y = psi_plane_3(max_dis3);
scatter(max_dis3,y,color_list{3});

y_std = standard_model(max_dis1);
scatter(max_dis1, y_std);

legend('1.0PSI','1.5PSI','2.0PSI','Hertz Model');
title_name = sprintf("Force-indentation results upon contact with plane");
ylabel('F_z [N]'); xlabel("\delta_{max} [m]");
title(title_name);
grid;

%%
function [y] = standard_model(x)
    y = 1100 * x.^(3/2);
end


%%
function [force1, max_dis1] = import_data(psi)
    file_name = sprintf("%0.1fpsi.csv",psi);
    fileID = fopen(file_name,'r');
    formatSpec = '%f, %f%[^\n\r]';
    dataArray = textscan(fileID,formatSpec);
    force_0 = dataArray{1}; 
    max_dis_0 = dataArray{2};
    force_limit = 0; 
    force1 = []; max_dis1 = []; 
    for i = 1:size(force_0,1)
         if max_dis_0(i) < 0.04
             force1 = [force1; force_0(i)];
             max_dis1 = [max_dis1 ;max_dis_0(i)];
         end
             
    end
    fclose(fileID);
end