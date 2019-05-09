eco = 50; psi = "1";
eco_list = [10,50]; psi_list = {"1","1.5","2"};
color_list = {'b','r','g'};
for i = 1:2
    subplot(1,2,i);
    sum_slope = 0;
    for j = 1:3
        eco = eco_list(i); psi = psi_list{j};
        [max_dis, z] = import_data(eco,psi);
        f = fit(z,max_dis,'poly1');
        plot(f,color_list{j});
        sum_slope = sum_slope + f.p1;
        hold on
    end
    legend('1psi','1.5psi','2psi');
    slope = sum_slope/3; estimated_stiffness =  1/(1 - slope) - 1;
    title_name = sprintf('ecofelx%d, average slope=%f, estimated stiffness=%f'...
                    ,eco_list(i),slope,estimated_stiffness);
    ylabel('displacement of membrane');xlabel('arm z movement');
    title(title_name);
end

function [max_dis, z] = import_data(eco,psi)
    file_name = sprintf("eco%d_%spsi.csv",eco,psi);
    fileID = fopen(file_name,'r');
    formatSpec = '%f, %f%[^\n\r]';
    dataArray = textscan(fileID,formatSpec);
    max_dis0 = dataArray{1}; z0 = dataArray{2};
    contact_z = 0.09;
    max_dis = []; z = [];
    for i = 1:size(max_dis0,1)
        if(z0(i) < contact_z)
            max_dis = [max_dis; max_dis0(i)];
            z = [z ;z0(i)];
        end
    end
    z = contact_z * ones(size(z,1),1) - z;
    fclose(fileID);
end