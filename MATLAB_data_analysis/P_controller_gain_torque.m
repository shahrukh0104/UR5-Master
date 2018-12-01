%% TTK4900 Teknisk kybernetikk - Master thesis
%Mads Johan Laastad
%Spring 2017
clc;
close all;
clear all;
%% Load data from test 1
fileID = fopen('datalogs/controller_gain_estimation_torque/P-controller-torque/Kp_T=0.10');
dim = 55; %time(1), q(6), s(6), etc..
data_format = repmat('%f ', 1, dim);
raw_data = textscan(fileID, data_format); %Remember to delete any incomplete log entries in the final row.
%[N, M] = size(raw_data{1,1});
data = cell2mat(raw_data); %Convert cell array
fclose(fileID);

%All data parameters avaliable from logfile
elapsTime = data(:,1);
speed = data(:, 2:7);
q = data(:, 8:13);
rawFTdata = data(:, 14:19);
Forces = data(:, 20:22);
Torques = data(:, 23:25);
errors_F = data(:, 26:28);
errors_T = data(:, 29:31);
u_F = data(:, 32:34);
u_T = data(:, 35:37);
biasFT = data(:, 38:40);
biasForce = data(:, 41:43);
tool_coordinates = data(:, 44:46);

%% Load data from test 2
fileID1 = fopen('datalogs/controller_gain_estimation_torque/P-controller-torque/Kp_T=0.25');
raw_data1 = textscan(fileID1, data_format); %Remember to delete any incomplete log entries in the final row.
data1 = cell2mat(raw_data1); %Convert cell array
fclose(fileID1);

elapsTime1 = data1(:,1);
Torques1 = data1(:, 23:25);
errors_F1 = data1(:, 26:28);

%% Load data from test 3
fileID2 = fopen('datalogs/controller_gain_estimation_torque/P-controller-torque/Kp_T=0.50');
raw_data2 = textscan(fileID2, data_format); %Remember to delete any incomplete log entries in the final row.
data2 = cell2mat(raw_data2); %Convert cell array
fclose(fileID2);

elapsTime2 = data2(:,1);
Torques2 = data2(:, 23:25);
errors_F2 = data2(:, 26:28);

%% Load data from test 4
fileID3 = fopen('datalogs/controller_gain_estimation_torque/P-controller-torque/Kp_T=0.75');
raw_data3 = textscan(fileID3, data_format); %Remember to delete any incomplete log entries in the final row.
data3 = cell2mat(raw_data3); %Convert cell array
fclose(fileID3);

elapsTime3 = data3(:,1);
Torques3 = data3(:, 23:25);
errors_F3 = data3(:, 26:28);

%% Plot step response
figure('Name','Step response test');
plot(elapsTime(:), Torques(:,3));
hold on;
plot(elapsTime(:), Torques(:,3));
plot(elapsTime1(:), Torques1(:,3));
plot(elapsTime2(:), Torques2(:,3));
plot(elapsTime3(:), Torques3(:,3));
plot(elapsTime(:), ones(size(Torques))*1);
hold off;
lgd = legend('\fontsize{15} Referance torque','\fontsize{15} K_p\_T = 0.10', '\fontsize{15} K_p\_T = 0.25', '\fontsize{15} K_p\_T = 0.50', '\fontsize{15} K_p\_T = 0.75','Location','east');
title(lgd,'\fontsize{15} Gain parameters')
title('\fontsize{15} P-controller - Step response for Tz');
xlabel('\fontsize{15} Time [s]')
ylabel('\fontsize{15} Torque [Nm]')
grid on;