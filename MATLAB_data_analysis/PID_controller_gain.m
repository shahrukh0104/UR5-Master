%% TTK4900 Teknisk kybernetikk - Master thesis
%Mads Johan Laastad
%Spring 2017
clc;
close all;
clear all;
%% Load data from test 1
fileID = fopen('../data/log_library/controller_gain_estimation_force/PID-controller/Kp=0.0075_Ki=0.000001_Kd=0.0000075');
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
fileID1 = fopen('../data/log_library/controller_gain_estimation_force/PID-controller/Kp=0.0075_Ki=0.000005_Kd=0.0000075');
raw_data1 = textscan(fileID1, data_format); %Remember to delete any incomplete log entries in the final row.
data1 = cell2mat(raw_data1); %Convert cell array
fclose(fileID1);

elapsTime1 = data1(:,1);
Forces1 = data1(:, 20:22);
errors_F1 = data1(:, 26:28);

%% Load data from test 3
fileID2 = fopen('../data/log_library/controller_gain_estimation_force/PID-controller/Kp=0.00025_Ki=0.00025_Kd=0.0075');
raw_data2 = textscan(fileID2, data_format); %Remember to delete any incomplete log entries in the final row.
data2 = cell2mat(raw_data2); %Convert cell array
fclose(fileID2);

elapsTime2 = data2(:,1);
Forces2 = data2(:, 20:22);
errors_F2 = data2(:, 26:28);

%% Load data from test 4
fileID3 = fopen('../data/log_library/controller_gain_estimation_force/PID-controller/Kp=0.005_Ki=0.000001_Kd=0.00075');
raw_data3 = textscan(fileID3, data_format); %Remember to delete any incomplete log entries in the final row.
data3 = cell2mat(raw_data3); %Convert cell array
fclose(fileID3);

elapsTime3 = data3(:,1);
Forces3 = data3(:, 20:22);
errors_F3 = data3(:, 26:28);

%% Load data from test 5
fileID4 = fopen('../data/log_library/controller_gain_estimation_force/PID-controller/Kp=0.0077_Ki=0.00015_Kd=0.08');
raw_data4 = textscan(fileID3, data_format); %Remember to delete any incomplete log entries in the final row.
data4 = cell2mat(raw_data4); %Convert cell array
fclose(fileID3);

elapsTime4 = data4(:,1);
Forces4 = data4(:, 20:22);
errors_F4 = data4(:, 26:28);
%% Plot step response
figure('Name','Step response test');
line([0 5],[2 2], 'Color','red')
hold on;
plot(elapsTime(:), -Forces(:,3));
plot(elapsTime1(:), -Forces1(:,3));
plot(elapsTime2(:), -Forces2(:,3));
plot(elapsTime3(:), -Forces3(:,3));
plot(elapsTime4(:), -Forces4(:,3));


legend({'Reference', 'K_p=0.0075, K_i=0.000001, K_d=0.0000075', 'K_p=0.0075, K_i=0.000005, K_d=0.0000075', 'K_p=0.00025, K_i=0.00025, K_d=0.0075', 'K_p=0.005, K_i=0.000001, K_d=0.00075', 'K_p=0.0077, K_i=0.00015, K_d=0.08'}, 'Location', 'southeast', 'Fontsize', 14);
%title(lgd,'Gain parameters','FontSize',15)
title('PID-controller - Step response for Fz','FontSize',15);
xlabel('Time [s]', 'FontSize',15)
ylabel('Force [N]', 'FontSize',15)
grid on;
hold off;

%export_fig ForcePIDcont -eps -transparent