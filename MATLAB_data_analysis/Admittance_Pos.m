%% TTK4900 Teknisk kybernetikk - Master thesis
%Mads Johan Laastad
%Spring 2017
clc;
close all;
clear all;
%% Load data from test 1
fileID = fopen('../data/log_library/admittance_control_pos_data/Y_m05_f9_c02');
dim = 55; %time(1), q(6), s(6), etc..
data_format = repmat('%f ', 1, dim);
raw_data = textscan(fileID, data_format); %Remember to delete any incomplete log entries in the final row.
[N, M] = size(raw_data{1,1});
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
fileID1 = fopen('../data/log_library/admittance_control_pos_data/Y_m05_f9_c05');
raw_data1 = textscan(fileID1, data_format); %Remember to delete any incomplete log entries in the final row.
data1 = cell2mat(raw_data1); %Convert cell array
fclose(fileID1);

elapsTime1 = data1(:,1);
q1 = data1(:,8:13);
Forces1 = data1(:, 20:22);
errors_F1 = data1(:, 26:28);

%% Load data from test 3
fileID2 = fopen('../data/log_library/admittance_control_pos_data/Y_m05_f9_c1');
raw_data2 = textscan(fileID2, data_format); %Remember to delete any incomplete log entries in the final row.
data2 = cell2mat(raw_data2); %Convert cell array
fclose(fileID2);

elapsTime2 = data2(:,1);
q2 = data2(:,8:13);
Forces2 = data2(:, 20:22);
errors_F2 = data2(:, 26:28);

%% Load data from test 4
fileID3 = fopen('../data/log_library/admittance_control_pos_data/Y_m05_f1_c03');
raw_data3 = textscan(fileID3, data_format); %Remember to delete any incomplete log entries in the final row.
data3 = cell2mat(raw_data3); %Convert cell array
fclose(fileID3);

elapsTime3 = data3(:,1);
q3 = data3(:,8:13);
Forces3 = data3(:, 20:22);
errors_F3 = data3(:, 26:28);
%% Plot step response
figure('Name','Admittance Controller Position Test');
plot(elapsTime(:),  q(:,1));
hold on;
plot(elapsTime1(:), q1(:,1));
plot(elapsTime2(:), q2(:,1));
plot(elapsTime3(:), q3(:,1));

legend({'c = 0.2','c = 0.5','c = 1', 'f = 1Hz, c = 0.3'}, 'Location', 'southeast', 'Fontsize', 14);
%title(lgd,'Gain parameters','FontSize',15)
title('Admittance Controller - Step Response for Fz','FontSize',15);
xlabel('Time [s]', 'FontSize',15)
ylabel('Position [q]', 'FontSize',15)
grid on;
hold off;

%export_fig ForcePcont -eps -transparent

