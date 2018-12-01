%% TTK4900 Teknisk kybernetikk - Master thesis
%Shahrukh Khan
%Fall 2018
clc;
close all;
clear all;
%% Load data from file
fileID = fopen('../data/log_library/gravity_compensation_testing/w_out_vector/tool_bias_Rx');
dim = 58; %time(1), q(6), s(6), etc..
data_format = repmat('%f ', 1, dim);
raw_data = textscan(fileID, data_format); %Remember to delete any incomplete log entries in the final row.
[N, M] = size(raw_data{1,1});
data = cell2mat(raw_data); %Convert cell array
fclose(fileID);

fileID1 = fopen('../data/log_library/gravity_compensation_testing/w_out_vector/tool_bias_Ry');
%dim = 46; %time(1), q(6), s(6), etc..
%data_format = repmat('%f ', 1, dim);
raw_data1 = textscan(fileID1, data_format); %Remember to delete any incomplete log entries in the final row.
[N1, M1] = size(raw_data1{1,1});
data1 = cell2mat(raw_data1); %Convert cell array
fclose(fileID1);

fileID2 = fopen('../data/log_library/gravity_compensation_testing/w_out_vector/tool_bias_Rz');
%dim = 46; %time(1), q(6), s(6), etc..
%data_format = repmat('%f ', 1, dim);
raw_data2 = textscan(fileID2, data_format); %Remember to delete any incomplete log entries in the final row.
[N2, M2] = size(raw_data2{1,1});
data2 = cell2mat(raw_data2); %Convert cell array
fclose(fileID2);

% Distribute data into usefull matricies
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
bias_tool_TF = data(:, 44:46);

elapsTime1 = data1(:,1);
Forces1 = data1(:, 20:22);
elapsTime2 = data2(:,1);
Forces2 = data2(:, 20:22);

%% Plot force sensor input
figure('Name','Tool bias test force sensor data');
subplot(3,1,1)
plot(elapsTime,Forces(:,1),elapsTime,Forces(:,2),elapsTime,Forces(:,3))
legend('F_x','F_y','F_z')
title('Tool bias test - Rotation in X, Y, Z-axis');
ylabel('Rx - Force sensor input [N]')
grid on;

subplot(3,1,2)
plot(elapsTime1,Forces1(:,1),elapsTime1,Forces1(:,2),elapsTime1,Forces1(:,3))
legend('F_x','F_y','F_z')
ylabel('Ry - Force sensor input [N]')
grid on;

subplot(3,1,3)
plot(elapsTime2,Forces2(:,1),elapsTime2,Forces2(:,2),elapsTime2,Forces2(:,3))
legend('F_x','F_y','F_z')
xlabel('Elapsed time [s]')
ylabel('Rz - Force sensor input [N]')
grid on;
