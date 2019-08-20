%  Graph Plot Code
%
%  This code generates graphs from the measurement file created by the Arduino nano saved as .txt file.
%
% // Created: BJGW DU PLESSIS
% // Student Number: 18989780
% // Modified: 2019/07/16
% // Version: 0.1

clear all;
close all;                   
clc


% Read in .tx.t file to array
%IMU_data=dlmread('complementary_filter_pitch_a01.csv');
% Mag_data=dlmread('raw_mag_outside_balcony.csv');
% Mag_data=dlmread('coetzenburg_mag_3.csv');
gndtest_data = dlmread('2.csv');

% Plot Airspeed Data
airspeed = gndtest_data(:,7);
[n,p] = size(airspeed);
t= 1:n;
figure(1);
plot(t,airspeed);



% Copy Accelerometer data to inidividual axis array
% a_x=IMU_data(:,1);
% a_y=IMU_data(:,2);
% a_z=IMU_data(:,3);

% Copy Gyroscope data to inidividual axis array
% g_x=IMU_data(:,1);
% g_y=IMU_data(:,2);
% g_z=IMU_data(:,3);

% Plot Accelerometer Graphs
% [n,p] = size(a_x);
% t= 1:n;
% figure(1);
% plot(t,a_x);
% hold on
% plot(t,a_y);
% plot(t,a_z);
% legend('a_x','a_y','a_z');
% xlabel('Samples'), ylabel('Acceleration (g)')
% title('Accelerometer Data ')
% hold off

% Plot Gyroscope Graphs and get mean
% [n,p] = size(g_x);
% t= 1:n;
% figure(1);
% plot(t,g_x);
% hold on
% plot(t,g_y);
% plot(t,g_z);
% legend('g_x','g_y','g_z');
% xlabel('Samples'), ylabel('Angular velocity (dps)')
% title('Gyroscope Data ')
% hold off
% 
% Mean_gx = mean(g_x)
% Mean_gy = mean(g_y)
% Mean_gz = mean(g_z)

% Plot Yaw Graph
% figure(3);
% plot(t,IMU_data(:,7));
% xlabel('Samples'), ylabel('Yaw (degrees)')
% title('Yaw ')

% Copy Magnetometer data to inidividual axis array
% m_x=Mag_data(:,1);
% m_y=Mag_data(:,2);
% m_z=Mag_data(:,3);
% 
% % m_x2=Mag_data2(:,1);
% % m_y2=Mag_data2(:,2);
% % m_z2=Mag_data2(:,3);
% % % 
% % Plot Magnetometer 3D Scatter Plot 
% figure(1);
% scatter3(m_x,m_y,m_z);

% figure(2);
% scatter3(m_x2,m_y2,m_z2);

%#######################################################################%
% Copy Filtered and Non Filtered Roll test data data to inidividual axis array
% roll_nf=IMU_data(:,2);
% roll_f=IMU_data(:,4);

% Plot Filtered and Non Filtered Roll test Plot 
% [n,p] = size(roll_nf);
% t= 1:n;
% figure(1);
% plot(t,roll_nf);
% hold on
% plot(t,roll_f);
% legend('roll_nf','roll_f');
% xlabel('Samples'), ylabel('Roll (degrees)')
% title('Complementary Filter on Roll data ')
% hold off

% 
% Copy Filtered and Non Filtered Pitch test data data to inidividual axis array
% pitch_nf=IMU_data(:,1);
% pitch_f=IMU_data(:,3);
% 
% Plot Filtered and Non Filtered Pitch test Plot 
% figure(2);
% plot(t,pitch_nf);
% hold on
% plot(t,pitch_f);
% legend('pitch_nf','pitch_f');
% xlabel('Samples'), ylabel('Pitch (degrees)')
% title('Complementary Filter on Pitch data ')
% hold off

%#######################################################################%
















