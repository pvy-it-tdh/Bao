clc;
clear all;
close all;

% Thông số của hệ thống 
Kt = 0.07;  % Hằng số momen-xoắn của động cơ 
Kb = 0.07;  % Hằng số điện áp cảm ứng
mr = 0.2;  % Khối lượng của arm 
mp = 0.05388;  % Khối lượng của pendulum 
Lp = 0.15;  % Chiều dài của pendulum 
Lr = 0.25;  % Chiều dài của arm 
Br = 0.1;  % Hệ số cản của arm 
Bp = 0.001;  % Hệ số cản của pendulum
Rm = 1.7;  % Điện trở của động cơ 
Jm = 2.52e-5;  % Hằng số momen quán tính của động cơ 
g = 9.81 ;
T = 0.01;

% Thông số điều khiển PID 
% Thông số PID của arm
%kp1 = 60; 
%ki1 = 0; 
%kd1 = 12; 
% % Thông số PID của pendulum 
%kp2 = 2; 
%ki2 =0; 
%kd2 =1; 

% 
% kp1 =-19.1400
% 
% ki1 =0
% 
% kd1 =-11.4500
% 
% kp2 =-98.6900
% 
% ki2 = 0
% 
% kd2 =-15.7400
% 


% 
% kp1 =-9.4100
% 
% ki1 =0
% 
% kd1 =-8.9
% 
% kp2 =-180.6900
% 
% ki2 = 0
% 
% kd2 =-12.7400


% hơi tệ 
% kp1 = -5.4100
% ki1 =0
% kd1 =-9.5400
% kp2 =-96.7800
% ki2 =0
% kd2 = -14.7500




% Giá trị ban đầu 
x1_init = 0.1;  % Vị trí ban đầu của arm, đơn vị rad  
x3_init = 0;  % Vận tốc góc ban đầu của arm, đơn vị rad/s
x2_init = -0.1;  % Vị trí ban đầu của pendulum, đơn vị rad 
x4_init = 0;  % Vận tốc góc ban đầu của pendulum, đơn vị rad/s