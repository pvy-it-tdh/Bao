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
Jr = 0.008591;
Cm = 0.006408; 
Jp = 0.000217;
Bp = 0.001;  % Hệ số cản của pendulum
Rm = 1.7;  % Điện trở của động cơ 
Jm = 2.52e-5;  % Hằng số momen quán tính của động cơ 
g = 9.81 ;
T=0.01;

% Thông số điều khiển PID 
% Thông số PID của arm
kp1 = 50; 
ki1 = 0; 
kd1 = 0; 
% Thông số PID của pendulum 
kp2 = 7; 
ki2 = 0; 
kd2 = 0; 

% Giá trị ban đầu 
x1_init = pi/10;  % Vị trí ban đầu của arm, đơn vị rad  
x2_init = pi/20;  % Vận tốc góc ban đầu của arm, đơn vị rad/s
x3_init = 0.01;  % Vị trí ban đầu của pendulum, đơn vị rad 
x4_init = 0.1;  % Vận tốc góc ban đầu của pendulum, đơn vị rad/s