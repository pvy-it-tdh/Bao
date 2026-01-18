clc 
clear all 
close all 

% đặt biến cho thông số hệ thống 
syms mp mr Lp Lr Jr Jp Br Bp Kt Kb Rm Jm Cm K1 K2 K3 g;

% đặt biến điều khiển hệ thống
syms x1 x2 x3 x4 x5 x6 V

K1 = Jm ; 
K2 =((Kt*Kb)/Rm);
K3=Kt/Rm ;

% ta đặt trạng thái như sau: 
% x1=theta1 , x2 = theta2,  x3=theta1_dot , x4=theta2_dot, 
% x5=theta1_2dot, x6=theta2_2dot 

% ma tran M 

M1=[mp*Lp^2*sin(x2)^2 + mp*Lr^2 + 1.5*mr*Lr^2 , -mp*Lr*Lp*cos(x2) ; -mp*Lr*Lp*cos(x2), 1.5*mp*Lp^2];
M2=[(1/2)*mp*Lp^2*sin(2*x2)*x4+Br, mp*Lr*Lp*sin(x2)*x4; -(1/2)*mp*Lp^2*sin(2*x2)*x3 , Bp];
M3=[0;-mp*g*Lp*sin(x2)];


H = M1*[x5;x6] + M2*[x3;x4] + M3 -[-K1*x5-K2*x3+K3*V;0];
h1 = H(1); 
h2 = H(2); 
[x5,x6] = solve(h1,h2,x5,x6)










