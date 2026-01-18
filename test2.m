clc;
clear all;
close all;

% Đặt biến cho thông số hệ thống
syms mp mr Lp Lr Jr Jp Br Bp Kt Kb Rm Jm Cm g real;
syms x1 x2 x3 x4 x5 x6 V ;  % x1=alpha, x2=beta, x3=alpha_dot, x4=beta_dot, x5=alpha_ddot, x6=beta_ddot

% Các hằng số của động cơ
K1 = Jm;
K2 = ((Kt * Kb) / Rm);
K3 = Kt / Rm;

% Ma trận quán tính (Inertia Matrix) M1
M1 = [Jr + mp * Lr^2 + mp * Lp^2 * sin(x2)^2, -mp * Lr * Lp * cos(x2);
      -mp * Lr * Lp * cos(x2), Jp + mp * Lp^2];

% Ma trận Coriolis và lực ly tâm (Coriolis and Centrifugal Matrix) M2
M2 = [(Cm + (1/2) * mp * Lp^2 * x4 * sin(2 * x2)),( mp * Lr * Lp * x4 * sin(x2) + (1/2) * mp * Lp^2 * x3 * sin(2 * x2)); ...
     - (1/2) * mp * Lp^2 * x3 * sin(2 * x2), Bp];

% Vectơ lực trọng trường (Gravity Vector) M3
M3 = [0;
      -mp * g * Lp * sin(x2)];

% Phương trình động lực học H
H = M1 * [x5; x6] + M2 * [x3; x4] + M3 - [-K1*x5-K2*x3+K3*V;0];

% Tách các phương trình cho alpha_ddot (x5) và beta_ddot (x6)
h1 = H(1);
h2 = H(2);

% Giải hệ phương trình cho x5 (alpha_ddot) và x6 (beta_ddot)
[x5, x6] = solve(h1, h2, x5, x6)

