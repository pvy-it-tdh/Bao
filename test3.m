clc;
clear all;
close all;

% Khai báo thông số hệ thống
syms mp mr Lp Lr Jm b1 b2 Kt Kb Rm g x1 x2 x3 x4 x5 x6 V;

% Đặt các hệ số K
K1 = Jm;
K2 = (Kt * Kb) / Rm;
K3 = Kt / Rm;

% Ma trận M1 (Ma trận quán tính)
M1 = [mp*Lp^2*sin(x2)^2 + mp*Lr^2 + (3/2)*mr*Lr^2 + Jm, -mp*Lr*Lp*cos(x2);
      -mp*Lr*Lp*cos(x2), (3/2)*mp*Lp^2];

% Ma trận M2 (Thành phần tác động bởi vận tốc góc)
M2 = [mp*Lp^2*sin(x2)*cos(x2) * x4, mp*Lr*Lp*sin(x2)*x4;
     -mp*Lp^2*sin(x2)*cos(x2)*x3, b2];

% Vectơ M3 (Mô-men trọng lực)
M3 = [0;
      -mp*g*Lp*sin(x2)];

% Vectơ M4 (Điều khiển)
M4 = [-K1*x5 - K2*x3 + K3*V;
      0];

% Thiết lập phương trình hệ thống H
H = M1 * [x5; x6] + M2 * [x3; x4] + M3 - M4;

% Tách hai phương trình
h1 = H(1);
h2 = H(2);

% Giải hệ phương trình cho x5 và x6
sol = solve([h1 == 0, h2 == 0], [x5, x6]);

% Hiển thị kết quả
disp('Nghiệm cho x5 (theta1_2dot):');
disp(sol.x5);
disp('Nghiệm cho x6 (theta2_2dot):');
disp(sol.x6);
