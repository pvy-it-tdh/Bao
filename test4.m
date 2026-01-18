% Define system parameters
syms mp mr Lp Lr Jm g b2 real
syms x1 x2 x3 x4 x5 x6 V real

% Motor constants
syms Kt Kb Rm real
K1 = Jm;
K2 = (Kt * Kb) / Rm;
K3 = Kt / Rm;

% Define Matrix M1 (Inertia Matrix)
M1 = [mp*Lp^2*sin(x2)^2 + mp*Lr^2 + (3/2)*mr*Lr^2 + Jm, -mp*Lr*Lp*cos(x2);
      -mp*Lr*Lp*cos(x2), (3/2)*mp*Lp^2];

% Define Matrix M2 (Velocity-dependent terms)
M2 = [mp*Lp^2*sin(x2)*cos(x2) * x4, mp*Lr*Lp*sin(x2)*x4;
     -mp*Lp^2*sin(x2)*cos(x2)*x3, b2];

% Define Vector M3 (Gravity effect)
M3 = [0;
      -mp*g*Lp*sin(x2)];

% Define Vector M4 (Control input)
M4 = [-K1*x5 - K2*x3 + K3*V;
       0];

% Full equation M1*[x5; x6] = M2*[x3; x4] + M3 + M4
% Rearrange to solve for [x5; x6]
rhs = M2 * [x3; x4] + M3 + M4;
solution = linsolve(M1, rhs);

% Extract solutions for x5 and x6
x5_solution = solution(1);
x6_solution = solution(2);

% Display solutions
disp('x5 = ');
disp(x5_solution);
disp('x6 = ');
disp(x6_solution);
