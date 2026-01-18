clear,clc
%% 1 Biến & tham số ký hiệu
syms th1 th2 dth1 dth2 ddth1 ddth2 real
syms l_a l_p m_a m_p g I_a I_p positive

c1 = cos(th1); s1 = sin(th1);
c2 = cos(th2); s2 = sin(th2);

%% 2 Thuần nhất & Jacôbi
T01 = [ c1 -s1 0 l_a*c1;
        s1  c1 0 l_a*s1;
        0   0  1      0;
        0   0  0      1];

T12 = [ c2 0  s2  0;
        0  1  0   0;
       -s2 0  c2 l_p;
        0  0  0   1];

T02 = simplify(T01*T12)

p_a = T01*[l_a/2;0; 0;1]
%p_a = p_a(1:3)
p_p = T02*[0;0;l_p/2;1]
%p_p = p_p(1:3)

J_a = jacobian(p_a,[th1 th2]);
J_p = jacobian(p_p,[th1 th2]);

dq  = [dth1; dth2];
v_a = J_a*dq; v_p = J_p*dq;
va2 = simplify(v_a.'*v_a);
vp2 = simplify(v_p.'*v_p);

%% 3 Năng lượng
T = 1/2*( m_a*va2 + m_p*vp2 + I_a*dth1^2 + I_p*dth2^2 );
V = m_p*g*l_p/2*cos(th2);
L = T - V;

%% 4 Lagrange   𝐌(q)·ddq + (C+G)=0
q  = [th1; th2];
dq = [dth1; dth2];
ddq= [ddth1; ddth2];

E = sym(zeros(2,1));
for k = 1:2
    dLdq   = diff(L, q(k));
    dLdqd  = diff(L, dq(k));

    % d/dt[dL/dq̇]  = ∂/∂q *dq̇  + ∂/∂q̇ *ddq̇
    ddt = jacobian(dLdqd, [th1 th2])*dq + ...
          jacobian(dLdqd, [dth1 dth2])*ddq;

    E(k) = simplify(ddt - dLdq);
end

% Tách M, rhs
M = jacobian(E, ddq)              % 2×2
rhs = simplify( E - M*ddq )

%% 5 Kết quả
%disp('M(q) ='); pretty(simplify(M)), disp(' ')
%disp('C(q,dq)+G(q) ='); pretty(rhs)
