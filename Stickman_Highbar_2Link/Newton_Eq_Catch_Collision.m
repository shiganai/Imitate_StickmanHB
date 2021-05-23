clear all

syms th_Wrist_Pre(t)
syms th_Hip_Pre(t)
syms x_Wrist_Pre(t)
syms y_Wrist_Pre(t)
syms g m_Body m_Leg real
syms l_Body l_Leg real
syms l_Wrist_Bar_Pre(t)
syms l_N_Wrist_Bar_Pre(t)

p_Origin = [x_Wrist_Pre, y_Wrist_Pre] + l_N_Wrist_Bar_Pre .* [cos(th_Wrist_Pre), sin(th_Wrist_Pre)];
p_Wrist = p_Origin +  -l_Wrist_Bar_Pre .* [cos(th_Wrist_Pre + 1/2 * pi), sin(th_Wrist_Pre + 1/2 * pi)];
p_Hip = p_Wrist + l_Body .* [cos(th_Wrist_Pre + 1/2 * pi), sin(th_Wrist_Pre + 1/2 * pi)];
p_Toe = p_Hip + l_Leg .* [cos(th_Wrist_Pre + 1/2 * pi + th_Hip_Pre), sin(th_Wrist_Pre + 1/2 * pi + th_Hip_Pre)];

p_Body = formula(1/2 * (p_Wrist + p_Hip));
p_Leg = formula(1/2 * (p_Hip + p_Toe));

p_G = formula((m_Body * p_Body + m_Leg * p_Leg)/(m_Body + m_Leg));

v_Body = diff(p_Body);
v_Leg = diff(p_Leg);
v_G = formula(diff(p_G));

M = v_G * (m_Body + m_Leg);
d_M = formula(diff(M,t));

T = 1/2 * m_Body * (v_Body * v_Body') + 1/2 * m_Leg * (v_Leg * v_Leg') ...
    + 1/2 * (1/12 * m_Body * l_Body^2) * diff(th_Wrist_Pre)^2 ...
    + 1/2 * (1/12 * m_Leg * l_Leg^2) * diff(th_Wrist_Pre + th_Hip_Pre)^2;
U = p_Body(2) * m_Body * g + p_Leg(2) * m_Leg * g;

L = T - U;

f_X = subs(-functionalDerivative(L, x_Wrist_Pre), [x_Wrist_Pre, y_Wrist_Pre], sym([0,0]));
f_Y = subs(-functionalDerivative(L, y_Wrist_Pre), [x_Wrist_Pre, y_Wrist_Pre], sym([0,0]));

L = subs(L, [x_Wrist_Pre, y_Wrist_Pre], sym([0,0]));
d_M = subs(d_M, [x_Wrist_Pre, y_Wrist_Pre], sym([0,0]));
v_G = subs(v_G, [x_Wrist_Pre, y_Wrist_Pre], sym([0,0]));

syms f_Wrist_Bar real
syms N_Wrist_Bar real
syms tau_Hip

laglange_Eqs = [
    -functionalDerivative(L, l_N_Wrist_Bar_Pre) == N_Wrist_Bar;
    -functionalDerivative(L, l_Wrist_Bar_Pre) == f_Wrist_Bar;
    -functionalDerivative(L, th_Wrist_Pre) == 0;
    -functionalDerivative(L, th_Hip_Pre) == tau_Hip;
    ];

syms l_N_Wrist_Bar dl_N_Wrist_Bar ddl_N_Wrist_Bar real
syms l_Wrist_Bar dl_Wrist_Bar ddl_Wrist_Bar real
syms th_Wrist dth_Wrist ddth_Wrist real
syms th_Hip dth_Hip ddth_Hip real

syms_Replaced = [
    l_N_Wrist_Bar_Pre, diff(l_N_Wrist_Bar_Pre,t), diff(l_N_Wrist_Bar_Pre,t,t), ...
    l_Wrist_Bar_Pre, diff(l_Wrist_Bar_Pre,t), diff(l_Wrist_Bar_Pre,t,t), ...
    th_Wrist_Pre, diff(th_Wrist_Pre,t), diff(th_Wrist_Pre,t,t), ...
    th_Hip_Pre, diff(th_Hip_Pre,t), diff(th_Hip_Pre,t,t), ...
    ];

syms_Replacing = [
    l_N_Wrist_Bar, dl_N_Wrist_Bar, ddl_N_Wrist_Bar, ...
    l_Wrist_Bar, dl_Wrist_Bar, ddl_Wrist_Bar, ...
    th_Wrist, dth_Wrist, ddth_Wrist, ...
    th_Hip, dth_Hip, ddth_Hip, ...
    ];

laglange_Eqs = simplify(subs(laglange_Eqs, syms_Replaced, syms_Replacing));
f_X = subs(f_X, syms_Replaced, syms_Replacing);
f_Y = subs(f_Y, syms_Replaced, syms_Replacing);
d_M = simplify(subs(d_M, syms_Replaced, syms_Replacing));
v_G = simplify(subs(v_G, syms_Replaced, syms_Replacing));

parallel.defaultClusterProfile('local');
c = parcluster();

variables = [ddl_N_Wrist_Bar, ddl_Wrist_Bar, ddth_Wrist, ddth_Hip];
% variables = [ddx, ddy, ddl_Wrist_Bar, ddth_Wrist, ddth_Hip];

[A, B] = equationsToMatrix(laglange_Eqs, variables);
tic
X = simplify(inv(A)*B);
toc

ddl_N_Wrist_Bar_Eq = simplify(X(1));
ddl_Wrist_Bar_Eq = simplify(X(2));
ddth_Wrist_Eq = simplify(X(3));
ddth_Hip_Eq = simplify(X(4));

% check [f_X, f_Y] == [f_Wrist_Bar, N_Wrist_Bar]
%{/
f_X = subs(f_X, variables, X');
f_Y = subs(f_Y, variables, X');

simplify(f_Wrist_Bar - (f_X * sin(th_Wrist) + f_Y * -cos(th_Wrist)));
simplify(N_Wrist_Bar - (f_X * cos(th_Wrist) + f_Y * sin(th_Wrist)));
%}

simplify(subs(d_M, variables, X'));

syms dl_N_Wrist_Bar_After dl_N_Wrist_Bar_Before real
syms dl_Wrist_Bar_After dl_Wrist_Bar_Before real
syms dth_Wrist_After dth_Wrist_Before real
syms dth_Hip_After dth_Hip_Before real
syms I_N_Wrist_Bar real
syms myu real

impulsive_Force = [f_Wrist_Bar, N_Wrist_Bar];
impulsive_Force_Zero = sym(zeros(size(impulsive_Force)));
impulsive_Force_Replacing = [I_N_Wrist_Bar * myu, I_N_Wrist_Bar];

assuming_Variable = [dl_N_Wrist_Bar_After];
assuming_Variable_Replacing = sym(zeros(size(assuming_Variable)));


equations = [
    dl_N_Wrist_Bar_After - dl_N_Wrist_Bar_Before == simplify(ddl_N_Wrist_Bar_Eq - subs(ddl_N_Wrist_Bar_Eq, impulsive_Force, impulsive_Force_Zero));
    dl_Wrist_Bar_After - dl_Wrist_Bar_Before == simplify(ddl_Wrist_Bar_Eq - subs(ddl_Wrist_Bar_Eq, impulsive_Force, impulsive_Force_Zero));
    dth_Wrist_After - dth_Wrist_Before == simplify(ddth_Wrist_Eq - subs(ddth_Wrist_Eq, impulsive_Force, impulsive_Force_Zero));
    dth_Hip_After - dth_Hip_Before == simplify(ddth_Hip_Eq - subs(ddth_Hip_Eq, impulsive_Force, impulsive_Force_Zero));
    ];

equations = subs(equations, impulsive_Force, impulsive_Force_Replacing);
equations = subs(equations, assuming_Variable, assuming_Variable_Replacing);

% equations = subs(equations, dl_N_Wrist_Bar_After, sym(0));
% equations = subs(equations, f_Wrist_Bar, myu * N_Wrist_Bar);
% equations = subs(equations, N_Wrist_Bar, I_N_Wrist_Bar);

variables = [dth_Wrist_After, dth_Hip_After, dl_Wrist_Bar_After, I_N_Wrist_Bar];

[A, B] = equationsToMatrix(equations, variables);
X = simplify(inv(A)*B);

dth_Wrist_After_Eq = simplify(X(1));
dth_Hip_After_Eq = simplify(X(2));
dl_Wrist_Bar_After_Eq = simplify(X(3));
I_N_Wrist_Bar_Eq = simplify(X(4));

impulsive_Force_Replacing_Eq = [I_N_Wrist_Bar_Eq * myu, I_N_Wrist_Bar_Eq];

v_G_Delta = subs(v_G, [dth_Wrist, dth_Hip, dl_Wrist_Bar, dl_N_Wrist_Bar], [dth_Wrist_After_Eq, dth_Hip_After_Eq, dl_Wrist_Bar_After_Eq, 0]) ...
    - subs(v_G, [dth_Wrist, dth_Hip, dl_Wrist_Bar, dl_N_Wrist_Bar], [dth_Wrist_Before, dth_Hip_Before, dl_Wrist_Bar_Before, dl_N_Wrist_Bar_Before]);
I_G_Delta = v_G_Delta * (m_Body + m_Leg);

I_F_Wrist_Bar = I_N_Wrist_Bar_Eq * myu;
I_F_X = simplify(subs(f_X - subs(f_X, impulsive_Force, impulsive_Force_Zero), impulsive_Force, impulsive_Force_Replacing_Eq));
I_F_Y = simplify(subs(f_Y - subs(f_Y, impulsive_Force, impulsive_Force_Zero), impulsive_Force, impulsive_Force_Replacing_Eq));

I_F_X_Manual = I_N_Wrist_Bar_Eq*cos(th_Wrist) + I_F_Wrist_Bar*sin(th_Wrist);
I_F_Y_Manual = I_N_Wrist_Bar_Eq*sin(th_Wrist) - I_F_Wrist_Bar*cos(th_Wrist);

simplify([I_F_X, I_F_Y] - I_G_Delta)
simplify(I_F_X - I_F_X_Manual)
simplify(I_F_Y - I_F_Y_Manual)

% matlabFunction(dth_Wrist_After_Eq, dth_Hip_After_Eq, dl_Wrist_Bar_After_Eq, I_N_Wrist_Bar_Eq, 'file', 'find_Status_After_Collision.m', 'outputs', {'dth_Wrist_After', 'dth_Hip_After', 'dl_Wrist_Bar_After', 'I_N_Wrist_Bar'})
































