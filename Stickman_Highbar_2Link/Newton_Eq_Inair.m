clear all

syms th_Wrist_Pre(t)
syms th_Hip_Pre(t)
syms x_Wrist_Pre(t)
syms y_Wrist_Pre(t)
syms g m_Body m_Leg real
syms l_Body l_Leg real

p_Wrist = [x_Wrist_Pre, y_Wrist_Pre];
p_Hip = p_Wrist + l_Body .* [cos(th_Wrist_Pre + 1/2 * pi), sin(th_Wrist_Pre + 1/2 * pi)];
p_Toe = p_Hip + l_Leg .* [cos(th_Wrist_Pre + 1/2 * pi + th_Hip_Pre), sin(th_Wrist_Pre + 1/2 * pi + th_Hip_Pre)];

p_Body = formula(1/2 * (p_Wrist + p_Hip));
p_Leg = formula(1/2 * (p_Hip + p_Toe));

p_G = formula((m_Body * p_Body + m_Leg * p_Leg)/(m_Body + m_Leg));

v_Body = diff(p_Body);
v_Leg = diff(p_Leg);
v_G = diff(p_G);

T = 1/2 * m_Body * (v_Body * v_Body') + 1/2 * m_Leg * (v_Leg * v_Leg') ...
    + 1/2 * (1/12 * m_Body * l_Body^2) * diff(th_Wrist_Pre)^2 ...
    + 1/2 * (1/12 * m_Leg * l_Leg^2) * diff(th_Wrist_Pre + th_Hip_Pre)^2;
U = p_Body(2) * m_Body * g + p_Leg(2) * m_Leg * g;

L = T - U;

K = (m_Body + m_Leg) * v_G;
d_K = diff(K);

syms f_Wrist_Bar real
syms tau_Hip

laglange_Eqs = [
    -functionalDerivative(L, x_Wrist_Pre) == 0;
    -functionalDerivative(L, y_Wrist_Pre) == 0;
    -functionalDerivative(L, th_Wrist_Pre) == 0;
    -functionalDerivative(L, th_Hip_Pre) == tau_Hip;
    ];

syms x dx ddx real
syms y dy ddy real
syms th_Wrist dth_Wrist ddth_Wrist real
syms th_Hip dth_Hip ddth_Hip real

syms_Replaced = [
    x_Wrist_Pre, diff(x_Wrist_Pre,t), diff(x_Wrist_Pre,t,t), ...
    y_Wrist_Pre, diff(y_Wrist_Pre,t), diff(y_Wrist_Pre,t,t), ...
    th_Wrist_Pre, diff(th_Wrist_Pre,t), diff(th_Wrist_Pre,t,t), ...
    th_Hip_Pre, diff(th_Hip_Pre,t), diff(th_Hip_Pre,t,t), ...
    ];

syms_Replacing = [
    x, dx, ddx, ...
    y, dy, ddy, ...
    th_Wrist, dth_Wrist, ddth_Wrist, ...
    th_Hip, dth_Hip, ddth_Hip, ...
    ];

laglange_Eqs = simplify(subs(laglange_Eqs, syms_Replaced, syms_Replacing));
d_K = simplify(subs(d_K, syms_Replaced, syms_Replacing));


parallel.defaultClusterProfile('local');
c = parcluster();

variables = [ddx, ddy, ddth_Wrist, ddth_Hip];

[A, B] = equationsToMatrix(laglange_Eqs, variables);
tic
X = simplify(inv(A)*B);
toc

ddx_Wrist_Eq = simplify(X(1));
ddy_Wrist_Eq = simplify(X(2));
ddth_Wrist_Eq = simplify(X(3));
ddth_Hip_Eq = simplify(X(4));

matlabFunction(ddth_Wrist_Eq, ddth_Hip_Eq, ddx_Wrist_Eq, ddy_Wrist_Eq, 'file', 'find_dd_Inair.m', 'outputs', {'ddth_Wrist', 'ddth_Hip', 'ddx', 'ddy'})

simplify(subs(d_K, variables, X'))

































