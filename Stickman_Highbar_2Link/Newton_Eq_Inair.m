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

ang_M_Center_Segments = formula(m_Body * cross([p_Body - p_G, 0], [v_Body - v_G, 0]) + m_Leg * cross([p_Leg - p_G, 0], [v_Leg - v_G, 0]));
ang_M_Center_Segments = ang_M_Center_Segments(3);
ang_M = ang_M_Center_Segments + (1/12 * m_Body * l_Body^2) * diff(th_Wrist_Pre) + (1/12 * m_Leg * l_Leg^2) * diff(th_Wrist_Pre + th_Hip_Pre);
dang_M = diff(ang_M);

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
ang_M = simplify(subs(ang_M, syms_Replaced, syms_Replacing));
dang_M = simplify(subs(dang_M, syms_Replaced, syms_Replacing));

% matlabFunction(formula(ang_M), 'file', 'find_Ang_M.m', 'outputs', {'ang_M'})


parallel.defaultClusterProfile('local');
c = parcluster();

%{
variables = [ddx, ddy, ddth_Wrist, ddth_Hip];

[A, B] = equationsToMatrix(laglange_Eqs, variables);
tic
X = simplify(inv(A)*B);
toc

ddx_Wrist_Eq = simplify(X(1));
ddy_Wrist_Eq = simplify(X(2));
ddth_Wrist_Eq = simplify(X(3));
ddth_Hip_Eq = simplify(X(4));

% matlabFunction(ddth_Wrist_Eq, ddth_Hip_Eq, ddx_Wrist_Eq, ddy_Wrist_Eq, 'file', 'find_dd_Inair.m', 'outputs', {'ddth_Wrist', 'ddth_Hip', 'ddx', 'ddy'})
%}

%{/
variables = [ddx, ddy, ddth_Wrist, tau_Hip];

[A, B] = equationsToMatrix(laglange_Eqs, variables);
tic
X = simplify(inv(A)*B);
toc

ddx_Wrist_Eq = simplify(X(1));
ddy_Wrist_Eq = simplify(X(2));
ddth_Wrist_Eq = simplify(X(3));
tau_Hip_Eq = simplify(X(4));

matlabFunction(tau_Hip_Eq, 'file', 'find_Tau_Hip_Inair.m', 'outputs', {'tau_Hip'})

simplify(subs(d_K, variables, X'))
simplify(subs(dang_M, variables, X'))
%}


































