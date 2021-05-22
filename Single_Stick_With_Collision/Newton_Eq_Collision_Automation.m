clear all

syms theta_Pre(t)
syms Lc real
syms M real
syms g real
syms x_Pre(t) y_Pre(t)
syms tau real
syms f_X f_Y real

% p_G = Lc * [cos(theta_Pre), sin(theta_Pre)];
% p_G = formula([x_Pre, y_Pre]);
p_G = formula([x_Pre, y_Pre] + Lc * [cos(theta_Pre), sin(theta_Pre)]);
% p_G = sym([0,0]);

restraint_P_Origin = formula(diff(p_G - [x_Pre, y_Pre], t));

v_G = diff(p_G, t);

% T = 1/2 * M * (v_G * v_G');
T = 1/2 * M * (v_G * v_G') + 1/2 * 1/12 * M * (2 * Lc)^2 * diff(theta_Pre,t)^2;
% U = M * g * p_G(1,2);
U = 0;

momentumG = 1/12 * M * (2*Lc)^2 * diff(theta_Pre,t);
dmomentumG = diff(momentumG, t);

K = M * v_G;
d_K = diff(K, t);

L = T - U;

Equations = [
    -functionalDerivative(L, x_Pre) == f_X;
    -functionalDerivative(L, y_Pre) == f_Y;
    -functionalDerivative(L, theta_Pre) == tau;
    ];

syms x dx ddx real
syms y dy ddy real
syms theta dtheta ddtheta real

syms_Replaced = [
    x_Pre, diff(x_Pre, t), diff(x_Pre, t, t), ...
    y_Pre, diff(y_Pre, t), diff(y_Pre, t, t), ...
    theta_Pre, diff(theta_Pre, t), diff(theta_Pre, t, t), ...
    ];
syms_Replacing = [
    x, dx, ddx, ...
    y, dy, ddy, ...
    theta, dtheta, ddtheta, ...
    ];

Equations = subs(Equations, syms_Replaced, syms_Replacing);

dmomentumG = subs(dmomentumG, syms_Replaced, syms_Replacing);
p_G = subs(p_G, syms_Replaced, syms_Replacing);
v_G = subs(v_G, syms_Replaced, syms_Replacing);
d_K = subs(d_K, syms_Replaced, syms_Replacing);
restraint_P_Origin = subs(restraint_P_Origin, syms_Replaced, syms_Replacing);

% variables = [ddtheta];
variables = [ddx, ddy, ddtheta];

[A, B] = equationsToMatrix(Equations, variables);
detA = det(A);
X = simplify(inv(A)*B);

ddx_Eq = simplify(X(1));
ddy_Eq = simplify(X(2));
ddtheta_Eq = simplify(X(3));

% matlabFunction(ddx_eq, ddy_eq, ddtheta_eq, 'file', 'find_dd_Inair.m', 'outputs', {'ddx_eq', 'ddy_eq', 'ddtheta_eq'})

Torque_F = formula(cross([x, y, 0] - [p_G, 0],[f_X, f_Y, 0]));
Torque_F = expand(simplify(Torque_F(3)));

dmomentumG = simplify(subs(dmomentumG, variables, X'));
simplify(dmomentumG - Torque_F);
d_K = simplify(subs(d_K, variables, X'));

syms dtheta_After dtheta_Before real
syms dx_After dx_Before real
syms dy_After dy_Before real
syms I_F_X I_F_Y real
syms dx_G_Before dy_G_Before real

% calc around the center of mass
% equations = ([
%     subs(M * v_G(1), [dx, dtheta], [dx_After, dtheta_After]) - M * dx_G_Before == f_X;
%     subs(M * v_G(2), [dy, dtheta], [dy_After, dtheta_After]) - M *dy_G_Before == f_Y;
%     dtheta_After - dtheta_Before == simplify(ddtheta_Eq - subs(ddtheta_Eq, [f_X, f_Y], sym([0,0])));
%     ]);

% calc around the point to fix
equations = ([
    dx_After - dx_Before == simplify(ddx_Eq - subs(ddx_Eq, [f_X, f_Y], sym([0,0])));
    dy_After - dy_Before == simplify(ddy_Eq - subs(ddy_Eq, [f_X, f_Y], sym([0,0])));
    dtheta_After - dtheta_Before == simplify(ddtheta_Eq - subs(ddtheta_Eq, [f_X, f_Y], sym([0,0])));
    ]);

equations = subs(equations, [f_X, f_Y], [I_F_X, I_F_Y]);

equations = (subs(equations, [dx_After, dy_After], [sym(0), sym(0)]));

% equations(3)

variables = [I_F_X, I_F_Y, dtheta_After];

[A, B] = equationsToMatrix(equations, variables);
X = simplify(inv(A)*B);

I_F_X_Eq = simplify(X(1));
I_F_Y_Eq = simplify(X(2));
dtheta_After_Eq = simplify(X(3));

v_G_Delta = subs(v_G, [dx, dy, dtheta], [0, 0, dtheta_After_Eq]) ...
    - subs(v_G, [dx, dy, dtheta], [dx_Before, dy_Before, dtheta_Before]);
I_G_Delta = v_G_Delta * M;
I_G_Delta(1);

simplify([I_F_X_Eq, I_F_Y_Eq] - I_G_Delta)






















