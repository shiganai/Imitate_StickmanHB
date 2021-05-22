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

v_G = diff(p_G, t);

% T = 1/2 * M * (v_G * v_G');
T = 1/2 * M * (v_G * v_G') + 1/2 * 1/12 * M * (2 * Lc)^2 * diff(theta_Pre,t)^2;
U = M * g * p_G(1,2);
% U = 0;

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
d_K = subs(d_K, syms_Replaced, syms_Replacing);

% variables = [ddtheta];
variables = [ddx, ddy, ddtheta];

[A, B] = equationsToMatrix(Equations, variables);
detA = det(A);
X = simplify(inv(A)*B);

ddx_eq = simplify(X(1));
ddy_eq = simplify(X(2));
ddtheta_eq = simplify(X(3));

% matlabFunction(ddx_eq, ddy_eq, ddtheta_eq, 'file', 'find_dd_Inair.m', 'outputs', {'ddx_eq', 'ddy_eq', 'ddtheta_eq'})

Torque_F = formula(cross([p_G, 0] - [x, y, 0],[f_X, f_Y, 0]));
Torque_F = simplify(Torque_F(3));

dmomentumG = simplify(subs(dmomentumG, variables, X') - Torque_F)
d_K = simplify(subs(d_K, variables, X'))

























