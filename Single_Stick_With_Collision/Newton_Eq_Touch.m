clear all

syms theta_Pre(t)
syms Lc M real
syms g real
syms x_Pre(t) y_Pre(t)
syms tau real

p_G = [x_Pre, y_Pre] + Lc * [cos(theta_Pre), sin(theta_Pre)];

p_G_Formula = formula(p_G);

v_G = diff(p_G, t);

% T = 1/2 * M * (vG * vG');
T = 1/2 * M * (v_G * v_G') + 1/2 * 1/12 * M * (2*Lc)^2 * diff(theta_Pre,t)^2;
U = M * g * p_G_Formula(1,2);

momentumG = 1/12 * M * (2*Lc)^2 * diff(theta_Pre,t);
dmomentumG = diff(momentumG, t);

K = M * v_G;
d_K = diff(K);

L = T - U;

f_X = -functionalDerivative(L, x_Pre);
f_Y = -functionalDerivative(L, y_Pre);

L = subs(L, [x_Pre, y_Pre], [sym(0), sym(0)]);
f_X = subs(f_X, [x_Pre, y_Pre], [sym(0), sym(0)]);
f_Y = subs(f_Y, [x_Pre, y_Pre], [sym(0), sym(0)]);
dmomentumG = subs(dmomentumG, [x_Pre, y_Pre], [sym(0), sym(0)]);
d_K = subs(d_K, [x_Pre, y_Pre], [sym(0), sym(0)]);

theta_eq = -functionalDerivative(L, theta_Pre) == tau;

Equations = [theta_eq];

syms theta dtheta ddtheta real

SymbolReplaced = [theta_Pre, diff(theta_Pre, t), diff(theta_Pre, t, t)];
SymbolReplacing = [theta, dtheta, ddtheta];

Equations = subs(Equations, SymbolReplaced, SymbolReplacing);

f_X = expand(subs(f_X, SymbolReplaced, SymbolReplacing));
f_Y = expand(subs(f_Y, SymbolReplaced, SymbolReplacing));
dmomentumG = subs(dmomentumG, SymbolReplaced, SymbolReplacing);
d_K = subs(d_K, SymbolReplaced, SymbolReplacing);

variables = ddtheta;

[A, B] = equationsToMatrix(Equations, variables);
detA = det(A);
X = simplify(inv(A)*B);

ddtheta_eq = X(1,1);

matlabFunction(ddtheta_eq, 'file', 'find_dd_Touch.m', 'outputs', {'ddtheta'})

f_X = subs(f_X, ddtheta, ddtheta_eq);
f_Y = subs(f_Y, ddtheta, ddtheta_eq);

matlabFunction(formula(f_X), formula(f_Y), 'file', 'find_F_Touch.m', 'outputs', {'f_X', 'f_Y'})

dmomentumG = subs(dmomentumG, ddtheta, ddtheta_eq)
d_K = subs(d_K, ddtheta, ddtheta_eq);

simplify(d_K - [f_X, f_Y])
























