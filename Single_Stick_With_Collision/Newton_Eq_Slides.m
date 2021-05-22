clear all

syms theta_Pre(t)
syms Lc M real
syms g real
syms x_Pre(t) y_Pre(t)
syms tau real
syms myu real

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

f_Y = -functionalDerivative(L, y_Pre);

L = subs(L, y_Pre, sym(0));
f_Y = subs(f_Y, y_Pre, sym(0));
dmomentumG = subs(dmomentumG, y_Pre, sym(0));
d_K = subs(d_K, y_Pre, sym(0));

equations = [
    -functionalDerivative(L, x_Pre) == myu * f_Y;
    -functionalDerivative(L, theta_Pre) == tau;
    ];

syms x dx ddx real
syms theta dtheta ddtheta real

syms_Replaced = [
    x_Pre, diff(x_Pre, t), diff(x_Pre, t, t), ...
    theta_Pre, diff(theta_Pre, t), diff(theta_Pre, t, t), ...
    ];
syms_Replacing = [
    x, dx, ddx, ...
    theta, dtheta, ddtheta, ...
    ];

equations = subs(equations, syms_Replaced, syms_Replacing);

f_Y = expand(subs(f_Y, syms_Replaced, syms_Replacing));
dmomentumG = subs(dmomentumG, syms_Replaced, syms_Replacing);
d_K = subs(d_K, syms_Replaced, syms_Replacing);

variables = [ddx, ddtheta];

[A, B] = equationsToMatrix(equations, variables);
detA = det(A);
X = simplify(inv(A)*B);

ddx_eq = X(1,1);
ddtheta_eq = X(2,1);

f_Y = subs(f_Y, variables, X');
f_X = f_Y * myu;

matlabFunction(ddtheta_eq, ddx_eq, formula(f_X), formula(f_Y), 'file', 'find_dd_Slides.m', 'outputs', {'ddtheta', 'ddx', 'f_X', 'f_Y'})
% matlabFunction(formula(f_X), formula(f_Y), 'file', 'find_F_Slides.m', 'outputs', {'f_Y', 'f_X'})

dmomentumG = subs(dmomentumG, variables, X');
d_K = subs(d_K, variables, X');

simplify(d_K - [f_X, f_Y])
























