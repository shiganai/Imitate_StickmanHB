
clear all

syms theta_Pre(t)
syms x_Pre(t) y_Pre(t)

syms dtheta_Before dtheta_After real
syms dx_Before real
syms dy_Before real
syms Lc real
syms I_F_X I_F_Y real
syms I M real

p_G = [x_Pre, y_Pre] + Lc * [cos(theta_Pre), sin(theta_Pre)];
v_G = formula(diff(p_G));

syms x y real
syms theta real

dx_After = subs(v_G(1), [diff(x_Pre), diff(y_Pre), theta_Pre, diff(theta_Pre)], [0,0, theta, dtheta_After]);
dy_After = subs(v_G(2), [diff(x_Pre), diff(y_Pre), theta_Pre, diff(theta_Pre)], [0,0, theta, dtheta_After]);

p_G = subs(p_G, [theta_Pre, x_Pre, y_Pre], [theta, x, y]);

Torque_F = formula(cross([x,y,0] - [p_G, 0], [I_F_X, I_F_Y,0]));
Torque_F = formula(Torque_F(3));


equations = ([
    M * (dx_After - dx_Before) == I_F_X;
    M * (dy_After - dy_Before) == I_F_Y;
    (1/12 * M * (2 * Lc)^2) * (dtheta_After - dtheta_Before) == Torque_F;
    ]);

% equations(3)

variables = [I_F_X, I_F_Y, dtheta_After];

[A, B] = equationsToMatrix(equations, variables);
X = simplify(inv(A)*B);

I_F_X_Eq = simplify(X(1));
I_F_Y_Eq = simplify(X(2));
dtheta_After_Eq = simplify(X(3));

% The input value X and Y are location of center of mass, not of top of the stick
% matlabFunction(dtheta_After_Eq, I_F_X_Eq, I_F_Y_Eq, 'file', 'find_Status_After_Collision', 'outputs', {'dtheta_After', 'I_F_X', 'I_F_Y'})

simplify(I_F_X_Eq - subs(M * (dx_After - dx_Before), dtheta_After, dtheta_After_Eq));
simplify(subs(M * (dx_After - dx_Before), dtheta_After, dtheta_After_Eq));











































