
clear all

syms theta_Pre(t)
syms x_Pre(t) y_Pre(t)

syms dtheta_Before dtheta_After real
syms dx_Before real
syms dy_Before real
syms dx_After real
syms Lc real
syms I_F_Y real
syms I m real
syms myu real

syms x y real
syms theta real

I_F_X = myu * I_F_Y;

p_G = [x_Pre, y_Pre] + Lc * [cos(theta_Pre), sin(theta_Pre)];
v_G = formula(diff(p_G));

% dx_After = subs(v_G(1), [diff(x_Pre), diff(y_Pre), theta_Pre, diff(theta_Pre)], [0,0, theta, dtheta_After]);
dy_After = subs(v_G(2), [diff(x_Pre), diff(y_Pre), theta_Pre, diff(theta_Pre)], [0,0, theta, dtheta_After]);

p_G = subs(p_G, [theta_Pre, x_Pre, y_Pre], [theta, x, y]);

Torque_F = formula(cross([x,y,0] - [p_G, 0], [I_F_X, I_F_Y,0]));
Torque_F = formula(Torque_F(3));

equations = simplify([
    I * (dtheta_After - dtheta_Before) == Torque_F;
    m * (dx_After - dx_Before) == I_F_X;
    m * (dy_After - dy_Before) == I_F_Y;
    ]);

variables = [dtheta_After, dx_After, I_F_Y];

[A, B] = equationsToMatrix(equations, variables);
X = simplify(inv(A)*B);

dtheta_After_Eq = simplify(X(1));
dx_After_Eq = simplify(X(2));
I_F_Y_Eq = simplify(X(3));

matlabFunction(dtheta_After_Eq, dx_After_Eq, I_F_Y_Eq, 'file', 'find_Status_After_Collision_Slides', 'outputs', {'dtheta_After', 'dx_After', 'I_F_Y'})












































