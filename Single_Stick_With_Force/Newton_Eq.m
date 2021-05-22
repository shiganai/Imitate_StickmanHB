clear all

syms theta_pre(t)
syms Lc M real
syms g real
syms xTop(t) yTop(t)
syms Mtheta real

pG = [xTop, yTop] + Lc * [cos(theta_pre + 3/2*pi), sin(theta_pre + 3/2*pi)];

pG_formula = formula(pG);

vG = diff(pG, t);

% T = 1/2 * M * (vG * vG');
T = 1/2 * M * (vG * vG') + 1/2 * 1/12 * M * (2*Lc)^2 * diff(theta_pre,t)^2;
U = M * g * pG_formula(1,2);

momentumG = 1/12 * M * (2*Lc)^2 * diff(theta_pre,t);
dmomentumG = diff(momentumG, t);

L = T - U;

Fx_out = -functionalDerivative(L, xTop);
Fy_out = -functionalDerivative(L, yTop);

L = subs(L, [xTop, yTop], [sym(0), sym(0)]);
Fx_out = subs(Fx_out, [xTop, yTop], [sym(0), sym(0)]);
Fy_out = subs(Fy_out, [xTop, yTop], [sym(0), sym(0)]);

theta_eq = -functionalDerivative(L, theta_pre) == Mtheta;

Equations = [theta_eq];

syms theta dtheta ddtheta real

SymbolReplaced = [theta_pre, diff(theta_pre, t), diff(theta_pre, t, t)];
SymbolReplacing = [theta, dtheta, ddtheta];

Equations = subs(Equations, SymbolReplaced, SymbolReplacing);

Fx_out = expand(subs(Fx_out, SymbolReplaced, SymbolReplacing));
Fy_out = expand(subs(Fy_out, SymbolReplaced, SymbolReplacing));
dmomentumG = subs(dmomentumG, SymbolReplaced, SymbolReplacing);

variables = ddtheta;

[A, B] = equationsToMatrix(Equations, variables);
detA = det(A);
X = simplify(inv(A)*B);

ddtheta_eq = X(1,1);

% parallel.defaultClusterProfile('local');
% c = parcluster();

% matlabFunction(ddtheta_eq, 'file', 'find_ddtheta.m', 'outputs', {'ddtheta'})

% job = createJob(c);
% createTask(job, @matlabFunction, 1,{ddtheta_eq, 'file', 'find_ddtheta.m', 'outputs', {'ddtheta'}});
% submit(job)
% job.Tasks

Fx_out = subs(Fx_out, ddtheta, ddtheta_eq);
Fy_out = subs(Fy_out, ddtheta, ddtheta_eq);
% Fx_out = expand(subs(subs(Fx_out, ddtheta, ddtheta_eq), Mtheta, 0))
% Fy_out = expand(subs(subs(Fy_out, ddtheta, ddtheta_eq), Mtheta, 0))

% matlabFunction(formula(Fx_out), 'file', 'find_Fx_out.m', 'outputs', {'Fx_out'})
% matlabFunction(formula(Fy_out), 'file', 'find_Fy_out.m', 'outputs', {'Fy_out'})

% job = createJob(c);
% createTask(job, @matlabFunction, 1,{Fx_out, 'file', 'find_Fx_out.m', 'outputs', {'Fx_out'}});
% submit(job)
% job.Tasks
% 
% job = createJob(c);
% createTask(job, @matlabFunction, 1,{Fy_out, 'file', 'find_Fy_out.m', 'outputs', {'Fy_out'}});
% submit(job)
% job.Tasks

dmomentumG = subs(dmomentumG, ddtheta, ddtheta_eq)

























