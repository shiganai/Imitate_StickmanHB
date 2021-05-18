function dotq = ddt(t, q, Lc, M, g, Mtheta)
theta = q(1);
dtheta = q(2);

% Mtheta = 0;

ddtheta = find_ddtheta(Lc,M,Mtheta,g,theta);

dotq = [dtheta, ddtheta]';
end