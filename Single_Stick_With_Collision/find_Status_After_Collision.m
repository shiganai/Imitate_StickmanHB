function [dtheta_After,I_F_X,I_F_Y] = find_Status_After_Collision(I,Lc,dtheta_Before,dx_Before,dy_Before,m,theta)
%FIND_STATUS_AFTER_COLLISION
%    [DTHETA_AFTER,I_F_X,I_F_Y] = FIND_STATUS_AFTER_COLLISION(I,LC,DTHETA_BEFORE,DX_BEFORE,DY_BEFORE,M,THETA)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    22-May-2021 17:23:05

t2 = cos(theta);
t3 = sin(theta);
t4 = Lc.^2;
t5 = t2.^2;
t6 = m.*t4;
t7 = I+t6;
t8 = 1.0./t7;
dtheta_After = t8.*(I.*dtheta_Before-Lc.*dx_Before.*m.*t3+Lc.*dy_Before.*m.*t2);
if nargout > 1
    I_F_X = -m.*t8.*(I.*dx_Before+dx_Before.*t5.*t6+dy_Before.*t2.*t3.*t6+I.*Lc.*dtheta_Before.*t3);
end
if nargout > 2
    I_F_Y = -m.*t8.*(I.*dy_Before+dy_Before.*t6+(dx_Before.*t6.*sin(theta.*2.0))./2.0-dy_Before.*t5.*t6-I.*Lc.*dtheta_Before.*t2);
end
end