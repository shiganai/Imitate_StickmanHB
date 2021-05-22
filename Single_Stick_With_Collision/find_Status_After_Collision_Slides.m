function [dtheta_After,dx_After,I_F_Y] = find_Status_After_Collision_Slides(Lc,M,dtheta_Before,dx_Before,dy_Before,myu,theta)
%FIND_STATUS_AFTER_COLLISION_SLIDES
%    [DTHETA_AFTER,DX_AFTER,I_F_Y] = FIND_STATUS_AFTER_COLLISION_SLIDES(LC,M,DTHETA_BEFORE,DX_BEFORE,DY_BEFORE,MYU,THETA)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    22-May-2021 22:59:15

t2 = cos(theta);
t3 = sin(theta);
dtheta_After = dtheta_Before+(dy_Before.*(t2-myu.*t3).*6.0)./(Lc.*(t2.^2.*6.0-myu.*t2.*t3.*6.0+2.0));
if nargout > 1
    t4 = theta.*2.0;
    t5 = cos(t4);
    t6 = sin(t4);
    t7 = t5.*3.0;
    t8 = myu.*t6.*3.0;
    t9 = -t8;
    t10 = t7+t9+5.0;
    t11 = 1.0./t10;
    dx_After = dx_Before+dy_Before.*t11.*(myu.*-5.0+t6.*3.0+myu.*t7);
end
if nargout > 2
    I_F_Y = M.*dy_Before.*t11.*-2.0;
end
end