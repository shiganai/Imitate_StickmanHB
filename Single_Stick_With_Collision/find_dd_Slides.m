function [ddx,ddtheta,f_X,f_Y] = find_dd_Slides(Lc,M,dtheta,g,myu,tau,theta)
%FIND_DD_SLIDES
%    [DDX,DDTHETA,F_X,F_Y] = FIND_DD_SLIDES(LC,M,DTHETA,G,MYU,TAU,THETA)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    22-May-2021 20:08:54

t2 = cos(theta);
t3 = sin(theta);
t4 = M.*g;
t5 = Lc.^2;
t6 = dtheta.^2;
t7 = tau.*3.0;
t8 = theta.*2.0;
t11 = 1.0./Lc;
t12 = 1.0./M;
t9 = cos(t8);
t10 = sin(t8);
t15 = Lc.*t2.*t4.*3.0;
t18 = Lc.*M.*t3.*t6;
t19 = Lc.*myu.*t3.*t4.*3.0;
t21 = M.*myu.*t5.*t6.*(3.0./2.0);
t13 = t9.*3.0;
t14 = myu.*t10.*3.0;
t17 = -t15;
t20 = -t18;
t22 = -t21;
t23 = M.*t5.*t6.*t10.*(3.0./2.0);
t25 = t9.*t21;
t16 = -t14;
t27 = t7+t17+t19+t22+t23+t25;
t24 = t13+t16+5.0;
t26 = 1.0./t24;
ddx = t11.*t12.*t26.*(t3.*t7+Lc.*myu.*t4.*(5.0./2.0)-Lc.*t4.*t10.*(3.0./2.0)+myu.*t2.*t7-Lc.*myu.*t4.*t9.*(3.0./2.0)+M.*t2.*t5.*t6.*4.0-M.*myu.*t3.*t5.*t6.*4.0).*2.0;
if nargout > 1
    ddtheta = (t12.*t26.*t27.*2.0)./t5;
end
if nargout > 2
    t28 = t2.*t11.*t26.*t27.*2.0;
    t29 = t4+t20+t28;
    f_X = myu.*t29;
end
if nargout > 3
    f_Y = t29;
end
