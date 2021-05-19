function Fy_out = find_Fy_out(Lc,M,Mtheta,dtheta,g,theta)
%FIND_FY_OUT
%    FY_OUT = FIND_FY_OUT(LC,M,MTHETA,DTHETA,G,THETA)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    17-May-2021 15:39:51

t2 = sin(theta);
Fy_out = M.*g+(t2.*(Mtheta.*3.0-Lc.*M.*g.*t2.*3.0))./(Lc.*4.0)+Lc.*M.*dtheta.^2.*cos(theta);