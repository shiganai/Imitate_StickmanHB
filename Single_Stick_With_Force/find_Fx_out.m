function Fx_out = find_Fx_out(Lc,M,Mtheta,dtheta,g,theta)
%FIND_FX_OUT
%    FX_OUT = FIND_FX_OUT(LC,M,MTHETA,DTHETA,G,THETA)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    17-May-2021 15:39:51

t2 = sin(theta);
Fx_out = (cos(theta).*(Mtheta.*3.0-Lc.*M.*g.*t2.*3.0))./(Lc.*4.0)-Lc.*M.*dtheta.^2.*t2;