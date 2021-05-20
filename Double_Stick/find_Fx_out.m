function Fx_out = find_Fx_out(Lc1,Lc2,M1,M2,dth1,dth2,g,tau1,tau2,th1,th2)
%FIND_FX_OUT
%    FX_OUT = FIND_FX_OUT(LC1,LC2,M1,M2,DTH1,DTH2,G,TAU1,TAU2,TH1,TH2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    19-May-2021 23:24:35

t2 = cos(th1);
t3 = cos(th2);
t4 = sin(th1);
t5 = Lc1.^2;
t6 = Lc2.^2;
t7 = M1.^2;
t8 = dth1.^2;
t9 = dth2.^2;
t10 = th1.*2.0;
t11 = th2.*2.0;
t13 = -th2;
t12 = sin(t10);
t14 = -t11;
t17 = t10+t13;
t15 = t14+th1;
t18 = cos(t17);
t16 = cos(t15);
Fx_out = -(Lc2.*M1.*t2.*tau1.*-2.0-Lc1.*M1.*t3.*tau2+Lc2.*M1.*t2.*tau2.*2.0-Lc2.*M2.*t2.*tau1-Lc1.*M2.*t3.*tau2+Lc2.*M2.*t2.*tau2+Lc2.*M2.*t16.*tau1+Lc1.*M1.*t18.*tau2-Lc2.*M2.*t16.*tau2+Lc1.*M2.*t18.*tau2+Lc1.*Lc2.*g.*t7.*t12+Lc2.*t4.*t5.*t7.*t8.*2.0+Lc1.*M1.*M2.*t6.*t9.*sin(t17)+Lc1.*M1.*M2.*t6.*t9.*sin(th2)+Lc1.*Lc2.*M1.*M2.*g.*t12+Lc2.*M1.*M2.*t4.*t5.*t8.*2.0)./(Lc1.*Lc2.*(M1.*2.0+M2-M2.*cos(t10+t14)));
end