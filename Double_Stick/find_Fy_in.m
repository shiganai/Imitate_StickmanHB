function Fy_in = find_Fy_in(Lc1,Lc2,M1,M2,dth1,dth2,g,tau1,tau2,th1,th2)
%FIND_FY_IN
%    FY_IN = FIND_FY_IN(LC1,LC2,M1,M2,DTH1,DTH2,G,TAU1,TAU2,TH1,TH2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    19-May-2021 23:24:35

t2 = cos(th1);
t3 = cos(th2);
t4 = sin(th1);
t5 = sin(th2);
t6 = Lc1.^2;
t7 = Lc2.^2;
t8 = M2.^2;
t9 = dth1.^2;
t10 = dth2.^2;
t15 = Lc1.*Lc2.*M1.*2.0;
t11 = t2.^2;
t12 = t3.^2;
t13 = t4.^2;
t14 = t5.^2;
t19 = Lc1.*Lc2.*M2.*t2.*t3.*t4.*t5.*4.0;
t16 = Lc1.*Lc2.*M2.*t11.*2.0;
t17 = Lc1.*Lc2.*M2.*t12.*2.0;
t18 = Lc1.*Lc2.*M2.*t11.*t12.*4.0;
t21 = -t19;
t20 = -t18;
t22 = t15+t16+t17+t20+t21;
t23 = 1.0./t22;
Fy_in = M2.*g+Lc1.*M2.*t2.*t9+Lc2.*M2.*t3.*t10+Lc1.*M1.*t5.*t23.*tau2.*2.0+Lc2.*M2.*t4.*t23.*tau1.*2.0+Lc1.*M2.*t5.*t23.*tau2.*2.0-Lc2.*M2.*t4.*t23.*tau2.*2.0-Lc1.*M2.*t5.*t13.*t23.*tau2.*2.0-Lc2.*M2.*t4.*t14.*t23.*tau1.*2.0+Lc2.*M2.*t4.*t14.*t23.*tau2.*2.0-Lc1.*Lc2.*g.*t8.*t11.*t14.*t23.*2.0-Lc1.*Lc2.*g.*t8.*t12.*t13.*t23.*2.0-Lc1.*M2.*t2.*t3.*t4.*t23.*tau2.*2.0-Lc2.*M2.*t2.*t3.*t5.*t23.*tau1.*2.0+Lc2.*M2.*t2.*t3.*t5.*t23.*tau2.*2.0+Lc2.*t2.*t6.*t8.*t9.*t13.*t23.*2.0-Lc2.*t2.*t6.*t8.*t9.*t14.*t23.*2.0-Lc1.*t3.*t7.*t8.*t10.*t13.*t23.*2.0+Lc1.*t3.*t7.*t8.*t10.*t14.*t23.*2.0-Lc1.*Lc2.*M1.*M2.*g.*t13.*t23.*2.0-Lc1.*Lc2.*M1.*M2.*g.*t11.*t14.*t23.*2.0-Lc2.*M1.*M2.*t2.*t6.*t9.*t14.*t23.*2.0+M2.*g.*t2.*t3.*t4.*t5.*t15.*t23-Lc2.*t2.*t6.*t8.*t9.*t12.*t13.*t23.*4.0-Lc1.*t3.*t7.*t8.*t10.*t11.*t14.*t23.*4.0+Lc1.*Lc2.*g.*t2.*t3.*t4.*t5.*t8.*t23.*4.0+Lc2.*t3.*t4.*t5.*t6.*t8.*t9.*t11.*t23.*4.0+Lc1.*t2.*t4.*t5.*t7.*t8.*t10.*t12.*t23.*4.0+Lc2.*M1.*M2.*t3.*t4.*t5.*t6.*t9.*t23.*2.0;
end