function [dth_Wrist_After,dth_Hip_After,I_F_Wrist_Bar] = find_Status_After_Slides_Collision(dl_Wrist_Bar_Before,dth_Hip_Before,dth_Wrist_Before,l_Body,l_Leg,l_Wrist_Bar,m_Body,m_Leg,th_Hip)
%FIND_STATUS_AFTER_SLIDES_COLLISION
%    [DTH_WRIST_AFTER,DTH_HIP_AFTER,I_F_WRIST_BAR] = FIND_STATUS_AFTER_SLIDES_COLLISION(DL_WRIST_BAR_BEFORE,DTH_HIP_BEFORE,DTH_WRIST_BEFORE,L_BODY,L_LEG,L_WRIST_BAR,M_BODY,M_LEG,TH_HIP)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    23-May-2021 12:49:54

t2 = cos(th_Hip);
t3 = sin(th_Hip);
t4 = l_Body.^2;
t5 = l_Wrist_Bar.^2;
t6 = m_Body.^2;
t7 = m_Leg.^2;
t8 = th_Hip.*2.0;
t12 = l_Body.*l_Wrist_Bar.*m_Body.*2.4e+1;
t13 = l_Body.*l_Wrist_Bar.*m_Leg.*3.0e+1;
t9 = cos(t8);
t10 = t2.^2;
t11 = sin(t8);
t14 = m_Body.*t4.*8.0;
t15 = m_Leg.*t4.*1.5e+1;
t16 = m_Body.*t5.*2.4e+1;
t17 = m_Leg.*t5.*1.5e+1;
t18 = -t12;
t19 = -t13;
t20 = l_Body.*l_Wrist_Bar.*m_Leg.*t9.*1.8e+1;
t21 = m_Leg.*t4.*t9.*9.0;
t22 = m_Leg.*t5.*t9.*9.0;
t23 = -t21;
t24 = -t22;
t25 = t14+t15+t16+t17+t18+t19+t20+t23+t24;
t26 = 1.0./t25;
dth_Wrist_After = t26.*(dth_Wrist_Before.*t14+dth_Wrist_Before.*t15+dth_Wrist_Before.*t16+dth_Wrist_Before.*t17+dth_Wrist_Before.*t20-dth_Wrist_Before.*l_Body.*l_Wrist_Bar.*m_Body.*2.4e+1-dth_Wrist_Before.*l_Body.*l_Wrist_Bar.*m_Leg.*3.0e+1-dl_Wrist_Bar_Before.*l_Body.*m_Leg.*t11.*9.0+dl_Wrist_Bar_Before.*l_Wrist_Bar.*m_Leg.*t11.*9.0-dth_Wrist_Before.*m_Leg.*t4.*t9.*9.0-dth_Wrist_Before.*m_Leg.*t5.*t9.*9.0);
if nargout > 1
    dth_Hip_After = (t26.*(dth_Hip_Before.*l_Leg.*t14+dth_Hip_Before.*l_Leg.*t15+dth_Hip_Before.*l_Leg.*t16+dth_Hip_Before.*l_Leg.*t17+dth_Hip_Before.*l_Leg.*t20+dl_Wrist_Bar_Before.*m_Body.*t3.*t4.*1.2e+1+dl_Wrist_Bar_Before.*m_Body.*t3.*t5.*3.6e+1+dl_Wrist_Bar_Before.*m_Leg.*t3.*t4.*3.6e+1+dl_Wrist_Bar_Before.*m_Leg.*t3.*t5.*3.6e+1-dth_Hip_Before.*l_Body.*l_Leg.*l_Wrist_Bar.*m_Body.*2.4e+1-dth_Hip_Before.*l_Body.*l_Leg.*l_Wrist_Bar.*m_Leg.*3.0e+1+dl_Wrist_Bar_Before.*l_Body.*l_Leg.*m_Leg.*t11.*9.0-dl_Wrist_Bar_Before.*l_Body.*l_Wrist_Bar.*m_Body.*t3.*3.6e+1-dl_Wrist_Bar_Before.*l_Body.*l_Wrist_Bar.*m_Leg.*t3.*7.2e+1-dl_Wrist_Bar_Before.*l_Leg.*l_Wrist_Bar.*m_Leg.*t11.*9.0-dth_Hip_Before.*l_Leg.*m_Leg.*t4.*t9.*9.0-dth_Hip_Before.*l_Leg.*m_Leg.*t5.*t9.*9.0))./l_Leg;
end
if nargout > 2
    I_F_Wrist_Bar = -(dl_Wrist_Bar_Before.*(m_Body.*t17+t4.*t6.*4.0+t4.*t7.*3.0+t5.*t6.*1.2e+1+t5.*t7.*3.0-l_Body.*l_Wrist_Bar.*t6.*1.2e+1-l_Body.*l_Wrist_Bar.*t7.*6.0+m_Body.*m_Leg.*t4.*1.3e+1-l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*2.7e+1-m_Body.*m_Leg.*t4.*t10.*6.0+l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*t10.*9.0))./(m_Body.*t4.*4.0+m_Body.*t5.*1.2e+1+m_Leg.*t4.*1.2e+1+m_Leg.*t5.*1.2e+1-l_Body.*l_Wrist_Bar.*m_Body.*1.2e+1-l_Body.*l_Wrist_Bar.*m_Leg.*2.4e+1-m_Leg.*t4.*t10.*9.0-m_Leg.*t5.*t10.*9.0+l_Body.*l_Wrist_Bar.*m_Leg.*t10.*1.8e+1);
end