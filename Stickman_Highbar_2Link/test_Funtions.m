ddl_Wrist_Bar = 0;
ddth_Hip = 0;
ddth_Wrist = 0;

dl_Wrist_Bar = 0;
dth_Hip = 0;
dth_Wrist = 0;
f_Wrist_Bar = 0;
g = 9.8;
l_Body = 1;
l_Leg = 1;
l_Wrist_Bar = 0;
m_Body = 1;
m_Leg = 1;
tau_Hip = 0;
th_Hip = 0;
th_Wrist = 0;

f_Wrist_Bar = find_F_Wrist_Bar(ddl_Wrist_Bar,ddth_Hip,ddth_Wrist,dth_Hip,dth_Wrist,g,l_Body,l_Leg,l_Wrist_Bar,m_Body,m_Leg,th_Hip,th_Wrist)
[f_X,f_Y] = find_F_Catch(dl_Wrist_Bar,dth_Hip,dth_Wrist,f_Wrist_Bar,g,l_Body,l_Leg,l_Wrist_Bar,m_Body,m_Leg,tau_Hip,th_Hip,th_Wrist)
[f_X,f_Y] = find_F_Onbar(dth_Hip,dth_Wrist,g,l_Body,l_Leg,l_Wrist_Bar,m_Body,m_Leg,tau_Hip,th_Hip,th_Wrist)