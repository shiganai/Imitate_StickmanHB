N = 2e7;
M = 3;

for jj = 1:M
    tic
    [ddth_Wrist,ddth_Hip] = find_dd_Onbar(0,0,1,1,0e-6,1,1,1,0,0);
    for ii = 1:N
        [ddth_Wrist,ddth_Hip] = find_dd_Onbar(0,0,1,1,0e-6,1,1,1,0,0);
    end
    toc
    
    tic
    [ddth_Wrist,ddth_Hip] = find_dd_Onbar_Debbug(0,0,1,1,0,1,1,1,0,0);
    for ii = 1:N
        [ddth_Wrist,ddth_Hip] = find_dd_Onbar_Debbug(0,0,1,1,0,1,1,1,0,0);
    end
    toc
    
    tic
    [ddth_Wrist,ddth_Hip] = find_dd_Onbar_Debbug2(0,0,1,1,0,1,1,1,0,0);
    for ii = 1:N
        [ddth_Wrist,ddth_Hip] = find_dd_Onbar_Debbug2(0,0,1,1,0,1,1,1,0,0);
    end
    toc
    
    tic
    [ddth_Wrist,ddth_Hip] = find_dd_Onbar_Debbug2(0,0,1,1,0,1,1,1,0,0);
    for ii = 1:N
        [ddth_Wrist,ddth_Hip] = find_dd_Onbar_Debbug3(0,0,1,1,0,1,1,1,0,0);
    end
    toc
end