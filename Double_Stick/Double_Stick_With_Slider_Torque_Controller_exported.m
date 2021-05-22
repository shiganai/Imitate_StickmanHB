classdef Double_Stick_With_Slider_Torque_Controller_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                  matlab.ui.Figure
        PlayingButton             matlab.ui.control.StateButton
        Slider                    matlab.ui.control.Slider
        ResetButton               matlab.ui.control.StateButton
        FrameButton               matlab.ui.control.Button
        PlaySpeedxEditFieldLabel  matlab.ui.control.Label
        PlaySpeedxEditField       matlab.ui.control.NumericEditField
        UIAxes                    matlab.ui.control.UIAxes
    end

    
    properties (Access = private)
        Lc1 double
        Lc2 double
        M1 double
        M2 double
        g double
        time_step double
        stick_Body matlab.graphics.chart.primitive.Line
        stick_Leg matlab.graphics.chart.primitive.Line
        th1 double
        dth1 double
        th2 double
        dth2 double
        tau1 double
        tau2 double
        quivers matlab.graphics.chart.primitive.Quiver
        quiver_Ratio double
    end
    
    methods (Access = private)
        
        function initialize_Data(app)
            app.th1 = 2/2 * pi;
            app.dth1 = 0;
            
            app.th2 = 2/2 * pi;
            app.dth2 = 0;
        end
        
        function refresh_Stick(app)
            x1 = app.Lc1 * cos(app.th1 + 3/2 * pi);
            y1 = app.Lc1 * sin(app.th1 + 3/2 * pi);
            x2 = x1 + app.Lc2 * cos(app.th2 + 3/2 * pi);
            y2 = y1 + app.Lc2 * sin(app.th2 + 3/2 * pi);
            
            app.stick_Body.XData(2) = x1;
            app.stick_Body.YData(2) = y1;
            
            app.stick_Leg.XData(1) = x1;
            app.stick_Leg.YData(1) = y1;
            app.stick_Leg.XData(2) = x2;
            app.stick_Leg.YData(2) = y2;
        end
        
        function refresh_Quivers(app)
            Fx_out = find_Fx_out(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,app.tau1,app.tau2,app.th1,app.th2);
            Fy_out = find_Fy_out(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,app.tau1,app.tau2,app.th1,app.th2);
%             Fx_out_G = find_Fx_out(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,0,0,app.th1,app.th2);
%             Fy_out_G = find_Fy_out(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,0,0,app.th1,app.th2);
%             Fx_out_tau2 = Fx_out - find_Fx_out(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,app.tau1,0,app.th1,app.th2);
%             Fy_out_tau2 = Fy_out - find_Fy_out(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,app.tau1,0,app.th1,app.th2);
            
            app.quivers(1).UData = Fx_out / app.quiver_Ratio;
            app.quivers(1).VData = Fy_out / app.quiver_Ratio;
%             app.quivers(2).UData = Fx_out_G / app.quiver_Ratio;
%             app.quivers(2).VData = Fy_out_G / app.quiver_Ratio;
%             app.quivers(3).UData = Fx_out_tau2 / app.quiver_Ratio;
%             app.quivers(3).VData = Fy_out_tau2 / app.quiver_Ratio;
            
%             Fx_in = find_Fx_in(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,app.tau1,app.tau2,app.th1,app.th2);
%             Fy_in = find_Fy_in(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,app.tau1,app.tau2,app.th1,app.th2);
%             Fx_in_G = find_Fx_in(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,0,0,app.th1,app.th2);
%             Fy_in_G = find_Fy_in(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,0,0,app.th1,app.th2);
%             Fx_in_tau2 = Fx_in - find_Fx_in(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,app.tau1,0,app.th1,app.th2);
%             Fy_in_tau2 = Fy_in - find_Fy_in(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,app.tau1,0,app.th1,app.th2);
            
%             app.quivers(4).UData = Fx_in / app.quiver_Ratio;
%             app.quivers(4).VData = Fy_in / app.quiver_Ratio;
%             app.quivers(5).UData = Fx_in_G / app.quiver_Ratio;
%             app.quivers(5).VData = Fy_in_G / app.quiver_Ratio;
%             app.quivers(6).UData = Fx_in_tau2 / app.quiver_Ratio;
%             app.quivers(6).VData = Fy_in_tau2 / app.quiver_Ratio;
            
%             app.quivers(4).XData = app.stick_Body.XData(2);
%             app.quivers(4).YData = app.stick_Body.YData(2);
%             app.quivers(5).XData = app.stick_Body.XData(2);
%             app.quivers(5).YData = app.stick_Body.YData(2);
%             app.quivers(6).XData = app.stick_Body.XData(2);
%             app.quivers(6).YData = app.stick_Body.YData(2);
            
        end
        
        function dotq = ddt(app, q)
            
            th1_Tmp = q(1);
            dth1_Tmp = q(2);
            th2_Tmp = q(3);
            dth2_Tmp = q(4);
            
            [ddth1,ddth2] = find_ddth1_ddth2(app.Lc1,app.Lc2,app.M1,app.M2,dth1_Tmp,dth2_Tmp,app.g,app.tau1,app.tau2,th1_Tmp,th2_Tmp);
            
            dotq = [dth1_Tmp, ddth1, dth2_Tmp, ddth2]';
        end
        
        function run_Ode(app)
            t = [0, app.time_step];
            q0 = [app.th1, app.dth1, app.th2, app.dth2]';
            
            [~, q] = ode45(@(t,q) ddt(app, q), t, q0);
%             [~, q] = ode45(@(t,q) ddt(app, q, app.Lc1, app.Lc2, app.M1, app.M2, app.g, app.tau1, app.tau2), t, q0);
            
            app.th1 = q(end,1);
            app.dth1 = q(end,2);
            app.th2 = q(end,3);
            app.dth2 = q(end,4);
        end
        
        function [ddth1,ddth2] = find_ddth1_ddth2(Lc1,Lc2,M1,M2,dth1,dth2,g,tau1,tau2,th1,th2)
            
            t2 = sin(th1);
            t3 = sin(th2);
            t4 = Lc1.^2;
            t5 = Lc2.^2;
            t6 = M1.*2.0;
            t7 = M2.^2;
            t8 = dth1.^2;
            t9 = dth2.^2;
            t10 = th1.*2.0;
            t11 = th2.*2.0;
            t12 = -th2;
            t13 = -t11;
            t14 = t12+th1;
            t17 = t10+t12;
            t15 = cos(t14);
            t16 = sin(t14);
            t18 = t10+t13;
            t20 = sin(t17);
            t19 = cos(t18);
            t21 = sin(t18);
            t22 = M2.*t19;
            t23 = -t22;
            t24 = M2+t6+t23;
            t25 = 1.0./t24;
            ddth1 = -(t25.*(Lc2.*tau1.*-2.0+Lc2.*tau2.*2.0+Lc1.*t15.*tau2.*2.0+Lc1.*Lc2.*M2.*g.*t2+Lc1.*Lc2.*g.*t2.*t6+Lc1.*M2.*t5.*t9.*t16.*2.0+Lc2.*M2.*t4.*t8.*t21+Lc1.*Lc2.*M2.*g.*sin(t13+th1)))./(Lc2.*t4);
            if nargout > 1
                ddth2 = (t25.*(Lc1.*M2.*tau2.*2.0+Lc1.*t6.*tau2-Lc2.*M2.*t15.*tau1.*2.0+Lc2.*M2.*t15.*tau2.*2.0-Lc1.*Lc2.*g.*t3.*t7+Lc1.*Lc2.*g.*t7.*t20+Lc2.*t4.*t7.*t8.*t16.*2.0+Lc1.*t5.*t7.*t9.*t21-Lc1.*Lc2.*M1.*M2.*g.*t3+Lc1.*Lc2.*M1.*M2.*g.*t20+Lc2.*M2.*t4.*t6.*t8.*t16))./(Lc1.*M2.*t5);
            end
        end
        
        function Fx_in = find_Fx_in(Lc1,Lc2,M1,M2,dth1,dth2,g,tau1,tau2,th1,th2)
            
            t2 = cos(th1);
            t3 = cos(th2);
            t4 = sin(th1);
            t5 = sin(th2);
            t6 = Lc1.^2;
            t7 = Lc2.^2;
            t8 = M2.^2;
            t9 = dth1.^2;
            t10 = dth2.^2;
            t13 = Lc1.*Lc2.*M1.*2.0;
            t11 = t2.^2;
            t12 = t3.^2;
            t17 = Lc1.*Lc2.*M2.*t2.*t3.*t4.*t5.*4.0;
            t14 = Lc1.*Lc2.*M2.*t11.*2.0;
            t15 = Lc1.*Lc2.*M2.*t12.*2.0;
            t16 = Lc1.*Lc2.*M2.*t11.*t12.*4.0;
            t19 = -t17;
            t18 = -t16;
            t20 = t13+t14+t15+t18+t19;
            t21 = 1.0./t20;
            Fx_in = -Lc1.*M2.*t4.*t9-Lc2.*M2.*t5.*t10+Lc1.*M1.*t3.*t21.*tau2.*2.0+Lc2.*M2.*t2.*t21.*tau1.*2.0+Lc1.*M2.*t3.*t21.*tau2.*2.0-Lc2.*M2.*t2.*t21.*tau2.*2.0-Lc1.*M2.*t3.*t11.*t21.*tau2.*2.0-Lc2.*M2.*t2.*t12.*t21.*tau1.*2.0+Lc2.*M2.*t2.*t12.*t21.*tau2.*2.0-Lc1.*M2.*t2.*t4.*t5.*t21.*tau2.*2.0-Lc2.*M2.*t3.*t4.*t5.*t21.*tau1.*2.0+Lc2.*M2.*t3.*t4.*t5.*t21.*tau2.*2.0+M2.*g.*t2.*t4.*t12.*t13.*t21+Lc2.*t4.*t6.*t8.*t9.*t11.*t21.*2.0+Lc2.*t4.*t6.*t8.*t9.*t12.*t21.*2.0+Lc1.*t5.*t7.*t8.*t10.*t11.*t21.*2.0+Lc1.*t5.*t7.*t8.*t10.*t12.*t21.*2.0+Lc1.*t2.*t3.^3.*t4.*t7.*t8.*t10.*t21.*4.0+Lc2.*t2.^3.*t3.*t5.*t6.*t8.*t9.*t21.*4.0-Lc1.*Lc2.*M1.*M2.*g.*t2.*t4.*t21.*2.0+Lc2.*M1.*M2.*t4.*t6.*t9.*t12.*t21.*2.0-Lc1.*t2.*t3.*t4.*t7.*t8.*t10.*t21.*4.0-Lc2.*t2.*t3.*t5.*t6.*t8.*t9.*t21.*4.0-Lc2.*t4.*t6.*t8.*t9.*t11.*t12.*t21.*4.0-Lc1.*t5.*t7.*t8.*t10.*t11.*t12.*t21.*4.0-Lc1.*Lc2.*M1.*M2.*g.*t3.*t5.*t11.*t21.*2.0-Lc2.*M1.*M2.*t2.*t3.*t5.*t6.*t9.*t21.*2.0;
        end
        
        function Fx_out = find_Fx_out(Lc1,Lc2,M1,M2,dth1,dth2,g,tau1,tau2,th1,th2)
            
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
        
        function Fy_in = find_Fy_in(Lc1,Lc2,M1,M2,dth1,dth2,g,tau1,tau2,th1,th2)
            
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
        
        function Fy_out = find_Fy_out(Lc1,Lc2,M1,M2,dth1,dth2,g,tau1,tau2,th1,th2)
            
            t2 = cos(th1);
            t3 = sin(th1);
            t4 = sin(th2);
            t5 = Lc1.^2;
            t6 = Lc2.^2;
            t7 = M1.^2;
            t8 = dth1.^2;
            t9 = dth2.^2;
            t10 = th1.*2.0;
            t11 = th2.*2.0;
            t13 = -th2;
            t12 = cos(t10);
            t14 = -t11;
            t17 = t10+t13;
            t15 = t14+th1;
            t18 = sin(t17);
            t16 = sin(t15);
            Fy_out = (Lc1.*Lc2.*g.*t7+Lc2.*M1.*t3.*tau1.*2.0+Lc1.*M1.*t4.*tau2-Lc2.*M1.*t3.*tau2.*2.0+Lc2.*M2.*t3.*tau1+Lc1.*M2.*t4.*tau2-Lc2.*M2.*t3.*tau2+Lc2.*M2.*t16.*tau1-Lc1.*M1.*t18.*tau2-Lc2.*M2.*t16.*tau2-Lc1.*M2.*t18.*tau2+Lc1.*Lc2.*g.*t7.*t12+Lc2.*t2.*t5.*t7.*t8.*2.0+Lc1.*Lc2.*M1.*M2.*g+Lc1.*M1.*M2.*t6.*t9.*cos(t17)+Lc1.*M1.*M2.*t6.*t9.*cos(th2)+Lc1.*Lc2.*M1.*M2.*g.*t12+Lc2.*M1.*M2.*t2.*t5.*t8.*2.0)./(Lc1.*Lc2.*(M1.*2.0+M2-M2.*cos(t10+t14)));
        end
        
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.Lc1 = 1;
            app.M1 = 2;
            app.Lc2 = 1;
            app.M2 = 2;
            
            app.g = 1;
            app.time_step = 0.05;
            
            initialize_Data(app)
            
            app.tau1 = 0;
            app.tau2 = 0;
            
            app.quiver_Ratio = 4;
            
            app.stick_Body = plot(app.UIAxes, ...
                [0, app.Lc1 * cos(app.th1 + 3/2 * pi)], ...
                [0, app.Lc1 * sin(app.th1 + 3/2 * pi)], ...
                '-', 'LineWidth', 3);
            
            hold(app.UIAxes, "on")
            app.stick_Leg = plot(app.UIAxes, ...
                [app.Lc1 * cos(app.th1 + 3/2 * pi), app.Lc1 * cos(app.th1 + 3/2 * pi) + app.Lc2 * cos(app.th2 + 3/2 * pi)], ...
                [app.Lc1 * sin(app.th1 + 3/2 * pi), app.Lc1 * sin(app.th1 + 3/2 * pi) + app.Lc2 * sin(app.th2 + 3/2 * pi)], ...
                '-', 'LineWidth', 3);
            hold(app.UIAxes, "off")
            
            xlim(app.UIAxes, [-(app.Lc1 + app.Lc2), app.Lc1 + app.Lc2])
            ylim(app.UIAxes, [-(app.Lc1 + app.Lc2), app.Lc1 + app.Lc2])
            
            Fx_out = find_Fx_out(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,app.tau1,app.tau2,app.th1,app.th2);
            Fy_out = find_Fy_out(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,app.tau1,app.tau2,app.th1,app.th2);
            Fx_out_G = find_Fx_out(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,0,0,app.th1,app.th2);
            Fy_out_G = find_Fy_out(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,0,0,app.th1,app.th2);
            Fx_out_tau2 = Fx_out - find_Fx_out(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,app.tau1,0,app.th1,app.th2);
            Fy_out_tau2 = Fy_out - find_Fy_out(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,app.tau1,0,app.th1,app.th2);
            
            Fx_in = find_Fx_in(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,app.tau1,app.tau2,app.th1,app.th2);
            Fy_in = find_Fy_in(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,app.tau1,app.tau2,app.th1,app.th2);
            Fx_in_G = find_Fx_in(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,0,0,app.th1,app.th2);
            Fy_in_G = find_Fy_in(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,0,0,app.th1,app.th2);
            Fx_in_tau2 = Fx_in - find_Fx_in(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,app.tau1,0,app.th1,app.th2);
            Fy_in_tau2 = Fy_in - find_Fy_in(app.Lc1,app.Lc2,app.M1,app.M2,app.dth1,app.dth2,app.g,app.tau1,0,app.th1,app.th2);
            
            hold(app.UIAxes, "on")
            app.quivers(1,1) = quiver(app.UIAxes, 0, 0, Fx_out / app.quiver_Ratio, Fy_out / app.quiver_Ratio);
            app.quivers(2,1) = quiver(app.UIAxes, 0, 0, Fx_out_G / app.quiver_Ratio, Fy_out_G / app.quiver_Ratio, 'Color', 'none');
            app.quivers(3,1) = quiver(app.UIAxes, 0, 0, Fx_out_tau2 / app.quiver_Ratio, Fy_out_tau2 / app.quiver_Ratio, 'Color', 'none');
            app.quivers(4,1) = quiver(app.UIAxes, app.stick_Body.XData(2), app.stick_Body.YData(2), Fx_in / app.quiver_Ratio, Fy_in / app.quiver_Ratio, 'Color', 'none');
            app.quivers(5,1) = quiver(app.UIAxes, app.stick_Body.XData(2), app.stick_Body.YData(2), Fx_in_G / app.quiver_Ratio, Fy_in_G / app.quiver_Ratio, 'Color', 'none');
            app.quivers(6,1) = quiver(app.UIAxes, app.stick_Body.XData(2), app.stick_Body.YData(2), Fx_in_tau2 / app.quiver_Ratio, Fy_in_tau2 / app.quiver_Ratio, 'Color', 'none');
            hold(app.UIAxes, "off")
            
        end

        % Value changed function: PlayingButton
        function PlayingButtonValueChanged(app, event)
            value = app.PlayingButton.Value;
            if value
                while true
                    
                    run_Ode(app)
                    
                    refresh_Stick(app)
                    refresh_Quivers(app)
                    
                    pause(app.time_step / app.PlaySpeedxEditField.Value)
                    drawnow
                    
                    if ~app.PlayingButton.Value
                        app.ResetButton.Value = false;
                        break
                    end
                end
            end
        end

        % Value changing function: Slider
        function SliderValueChanging(app, event)
            changingValue = event.Value;
            app.tau2 = changingValue;
            refresh_Quivers(app)
            drawnow
        end

        % Value changed function: ResetButton
        function ResetButtonValueChanged(app, event)
            value = app.ResetButton.Value;
            if value
                if app.PlayingButton.Value
                    app.PlayingButton.Value = false;
                else
                    app.ResetButton.Value = false;
                end
                
                initialize_Data(app)
                
                refresh_Stick(app)
                refresh_Quivers(app)
                drawnow
            end
        end

        % Button pushed function: FrameButton
        function FrameButtonPushed(app, event)
            if ~app.PlayingButton.Value
                run_Ode(app)
                
                refresh_Stick(app)
                refresh_Quivers(app)
                
                pause(app.time_step/1.2)
                drawnow
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [400 100 939 724];
            app.UIFigure.Name = 'MATLAB App';

            % Create PlayingButton
            app.PlayingButton = uibutton(app.UIFigure, 'state');
            app.PlayingButton.ValueChangedFcn = createCallbackFcn(app, @PlayingButtonValueChanged, true);
            app.PlayingButton.Text = 'Playing';
            app.PlayingButton.Position = [436 232 70 22];

            % Create Slider
            app.Slider = uislider(app.UIFigure);
            app.Slider.Limits = [-1 1];
            app.Slider.MajorTicks = [-1 -0.5 0 0.5 1];
            app.Slider.MajorTickLabels = {'-1', '-0.5', '0', '0.5', '1'};
            app.Slider.ValueChangingFcn = createCallbackFcn(app, @SliderValueChanging, true);
            app.Slider.MinorTicks = [-1 -0.96 -0.92 -0.88 -0.84 -0.8 -0.76 -0.72 -0.68 -0.64 -0.6 -0.56 -0.52 -0.48 -0.44 -0.4 -0.36 -0.32 -0.28 -0.24 -0.2 -0.16 -0.12 -0.08 -0.04 0 0.04 0.0800000000000001 0.12 0.16 0.2 0.24 0.28 0.32 0.36 0.4 0.44 0.48 0.52 0.56 0.6 0.64 0.68 0.72 0.76 0.8 0.84 0.88 0.92 0.96 1];
            app.Slider.Position = [358 79 226 3];

            % Create ResetButton
            app.ResetButton = uibutton(app.UIFigure, 'state');
            app.ResetButton.ValueChangedFcn = createCallbackFcn(app, @ResetButtonValueChanged, true);
            app.ResetButton.Text = 'Reset';
            app.ResetButton.Position = [436 196 70 22];

            % Create FrameButton
            app.FrameButton = uibutton(app.UIFigure, 'push');
            app.FrameButton.ButtonPushedFcn = createCallbackFcn(app, @FrameButtonPushed, true);
            app.FrameButton.Position = [436 160 70 22];
            app.FrameButton.Text = '1 Frame';

            % Create PlaySpeedxEditFieldLabel
            app.PlaySpeedxEditFieldLabel = uilabel(app.UIFigure);
            app.PlaySpeedxEditFieldLabel.HorizontalAlignment = 'right';
            app.PlaySpeedxEditFieldLabel.Position = [416 113 76 22];
            app.PlaySpeedxEditFieldLabel.Text = 'Play Speed x';

            % Create PlaySpeedxEditField
            app.PlaySpeedxEditField = uieditfield(app.UIFigure, 'numeric');
            app.PlaySpeedxEditField.Limits = [0.1 2];
            app.PlaySpeedxEditField.Position = [491 113 29 22];
            app.PlaySpeedxEditField.Value = 0.2;

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            app.UIAxes.DataAspectRatio = [1 1 1];
            app.UIAxes.XColor = 'none';
            app.UIAxes.YColor = 'none';
            app.UIAxes.FontSize = 12;
            app.UIAxes.Position = [253 265 439 439];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = Double_Stick_With_Slider_Torque_Controller_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end