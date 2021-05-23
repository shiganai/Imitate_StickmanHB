classdef Double_Stick_With_Slider_Torque_Controller_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                   matlab.ui.Figure
        PlayingButton              matlab.ui.control.StateButton
        Slider                     matlab.ui.control.Slider
        ResetButton                matlab.ui.control.StateButton
        FrameButton                matlab.ui.control.Button
        PlaySpeedxEditFieldLabel   matlab.ui.control.Label
        PlaySpeedxEditField        matlab.ui.control.NumericEditField
        Take_OffButton             matlab.ui.control.Button
        InitializationPanel        matlab.ui.container.Panel
        Status_Buttons             matlab.ui.container.ButtonGroup
        OnBarButton                matlab.ui.control.ToggleButton
        InAirButton                matlab.ui.control.ToggleButton
        xGSliderLabel              matlab.ui.control.Label
        xGSlider                   matlab.ui.control.Slider
        dxGSliderLabel             matlab.ui.control.Label
        dxGSlider                  matlab.ui.control.Slider
        yGSliderLabel              matlab.ui.control.Label
        yGSlider                   matlab.ui.control.Slider
        dyGSliderLabel             matlab.ui.control.Label
        dyGSlider                  matlab.ui.control.Slider
        WLabel                     matlab.ui.control.Label
        th_WristSlider             matlab.ui.control.Slider
        WLabel_2                   matlab.ui.control.Label
        dth_WristSlider            matlab.ui.control.Slider
        HLabel                     matlab.ui.control.Label
        th_HipSlider               matlab.ui.control.Slider
        HLabel_2                   matlab.ui.control.Label
        dth_HipSlider              matlab.ui.control.Slider
        StatusLabel                matlab.ui.control.Label
        AngularMomentumGaugeLabel  matlab.ui.control.Label
        AngularMomentumGauge       matlab.ui.control.SemicircularGauge
        UIAxes                     matlab.ui.control.UIAxes
    end

    
    properties (Access = private)
        l_Body = 1
        l_Leg = 1
        m_Body = 2
        m_Leg = 2
        g = 10
        myu = 0.5
        r_Head = 0.12
        
        l_Wrist_Bar double
        dl_Wrist_Bar double
        
        x_Wrist double
        dx_Wrist double
        y_Wrist double
        dy_Wrist double
        
        th_Wrist double
        dth_Wrist double
        th_Hip double
        dth_Hip double
        
        tau_Wrist = 0
        tau_Hip = 0
        
        f_X = 0
        f_Y = 0
        
        %         status char
        
        time_step = 0.01
        
        stick_Body matlab.graphics.chart.primitive.Line
        stick_Leg matlab.graphics.chart.primitive.Line
        head matlab.graphics.primitive.Rectangle
        
        quivers matlab.graphics.chart.primitive.Quiver
        quiver_Ratio double
        
        slider_Positions double
        Initialize_Data_Array double
        
        tau_M_0_Ext = 1 % 正
        theta_A_0_Ext = 42
        theta_A_W_Ext = 70 % 正
        omega_Max_Ext = 526 % 正
        t_V_E_Max_Ext = 1.12
        t_V_C_Half_Ext = 0.25
        theta_PE_0_Ext = -0 % 負
        theta_PE_1_Ext = -90 % 負
        
        tau_M_0_Flex = -1 % 負 文献値と正負逆
        theta_A_0_Flex = 61 % 文献値と正負逆
        theta_A_W_Flex = 61 % 正
        omega_Max_Flex = -517 % 負 文献値と正負逆
        t_V_E_Max_Flex = 1.10
        t_V_C_Half_Flex = 0.16
        theta_PE_0_Flex = 0 % 正
        theta_PE_1_Flex = 150 % 正
        
        c = 1e2
    end
    
    methods (Access = private)
        
        function initialize_Data(app)
            app.StatusLabel.Text = app.Status_Buttons.SelectedObject.Text;
            
            if isequal(app.StatusLabel.Text, 'InAir')
                
                app.th_Wrist = deg2rad(app.Initialize_Data_Array(5));
                app.dth_Wrist = app.Initialize_Data_Array(6);
                
                app.th_Hip = deg2rad(app.Initialize_Data_Array(7));
                app.dth_Hip = app.Initialize_Data_Array(8);
                
                p_Wrist = [0,0];
                p_Hip = p_Wrist + app.l_Body * [cos(app.th_Wrist + 1/2 * pi), sin(app.th_Wrist + 1/2 * pi)];
                p_Toe = p_Hip + app.l_Leg * [cos(app.th_Wrist + 1/2 * pi + app.th_Hip), sin(app.th_Wrist + 1/2 * pi + app.th_Hip)];
                
                p_Body = 1/2 * (p_Wrist + p_Hip);
                p_Leg = 1/2 * (p_Hip + p_Toe);
                
                p_G = (app.m_Body * p_Body + app.m_Leg * p_Leg)/(app.m_Body + app.m_Leg);
                
                v_Wrist = [0,0];
                v_Hip = v_Wrist + app.l_Body * app.dth_Wrist * [-sin(app.th_Wrist + 1/2 * pi), cos(app.th_Wrist + 1/2 * pi)];
                v_Toe = v_Hip + app.l_Leg * (app.dth_Wrist + app.dth_Hip) * ...
                    [-sin(app.th_Wrist + 1/2 * pi + app.th_Hip), cos(app.th_Wrist + 1/2 * pi + app.th_Hip)];
                
                v_Body = 1/2 * (v_Wrist + v_Hip);
                v_Leg = 1/2 * (v_Hip + v_Toe);
                
                v_G = (app.m_Body * v_Body + app.m_Leg * v_Leg)/(app.m_Body + app.m_Leg);
                
                app.x_Wrist = app.Initialize_Data_Array(1) - p_G(1);
                app.dx_Wrist = app.Initialize_Data_Array(2) - v_G(1);
                
                app.y_Wrist = app.Initialize_Data_Array(3) - p_G(2);
                app.dy_Wrist = app.Initialize_Data_Array(4) - v_G(2);
            elseif isequal(app.StatusLabel.Text, 'OnBar')
                app.x_Wrist = 0;
                app.dx_Wrist = 0;
                
                app.y_Wrist = 0;
                app.dy_Wrist = 0;
                
                app.th_Wrist = deg2rad(app.Initialize_Data_Array(5));
                app.dth_Wrist = app.Initialize_Data_Array(6);
                
                app.th_Hip = deg2rad(app.Initialize_Data_Array(7));
                app.dth_Hip = app.Initialize_Data_Array(8);
            end
            
            [x_Cross, y_Cross] = calc_Cross(app, app.th_Wrist, app.x_Wrist, app.y_Wrist);
            
            app.l_Wrist_Bar = vecnorm([x_Cross, y_Cross] - [app.x_Wrist, app.y_Wrist]);
            app.dl_Wrist_Bar = -app.dx_Wrist * cos(app.th_Wrist + 1/2 * pi) - app.dy_Wrist * sin(app.th_Wrist + 1/2 * pi);
            
            %{
            app.StatusLabel.Text = 'OnBar';
            
            app.l_Wrist_Bar = 0;
            
            app.th_Wrist = 0/2 * pi;
            app.dth_Wrist = 0;
            
            app.th_Hip = 0/2 * pi;
            app.dth_Hip = 0;
            
            app.x_Wrist = -app.l_Wrist_Bar * cos(app.th_Wrist + 1/2 * pi);
            app.dx_Wrist = app.l_Wrist_Bar * app.dth_Wrist * sin(app.th_Wrist + 1/2 * pi);
            
            app.y_Wrist = -app.l_Wrist_Bar * sin(app.th_Wrist + 1/2 * pi);
            app.dy_Wrist = -app.l_Wrist_Bar * app.dth_Wrist * cos(app.th_Wrist + 1/2 * pi);
            %}
        end
        
        function refresh_Stick(app)
            
            x0 = app.x_Wrist;
            y0 = app.y_Wrist;
            x1 = x0 + app.l_Body * cos(app.th_Wrist + 1/2 * pi);
            y1 = y0 + app.l_Body * sin(app.th_Wrist + 1/2 * pi);
            x2 = x1 + app.l_Leg * cos(app.th_Wrist + 1/2 * pi + app.th_Hip);
            y2 = y1 + app.l_Leg * sin(app.th_Wrist + 1/2 * pi + app.th_Hip);
            
            x_Head = x0 + 2/5 * app.l_Body * cos(app.th_Wrist + 1/2 * pi) + 1.2 * app.r_Head * cos(app.th_Wrist);
            y_Head = y0 + 2/5 * app.l_Body * sin(app.th_Wrist + 1/2 * pi) + 1.2 * app.r_Head * sin(app.th_Wrist);
            
            app.stick_Body.XData(1) = x0;
            app.stick_Body.YData(1) = y0;
            app.stick_Body.XData(2) = x1;
            app.stick_Body.YData(2) = y1;
            
            app.stick_Leg.XData(1) = x1;
            app.stick_Leg.YData(1) = y1;
            app.stick_Leg.XData(2) = x2;
            app.stick_Leg.YData(2) = y2;
            
            app.head.Position(1) = x_Head - app.r_Head;
            app.head.Position(2) = y_Head - app.r_Head;
            
            if isequal(app.StatusLabel.Text, 'InAir')
            else
                ang_M = find_Ang_M(app.dth_Hip,app.dth_Wrist,app.l_Body,app.l_Leg,app.m_Body,app.m_Leg,app.th_Hip);
                
                app.AngularMomentumGauge.Value = ang_M;
            end
        end
        
        function refresh_Quivers(app)
            if isequal(app.StatusLabel.Text, 'OnBar')
                [app.f_X,app.f_Y] = find_F_Onbar(app.dth_Hip,app.dth_Wrist,app.g,app.l_Body,app.l_Leg,app.l_Wrist_Bar,app.m_Body,app.m_Leg,app.tau_Hip,app.th_Hip,app.th_Wrist);
            elseif isequal(app.StatusLabel.Text, 'InAir')
                app.f_X = 0;
                app.f_Y = 0;
            elseif isequal(app.StatusLabel.Text, 'Catch')
                [app.f_X,app.f_Y] = find_F_Catch(app.dl_Wrist_Bar,app.dth_Hip,app.dth_Wrist,app.g,...
                    app.l_Body,app.l_Leg,app.l_Wrist_Bar,app.m_Body,app.m_Leg,app.myu,app.tau_Hip,app.th_Hip,app.th_Wrist);
                
                f_Wrist_Bar = app.f_X * sin(app.th_Wrist) + app.f_Y * -cos(app.th_Wrist);
                
                if sign(f_Wrist_Bar) ~= sign(app.dl_Wrist_Bar)
                    app.myu = -app.myu;
                    [app.f_X,app.f_Y] = find_F_Catch(app.dl_Wrist_Bar,app.dth_Hip,app.dth_Wrist,app.g,...
                        app.l_Body,app.l_Leg,app.l_Wrist_Bar,app.m_Body,app.m_Leg,app.myu,app.tau_Hip,app.th_Hip,app.th_Wrist);
                end
            end
            app.quivers(1).UData = app.f_X / app.quiver_Ratio;
            app.quivers(1).VData = app.f_Y / app.quiver_Ratio;
        end
        
        function run_Ode(app)
            if isequal(app.StatusLabel.Text, 'Failed')
                initialize_Data(app)
            else
                if isequal(app.StatusLabel.Text, 'OnBar')
                    run_Ode_Onbar(app)
                elseif isequal(app.StatusLabel.Text, 'InAir')
                    run_Ode_Inair(app)
                elseif isequal(app.StatusLabel.Text, 'Catch')
                    run_Ode_Catch(app)
                else
                    error('The status has not been determined')
                end
            end
        end
        
        function run_Ode_Onbar(app)
            t = [0, app.time_step];
            
            q0 = [app.th_Wrist, app.dth_Wrist, app.th_Hip, app.dth_Hip]';
            [~, q] = ode45(@(t,q) ddt_Onbar(app, q), t, q0);
            
            app.th_Wrist = q(end,1);
            app.dth_Wrist = q(end,2);
            app.th_Hip = q(end,3);
            app.dth_Hip = q(end,4);
            
            app.x_Wrist = -app.l_Wrist_Bar * cos(app.th_Wrist + 1/2 * pi);
            app.dx_Wrist = app.l_Wrist_Bar * app.dth_Wrist * sin(app.th_Wrist + 1/2 * pi);
            app.y_Wrist = -app.l_Wrist_Bar * sin(app.th_Wrist + 1/2 * pi);
            app.dy_Wrist = -app.l_Wrist_Bar * app.dth_Wrist * cos(app.th_Wrist + 1/2 * pi);
        end
        
        function run_Ode_Inair(app)
            t = [0, app.time_step];
            
            q0 = [app.th_Wrist, app.dth_Wrist, app.th_Hip, app.dth_Hip, app.x_Wrist, app.dx_Wrist, app.y_Wrist, app.dy_Wrist]';
            ode_Event = @(t,q) event_Inair(app, q);
            ode_Options = odeset('Events', ode_Event);
            [time, q, te, ~, ie] = ode45(@(t,q) ddt_Inair(app, q), t, q0, ode_Options);
            
            app.th_Wrist = q(end,1);
            app.dth_Wrist = q(end,2);
            app.th_Hip = q(end,3);
            app.dth_Hip = q(end,4);
            app.x_Wrist = q(end,5);
            app.dx_Wrist = q(end,6);
            app.y_Wrist = q(end,7);
            app.dy_Wrist = q(end,8);
            
            if ~isempty(ie)
                if time(end) == te(end)
                    if ie(end) == 1
                        app.StatusLabel.Text = 'Catch';
                        
                        [x_Cross, y_Cross] = calc_Cross(app, app.th_Wrist, app.x_Wrist, app.y_Wrist);
                        
                        app.l_Wrist_Bar = vecnorm([x_Cross, y_Cross] - [app.x_Wrist, app.y_Wrist]);
                        
                        app.dl_Wrist_Bar = -app.dx_Wrist * cos(app.th_Wrist + 1/2 * pi) - app.dy_Wrist * sin(app.th_Wrist + 1/2 * pi);
                        
                        dl_N_Wrist_Bar = app.dx_Wrist * sin(app.th_Wrist + 1/2 * pi) - app.dy_Wrist * cos(app.th_Wrist + 1/2 * pi) - app.l_Wrist_Bar * app.dth_Wrist;
                        
                        [dth_Wrist_After,dth_Hip_After,dl_Wrist_Bar_After,I_N_Wrist_Bar] = ...
                            find_Status_After_Collision(app.dl_Wrist_Bar,dl_N_Wrist_Bar,app.dth_Hip,app.dth_Wrist,...
                            app.l_Body,app.l_Leg, 0,app.l_Wrist_Bar,app.m_Body,app.m_Leg,app.myu,app.th_Hip);
                        
                        if sign(dl_N_Wrist_Bar) ~= sign(I_N_Wrist_Bar)
                            app.myu = -app.myu;
                            [dth_Wrist_After,dth_Hip_After,dl_Wrist_Bar_After] = ...
                                find_Status_After_Collision(app.dl_Wrist_Bar,dl_N_Wrist_Bar,app.dth_Hip,app.dth_Wrist,...
                                app.l_Body,app.l_Leg, 0,app.l_Wrist_Bar,app.m_Body,app.m_Leg,app.myu,app.th_Hip);
                        end
                        app.dth_Wrist = dth_Wrist_After;
                        app.dth_Hip = dth_Hip_After;
                        app.dl_Wrist_Bar = dl_Wrist_Bar_After;
                    elseif ie(end) == 2
                        app.StatusLabel.Text = 'Failed';
                        
                        app.PlayingButton.Value = false;
                    end
                end
            end
        end
        
        function [value,isterminal,direction] = event_Inair(app, q)
            
            th_Wrist_Tmp = q(1);
            th_Hip_Tmp = q(3);
            
            x_Wrist_Tmp = q(5);
            y_Wrist_Tmp = q(7);
            
            [x_Cross, y_Cross] = calc_Cross(app, th_Wrist_Tmp, x_Wrist_Tmp, y_Wrist_Tmp);
            
            l_Wrist_Bar_Tmp = ([x_Cross, y_Cross] - [x_Wrist_Tmp, y_Wrist_Tmp]) ./ [cos(th_Wrist_Tmp + 1/2 * pi), sin(th_Wrist_Tmp + 1/2 * pi)];
            
            cross_Vec = [cos(th_Wrist_Tmp), sin(th_Wrist_Tmp)];
            l_N_Wrist_Bar_Tmp = [x_Cross, y_Cross] ./ cross_Vec;
            
            % To check if calc is correct, but not installed
            %{
            if abs(x_Cross * cos(th_Wrist_Tmp) - y_Cross * sin(th_Wrist_Tmp)) < 1e-1
                l_N_Wrist_Bar_Tmp = [x_Cross, y_Cross] ./ cross_Vec;
            else
                error('l_N_Wrist_Bar is not calculated correctly')
            end
            %}
            
            value(1) = l_N_Wrist_Bar_Tmp(1);
            direction(1) = 0;
            if l_Wrist_Bar_Tmp(1) > 0 && l_Wrist_Bar_Tmp(1) < app.l_Body
                isterminal(1) = 1;
            else
                isterminal(1) = 0;
            end
            
            y0 = y_Wrist_Tmp;
            y1 = y0 + app.l_Body * sin(th_Wrist_Tmp + 1/2 * pi);
            y2 = y1 + app.l_Leg * sin(th_Wrist_Tmp + 1/2 * pi + th_Hip_Tmp);
            
            value(2) = (app.m_Body * (y0 + y1)/2 + app.m_Leg * (y1 + y2)/2)/(app.m_Body + app.m_Leg) - (-app.l_Body - app.l_Leg);
            direction(2) = 0;
            isterminal(2) = 1;
        end
        
        function run_Ode_Catch(app)
            t = [0, app.time_step];
            
            q0 = [app.th_Wrist, app.dth_Wrist, app.th_Hip, app.dth_Hip, app.l_Wrist_Bar, app.dl_Wrist_Bar]';
            ode_Event = @(t,q) event_Catch(app, q);
            ode_Options = odeset('Events', ode_Event);
            [~, q, ~, ~, ie] = ode45(@(t,q) ddt_Catch(app, q), t, q0, ode_Options);
            
            app.th_Wrist = q(end,1);
            app.dth_Wrist = q(end,2);
            app.th_Hip = q(end,3);
            app.dth_Hip = q(end,4);
            app.l_Wrist_Bar = q(end,5);
            app.dl_Wrist_Bar = q(end,6);
            
            app.x_Wrist = -app.l_Wrist_Bar * cos(app.th_Wrist + 1/2 * pi);
            app.dx_Wrist = app.l_Wrist_Bar * app.dth_Wrist * sin(app.th_Wrist + 1/2 * pi);
            app.y_Wrist = -app.l_Wrist_Bar * sin(app.th_Wrist + 1/2 * pi);
            app.dy_Wrist = -app.l_Wrist_Bar * app.dth_Wrist * cos(app.th_Wrist + 1/2 * pi);
            
            if ~isempty(ie)
                if ie(end) == 1
                    app.StatusLabel.Text = 'OnBar';
                    
                    [dth_Wrist_After,dth_Hip_After] = ...
                        find_Status_After_Slides_Collision(app.dl_Wrist_Bar,app.dth_Hip,app.dth_Wrist,...
                        app.l_Body,app.l_Leg,app.l_Wrist_Bar,app.m_Body,app.m_Leg,app.th_Hip);
                    
                    app.dth_Wrist = dth_Wrist_After;
                    app.dth_Hip = dth_Hip_After;
                elseif ie(end) == 2
                    app.StatusLabel.Text = 'failed';
                    
                    app.PlayingButton.Value = false;
                end
            end
        end
        
        function [value,isterminal,direction] = event_Catch(app, q)
            l_Wrist_Bar_Tmp = q(5);
            %             dl_Wrist_Bar_Tmp = q(6);
            
            value(1) = l_Wrist_Bar_Tmp;
            isterminal(1) = 1;
            direction(1) = 0;
            
            value(2) = l_Wrist_Bar_Tmp - app.l_Body;
            isterminal(2) = 1;
            direction(2) = 0;
        end
        
        function dotq = ddt_Onbar(app, q)
            th_Wrist_Tmp = q(1);
            dth_Wrist_Tmp = q(2);
            th_Hip_Tmp = q(3);
            dth_Hip_Tmp = q(4);
            
            tau_Hip_Tmp = app.tau_Hip;
            
            if rad2deg(th_Hip_Tmp) < app.theta_PE_1_Ext
                if dth_Hip_Tmp < 0
                    tau_Hip_Tmp = tau_Hip_Tmp - app.c * dth_Hip_Tmp;
                end
            elseif rad2deg(th_Hip_Tmp) > app.theta_PE_1_Flex
                if dth_Hip_Tmp > 0
                    tau_Hip_Tmp = tau_Hip_Tmp - app.c * dth_Hip_Tmp;
                end
            end
            
            [ddth_Wrist,ddth_Hip] = find_dd_Onbar(dth_Hip_Tmp,dth_Wrist_Tmp,app.g,app.l_Body,app.l_Leg,app.l_Wrist_Bar,app.m_Body,app.m_Leg,tau_Hip_Tmp,th_Hip_Tmp,th_Wrist_Tmp);
            
            dotq = [dth_Wrist_Tmp, ddth_Wrist, dth_Hip_Tmp, ddth_Hip]';
        end
        
        function dotq = ddt_Inair(app, q)
            th_Wrist_Tmp = q(1);
            dth_Wrist_Tmp = q(2);
            th_Hip_Tmp = q(3);
            dth_Hip_Tmp = q(4);
            
            dx_Wrist_Tmp = q(6);
            dy_Wrist_Tmp = q(8);
            
            tau_Hip_Tmp = app.tau_Hip;
            
            if rad2deg(th_Hip_Tmp) < app.theta_PE_1_Ext
                if dth_Hip_Tmp < 0
                    tau_Hip_Tmp = tau_Hip_Tmp - app.c * dth_Hip_Tmp;
                end
            elseif rad2deg(th_Hip_Tmp) > app.theta_PE_1_Flex
                if dth_Hip_Tmp > 0
                    tau_Hip_Tmp = tau_Hip_Tmp - app.c * dth_Hip_Tmp;
                end
            end
            
            [ddth_Wrist,ddth_Hip,ddx,ddy] = find_dd_Inair(dth_Hip_Tmp,dth_Wrist_Tmp,app.g,app.l_Body,app.l_Leg,app.m_Body,app.m_Leg,tau_Hip_Tmp,th_Hip_Tmp,th_Wrist_Tmp);
            
            dotq = [dth_Wrist_Tmp, ddth_Wrist, dth_Hip_Tmp, ddth_Hip, dx_Wrist_Tmp, ddx, dy_Wrist_Tmp, ddy]';
        end
        
        function dotq = ddt_Catch(app, q)
            
            th_Wrist_Tmp = q(1);
            dth_Wrist_Tmp = q(2);
            th_Hip_Tmp = q(3);
            dth_Hip_Tmp = q(4);
            
            l_Wrist_Bar_Tmp = q(5);
            dl_Wrist_Bar_Tmp = q(6);
            
            tau_Hip_Tmp = app.tau_Hip;
            
            if rad2deg(th_Hip_Tmp) < app.theta_PE_1_Ext
                if dth_Hip_Tmp < 0
                    tau_Hip_Tmp = tau_Hip_Tmp - app.c * dth_Hip_Tmp;
                end
            elseif rad2deg(th_Hip_Tmp) > app.theta_PE_1_Flex
                if dth_Hip_Tmp > 0
                    tau_Hip_Tmp = tau_Hip_Tmp - app.c * dth_Hip_Tmp;
                end
            end
            
            [ddth_Wrist,ddth_Hip,ddl_Wrist_Bar,f_Wrist_Bar] ...
                = find_dd_Catch(dl_Wrist_Bar_Tmp,dth_Hip_Tmp,dth_Wrist_Tmp,app.g,app.l_Body,app.l_Leg,...
                l_Wrist_Bar_Tmp,app.m_Body,app.m_Leg,app.myu,tau_Hip_Tmp,th_Hip_Tmp,th_Wrist_Tmp);
            
            if sign(f_Wrist_Bar) ~= sign(dl_Wrist_Bar_Tmp)
                app.myu = -app.myu;
                [ddth_Wrist,ddth_Hip,ddl_Wrist_Bar] ...
                    = find_dd_Catch(dl_Wrist_Bar_Tmp,dth_Hip_Tmp,dth_Wrist_Tmp,app.g,app.l_Body,app.l_Leg,...
                    l_Wrist_Bar_Tmp,app.m_Body,app.m_Leg,app.myu,tau_Hip_Tmp,th_Hip_Tmp,th_Wrist_Tmp);
            end
            
            dotq = [dth_Wrist_Tmp, ddth_Wrist, dth_Hip_Tmp, ddth_Hip, dl_Wrist_Bar_Tmp, ddl_Wrist_Bar]';
        end
        
        function [x_Cross, y_Cross]= calc_Cross(~, th_Wrist_Tmp, x_Wrist_Tmp, y_Wrist_Tmp)
            theta_Tmp = th_Wrist_Tmp + 1/2 * pi;
            x_Cross = sin(theta_Tmp)^2 * x_Wrist_Tmp - sin(theta_Tmp) * cos(theta_Tmp) * y_Wrist_Tmp;
            if nargout > 1
                y_Cross = -sin(theta_Tmp) * cos(theta_Tmp) * x_Wrist_Tmp + cos(theta_Tmp)^2 * y_Wrist_Tmp;
            end
        end
        
        function [ddth_Wrist,ddth_Hip] = find_dd_Onbar(dth_Hip,dth_Wrist,g,l_Body,l_Leg,l_Wrist_Bar,m_Body,m_Leg,tau_Hip,th_Hip,th_Wrist)
            t2 = cos(th_Hip);
            t3 = sin(th_Hip);
            t4 = sin(th_Wrist);
            t5 = th_Hip+th_Wrist;
            t6 = dth_Hip.^2;
            t7 = dth_Wrist.^2;
            t8 = l_Body.^2;
            t9 = l_Leg.^2;
            t10 = l_Wrist_Bar.^2;
            t11 = th_Hip.*2.0;
            t16 = 1.0./l_Leg;
            t20 = l_Body.*l_Wrist_Bar.*m_Body.*1.2e+1;
            t21 = l_Body.*l_Wrist_Bar.*m_Leg.*2.4e+1;
            t12 = cos(t11);
            t13 = t2.^2;
            t14 = sin(t11);
            t15 = sin(t5);
            t17 = t5+th_Hip;
            t19 = m_Body.*t8.*4.0;
            t22 = m_Leg.*t8.*1.2e+1;
            t23 = m_Body.*t10.*1.2e+1;
            t24 = m_Leg.*t10.*1.2e+1;
            t25 = -t20;
            t26 = -t21;
            t18 = sin(t17);
            ddth_Wrist = -(t16.*(l_Leg.*tau_Hip.*2.4e+1+l_Body.*t2.*tau_Hip.*3.6e+1-l_Wrist_Bar.*t2.*tau_Hip.*3.6e+1-g.*l_Body.*l_Leg.*m_Body.*t4.*1.2e+1-g.*l_Body.*l_Leg.*m_Leg.*t4.*1.5e+1+g.*l_Body.*l_Leg.*m_Leg.*t18.*9.0+g.*l_Leg.*l_Wrist_Bar.*m_Body.*t4.*2.4e+1+g.*l_Leg.*l_Wrist_Bar.*m_Leg.*t4.*1.5e+1-g.*l_Leg.*l_Wrist_Bar.*m_Leg.*t18.*9.0-l_Body.*m_Leg.*t3.*t6.*t9.*1.2e+1-l_Body.*m_Leg.*t3.*t7.*t9.*1.2e+1-l_Leg.*m_Leg.*t7.*t8.*t14.*9.0-l_Leg.*m_Leg.*t7.*t10.*t14.*9.0+l_Wrist_Bar.*m_Leg.*t3.*t6.*t9.*1.2e+1+l_Wrist_Bar.*m_Leg.*t3.*t7.*t9.*1.2e+1-dth_Hip.*dth_Wrist.*l_Body.*m_Leg.*t3.*t9.*2.4e+1+dth_Hip.*dth_Wrist.*l_Wrist_Bar.*m_Leg.*t3.*t9.*2.4e+1+l_Body.*l_Leg.*l_Wrist_Bar.*m_Leg.*t7.*t14.*1.8e+1))./(m_Body.*t8.*8.0+m_Body.*t10.*2.4e+1+m_Leg.*t8.*1.5e+1+m_Leg.*t10.*1.5e+1-l_Body.*l_Wrist_Bar.*m_Body.*2.4e+1-l_Body.*l_Wrist_Bar.*m_Leg.*3.0e+1-m_Leg.*t8.*t12.*9.0-m_Leg.*t10.*t12.*9.0+l_Body.*l_Wrist_Bar.*m_Leg.*t12.*1.8e+1);
            if nargout > 1
                t27 = l_Body.*l_Wrist_Bar.*m_Leg.*t13.*1.8e+1;
                t28 = m_Leg.*t8.*t13.*9.0;
                t29 = m_Leg.*t10.*t13.*9.0;
                t30 = -t28;
                t31 = -t29;
                t32 = t19+t22+t23+t24+t25+t26+t27+t30+t31;
                t33 = 1.0./t32;
                ddth_Hip = -t16.*t33.*(l_Leg.*2.0+l_Body.*t2.*3.0-l_Wrist_Bar.*t2.*3.0).*(g.*l_Body.*m_Body.*t4.*3.0+g.*l_Body.*m_Leg.*t4.*6.0+g.*l_Leg.*m_Leg.*t15.*3.0-g.*l_Wrist_Bar.*m_Body.*t4.*6.0-g.*l_Wrist_Bar.*m_Leg.*t4.*6.0+l_Body.*l_Leg.*m_Leg.*t3.*t6.*3.0-l_Leg.*l_Wrist_Bar.*m_Leg.*t3.*t6.*3.0+dth_Hip.*dth_Wrist.*l_Body.*l_Leg.*m_Leg.*t3.*6.0-dth_Hip.*dth_Wrist.*l_Leg.*l_Wrist_Bar.*m_Leg.*t3.*6.0)+(t33.*(tau_Hip.*1.2e+1+g.*l_Leg.*m_Leg.*t15.*6.0-l_Body.*l_Leg.*m_Leg.*t3.*t7.*6.0+l_Leg.*l_Wrist_Bar.*m_Leg.*t3.*t7.*6.0).*(m_Body.*t8+m_Body.*t10.*3.0+m_Leg.*t8.*3.0+m_Leg.*t9+m_Leg.*t10.*3.0-l_Body.*l_Wrist_Bar.*m_Body.*3.0-l_Body.*l_Wrist_Bar.*m_Leg.*6.0+l_Body.*l_Leg.*m_Leg.*t2.*3.0-l_Leg.*l_Wrist_Bar.*m_Leg.*t2.*3.0))./(m_Leg.*t9);
            end
        end
        
        function [ddth_Wrist,ddth_Hip,ddx,ddy] = find_dd_Inair(dth_Hip,dth_Wrist,g,l_Body,l_Leg,m_Body,m_Leg,tau_Hip,th_Hip,th_Wrist)
            t2 = cos(th_Hip);
            t3 = cos(th_Wrist);
            t4 = sin(th_Hip);
            t5 = sin(th_Wrist);
            t6 = th_Hip+th_Wrist;
            t7 = dth_Hip.^2;
            t8 = dth_Wrist.^2;
            t9 = l_Body.^2;
            t10 = l_Body.^3;
            t11 = l_Leg.^2;
            t12 = l_Leg.^3;
            t13 = m_Body.^2;
            t14 = m_Body.^3;
            t15 = m_Leg.^2;
            t16 = m_Leg.^3;
            t17 = th_Hip.*2.0;
            t22 = 1.0./l_Body;
            t24 = 1.0./l_Leg;
            t25 = 1.0./m_Body;
            t26 = -th_Wrist;
            t27 = m_Body.*m_Leg.*2.5e+1;
            t18 = cos(t17);
            t19 = sin(t17);
            t20 = cos(t6);
            t21 = sin(t6);
            t23 = 1.0./t9;
            ddth_Wrist = (t23.*t24.*t25.*(l_Leg.*t13.*tau_Hip.*-4.8e+1-l_Leg.*t15.*tau_Hip.*1.2e+1-l_Leg.*m_Body.*m_Leg.*tau_Hip.*6.0e+1-l_Body.*t2.*t13.*tau_Hip.*3.6e+1-l_Body.*m_Body.*m_Leg.*t2.*tau_Hip.*3.6e+1+l_Body.*m_Body.*t4.*t7.*t11.*t15.*3.0+l_Body.*m_Body.*t4.*t8.*t11.*t15.*3.0+l_Body.*m_Leg.*t4.*t7.*t11.*t13.*1.2e+1+l_Body.*m_Leg.*t4.*t8.*t11.*t13.*1.2e+1+l_Leg.*m_Leg.*t8.*t9.*t13.*t19.*(9.0./2.0)+dth_Hip.*dth_Wrist.*l_Body.*m_Body.*t4.*t11.*t15.*6.0+dth_Hip.*dth_Wrist.*l_Body.*m_Leg.*t4.*t11.*t13.*2.4e+1))./(t13.*4.0+t15.*4.0+m_Body.*m_Leg.*1.7e+1-m_Body.*m_Leg.*t2.^2.*9.0);
            if nargout > 1
                t28 = t6+th_Hip;
                t31 = t13.*8.0;
                t32 = t15.*8.0;
                t33 = t26+th_Hip;
                t29 = cos(t28);
                t30 = sin(t28);
                t34 = cos(t33);
                t35 = sin(t33);
                t36 = m_Body.*m_Leg.*t18.*9.0;
                t37 = -t36;
                t38 = t27+t31+t32+t37;
                t39 = 1.0./t38;
                ddth_Hip = (t23.*t25.*t39.*(t9.*t14.*tau_Hip.*-1.2e+1-t11.*t16.*tau_Hip.*1.2e+1-m_Body.*t9.*t15.*tau_Hip.*4.8e+1-m_Body.*t11.*t15.*tau_Hip.*6.0e+1-m_Leg.*t9.*t13.*tau_Hip.*6.0e+1-m_Leg.*t11.*t13.*tau_Hip.*4.8e+1-l_Body.*l_Leg.*m_Body.*t2.*t15.*tau_Hip.*7.2e+1-l_Body.*l_Leg.*m_Leg.*t2.*t13.*tau_Hip.*7.2e+1+l_Body.*m_Body.*t4.*t7.*t12.*t16.*3.0+l_Body.*m_Body.*t4.*t8.*t12.*t16.*3.0+l_Leg.*m_Leg.*t4.*t8.*t10.*t14.*3.0+l_Body.*t4.*t7.*t12.*t13.*t15.*1.2e+1+l_Body.*t4.*t8.*t12.*t13.*t15.*1.2e+1+l_Leg.*t4.*t8.*t10.*t13.*t15.*1.2e+1+t7.*t9.*t11.*t13.*t15.*t19.*(9.0./2.0)+t8.*t9.*t11.*t13.*t15.*t19.*9.0+dth_Hip.*dth_Wrist.*l_Body.*m_Body.*t4.*t12.*t16.*6.0+dth_Hip.*dth_Wrist.*l_Body.*t4.*t12.*t13.*t15.*2.4e+1+dth_Hip.*dth_Wrist.*t9.*t11.*t13.*t15.*t19.*9.0).*-2.0)./(m_Leg.*t11);
            end
            if nargout > 2
                ddx = -t22.*t24.*t25.*t39.*(l_Body.*t13.*t20.*tau_Hip.*6.0+l_Body.*t13.*t34.*tau_Hip.*1.8e+1+l_Leg.*t3.*t13.*tau_Hip.*4.8e+1+l_Leg.*t3.*t15.*tau_Hip.*2.4e+1-l_Body.*m_Body.*m_Leg.*t20.*tau_Hip.*1.2e+1+l_Body.*m_Body.*m_Leg.*t34.*tau_Hip.*3.6e+1+l_Leg.*m_Body.*m_Leg.*t3.*tau_Hip.*9.0e+1-l_Leg.*m_Body.*m_Leg.*t29.*tau_Hip.*1.8e+1+l_Leg.*t5.*t8.*t9.*t14.*4.0+l_Body.*m_Body.*t7.*t11.*t15.*t21+l_Body.*m_Body.*t8.*t11.*t15.*t21-l_Body.*m_Body.*t7.*t11.*t15.*t35.*3.0-l_Body.*m_Body.*t8.*t11.*t15.*t35.*3.0-l_Body.*m_Leg.*t7.*t11.*t13.*t21.*2.0-l_Body.*m_Leg.*t8.*t11.*t13.*t21.*2.0+l_Leg.*m_Body.*t5.*t8.*t9.*t32-l_Body.*m_Leg.*t7.*t11.*t13.*t35.*6.0-l_Body.*m_Leg.*t8.*t11.*t13.*t35.*6.0+l_Leg.*m_Leg.*t5.*t8.*t9.*t13.*1.5e+1-l_Leg.*m_Leg.*t8.*t9.*t13.*t30.*3.0+dth_Hip.*dth_Wrist.*l_Body.*m_Body.*t11.*t15.*t21.*2.0-dth_Hip.*dth_Wrist.*l_Body.*m_Body.*t11.*t15.*t35.*6.0-dth_Hip.*dth_Wrist.*l_Body.*m_Leg.*t11.*t13.*t21.*4.0-dth_Hip.*dth_Wrist.*l_Body.*m_Leg.*t11.*t13.*t35.*1.2e+1);
            end
            if nargout > 3
                ddy = t22.*t24.*t25.*t39.*(g.*l_Body.*l_Leg.*t14.*-8.0-l_Body.*t13.*t21.*tau_Hip.*6.0+l_Body.*t13.*t35.*tau_Hip.*1.8e+1-l_Leg.*t5.*t13.*tau_Hip.*4.8e+1-l_Leg.*t5.*t15.*tau_Hip.*2.4e+1-g.*l_Body.*l_Leg.*m_Body.*t15.*8.0-g.*l_Body.*l_Leg.*m_Leg.*t13.*2.5e+1+l_Body.*m_Body.*m_Leg.*t21.*tau_Hip.*1.2e+1+l_Body.*m_Body.*m_Leg.*t35.*tau_Hip.*3.6e+1-l_Leg.*m_Body.*m_Leg.*t5.*tau_Hip.*9.0e+1+l_Leg.*m_Body.*m_Leg.*t30.*tau_Hip.*1.8e+1+l_Leg.*t3.*t8.*t9.*t14.*4.0+l_Body.*m_Body.*t7.*t11.*t15.*t20+l_Body.*m_Body.*t8.*t11.*t15.*t20+l_Body.*m_Body.*t7.*t11.*t15.*t34.*3.0+l_Body.*m_Body.*t8.*t11.*t15.*t34.*3.0-l_Body.*m_Leg.*t7.*t11.*t13.*t20.*2.0-l_Body.*m_Leg.*t8.*t11.*t13.*t20.*2.0+l_Leg.*m_Body.*t3.*t8.*t9.*t32+l_Body.*m_Leg.*t7.*t11.*t13.*t34.*6.0+l_Body.*m_Leg.*t8.*t11.*t13.*t34.*6.0+l_Leg.*m_Leg.*t3.*t8.*t9.*t13.*1.5e+1-l_Leg.*m_Leg.*t8.*t9.*t13.*t29.*3.0+g.*l_Body.*l_Leg.*m_Leg.*t13.*t18.*9.0+dth_Hip.*dth_Wrist.*l_Body.*m_Body.*t11.*t15.*t20.*2.0+dth_Hip.*dth_Wrist.*l_Body.*m_Body.*t11.*t15.*t34.*6.0-dth_Hip.*dth_Wrist.*l_Body.*m_Leg.*t11.*t13.*t20.*4.0+dth_Hip.*dth_Wrist.*l_Body.*m_Leg.*t11.*t13.*t34.*1.2e+1);
            end
        end
        
        function [ddth_Wrist,ddth_Hip,ddl_Wrist_Bar,f_Wrist_Bar] = find_dd_Catch(dl_Wrist_Bar,dth_Hip,dth_Wrist,g,l_Body,l_Leg,l_Wrist_Bar,m_Body,m_Leg,myu,tau_Hip,th_Hip,th_Wrist)
            t2 = cos(th_Hip);
            t3 = cos(th_Wrist);
            t4 = sin(th_Hip);
            t5 = sin(th_Wrist);
            t6 = dth_Hip.^2;
            t7 = dth_Wrist.^2;
            t8 = l_Body.^2;
            t9 = l_Body.^3;
            t10 = l_Leg.^2;
            t11 = l_Leg.^3;
            t12 = l_Wrist_Bar.^2;
            t13 = l_Wrist_Bar.^3;
            t14 = m_Body.^2;
            t15 = m_Leg.^2;
            t16 = m_Leg.^3;
            t17 = th_Hip.*2.0;
            t21 = 1.0./l_Leg;
            t22 = l_Leg.*m_Leg.*tau_Hip.*6.0;
            t23 = pi./2.0;
            t24 = l_Leg.*m_Body.*tau_Hip.*2.4e+1;
            t30 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*9.0e+1;
            t36 = l_Body.*l_Leg.*m_Leg.*myu.*tau_Hip.*3.0e+1;
            t37 = l_Leg.*l_Wrist_Bar.*m_Body.*myu.*tau_Hip.*4.8e+1;
            t38 = l_Leg.*l_Wrist_Bar.*m_Leg.*myu.*tau_Hip.*3.0e+1;
            t51 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*tau_Hip.*2.16e+2;
            t52 = l_Body.*l_Leg.*m_Body.*myu.*tau_Hip.*-2.4e+1;
            t54 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*l_Leg.*m_Body.*m_Leg.*5.4e+1;
            t55 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*6.0e+1;
            t18 = cos(t17);
            t19 = t2.^2;
            t20 = sin(t17);
            t25 = t23+th_Wrist;
            t26 = l_Body.*l_Wrist_Bar.*t14.*4.8e+1;
            t27 = l_Body.*l_Wrist_Bar.*t15.*2.4e+1;
            t28 = m_Body.*m_Leg.*t8.*4.0e+1;
            t29 = m_Body.*m_Leg.*t12.*6.0e+1;
            t31 = l_Body.*m_Body.*t2.*tau_Hip.*3.6e+1;
            t32 = l_Body.*m_Leg.*t2.*tau_Hip.*3.6e+1;
            t33 = l_Wrist_Bar.*m_Body.*t2.*tau_Hip.*3.6e+1;
            t34 = l_Wrist_Bar.*m_Leg.*t2.*tau_Hip.*3.6e+1;
            t35 = l_Body.*myu.*t24;
            t44 = -t30;
            t47 = t10.*t15.*tau_Hip.*6.0;
            t48 = m_Body.*m_Leg.*t10.*tau_Hip.*2.4e+1;
            t53 = -t36;
            t56 = t8.*t14.*1.6e+1;
            t57 = t8.*t15.*1.2e+1;
            t58 = t12.*t14.*4.8e+1;
            t59 = t12.*t15.*1.2e+1;
            t60 = l_Body.*l_Wrist_Bar.*m_Body.*t4.*tau_Hip.*7.2e+1;
            t61 = l_Body.*l_Wrist_Bar.*m_Leg.*t4.*tau_Hip.*1.44e+2;
            t62 = g.*l_Body.*l_Leg.*t5.*t15.*6.0;
            t63 = g.*l_Leg.*l_Wrist_Bar.*t5.*t15.*6.0;
            t65 = g.*l_Body.*l_Leg.*m_Body.*m_Leg.*t5.*2.7e+1;
            t66 = g.*l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*t5.*3.0e+1;
            t67 = t8.*t14.*tau_Hip.*2.4e+1;
            t69 = l_Body.*l_Wrist_Bar.*t14.*tau_Hip.*7.2e+1;
            t70 = l_Body.*l_Wrist_Bar.*t15.*tau_Hip.*1.44e+2;
            t71 = m_Body.*m_Leg.*t8.*tau_Hip.*9.6e+1;
            t73 = m_Body.*m_Leg.*t12.*tau_Hip.*1.44e+2;
            t74 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*l_Leg.*t14.*2.4e+1;
            t75 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*l_Leg.*t15.*1.2e+1;
            t76 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*l_Wrist_Bar.*t14.*4.8e+1;
            t77 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*l_Wrist_Bar.*t15.*1.2e+1;
            t78 = -t54;
            t80 = m_Body.*t4.*t8.*tau_Hip.*2.4e+1;
            t84 = m_Body.*myu.*t2.*t8.*tau_Hip.*1.2e+1;
            t85 = g.*l_Body.*l_Leg.*t5.*t14.*1.2e+1;
            t87 = g.*l_Leg.*l_Wrist_Bar.*t5.*t14.*2.4e+1;
            t88 = l_Body.*l_Leg.*m_Body.*m_Leg.*t2.*tau_Hip.*7.2e+1;
            t89 = l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*t2.*tau_Hip.*7.2e+1;
            t95 = t8.*t15.*tau_Hip.*7.2e+1;
            t96 = t12.*t14.*tau_Hip.*7.2e+1;
            t97 = t12.*t15.*tau_Hip.*7.2e+1;
            t102 = l_Body.*l_Leg.*m_Body.*m_Leg.*myu.*t4.*tau_Hip.*1.8e+1;
            t103 = l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*myu.*t4.*tau_Hip.*3.6e+1;
            t104 = m_Leg.*t4.*t8.*tau_Hip.*7.2e+1;
            t105 = m_Body.*t4.*t12.*tau_Hip.*7.2e+1;
            t106 = m_Leg.*t4.*t12.*tau_Hip.*7.2e+1;
            t107 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*t10.*t16.*1.2e+1;
            t108 = dl_Wrist_Bar.*dth_Wrist.*l_Wrist_Bar.*t10.*t16.*1.2e+1;
            t109 = l_Leg.*m_Body.*m_Leg.*t7.*t9.*2.5e+1;
            t110 = l_Leg.*m_Body.*m_Leg.*t7.*t13.*6.0e+1;
            t112 = l_Body.*l_Leg.*t2.*t15.*tau_Hip.*7.2e+1;
            t113 = l_Leg.*l_Wrist_Bar.*t2.*t15.*tau_Hip.*7.2e+1;
            t122 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*m_Body.*m_Leg.*myu.*t8.*2.0e+1;
            t127 = g.*l_Body.*t5.*t10.*t16.*6.0;
            t128 = g.*l_Wrist_Bar.*t5.*t10.*t16.*6.0;
            t130 = g.*l_Leg.*t3.*t30;
            t134 = l_Body.*l_Leg.*myu.*t4.*t15.*tau_Hip.*3.6e+1;
            t135 = l_Leg.*l_Wrist_Bar.*myu.*t4.*t15.*tau_Hip.*3.6e+1;
            t138 = dth_Hip.*dth_Wrist.*l_Body.*t4.*t10.*t15.*6.0;
            t139 = dth_Hip.*dth_Wrist.*l_Body.*t4.*t11.*t16.*6.0;
            t140 = dth_Hip.*dth_Wrist.*l_Wrist_Bar.*t4.*t10.*t15.*6.0;
            t141 = dth_Hip.*dth_Wrist.*l_Wrist_Bar.*t4.*t11.*t16.*6.0;
            t142 = g.*l_Leg.*myu.*t5.*t8.*t14.*4.0;
            t143 = dth_Hip.*dth_Wrist.*l_Body.*m_Body.*m_Leg.*t4.*t10.*2.4e+1;
            t144 = dth_Hip.*dth_Wrist.*l_Wrist_Bar.*m_Body.*m_Leg.*t4.*t10.*2.4e+1;
            t145 = l_Leg.*t7.*t9.*t14.*8.0;
            t146 = l_Leg.*t7.*t9.*t15.*1.2e+1;
            t147 = l_Leg.*t7.*t13.*t14.*4.8e+1;
            t148 = l_Leg.*t7.*t13.*t15.*1.2e+1;
            t150 = g.*l_Leg.*m_Body.*m_Leg.*myu.*t5.*t8.*1.0e+1;
            t154 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*m_Leg.*t10.*t14.*2.4e+1;
            t155 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*m_Body.*t10.*t15.*5.4e+1;
            t156 = dl_Wrist_Bar.*dth_Wrist.*l_Wrist_Bar.*m_Leg.*t10.*t14.*4.8e+1;
            t157 = dl_Wrist_Bar.*dth_Wrist.*l_Wrist_Bar.*m_Body.*t10.*t15.*6.0e+1;
            t158 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*myu.*t8.*t14.*8.0;
            t164 = g.*l_Leg.*m_Body.*m_Leg.*t3.*t8.*-4.0e+1;
            t165 = g.*l_Leg.*m_Body.*m_Leg.*t3.*t12.*-6.0e+1;
            t170 = l_Body.*t4.*t6.*t10.*t15.*3.0;
            t171 = l_Body.*t4.*t6.*t11.*t16.*3.0;
            t172 = l_Body.*t4.*t7.*t10.*t15.*3.0;
            t173 = l_Body.*t4.*t7.*t11.*t16.*3.0;
            t174 = l_Wrist_Bar.*t4.*t6.*t10.*t15.*3.0;
            t175 = l_Wrist_Bar.*t4.*t6.*t11.*t16.*3.0;
            t176 = l_Wrist_Bar.*t4.*t7.*t10.*t15.*3.0;
            t177 = l_Wrist_Bar.*t4.*t7.*t11.*t16.*3.0;
            t180 = l_Body.*m_Body.*m_Leg.*t4.*t6.*t10.*1.2e+1;
            t181 = l_Body.*m_Body.*m_Leg.*t4.*t7.*t10.*1.2e+1;
            t182 = l_Wrist_Bar.*m_Body.*m_Leg.*t4.*t6.*t10.*1.2e+1;
            t183 = l_Wrist_Bar.*m_Body.*m_Leg.*t4.*t7.*t10.*1.2e+1;
            t184 = g.*l_Body.*m_Leg.*t5.*t10.*t14.*1.2e+1;
            t185 = g.*l_Body.*m_Body.*t5.*t10.*t15.*2.7e+1;
            t186 = g.*l_Wrist_Bar.*m_Leg.*t5.*t10.*t14.*2.4e+1;
            t187 = g.*l_Wrist_Bar.*m_Body.*t5.*t10.*t15.*3.0e+1;
            t193 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*l_Leg.*l_Wrist_Bar.*m_Body.*t2.*t15.*3.6e+1;
            t194 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*l_Leg.*l_Wrist_Bar.*m_Leg.*t2.*t14.*3.6e+1;
            t195 = dth_Hip.*dth_Wrist.*l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*t2.*t10.*4.8e+1;
            t199 = l_Leg.*l_Wrist_Bar.*t7.*t8.*t14.*4.0e+1;
            t200 = l_Body.*l_Leg.*t7.*t12.*t15.*3.6e+1;
            t201 = l_Leg.*l_Wrist_Bar.*t7.*t8.*t15.*3.6e+1;
            t206 = l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*t7.*t8.*1.0e+2;
            t207 = l_Body.*l_Leg.*m_Body.*m_Leg.*t7.*t12.*1.35e+2;
            t208 = g.*l_Leg.*t3.*t8.*t14.*-1.6e+1;
            t209 = g.*l_Leg.*t3.*t8.*t15.*-1.2e+1;
            t210 = g.*l_Leg.*t3.*t12.*t14.*-4.8e+1;
            t211 = g.*l_Leg.*t3.*t12.*t15.*-1.2e+1;
            t213 = g.*l_Body.*l_Leg.*l_Wrist_Bar.*m_Body.*t2.*t5.*t15.*1.8e+1;
            t214 = g.*l_Body.*l_Leg.*l_Wrist_Bar.*m_Leg.*t2.*t5.*t14.*1.8e+1;
            t226 = dth_Hip.*dth_Wrist.*m_Body.*myu.*t8.*t10.*t15.*6.0;
            t230 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*m_Leg.*t2.*t8.*t14.*1.2e+1;
            t231 = dth_Hip.*dth_Wrist.*m_Body.*m_Leg.*t2.*t8.*t10.*1.6e+1;
            t232 = dth_Hip.*dth_Wrist.*m_Body.*m_Leg.*t2.*t10.*t12.*4.8e+1;
            t233 = dth_Hip.*dth_Wrist.*l_Body.*l_Wrist_Bar.*m_Body.*myu.*t10.*t15.*1.8e+1;
            t234 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*t2.*t6.*t10.*2.4e+1;
            t235 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*t2.*t7.*t10.*2.4e+1;
            t236 = l_Leg.*m_Leg.*t4.*t7.*t9.*t14.*6.0;
            t241 = dth_Hip.*dth_Wrist.*l_Body.*m_Body.*t4.*t11.*t15.*2.4e+1;
            t242 = dth_Hip.*dth_Wrist.*l_Wrist_Bar.*m_Body.*t4.*t11.*t15.*2.4e+1;
            t243 = g.*l_Body.*l_Leg.*m_Body.*m_Leg.*myu.*t2.*t4.*t5.*9.0;
            t246 = l_Body.*l_Leg.*t7.*t12.*t14.*7.2e+1;
            t250 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*m_Leg.*myu.*t4.*t8.*t14.*6.0;
            t252 = g.*l_Leg.*m_Body.*t2.*t5.*t8.*t15.*6.0;
            t253 = g.*l_Leg.*m_Leg.*t2.*t5.*t8.*t14.*6.0;
            t258 = t2.*t6.*t8.*t10.*t15.*6.0;
            t259 = t2.*t7.*t8.*t10.*t15.*6.0;
            t260 = t2.*t6.*t10.*t12.*t15.*6.0;
            t261 = t2.*t7.*t10.*t12.*t15.*6.0;
            t262 = m_Body.*myu.*t6.*t8.*t10.*t15.*3.0;
            t263 = m_Body.*myu.*t7.*t8.*t10.*t15.*3.0;
            t269 = l_Body.*l_Wrist_Bar.*t2.*t6.*t10.*t15.*1.2e+1;
            t270 = l_Body.*l_Wrist_Bar.*t2.*t7.*t10.*t15.*1.2e+1;
            t272 = m_Body.*m_Leg.*t2.*t6.*t8.*t10.*8.0;
            t273 = m_Body.*m_Leg.*t2.*t7.*t8.*t10.*8.0;
            t274 = m_Body.*m_Leg.*t2.*t6.*t10.*t12.*2.4e+1;
            t275 = m_Body.*m_Leg.*t2.*t7.*t10.*t12.*2.4e+1;
            t276 = l_Body.*l_Wrist_Bar.*m_Body.*myu.*t6.*t10.*t15.*9.0;
            t277 = l_Body.*l_Wrist_Bar.*m_Body.*myu.*t7.*t10.*t15.*9.0;
            t278 = dth_Hip.*dth_Wrist.*l_Body.*l_Wrist_Bar.*t2.*t10.*t15.*-2.4e+1;
            t280 = g.*l_Leg.*m_Leg.*myu.*t4.*t5.*t8.*t14.*3.0;
            t283 = l_Body.*m_Body.*t4.*t6.*t11.*t15.*1.2e+1;
            t284 = l_Body.*m_Body.*t4.*t7.*t11.*t15.*1.2e+1;
            t285 = l_Leg.*m_Body.*t4.*t7.*t9.*t15.*1.8e+1;
            t286 = l_Wrist_Bar.*m_Body.*t4.*t6.*t11.*t15.*1.2e+1;
            t287 = l_Wrist_Bar.*m_Body.*t4.*t7.*t11.*t15.*1.2e+1;
            t293 = m_Body.*m_Leg.*myu.*t4.*t6.*t8.*t10.*4.0;
            t294 = m_Body.*m_Leg.*myu.*t4.*t7.*t8.*t10.*4.0;
            t295 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*m_Body.*myu.*t4.*t8.*t15.*2.4e+1;
            t296 = dth_Hip.*dth_Wrist.*m_Body.*m_Leg.*myu.*t4.*t8.*t10.*8.0;
            t300 = dth_Hip.*dth_Wrist.*l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*myu.*t4.*t10.*-2.4e+1;
            t313 = l_Body.*l_Leg.*m_Body.*t4.*t7.*t12.*t15.*1.8e+1;
            t314 = l_Body.*l_Leg.*m_Leg.*t4.*t7.*t12.*t14.*1.8e+1;
            t315 = l_Leg.*l_Wrist_Bar.*m_Leg.*t4.*t7.*t8.*t14.*1.8e+1;
            t317 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*myu.*t4.*t6.*t10.*-1.2e+1;
            t318 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*myu.*t4.*t7.*t10.*-1.2e+1;
            t328 = l_Leg.*l_Wrist_Bar.*m_Body.*t4.*t7.*t8.*t15.*-3.6e+1;
            t332 = g.*l_Body.*m_Body.*myu.*t2.*t4.*t5.*t10.*t15.*9.0;
            t39 = cos(t25);
            t40 = sin(t25);
            t41 = t25+th_Hip;
            t42 = -t26;
            t43 = -t27;
            t49 = -t33;
            t50 = -t34;
            t64 = l_Wrist_Bar.*myu.*t31;
            t68 = -t47;
            t72 = -t48;
            t79 = l_Leg.*m_Leg.*t19.*tau_Hip.*1.8e+1;
            t81 = -t60;
            t82 = -t61;
            t83 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*t18.*1.8e+1;
            t86 = -t62;
            t90 = l_Body.*l_Leg.*m_Leg.*t20.*tau_Hip.*1.8e+1;
            t91 = l_Leg.*l_Wrist_Bar.*m_Leg.*t20.*tau_Hip.*1.8e+1;
            t92 = -t65;
            t93 = l_Leg.*m_Leg.*myu.*t20.*tau_Hip.*9.0;
            t94 = -t67;
            t98 = -t71;
            t99 = -t73;
            t100 = -t74;
            t101 = -t75;
            t111 = m_Body.*m_Leg.*t8.*t18.*1.2e+1;
            t114 = -t84;
            t115 = -t85;
            t116 = -t88;
            t119 = -t95;
            t120 = -t96;
            t121 = -t97;
            t123 = g.*l_Leg.*t3.*t26;
            t124 = g.*l_Leg.*t3.*t27;
            t125 = g.*l_Leg.*t3.*t28;
            t126 = g.*l_Leg.*t3.*t29;
            t129 = m_Body.*m_Leg.*myu.*t8.*t20.*3.0;
            t131 = l_Body.*l_Leg.*m_Leg.*myu.*t18.*tau_Hip.*1.8e+1;
            t132 = l_Leg.*l_Wrist_Bar.*m_Leg.*myu.*t18.*tau_Hip.*1.8e+1;
            t133 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*myu.*t20.*9.0;
            t136 = -t103;
            t137 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*l_Leg.*m_Body.*m_Leg.*t19.*1.8e+1;
            t149 = -t108;
            t151 = -t110;
            t153 = -t112;
            t159 = g.*l_Leg.*t3.*t56;
            t160 = g.*l_Leg.*t3.*t57;
            t161 = g.*l_Leg.*t3.*t58;
            t162 = g.*l_Leg.*t3.*t59;
            t163 = -t122;
            t166 = -t128;
            t169 = -t135;
            t178 = -t138;
            t179 = -t141;
            t188 = -t142;
            t189 = -t143;
            t190 = -t147;
            t191 = -t148;
            t192 = -t150;
            t196 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*m_Body.*m_Leg.*t8.*t20.*6.0;
            t197 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*t20.*1.8e+1;
            t198 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*l_Leg.*m_Body.*m_Leg.*myu.*t20.*9.0;
            t202 = g.*l_Body.*l_Leg.*m_Body.*m_Leg.*t5.*t19.*9.0;
            t203 = -t156;
            t204 = -t157;
            t205 = -t158;
            t212 = t10.*t15.*t19.*tau_Hip.*1.8e+1;
            t215 = g.*l_Leg.*m_Body.*m_Leg.*t5.*t8.*t20.*3.0;
            t216 = -t170;
            t217 = -t172;
            t218 = -t175;
            t219 = -t177;
            t220 = g.*l_Body.*l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*t5.*t20.*9.0;
            t221 = -t180;
            t222 = -t181;
            t223 = -t186;
            t224 = -t187;
            t225 = myu.*t10.*t15.*t20.*tau_Hip.*9.0;
            t227 = l_Leg.*m_Body.*m_Leg.*t7.*t9.*t18.*3.0;
            t228 = dth_Hip.*dth_Wrist.*t2.*t10.*t27;
            t229 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*m_Body.*t2.*t57;
            t237 = -t193;
            t238 = -t194;
            t239 = -t195;
            t245 = -t199;
            t247 = -t201;
            t248 = -t206;
            t254 = l_Wrist_Bar.*myu.*t143;
            t255 = -t213;
            t256 = -t214;
            t264 = dth_Hip.*dth_Wrist.*t2.*t10.*t57;
            t265 = dth_Hip.*dth_Wrist.*t2.*t10.*t59;
            t267 = g.*l_Leg.*m_Body.*m_Leg.*myu.*t5.*t8.*t18.*6.0;
            t268 = -t226;
            t281 = -t234;
            t282 = -t235;
            t288 = -t242;
            t289 = -t243;
            t292 = l_Leg.*m_Body.*m_Leg.*myu.*t7.*t9.*t20.*3.0;
            t297 = g.*l_Body.*l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*t3.*t18.*-1.8e+1;
            t298 = l_Wrist_Bar.*myu.*t180;
            t299 = l_Wrist_Bar.*myu.*t181;
            t301 = -t262;
            t302 = -t263;
            t303 = g.*l_Leg.*m_Body.*m_Leg.*myu.*t3.*t8.*t20.*-3.0;
            t304 = -t269;
            t305 = -t270;
            t306 = g.*l_Leg.*m_Body.*myu.*t4.*t5.*t57;
            t307 = -t286;
            t308 = -t287;
            t309 = l_Body.*l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*t7.*t20.*(9.0./2.0);
            t310 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*m_Body.*t10.*t15.*t19.*1.8e+1;
            t311 = l_Body.*l_Leg.*m_Body.*m_Leg.*t7.*t12.*t18.*9.0;
            t316 = m_Body.*t4.*t201;
            t319 = dth_Hip.*dth_Wrist.*l_Body.*l_Wrist_Bar.*m_Body.*t10.*t15.*t20.*1.8e+1;
            t320 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*m_Body.*myu.*t10.*t15.*t20.*9.0;
            t321 = l_Body.*l_Leg.*m_Body.*m_Leg.*myu.*t7.*t12.*t20.*9.0;
            t322 = l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*myu.*t7.*t8.*t20.*1.2e+1;
            t323 = g.*l_Body.*m_Body.*t5.*t10.*t15.*t19.*9.0;
            t324 = l_Leg.*m_Body.*m_Leg.*t7.*t8.*t20.*(9.0./2.0);
            t327 = -t315;
            t329 = m_Body.*t6.*t8.*t10.*t15.*t20.*6.0;
            t330 = dth_Hip.*dth_Wrist.*m_Body.*t10.*t20.*t57;
            t331 = l_Body.*l_Wrist_Bar.*m_Body.*t6.*t10.*t15.*t20.*9.0;
            t337 = t19.*t226;
            t338 = t19.*t233;
            t340 = t19.*t262;
            t341 = t19.*t263;
            t342 = t19.*t276;
            t343 = t19.*t277;
            t344 = dth_Hip.*dth_Wrist.*l_Body.*l_Wrist_Bar.*m_Body.*myu.*t10.*t15.*t19.*-1.8e+1;
            t345 = l_Body.*l_Wrist_Bar.*m_Body.*t7.*t10.*t15.*t20.*(2.7e+1./2.0);
            t346 = l_Body.*l_Wrist_Bar.*m_Body.*myu.*t6.*t10.*t15.*t19.*-9.0;
            t347 = l_Body.*l_Wrist_Bar.*m_Body.*myu.*t7.*t10.*t15.*t19.*-9.0;
            t348 = m_Body.*t7.*t8.*t10.*t15.*t20.*(2.1e+1./2.0);
            t45 = cos(t41);
            t46 = sin(t41);
            t117 = -t91;
            t118 = -t93;
            t152 = -t111;
            t167 = -t132;
            t168 = -t133;
            t240 = -t196;
            t244 = -t198;
            t249 = -t212;
            t251 = g.*l_Leg.*t3.*t83;
            t257 = -t215;
            t266 = g.*l_Leg.*t3.*t129;
            t271 = -t227;
            t279 = g.*l_Leg.*t3.*t133;
            t290 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*myu.*t111;
            t291 = g.*l_Leg.*t3.*t111;
            t312 = l_Leg.*l_Wrist_Bar.*t7.*t111;
            t325 = -t310;
            t326 = -t311;
            t333 = -t319;
            t334 = -t322;
            t335 = -t323;
            t336 = -t324;
            t339 = -t331;
            t349 = -t345;
            t350 = t28+t29+t42+t43+t44+t56+t57+t58+t59+t83+t129+t152+t168;
            t352 = t22+t24+t31+t32+t49+t50+t55+t63+t66+t76+t77+t78+t79+t86+t87+t92+t100+t101+t115+t118+t137+t140+t144+t174+t176+t178+t182+t183+t189+t202+t216+t217+t221+t222+t244+t289+t309+t336;
            t353 = t37+t38+t52+t53+t64+t80+t81+t82+t90+t104+t105+t106+t109+t114+t117+t123+t124+t130+t131+t145+t146+t151+t163+t164+t165+t167+t188+t190+t191+t192+t197+t200+t205+t207+t208+t209+t210+t211+t220+t231+t232+t239+t240+t245+t246+t247+t248+t257+t258+t259+t260+t261+t264+t265+t267+t271+t272+t273+t274+t275+t278+t279+t281+t282+t290+t291+t292+t293+t294+t296+t297+t300+t303+t304+t305+t312+t317+t318+t321+t326+t334;
            t354 = t51+t68+t69+t70+t72+t89+t94+t98+t99+t102+t107+t113+t116+t119+t120+t121+t127+t134+t136+t139+t149+t153+t154+t155+t166+t169+t171+t173+t179+t184+t185+t203+t204+t218+t219+t223+t224+t225+t229+t230+t233+t236+t237+t238+t241+t249+t250+t252+t253+t255+t256+t268+t276+t277+t280+t283+t284+t285+t288+t295+t301+t302+t306+t307+t308+t313+t314+t320+t325+t327+t328+t329+t330+t332+t333+t335+t337+t339+t340+t341+t344+t346+t347+t348+t349;
            t351 = 1.0./t350;
            ddth_Wrist = t21.*t351.*t352.*-2.0;
            if nargout > 1
                ddth_Hip = (t351.*t354.*-2.0)./(m_Leg.*t10);
            end
            if nargout > 2
                ddl_Wrist_Bar = -t21.*t351.*t353;
            end
            if nargout > 3
                f_Wrist_Bar = myu.*(g.*m_Body.*t5+g.*m_Leg.*t5+dl_Wrist_Bar.*dth_Wrist.*m_Body.*t3.*t40.*2.0-dl_Wrist_Bar.*dth_Wrist.*m_Body.*t5.*t39.*2.0+dl_Wrist_Bar.*dth_Wrist.*m_Leg.*t3.*t40.*2.0-dl_Wrist_Bar.*dth_Wrist.*m_Leg.*t5.*t39.*2.0-(l_Body.*m_Body.*t3.*t7.*t39)./2.0-(l_Body.*m_Body.*t5.*t7.*t40)./2.0-l_Body.*m_Leg.*t3.*t7.*t39-l_Body.*m_Leg.*t5.*t7.*t40-(l_Leg.*m_Leg.*t3.*t6.*t45)./2.0-(l_Leg.*m_Leg.*t3.*t7.*t45)./2.0-(l_Leg.*m_Leg.*t5.*t6.*t46)./2.0-(l_Leg.*m_Leg.*t5.*t7.*t46)./2.0+l_Wrist_Bar.*m_Body.*t3.*t7.*t39+l_Wrist_Bar.*m_Body.*t5.*t7.*t40+l_Wrist_Bar.*m_Leg.*t3.*t7.*t39+l_Wrist_Bar.*m_Leg.*t5.*t7.*t40+m_Leg.*t3.*t46.*t351.*t352-m_Leg.*t5.*t45.*t351.*t352+t3.*t21.*t46.*t351.*t354-t5.*t21.*t45.*t351.*t354+m_Body.*t3.*t21.*t39.*t351.*t353+m_Body.*t5.*t21.*t40.*t351.*t353+m_Leg.*t3.*t21.*t39.*t351.*t353+m_Leg.*t5.*t21.*t40.*t351.*t353-dth_Hip.*dth_Wrist.*l_Leg.*m_Leg.*t3.*t45-dth_Hip.*dth_Wrist.*l_Leg.*m_Leg.*t5.*t46+l_Body.*m_Body.*t3.*t21.*t40.*t351.*t352-l_Body.*m_Body.*t5.*t21.*t39.*t351.*t352+l_Body.*m_Leg.*t3.*t21.*t40.*t351.*t352.*2.0-l_Body.*m_Leg.*t5.*t21.*t39.*t351.*t352.*2.0-l_Wrist_Bar.*m_Body.*t3.*t21.*t40.*t351.*t352.*2.0+l_Wrist_Bar.*m_Body.*t5.*t21.*t39.*t351.*t352.*2.0-l_Wrist_Bar.*m_Leg.*t3.*t21.*t40.*t351.*t352.*2.0+l_Wrist_Bar.*m_Leg.*t5.*t21.*t39.*t351.*t352.*2.0);
            end
        end
        
        function [f_X,f_Y] = find_F_Onbar(dth_Hip,dth_Wrist,g,l_Body,l_Leg,l_Wrist_Bar,m_Body,m_Leg,tau_Hip,th_Hip,th_Wrist)
            t2 = conj(tau_Hip);
            t3 = cos(th_Hip);
            t4 = sin(th_Hip);
            t5 = sin(th_Wrist);
            t6 = th_Hip+th_Wrist;
            t7 = dth_Hip.^2;
            t8 = dth_Wrist.^2;
            t9 = l_Leg.*2.0;
            t10 = l_Body.^2;
            t11 = l_Leg.^2;
            t12 = l_Wrist_Bar.^2;
            t13 = th_Hip.*2.0;
            t18 = 1.0./l_Leg;
            t20 = 1.0./m_Leg;
            t26 = l_Body.*l_Wrist_Bar.*m_Body.*3.0;
            t27 = l_Body.*l_Wrist_Bar.*m_Leg.*6.0;
            t29 = pi./2.0;
            t38 = l_Body.*l_Wrist_Bar.*m_Body.*1.2e+1;
            t39 = l_Body.*l_Wrist_Bar.*m_Body.*2.4e+1;
            t41 = l_Body.*l_Wrist_Bar.*m_Leg.*2.4e+1;
            t42 = l_Body.*l_Wrist_Bar.*m_Leg.*3.0e+1;
            t14 = cos(t13);
            t15 = t3.^2;
            t16 = sin(t13);
            t17 = sin(t6);
            t19 = 1.0./t11;
            t21 = t2.*1.2e+1;
            t22 = m_Body.*t10;
            t23 = m_Leg.*t11;
            t24 = l_Body.*t3.*3.0;
            t25 = l_Wrist_Bar.*t3.*3.0;
            t28 = t6+th_Hip;
            t32 = m_Leg.*t10.*3.0;
            t33 = m_Body.*t12.*3.0;
            t34 = m_Leg.*t12.*3.0;
            t35 = l_Leg.*t2.*2.4e+1;
            t37 = -t26;
            t40 = -t27;
            t45 = t29+th_Wrist;
            t46 = g.*l_Body.*m_Body.*t5.*3.0;
            t47 = g.*l_Body.*m_Leg.*t5.*6.0;
            t48 = g.*l_Wrist_Bar.*m_Body.*t5.*6.0;
            t49 = g.*l_Wrist_Bar.*m_Leg.*t5.*6.0;
            t51 = m_Leg.*t10.*1.2e+1;
            t52 = m_Leg.*t10.*1.5e+1;
            t53 = m_Body.*t12.*1.2e+1;
            t54 = m_Body.*t12.*2.4e+1;
            t55 = m_Leg.*t12.*1.2e+1;
            t56 = m_Leg.*t12.*1.5e+1;
            t57 = -t38;
            t58 = -t39;
            t59 = -t41;
            t60 = -t42;
            t61 = l_Body.*t2.*t3.*3.6e+1;
            t62 = l_Wrist_Bar.*t2.*t3.*3.6e+1;
            t63 = l_Leg.*l_Wrist_Bar.*m_Leg.*t3.*-3.0;
            t68 = t6+t29;
            t69 = dth_Hip.*dth_Wrist.*l_Body.*l_Leg.*m_Leg.*t4.*6.0;
            t70 = dth_Hip.*dth_Wrist.*l_Leg.*l_Wrist_Bar.*m_Leg.*t4.*6.0;
            t76 = g.*l_Body.*l_Leg.*m_Body.*t5.*1.2e+1;
            t77 = g.*l_Body.*l_Leg.*m_Leg.*t5.*1.5e+1;
            t78 = g.*l_Leg.*l_Wrist_Bar.*m_Body.*t5.*2.4e+1;
            t79 = g.*l_Leg.*l_Wrist_Bar.*m_Leg.*t5.*1.5e+1;
            t80 = l_Body.*l_Leg.*m_Leg.*t4.*t7.*3.0;
            t81 = l_Body.*l_Leg.*m_Leg.*t4.*t8.*6.0;
            t82 = l_Leg.*l_Wrist_Bar.*m_Leg.*t4.*t7.*3.0;
            t83 = l_Leg.*l_Wrist_Bar.*m_Leg.*t4.*t8.*6.0;
            t30 = sin(t28);
            t31 = t22.*4.0;
            t36 = -t25;
            t43 = l_Leg.*m_Leg.*t24;
            t44 = l_Leg.*m_Leg.*t25;
            t50 = t22.*8.0;
            t64 = -t48;
            t65 = -t49;
            t66 = cos(t45);
            t67 = sin(t45);
            t71 = cos(t68);
            t72 = g.*l_Leg.*m_Leg.*t17.*3.0;
            t73 = g.*l_Leg.*m_Leg.*t17.*6.0;
            t74 = sin(t68);
            t75 = -t62;
            t84 = -t70;
            t85 = l_Body.*l_Wrist_Bar.*m_Leg.*t14.*1.8e+1;
            t86 = l_Body.*l_Wrist_Bar.*m_Leg.*t15.*1.8e+1;
            t87 = -t76;
            t88 = -t77;
            t89 = -t81;
            t90 = -t82;
            t91 = m_Leg.*t10.*t14.*9.0;
            t92 = m_Leg.*t12.*t14.*9.0;
            t93 = m_Leg.*t10.*t15.*9.0;
            t94 = m_Leg.*t12.*t15.*9.0;
            t95 = dth_Hip.*dth_Wrist.*l_Body.*t4.*t23.*2.4e+1;
            t96 = dth_Hip.*dth_Wrist.*l_Wrist_Bar.*t4.*t23.*2.4e+1;
            t103 = l_Body.*t4.*t7.*t23.*1.2e+1;
            t104 = l_Body.*t4.*t8.*t23.*1.2e+1;
            t105 = l_Wrist_Bar.*t4.*t7.*t23.*1.2e+1;
            t106 = l_Wrist_Bar.*t4.*t8.*t23.*1.2e+1;
            t108 = l_Body.*l_Leg.*l_Wrist_Bar.*m_Leg.*t8.*t16.*1.8e+1;
            t112 = l_Leg.*m_Leg.*t8.*t10.*t16.*9.0;
            t113 = l_Leg.*m_Leg.*t8.*t12.*t16.*9.0;
            t97 = -t91;
            t98 = -t92;
            t99 = -t93;
            t100 = -t94;
            t101 = g.*l_Body.*l_Leg.*m_Leg.*t30.*9.0;
            t102 = g.*l_Leg.*l_Wrist_Bar.*m_Leg.*t30.*9.0;
            t107 = -t95;
            t110 = -t103;
            t111 = -t104;
            t114 = t9+t24+t36;
            t115 = -t112;
            t116 = -t113;
            t117 = t21+t73+t83+t89;
            t118 = t22+t23+t32+t33+t34+t37+t40+t43+t63;
            t122 = t46+t47+t64+t65+t69+t72+t80+t84+t90;
            t109 = -t102;
            t119 = t31+t51+t53+t55+t57+t59+t86+t99+t100;
            t120 = t50+t52+t54+t56+t58+t60+t85+t97+t98;
            t121 = 1.0./t119;
            t123 = 1.0./t120;
            t127 = t35+t61+t75+t78+t79+t87+t88+t96+t101+t105+t106+t107+t108+t109+t110+t111+t115+t116;
            t124 = t18.*t114.*t121.*t122;
            t125 = t19.*t20.*t117.*t118.*t121;
            t126 = -t125;
            t128 = t124+t126;
            f_X = l_Body.*m_Body.*t8.*t66.*(-1.0./2.0)-l_Body.*m_Leg.*t8.*t66-(l_Leg.*m_Leg.*t7.*t71)./2.0-(l_Leg.*m_Leg.*t8.*t71)./2.0+(l_Leg.*m_Leg.*t74.*t128)./2.0+l_Wrist_Bar.*m_Body.*t8.*t66+l_Wrist_Bar.*m_Leg.*t8.*t66+(m_Leg.*t74.*t123.*t127)./2.0-dth_Hip.*dth_Wrist.*l_Leg.*m_Leg.*t71+(l_Body.*m_Body.*t18.*t67.*t123.*t127)./2.0+l_Body.*m_Leg.*t18.*t67.*t123.*t127-l_Wrist_Bar.*m_Body.*t18.*t67.*t123.*t127-l_Wrist_Bar.*m_Leg.*t18.*t67.*t123.*t127;
            if nargout > 1
                f_Y = g.*m_Body+g.*m_Leg-(l_Body.*m_Body.*t8.*t67)./2.0-l_Body.*m_Leg.*t8.*t67-(l_Leg.*m_Leg.*t7.*t74)./2.0-(l_Leg.*m_Leg.*t8.*t74)./2.0-(l_Leg.*m_Leg.*t71.*t128)./2.0+l_Wrist_Bar.*m_Body.*t8.*t67+l_Wrist_Bar.*m_Leg.*t8.*t67-(m_Leg.*t71.*t123.*t127)./2.0-dth_Hip.*dth_Wrist.*l_Leg.*m_Leg.*t74-(l_Body.*m_Body.*t18.*t66.*t123.*t127)./2.0-l_Body.*m_Leg.*t18.*t66.*t123.*t127+l_Wrist_Bar.*m_Body.*t18.*t66.*t123.*t127+l_Wrist_Bar.*m_Leg.*t18.*t66.*t123.*t127;
            end
        end
        
        function [f_X,f_Y] = find_F_Catch(dl_Wrist_Bar,dth_Hip,dth_Wrist,g,l_Body,l_Leg,l_Wrist_Bar,m_Body,m_Leg,myu,tau_Hip,th_Hip,th_Wrist)
            t2 = cos(th_Hip);
            t3 = cos(th_Wrist);
            t4 = sin(th_Hip);
            t5 = sin(th_Wrist);
            t6 = dth_Hip.^2;
            t7 = dth_Wrist.^2;
            t8 = l_Body.^2;
            t9 = l_Body.^3;
            t10 = l_Leg.^2;
            t11 = l_Leg.^3;
            t12 = l_Wrist_Bar.^2;
            t13 = l_Wrist_Bar.^3;
            t14 = m_Body.^2;
            t15 = m_Leg.^2;
            t16 = m_Leg.^3;
            t17 = th_Hip.*2.0;
            t21 = 1.0./l_Leg;
            t22 = l_Leg.*m_Leg.*tau_Hip.*6.0;
            t23 = pi./2.0;
            t24 = l_Leg.*m_Body.*tau_Hip.*2.4e+1;
            t30 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*9.0e+1;
            t36 = l_Body.*l_Leg.*m_Leg.*myu.*tau_Hip.*3.0e+1;
            t37 = l_Leg.*l_Wrist_Bar.*m_Body.*myu.*tau_Hip.*4.8e+1;
            t38 = l_Leg.*l_Wrist_Bar.*m_Leg.*myu.*tau_Hip.*3.0e+1;
            t51 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*tau_Hip.*2.16e+2;
            t52 = l_Body.*l_Leg.*m_Body.*myu.*tau_Hip.*-2.4e+1;
            t54 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*l_Leg.*m_Body.*m_Leg.*5.4e+1;
            t55 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*6.0e+1;
            t18 = cos(t17);
            t19 = t2.^2;
            t20 = sin(t17);
            t25 = t23+th_Wrist;
            t26 = l_Body.*l_Wrist_Bar.*t14.*4.8e+1;
            t27 = l_Body.*l_Wrist_Bar.*t15.*2.4e+1;
            t28 = m_Body.*m_Leg.*t8.*4.0e+1;
            t29 = m_Body.*m_Leg.*t12.*6.0e+1;
            t31 = l_Body.*m_Body.*t2.*tau_Hip.*3.6e+1;
            t32 = l_Body.*m_Leg.*t2.*tau_Hip.*3.6e+1;
            t33 = l_Wrist_Bar.*m_Body.*t2.*tau_Hip.*3.6e+1;
            t34 = l_Wrist_Bar.*m_Leg.*t2.*tau_Hip.*3.6e+1;
            t35 = l_Body.*myu.*t24;
            t44 = -t30;
            t47 = t10.*t15.*tau_Hip.*6.0;
            t48 = m_Body.*m_Leg.*t10.*tau_Hip.*2.4e+1;
            t53 = -t36;
            t56 = t8.*t14.*1.6e+1;
            t57 = t8.*t15.*1.2e+1;
            t58 = t12.*t14.*4.8e+1;
            t59 = t12.*t15.*1.2e+1;
            t60 = l_Body.*l_Wrist_Bar.*m_Body.*t4.*tau_Hip.*7.2e+1;
            t61 = l_Body.*l_Wrist_Bar.*m_Leg.*t4.*tau_Hip.*1.44e+2;
            t62 = g.*l_Body.*l_Leg.*t5.*t15.*6.0;
            t63 = g.*l_Leg.*l_Wrist_Bar.*t5.*t15.*6.0;
            t65 = g.*l_Body.*l_Leg.*m_Body.*m_Leg.*t5.*2.7e+1;
            t66 = g.*l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*t5.*3.0e+1;
            t67 = t8.*t14.*tau_Hip.*2.4e+1;
            t69 = l_Body.*l_Wrist_Bar.*t14.*tau_Hip.*7.2e+1;
            t70 = l_Body.*l_Wrist_Bar.*t15.*tau_Hip.*1.44e+2;
            t71 = m_Body.*m_Leg.*t8.*tau_Hip.*9.6e+1;
            t73 = m_Body.*m_Leg.*t12.*tau_Hip.*1.44e+2;
            t74 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*l_Leg.*t14.*2.4e+1;
            t75 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*l_Leg.*t15.*1.2e+1;
            t76 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*l_Wrist_Bar.*t14.*4.8e+1;
            t77 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*l_Wrist_Bar.*t15.*1.2e+1;
            t78 = -t54;
            t80 = m_Body.*t4.*t8.*tau_Hip.*2.4e+1;
            t84 = m_Body.*myu.*t2.*t8.*tau_Hip.*1.2e+1;
            t85 = g.*l_Body.*l_Leg.*t5.*t14.*1.2e+1;
            t87 = g.*l_Leg.*l_Wrist_Bar.*t5.*t14.*2.4e+1;
            t88 = l_Body.*l_Leg.*m_Body.*m_Leg.*t2.*tau_Hip.*7.2e+1;
            t89 = l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*t2.*tau_Hip.*7.2e+1;
            t95 = t8.*t15.*tau_Hip.*7.2e+1;
            t96 = t12.*t14.*tau_Hip.*7.2e+1;
            t97 = t12.*t15.*tau_Hip.*7.2e+1;
            t102 = l_Body.*l_Leg.*m_Body.*m_Leg.*myu.*t4.*tau_Hip.*1.8e+1;
            t103 = l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*myu.*t4.*tau_Hip.*3.6e+1;
            t104 = m_Leg.*t4.*t8.*tau_Hip.*7.2e+1;
            t105 = m_Body.*t4.*t12.*tau_Hip.*7.2e+1;
            t106 = m_Leg.*t4.*t12.*tau_Hip.*7.2e+1;
            t107 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*t10.*t16.*1.2e+1;
            t108 = dl_Wrist_Bar.*dth_Wrist.*l_Wrist_Bar.*t10.*t16.*1.2e+1;
            t109 = l_Leg.*m_Body.*m_Leg.*t7.*t9.*2.5e+1;
            t110 = l_Leg.*m_Body.*m_Leg.*t7.*t13.*6.0e+1;
            t112 = l_Body.*l_Leg.*t2.*t15.*tau_Hip.*7.2e+1;
            t113 = l_Leg.*l_Wrist_Bar.*t2.*t15.*tau_Hip.*7.2e+1;
            t122 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*m_Body.*m_Leg.*myu.*t8.*2.0e+1;
            t127 = g.*l_Body.*t5.*t10.*t16.*6.0;
            t128 = g.*l_Wrist_Bar.*t5.*t10.*t16.*6.0;
            t130 = g.*l_Leg.*t3.*t30;
            t134 = l_Body.*l_Leg.*myu.*t4.*t15.*tau_Hip.*3.6e+1;
            t135 = l_Leg.*l_Wrist_Bar.*myu.*t4.*t15.*tau_Hip.*3.6e+1;
            t138 = dth_Hip.*dth_Wrist.*l_Body.*t4.*t10.*t15.*6.0;
            t139 = dth_Hip.*dth_Wrist.*l_Body.*t4.*t11.*t16.*6.0;
            t140 = dth_Hip.*dth_Wrist.*l_Wrist_Bar.*t4.*t10.*t15.*6.0;
            t141 = dth_Hip.*dth_Wrist.*l_Wrist_Bar.*t4.*t11.*t16.*6.0;
            t142 = g.*l_Leg.*myu.*t5.*t8.*t14.*4.0;
            t143 = dth_Hip.*dth_Wrist.*l_Body.*m_Body.*m_Leg.*t4.*t10.*2.4e+1;
            t144 = dth_Hip.*dth_Wrist.*l_Wrist_Bar.*m_Body.*m_Leg.*t4.*t10.*2.4e+1;
            t145 = l_Leg.*t7.*t9.*t14.*8.0;
            t146 = l_Leg.*t7.*t9.*t15.*1.2e+1;
            t147 = l_Leg.*t7.*t13.*t14.*4.8e+1;
            t148 = l_Leg.*t7.*t13.*t15.*1.2e+1;
            t150 = g.*l_Leg.*m_Body.*m_Leg.*myu.*t5.*t8.*1.0e+1;
            t154 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*m_Leg.*t10.*t14.*2.4e+1;
            t155 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*m_Body.*t10.*t15.*5.4e+1;
            t156 = dl_Wrist_Bar.*dth_Wrist.*l_Wrist_Bar.*m_Leg.*t10.*t14.*4.8e+1;
            t157 = dl_Wrist_Bar.*dth_Wrist.*l_Wrist_Bar.*m_Body.*t10.*t15.*6.0e+1;
            t158 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*myu.*t8.*t14.*8.0;
            t164 = g.*l_Leg.*m_Body.*m_Leg.*t3.*t8.*-4.0e+1;
            t165 = g.*l_Leg.*m_Body.*m_Leg.*t3.*t12.*-6.0e+1;
            t170 = l_Body.*t4.*t6.*t10.*t15.*3.0;
            t171 = l_Body.*t4.*t6.*t11.*t16.*3.0;
            t172 = l_Body.*t4.*t7.*t10.*t15.*3.0;
            t173 = l_Body.*t4.*t7.*t11.*t16.*3.0;
            t174 = l_Wrist_Bar.*t4.*t6.*t10.*t15.*3.0;
            t175 = l_Wrist_Bar.*t4.*t6.*t11.*t16.*3.0;
            t176 = l_Wrist_Bar.*t4.*t7.*t10.*t15.*3.0;
            t177 = l_Wrist_Bar.*t4.*t7.*t11.*t16.*3.0;
            t180 = l_Body.*m_Body.*m_Leg.*t4.*t6.*t10.*1.2e+1;
            t181 = l_Body.*m_Body.*m_Leg.*t4.*t7.*t10.*1.2e+1;
            t182 = l_Wrist_Bar.*m_Body.*m_Leg.*t4.*t6.*t10.*1.2e+1;
            t183 = l_Wrist_Bar.*m_Body.*m_Leg.*t4.*t7.*t10.*1.2e+1;
            t184 = g.*l_Body.*m_Leg.*t5.*t10.*t14.*1.2e+1;
            t185 = g.*l_Body.*m_Body.*t5.*t10.*t15.*2.7e+1;
            t186 = g.*l_Wrist_Bar.*m_Leg.*t5.*t10.*t14.*2.4e+1;
            t187 = g.*l_Wrist_Bar.*m_Body.*t5.*t10.*t15.*3.0e+1;
            t193 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*l_Leg.*l_Wrist_Bar.*m_Body.*t2.*t15.*3.6e+1;
            t194 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*l_Leg.*l_Wrist_Bar.*m_Leg.*t2.*t14.*3.6e+1;
            t195 = dth_Hip.*dth_Wrist.*l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*t2.*t10.*4.8e+1;
            t199 = l_Leg.*l_Wrist_Bar.*t7.*t8.*t14.*4.0e+1;
            t200 = l_Body.*l_Leg.*t7.*t12.*t15.*3.6e+1;
            t201 = l_Leg.*l_Wrist_Bar.*t7.*t8.*t15.*3.6e+1;
            t206 = l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*t7.*t8.*1.0e+2;
            t207 = l_Body.*l_Leg.*m_Body.*m_Leg.*t7.*t12.*1.35e+2;
            t208 = g.*l_Leg.*t3.*t8.*t14.*-1.6e+1;
            t209 = g.*l_Leg.*t3.*t8.*t15.*-1.2e+1;
            t210 = g.*l_Leg.*t3.*t12.*t14.*-4.8e+1;
            t211 = g.*l_Leg.*t3.*t12.*t15.*-1.2e+1;
            t213 = g.*l_Body.*l_Leg.*l_Wrist_Bar.*m_Body.*t2.*t5.*t15.*1.8e+1;
            t214 = g.*l_Body.*l_Leg.*l_Wrist_Bar.*m_Leg.*t2.*t5.*t14.*1.8e+1;
            t226 = dth_Hip.*dth_Wrist.*m_Body.*myu.*t8.*t10.*t15.*6.0;
            t230 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*m_Leg.*t2.*t8.*t14.*1.2e+1;
            t231 = dth_Hip.*dth_Wrist.*m_Body.*m_Leg.*t2.*t8.*t10.*1.6e+1;
            t232 = dth_Hip.*dth_Wrist.*m_Body.*m_Leg.*t2.*t10.*t12.*4.8e+1;
            t233 = dth_Hip.*dth_Wrist.*l_Body.*l_Wrist_Bar.*m_Body.*myu.*t10.*t15.*1.8e+1;
            t234 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*t2.*t6.*t10.*2.4e+1;
            t235 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*t2.*t7.*t10.*2.4e+1;
            t236 = l_Leg.*m_Leg.*t4.*t7.*t9.*t14.*6.0;
            t241 = dth_Hip.*dth_Wrist.*l_Body.*m_Body.*t4.*t11.*t15.*2.4e+1;
            t242 = dth_Hip.*dth_Wrist.*l_Wrist_Bar.*m_Body.*t4.*t11.*t15.*2.4e+1;
            t243 = g.*l_Body.*l_Leg.*m_Body.*m_Leg.*myu.*t2.*t4.*t5.*9.0;
            t246 = l_Body.*l_Leg.*t7.*t12.*t14.*7.2e+1;
            t250 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*m_Leg.*myu.*t4.*t8.*t14.*6.0;
            t252 = g.*l_Leg.*m_Body.*t2.*t5.*t8.*t15.*6.0;
            t253 = g.*l_Leg.*m_Leg.*t2.*t5.*t8.*t14.*6.0;
            t258 = t2.*t6.*t8.*t10.*t15.*6.0;
            t259 = t2.*t7.*t8.*t10.*t15.*6.0;
            t260 = t2.*t6.*t10.*t12.*t15.*6.0;
            t261 = t2.*t7.*t10.*t12.*t15.*6.0;
            t262 = m_Body.*myu.*t6.*t8.*t10.*t15.*3.0;
            t263 = m_Body.*myu.*t7.*t8.*t10.*t15.*3.0;
            t269 = l_Body.*l_Wrist_Bar.*t2.*t6.*t10.*t15.*1.2e+1;
            t270 = l_Body.*l_Wrist_Bar.*t2.*t7.*t10.*t15.*1.2e+1;
            t272 = m_Body.*m_Leg.*t2.*t6.*t8.*t10.*8.0;
            t273 = m_Body.*m_Leg.*t2.*t7.*t8.*t10.*8.0;
            t274 = m_Body.*m_Leg.*t2.*t6.*t10.*t12.*2.4e+1;
            t275 = m_Body.*m_Leg.*t2.*t7.*t10.*t12.*2.4e+1;
            t276 = l_Body.*l_Wrist_Bar.*m_Body.*myu.*t6.*t10.*t15.*9.0;
            t277 = l_Body.*l_Wrist_Bar.*m_Body.*myu.*t7.*t10.*t15.*9.0;
            t278 = dth_Hip.*dth_Wrist.*l_Body.*l_Wrist_Bar.*t2.*t10.*t15.*-2.4e+1;
            t280 = g.*l_Leg.*m_Leg.*myu.*t4.*t5.*t8.*t14.*3.0;
            t283 = l_Body.*m_Body.*t4.*t6.*t11.*t15.*1.2e+1;
            t284 = l_Body.*m_Body.*t4.*t7.*t11.*t15.*1.2e+1;
            t285 = l_Leg.*m_Body.*t4.*t7.*t9.*t15.*1.8e+1;
            t286 = l_Wrist_Bar.*m_Body.*t4.*t6.*t11.*t15.*1.2e+1;
            t287 = l_Wrist_Bar.*m_Body.*t4.*t7.*t11.*t15.*1.2e+1;
            t293 = m_Body.*m_Leg.*myu.*t4.*t6.*t8.*t10.*4.0;
            t294 = m_Body.*m_Leg.*myu.*t4.*t7.*t8.*t10.*4.0;
            t295 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*m_Body.*myu.*t4.*t8.*t15.*2.4e+1;
            t296 = dth_Hip.*dth_Wrist.*m_Body.*m_Leg.*myu.*t4.*t8.*t10.*8.0;
            t300 = dth_Hip.*dth_Wrist.*l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*myu.*t4.*t10.*-2.4e+1;
            t313 = l_Body.*l_Leg.*m_Body.*t4.*t7.*t12.*t15.*1.8e+1;
            t314 = l_Body.*l_Leg.*m_Leg.*t4.*t7.*t12.*t14.*1.8e+1;
            t315 = l_Leg.*l_Wrist_Bar.*m_Leg.*t4.*t7.*t8.*t14.*1.8e+1;
            t317 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*myu.*t4.*t6.*t10.*-1.2e+1;
            t318 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*myu.*t4.*t7.*t10.*-1.2e+1;
            t328 = l_Leg.*l_Wrist_Bar.*m_Body.*t4.*t7.*t8.*t15.*-3.6e+1;
            t332 = g.*l_Body.*m_Body.*myu.*t2.*t4.*t5.*t10.*t15.*9.0;
            t39 = cos(t25);
            t40 = sin(t25);
            t41 = t25+th_Hip;
            t42 = -t26;
            t43 = -t27;
            t49 = -t33;
            t50 = -t34;
            t64 = l_Wrist_Bar.*myu.*t31;
            t68 = -t47;
            t72 = -t48;
            t79 = l_Leg.*m_Leg.*t19.*tau_Hip.*1.8e+1;
            t81 = -t60;
            t82 = -t61;
            t83 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*t18.*1.8e+1;
            t86 = -t62;
            t90 = l_Body.*l_Leg.*m_Leg.*t20.*tau_Hip.*1.8e+1;
            t91 = l_Leg.*l_Wrist_Bar.*m_Leg.*t20.*tau_Hip.*1.8e+1;
            t92 = -t65;
            t93 = l_Leg.*m_Leg.*myu.*t20.*tau_Hip.*9.0;
            t94 = -t67;
            t98 = -t71;
            t99 = -t73;
            t100 = -t74;
            t101 = -t75;
            t111 = m_Body.*m_Leg.*t8.*t18.*1.2e+1;
            t114 = -t84;
            t115 = -t85;
            t116 = -t88;
            t119 = -t95;
            t120 = -t96;
            t121 = -t97;
            t123 = g.*l_Leg.*t3.*t26;
            t124 = g.*l_Leg.*t3.*t27;
            t125 = g.*l_Leg.*t3.*t28;
            t126 = g.*l_Leg.*t3.*t29;
            t129 = m_Body.*m_Leg.*myu.*t8.*t20.*3.0;
            t131 = l_Body.*l_Leg.*m_Leg.*myu.*t18.*tau_Hip.*1.8e+1;
            t132 = l_Leg.*l_Wrist_Bar.*m_Leg.*myu.*t18.*tau_Hip.*1.8e+1;
            t133 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*myu.*t20.*9.0;
            t136 = -t103;
            t137 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*l_Leg.*m_Body.*m_Leg.*t19.*1.8e+1;
            t149 = -t108;
            t151 = -t110;
            t153 = -t112;
            t159 = g.*l_Leg.*t3.*t56;
            t160 = g.*l_Leg.*t3.*t57;
            t161 = g.*l_Leg.*t3.*t58;
            t162 = g.*l_Leg.*t3.*t59;
            t163 = -t122;
            t166 = -t128;
            t169 = -t135;
            t178 = -t138;
            t179 = -t141;
            t188 = -t142;
            t189 = -t143;
            t190 = -t147;
            t191 = -t148;
            t192 = -t150;
            t196 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*m_Body.*m_Leg.*t8.*t20.*6.0;
            t197 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*t20.*1.8e+1;
            t198 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*l_Leg.*m_Body.*m_Leg.*myu.*t20.*9.0;
            t202 = g.*l_Body.*l_Leg.*m_Body.*m_Leg.*t5.*t19.*9.0;
            t203 = -t156;
            t204 = -t157;
            t205 = -t158;
            t212 = t10.*t15.*t19.*tau_Hip.*1.8e+1;
            t215 = g.*l_Leg.*m_Body.*m_Leg.*t5.*t8.*t20.*3.0;
            t216 = -t170;
            t217 = -t172;
            t218 = -t175;
            t219 = -t177;
            t220 = g.*l_Body.*l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*t5.*t20.*9.0;
            t221 = -t180;
            t222 = -t181;
            t223 = -t186;
            t224 = -t187;
            t225 = myu.*t10.*t15.*t20.*tau_Hip.*9.0;
            t227 = l_Leg.*m_Body.*m_Leg.*t7.*t9.*t18.*3.0;
            t228 = dth_Hip.*dth_Wrist.*t2.*t10.*t27;
            t229 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*m_Body.*t2.*t57;
            t237 = -t193;
            t238 = -t194;
            t239 = -t195;
            t245 = -t199;
            t247 = -t201;
            t248 = -t206;
            t254 = l_Wrist_Bar.*myu.*t143;
            t255 = -t213;
            t256 = -t214;
            t264 = dth_Hip.*dth_Wrist.*t2.*t10.*t57;
            t265 = dth_Hip.*dth_Wrist.*t2.*t10.*t59;
            t267 = g.*l_Leg.*m_Body.*m_Leg.*myu.*t5.*t8.*t18.*6.0;
            t268 = -t226;
            t281 = -t234;
            t282 = -t235;
            t288 = -t242;
            t289 = -t243;
            t292 = l_Leg.*m_Body.*m_Leg.*myu.*t7.*t9.*t20.*3.0;
            t297 = g.*l_Body.*l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*t3.*t18.*-1.8e+1;
            t298 = l_Wrist_Bar.*myu.*t180;
            t299 = l_Wrist_Bar.*myu.*t181;
            t301 = -t262;
            t302 = -t263;
            t303 = g.*l_Leg.*m_Body.*m_Leg.*myu.*t3.*t8.*t20.*-3.0;
            t304 = -t269;
            t305 = -t270;
            t306 = g.*l_Leg.*m_Body.*myu.*t4.*t5.*t57;
            t307 = -t286;
            t308 = -t287;
            t309 = l_Body.*l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*t7.*t20.*(9.0./2.0);
            t310 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*m_Body.*t10.*t15.*t19.*1.8e+1;
            t311 = l_Body.*l_Leg.*m_Body.*m_Leg.*t7.*t12.*t18.*9.0;
            t316 = m_Body.*t4.*t201;
            t319 = dth_Hip.*dth_Wrist.*l_Body.*l_Wrist_Bar.*m_Body.*t10.*t15.*t20.*1.8e+1;
            t320 = dl_Wrist_Bar.*dth_Wrist.*l_Body.*m_Body.*myu.*t10.*t15.*t20.*9.0;
            t321 = l_Body.*l_Leg.*m_Body.*m_Leg.*myu.*t7.*t12.*t20.*9.0;
            t322 = l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*myu.*t7.*t8.*t20.*1.2e+1;
            t323 = g.*l_Body.*m_Body.*t5.*t10.*t15.*t19.*9.0;
            t324 = l_Leg.*m_Body.*m_Leg.*t7.*t8.*t20.*(9.0./2.0);
            t327 = -t315;
            t329 = m_Body.*t6.*t8.*t10.*t15.*t20.*6.0;
            t330 = dth_Hip.*dth_Wrist.*m_Body.*t10.*t20.*t57;
            t331 = l_Body.*l_Wrist_Bar.*m_Body.*t6.*t10.*t15.*t20.*9.0;
            t337 = t19.*t226;
            t338 = t19.*t233;
            t340 = t19.*t262;
            t341 = t19.*t263;
            t342 = t19.*t276;
            t343 = t19.*t277;
            t344 = dth_Hip.*dth_Wrist.*l_Body.*l_Wrist_Bar.*m_Body.*myu.*t10.*t15.*t19.*-1.8e+1;
            t345 = l_Body.*l_Wrist_Bar.*m_Body.*t7.*t10.*t15.*t20.*(2.7e+1./2.0);
            t346 = l_Body.*l_Wrist_Bar.*m_Body.*myu.*t6.*t10.*t15.*t19.*-9.0;
            t347 = l_Body.*l_Wrist_Bar.*m_Body.*myu.*t7.*t10.*t15.*t19.*-9.0;
            t348 = m_Body.*t7.*t8.*t10.*t15.*t20.*(2.1e+1./2.0);
            t45 = cos(t41);
            t46 = sin(t41);
            t117 = -t91;
            t118 = -t93;
            t152 = -t111;
            t167 = -t132;
            t168 = -t133;
            t240 = -t196;
            t244 = -t198;
            t249 = -t212;
            t251 = g.*l_Leg.*t3.*t83;
            t257 = -t215;
            t266 = g.*l_Leg.*t3.*t129;
            t271 = -t227;
            t279 = g.*l_Leg.*t3.*t133;
            t290 = dl_Wrist_Bar.*dth_Wrist.*l_Leg.*myu.*t111;
            t291 = g.*l_Leg.*t3.*t111;
            t312 = l_Leg.*l_Wrist_Bar.*t7.*t111;
            t325 = -t310;
            t326 = -t311;
            t333 = -t319;
            t334 = -t322;
            t335 = -t323;
            t336 = -t324;
            t339 = -t331;
            t349 = -t345;
            t350 = t28+t29+t42+t43+t44+t56+t57+t58+t59+t83+t129+t152+t168;
            t352 = t22+t24+t31+t32+t49+t50+t55+t63+t66+t76+t77+t78+t79+t86+t87+t92+t100+t101+t115+t118+t137+t140+t144+t174+t176+t178+t182+t183+t189+t202+t216+t217+t221+t222+t244+t289+t309+t336;
            t353 = t37+t38+t52+t53+t64+t80+t81+t82+t90+t104+t105+t106+t109+t114+t117+t123+t124+t130+t131+t145+t146+t151+t163+t164+t165+t167+t188+t190+t191+t192+t197+t200+t205+t207+t208+t209+t210+t211+t220+t231+t232+t239+t240+t245+t246+t247+t248+t257+t258+t259+t260+t261+t264+t265+t267+t271+t272+t273+t274+t275+t278+t279+t281+t282+t290+t291+t292+t293+t294+t296+t297+t300+t303+t304+t305+t312+t317+t318+t321+t326+t334;
            t354 = t51+t68+t69+t70+t72+t89+t94+t98+t99+t102+t107+t113+t116+t119+t120+t121+t127+t134+t136+t139+t149+t153+t154+t155+t166+t169+t171+t173+t179+t184+t185+t203+t204+t218+t219+t223+t224+t225+t229+t230+t233+t236+t237+t238+t241+t249+t250+t252+t253+t255+t256+t268+t276+t277+t280+t283+t284+t285+t288+t295+t301+t302+t306+t307+t308+t313+t314+t320+t325+t327+t328+t329+t330+t332+t333+t335+t337+t339+t340+t341+t344+t346+t347+t348+t349;
            t351 = 1.0./t350;
            f_X = dl_Wrist_Bar.*dth_Wrist.*m_Body.*t40.*2.0+dl_Wrist_Bar.*dth_Wrist.*m_Leg.*t40.*2.0-(l_Body.*m_Body.*t7.*t39)./2.0-l_Body.*m_Leg.*t7.*t39-(l_Leg.*m_Leg.*t6.*t45)./2.0-(l_Leg.*m_Leg.*t7.*t45)./2.0+l_Wrist_Bar.*m_Body.*t7.*t39+l_Wrist_Bar.*m_Leg.*t7.*t39+m_Leg.*t46.*t351.*t352+t21.*t46.*t351.*t354-dth_Hip.*dth_Wrist.*l_Leg.*m_Leg.*t45+m_Body.*t21.*t39.*t351.*t353+m_Leg.*t21.*t39.*t351.*t353+l_Body.*m_Body.*t21.*t40.*t351.*t352+l_Body.*m_Leg.*t21.*t40.*t351.*t352.*2.0-l_Wrist_Bar.*m_Body.*t21.*t40.*t351.*t352.*2.0-l_Wrist_Bar.*m_Leg.*t21.*t40.*t351.*t352.*2.0;
            if nargout > 1
                f_Y = g.*m_Body+g.*m_Leg-dl_Wrist_Bar.*dth_Wrist.*m_Body.*t39.*2.0-dl_Wrist_Bar.*dth_Wrist.*m_Leg.*t39.*2.0-(l_Body.*m_Body.*t7.*t40)./2.0-l_Body.*m_Leg.*t7.*t40-(l_Leg.*m_Leg.*t6.*t46)./2.0-(l_Leg.*m_Leg.*t7.*t46)./2.0+l_Wrist_Bar.*m_Body.*t7.*t40+l_Wrist_Bar.*m_Leg.*t7.*t40-m_Leg.*t45.*t351.*t352-t21.*t45.*t351.*t354-dth_Hip.*dth_Wrist.*l_Leg.*m_Leg.*t46+m_Body.*t21.*t40.*t351.*t353+m_Leg.*t21.*t40.*t351.*t353-l_Body.*m_Body.*t21.*t39.*t351.*t352-l_Body.*m_Leg.*t21.*t39.*t351.*t352.*2.0+l_Wrist_Bar.*m_Body.*t21.*t39.*t351.*t352.*2.0+l_Wrist_Bar.*m_Leg.*t21.*t39.*t351.*t352.*2.0;
            end
        end
        
        function [dth_Wrist_After,dth_Hip_After,dl_Wrist_Bar_After,I_N_Wrist_Bar] = find_Status_After_Collision(dl_Wrist_Bar_Before,dl_N_Wrist_Bar_Before,dth_Hip_Before,dth_Wrist_Before,l_Body,l_Leg,l_N_Wrist_Bar,l_Wrist_Bar,m_Body,m_Leg,myu,th_Hip)
            t2 = cos(th_Hip);
            t3 = sin(th_Hip);
            t4 = l_Body.^2;
            t5 = l_N_Wrist_Bar.^2;
            t6 = l_Wrist_Bar.^2;
            t7 = m_Body.^2;
            t8 = m_Leg.^2;
            t9 = th_Hip.*2.0;
            t16 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*9.0e+1;
            t17 = l_Body.*l_N_Wrist_Bar.*m_Body.*m_Leg.*myu.*4.5e+1;
            t18 = l_N_Wrist_Bar.*l_Wrist_Bar.*m_Body.*m_Leg.*myu.*6.0e+1;
            t10 = cos(t9);
            t11 = sin(t9);
            t12 = l_Body.*l_Wrist_Bar.*t7.*4.8e+1;
            t13 = l_Body.*l_Wrist_Bar.*t8.*2.4e+1;
            t14 = m_Body.*m_Leg.*t4.*4.0e+1;
            t15 = m_Body.*m_Leg.*t6.*6.0e+1;
            t21 = -t16;
            t22 = l_Body.*l_N_Wrist_Bar.*myu.*t7.*2.4e+1;
            t23 = l_Body.*l_N_Wrist_Bar.*myu.*t8.*1.2e+1;
            t24 = l_N_Wrist_Bar.*l_Wrist_Bar.*myu.*t7.*4.8e+1;
            t25 = l_N_Wrist_Bar.*l_Wrist_Bar.*myu.*t8.*1.2e+1;
            t26 = -t18;
            t27 = t4.*t7.*1.6e+1;
            t28 = t4.*t8.*1.2e+1;
            t29 = t6.*t7.*4.8e+1;
            t30 = t6.*t8.*1.2e+1;
            t19 = -t12;
            t20 = -t13;
            t31 = -t24;
            t32 = -t25;
            t33 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*t10.*1.8e+1;
            t34 = m_Body.*m_Leg.*t4.*t10.*1.2e+1;
            t35 = l_Body.*l_N_Wrist_Bar.*m_Body.*m_Leg.*myu.*t10.*9.0;
            t36 = m_Body.*m_Leg.*myu.*t4.*t11.*3.0;
            t37 = l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*myu.*t11.*9.0;
            t38 = -t34;
            t39 = -t35;
            t40 = -t37;
            t41 = t14+t15+t17+t19+t20+t21+t22+t23+t26+t27+t28+t29+t30+t31+t32+t33+t36+t38+t39+t40;
            t42 = 1.0./t41;
            dth_Wrist_After = dth_Wrist_Before-dl_N_Wrist_Bar_Before.*t42.*(l_Body.*t7.*8.0+l_Body.*t8.*4.0-l_Wrist_Bar.*t7.*1.6e+1-l_Wrist_Bar.*t8.*4.0+l_Body.*m_Body.*m_Leg.*1.5e+1-l_Wrist_Bar.*m_Body.*m_Leg.*2.0e+1+l_N_Wrist_Bar.*myu.*t7.*1.6e+1+l_N_Wrist_Bar.*myu.*t8.*4.0+l_N_Wrist_Bar.*m_Body.*m_Leg.*myu.*2.0e+1-l_Body.*m_Body.*m_Leg.*t10.*3.0+l_Body.*m_Body.*m_Leg.*myu.*t11.*3.0).*3.0;
            if nargout > 1
                dth_Hip_After = dth_Hip_Before+(dl_N_Wrist_Bar_Before.*t42.*(l_Body.*l_Leg.*t7.*8.0+l_Body.*l_Leg.*t8.*4.0-l_Leg.*l_Wrist_Bar.*t7.*1.6e+1-l_Leg.*l_Wrist_Bar.*t8.*4.0+t2.*t4.*t7.*4.0+l_Body.*l_Leg.*m_Body.*m_Leg.*1.8e+1-l_Leg.*l_Wrist_Bar.*m_Body.*m_Leg.*2.0e+1+l_Leg.*l_N_Wrist_Bar.*myu.*t7.*1.6e+1+l_Leg.*l_N_Wrist_Bar.*myu.*t8.*4.0-l_Body.*l_Wrist_Bar.*t2.*t7.*1.2e+1+m_Body.*m_Leg.*t2.*t4.*4.0+myu.*t3.*t4.*t7.*2.0+l_Leg.*l_N_Wrist_Bar.*m_Body.*m_Leg.*myu.*2.0e+1-l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*t2.*1.2e+1+l_Body.*l_N_Wrist_Bar.*myu.*t2.*t7.*1.2e+1+m_Body.*m_Leg.*myu.*t3.*t4.*8.0-l_Body.*l_Leg.*m_Body.*m_Leg.*t2.^2.*6.0+l_Body.*l_Leg.*m_Body.*m_Leg.*myu.*t11.*3.0+l_Body.*l_N_Wrist_Bar.*m_Body.*m_Leg.*myu.*t2.*1.2e+1).*3.0)./l_Leg;
            end
            if nargout > 2
                dl_Wrist_Bar_After = dl_Wrist_Bar_Before-dl_N_Wrist_Bar_Before.*t42.*(l_Body.*l_N_Wrist_Bar.*t7.*2.4e+1+l_Body.*l_N_Wrist_Bar.*t8.*1.2e+1-l_N_Wrist_Bar.*l_Wrist_Bar.*t7.*4.8e+1-l_N_Wrist_Bar.*l_Wrist_Bar.*t8.*1.2e+1+myu.*t4.*t7.*4.0+myu.*t5.*t7.*4.8e+1+myu.*t5.*t8.*1.2e+1+l_Body.*l_N_Wrist_Bar.*m_Body.*m_Leg.*4.5e+1-l_N_Wrist_Bar.*l_Wrist_Bar.*m_Body.*m_Leg.*6.0e+1+m_Body.*m_Leg.*myu.*t4.*1.0e+1+m_Body.*m_Leg.*myu.*t5.*6.0e+1+m_Body.*m_Leg.*t4.*t11.*3.0-l_Body.*l_N_Wrist_Bar.*m_Body.*m_Leg.*t10.*9.0-l_Body.*l_Wrist_Bar.*m_Body.*m_Leg.*t11.*9.0-m_Body.*m_Leg.*myu.*t4.*t10.*6.0+l_Body.*l_N_Wrist_Bar.*m_Body.*m_Leg.*myu.*t11.*1.8e+1);
            end
            if nargout > 3
                I_N_Wrist_Bar = dl_N_Wrist_Bar_Before.*m_Body.*t4.*t42.*(t7.*8.0+t8.*8.0+m_Body.*m_Leg.*2.5e+1-m_Body.*m_Leg.*t10.*9.0).*(-1.0./2.0);
            end
        end
        
        function [dth_Wrist_After,dth_Hip_After,I_F_Wrist_Bar] = find_Status_After_Slides_Collision(dl_Wrist_Bar_Before,dth_Hip_Before,dth_Wrist_Before,l_Body,l_Leg,l_Wrist_Bar,m_Body,m_Leg,th_Hip)
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
            
            
        end
        
        function ang_M = find_Ang_M(dth_Hip,dth_Wrist,l_Body,l_Leg,m_Body,m_Leg,th_Hip)
            t2 = cos(th_Hip);
            t3 = l_Body.^2;
            t4 = l_Leg.^2;
            t5 = m_Leg.^2;
            ang_M = (dth_Wrist.*m_Body.^2.*t3+dth_Hip.*t4.*t5+dth_Wrist.*t4.*t5+dth_Hip.*m_Body.*m_Leg.*t4.*4.0+dth_Wrist.*m_Body.*m_Leg.*t3.*4.0+dth_Wrist.*m_Body.*m_Leg.*t4.*4.0+dth_Hip.*l_Body.*l_Leg.*m_Body.*m_Leg.*t2.*3.0+dth_Wrist.*l_Body.*l_Leg.*m_Body.*m_Leg.*t2.*6.0)./(m_Body.*1.2e+1+m_Leg.*1.2e+1);
        end
        
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            
            app.Initialize_Data_Array = [
                app.xGSlider.Value;
                app.dxGSlider.Value;
                app.yGSlider.Value;
                app.dyGSlider.Value;
                app.th_WristSlider.Value;
                app.dth_WristSlider.Value;
                app.th_HipSlider.Value;
                app.dth_HipSlider.Value;
                ];
            
            initialize_Data(app)
            
            app.quiver_Ratio = 4;
            
            app.stick_Body = plot(app.UIAxes, [0, 1], [0, 1], '-', 'LineWidth', 3);
            
            hold(app.UIAxes, "on")
            app.stick_Leg = plot(app.UIAxes, [0, 1], [0, 1], '-', 'LineWidth', 3);
            hold(app.UIAxes, "off")
            
            app.head = rectangle(app.UIAxes, 'Position', [0,0, app.r_Head * 2, app.r_Head * 2], 'Curvature', [1 1]);
            
            
            xlim(app.UIAxes, [-(app.l_Body + app.l_Leg), (app.l_Body + app.l_Leg) * 3] * 1.5)
            ylim(app.UIAxes, [-(app.l_Body + app.l_Leg), (app.l_Body + app.l_Leg) * 1] * 1.5)
            
            hold(app.UIAxes, "on")
            app.quivers(1,1) = quiver(app.UIAxes, 0, 0, 1, 1);
            hold(app.UIAxes, "off")
            
            hold(app.UIAxes, "on")
            scatter(app.UIAxes, 0,0,[],"black","filled")
            hold(app.UIAxes, "off")
            
            refresh_Stick(app)
            refresh_Quivers(app)
            
            app.slider_Positions = [
                app.xGSlider.Position;
                app.dxGSlider.Position;
                app.yGSlider.Position;
                app.dyGSlider.Position;
                app.th_WristSlider.Position;
                app.dth_WristSlider.Position;
                app.th_HipSlider.Position;
                app.dth_HipSlider.Position;
                ];
            
            if isequal(app.Status_Buttons.SelectedObject.Text, 'InAir')
                app.xGSlider.Visible = 'on';
                app.dxGSlider.Visible = 'on';
                app.yGSlider.Visible = 'on';
                app.dyGSlider.Visible = 'on';
            elseif isequal(app.Status_Buttons.SelectedObject.Text, 'OnBar')
                app.xGSlider.Visible = 'off';
                app.dxGSlider.Visible = 'off';
                app.yGSlider.Visible = 'off';
                app.dyGSlider.Visible = 'off';
            end
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
            app.tau_Hip = changingValue;
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

        % Button pushed function: Take_OffButton
        function Take_OffButtonPushed(app, event)
            app.StatusLabel.Text = 'InAir';
        end

        % Value changing function: dth_HipSlider, dth_WristSlider, 
        % dxGSlider, dyGSlider, th_HipSlider, th_WristSlider, 
        % xGSlider, yGSlider
        function Initialize_Data_SliderValueChanging(app, event)
            changingValue = event.Value;
            if ~app.PlayingButton.Value
                
                target_Index = prod(event.Source.Position == app.slider_Positions, 2);
                
                app.Initialize_Data_Array(logical(target_Index)) = changingValue;
                
                initialize_Data(app)
                refresh_Stick(app)
                drawnow
            end
        end

        % Selection changed function: Status_Buttons
        function Status_ButtonsSelectionChanged(app, event)
            selectedButton = app.Status_Buttons.SelectedObject;
            if isequal(selectedButton.Text, 'InAir')
                app.xGSlider.Visible = 'on';
                app.dxGSlider.Visible = 'on';
                app.yGSlider.Visible = 'on';
                app.dyGSlider.Visible = 'on';
            elseif isequal(selectedButton.Text, 'OnBar')
                app.xGSlider.Visible = 'off';
                app.dxGSlider.Visible = 'off';
                app.yGSlider.Visible = 'off';
                app.dyGSlider.Visible = 'off';
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [400 200 1031 731];
            app.UIFigure.Name = 'MATLAB App';

            % Create PlayingButton
            app.PlayingButton = uibutton(app.UIFigure, 'state');
            app.PlayingButton.ValueChangedFcn = createCallbackFcn(app, @PlayingButtonValueChanged, true);
            app.PlayingButton.Text = 'Playing';
            app.PlayingButton.Position = [424 167 70 22];

            % Create Slider
            app.Slider = uislider(app.UIFigure);
            app.Slider.Limits = [-10 10];
            app.Slider.ValueChangingFcn = createCallbackFcn(app, @SliderValueChanging, true);
            app.Slider.Position = [404 84 226 3];

            % Create ResetButton
            app.ResetButton = uibutton(app.UIFigure, 'state');
            app.ResetButton.ValueChangedFcn = createCallbackFcn(app, @ResetButtonValueChanged, true);
            app.ResetButton.Text = 'Reset';
            app.ResetButton.Position = [424 116 70 22];

            % Create FrameButton
            app.FrameButton = uibutton(app.UIFigure, 'push');
            app.FrameButton.ButtonPushedFcn = createCallbackFcn(app, @FrameButtonPushed, true);
            app.FrameButton.Position = [540 167 70 22];
            app.FrameButton.Text = '1 Frame';

            % Create PlaySpeedxEditFieldLabel
            app.PlaySpeedxEditFieldLabel = uilabel(app.UIFigure);
            app.PlaySpeedxEditFieldLabel.HorizontalAlignment = 'right';
            app.PlaySpeedxEditFieldLabel.Position = [454 206 76 22];
            app.PlaySpeedxEditFieldLabel.Text = 'Play Speed x';

            % Create PlaySpeedxEditField
            app.PlaySpeedxEditField = uieditfield(app.UIFigure, 'numeric');
            app.PlaySpeedxEditField.Limits = [1e-06 Inf];
            app.PlaySpeedxEditField.Position = [529 206 51 22];
            app.PlaySpeedxEditField.Value = 0.5;

            % Create Take_OffButton
            app.Take_OffButton = uibutton(app.UIFigure, 'push');
            app.Take_OffButton.ButtonPushedFcn = createCallbackFcn(app, @Take_OffButtonPushed, true);
            app.Take_OffButton.Position = [540 116 70 22];
            app.Take_OffButton.Text = 'Take_Off';

            % Create InitializationPanel
            app.InitializationPanel = uipanel(app.UIFigure);
            app.InitializationPanel.TitlePosition = 'centertop';
            app.InitializationPanel.Title = 'Initialization';
            app.InitializationPanel.FontSize = 20;
            app.InitializationPanel.Position = [722 19 211 280];

            % Create Status_Buttons
            app.Status_Buttons = uibuttongroup(app.InitializationPanel);
            app.Status_Buttons.SelectionChangedFcn = createCallbackFcn(app, @Status_ButtonsSelectionChanged, true);
            app.Status_Buttons.BorderType = 'none';
            app.Status_Buttons.Position = [10 194 187 44];

            % Create OnBarButton
            app.OnBarButton = uitogglebutton(app.Status_Buttons);
            app.OnBarButton.Text = 'OnBar';
            app.OnBarButton.Position = [1 11 86 22];
            app.OnBarButton.Value = true;

            % Create InAirButton
            app.InAirButton = uitogglebutton(app.Status_Buttons);
            app.InAirButton.Text = 'InAir';
            app.InAirButton.Position = [100 11 86 22];

            % Create xGSliderLabel
            app.xGSliderLabel = uilabel(app.InitializationPanel);
            app.xGSliderLabel.HorizontalAlignment = 'right';
            app.xGSliderLabel.Position = [1 169 25 22];
            app.xGSliderLabel.Text = 'xG';

            % Create xGSlider
            app.xGSlider = uislider(app.InitializationPanel);
            app.xGSlider.Limits = [-1 1];
            app.xGSlider.MajorTicks = [];
            app.xGSlider.ValueChangingFcn = createCallbackFcn(app, @Initialize_Data_SliderValueChanging, true);
            app.xGSlider.Position = [47 178 150 3];

            % Create dxGSliderLabel
            app.dxGSliderLabel = uilabel(app.InitializationPanel);
            app.dxGSliderLabel.HorizontalAlignment = 'right';
            app.dxGSliderLabel.Position = [-2 146 28 22];
            app.dxGSliderLabel.Text = 'dxG';

            % Create dxGSlider
            app.dxGSlider = uislider(app.InitializationPanel);
            app.dxGSlider.Limits = [-3 3];
            app.dxGSlider.MajorTicks = [];
            app.dxGSlider.ValueChangingFcn = createCallbackFcn(app, @Initialize_Data_SliderValueChanging, true);
            app.dxGSlider.Position = [47 155 150 3];

            % Create yGSliderLabel
            app.yGSliderLabel = uilabel(app.InitializationPanel);
            app.yGSliderLabel.HorizontalAlignment = 'right';
            app.yGSliderLabel.Position = [1 123 25 22];
            app.yGSliderLabel.Text = 'yG';

            % Create yGSlider
            app.yGSlider = uislider(app.InitializationPanel);
            app.yGSlider.Limits = [-1 1];
            app.yGSlider.MajorTicks = [];
            app.yGSlider.ValueChangingFcn = createCallbackFcn(app, @Initialize_Data_SliderValueChanging, true);
            app.yGSlider.Position = [47 132 150 3];

            % Create dyGSliderLabel
            app.dyGSliderLabel = uilabel(app.InitializationPanel);
            app.dyGSliderLabel.HorizontalAlignment = 'right';
            app.dyGSliderLabel.Position = [-2 100 28 22];
            app.dyGSliderLabel.Text = 'dyG';

            % Create dyGSlider
            app.dyGSlider = uislider(app.InitializationPanel);
            app.dyGSlider.Limits = [-3 3];
            app.dyGSlider.MajorTicks = [];
            app.dyGSlider.ValueChangingFcn = createCallbackFcn(app, @Initialize_Data_SliderValueChanging, true);
            app.dyGSlider.Position = [47 109 150 3];

            % Create WLabel
            app.WLabel = uilabel(app.InitializationPanel);
            app.WLabel.HorizontalAlignment = 'right';
            app.WLabel.Position = [1 77 25 22];
            app.WLabel.Text = 'θW';

            % Create th_WristSlider
            app.th_WristSlider = uislider(app.InitializationPanel);
            app.th_WristSlider.Limits = [-180 180];
            app.th_WristSlider.MajorTicks = [];
            app.th_WristSlider.ValueChangingFcn = createCallbackFcn(app, @Initialize_Data_SliderValueChanging, true);
            app.th_WristSlider.Position = [47 86 150 3];

            % Create WLabel_2
            app.WLabel_2 = uilabel(app.InitializationPanel);
            app.WLabel_2.HorizontalAlignment = 'right';
            app.WLabel_2.Position = [0 54 26 22];
            app.WLabel_2.Text = 'ωW';

            % Create dth_WristSlider
            app.dth_WristSlider = uislider(app.InitializationPanel);
            app.dth_WristSlider.Limits = [-3 3];
            app.dth_WristSlider.MajorTicks = [];
            app.dth_WristSlider.ValueChangingFcn = createCallbackFcn(app, @Initialize_Data_SliderValueChanging, true);
            app.dth_WristSlider.Position = [47 63 150 3];

            % Create HLabel
            app.HLabel = uilabel(app.InitializationPanel);
            app.HLabel.HorizontalAlignment = 'right';
            app.HLabel.Position = [1 31 25 22];
            app.HLabel.Text = 'θH';

            % Create th_HipSlider
            app.th_HipSlider = uislider(app.InitializationPanel);
            app.th_HipSlider.Limits = [-90 150];
            app.th_HipSlider.MajorTicks = [];
            app.th_HipSlider.ValueChangingFcn = createCallbackFcn(app, @Initialize_Data_SliderValueChanging, true);
            app.th_HipSlider.Position = [47 40 150 3];

            % Create HLabel_2
            app.HLabel_2 = uilabel(app.InitializationPanel);
            app.HLabel_2.HorizontalAlignment = 'right';
            app.HLabel_2.Position = [1 7 25 22];
            app.HLabel_2.Text = 'ωH';

            % Create dth_HipSlider
            app.dth_HipSlider = uislider(app.InitializationPanel);
            app.dth_HipSlider.Limits = [-3 3];
            app.dth_HipSlider.MajorTicks = [];
            app.dth_HipSlider.ValueChangingFcn = createCallbackFcn(app, @Initialize_Data_SliderValueChanging, true);
            app.dth_HipSlider.Position = [47 16 150 3];

            % Create StatusLabel
            app.StatusLabel = uilabel(app.UIFigure);
            app.StatusLabel.HorizontalAlignment = 'center';
            app.StatusLabel.FontSize = 30;
            app.StatusLabel.Position = [245 94 91 36];
            app.StatusLabel.Text = 'Status';

            % Create AngularMomentumGaugeLabel
            app.AngularMomentumGaugeLabel = uilabel(app.UIFigure);
            app.AngularMomentumGaugeLabel.HorizontalAlignment = 'center';
            app.AngularMomentumGaugeLabel.FontSize = 20;
            app.AngularMomentumGaugeLabel.Position = [201 165 180 24];
            app.AngularMomentumGaugeLabel.Text = 'Angular Momentum';

            % Create AngularMomentumGauge
            app.AngularMomentumGauge = uigauge(app.UIFigure, 'semicircular');
            app.AngularMomentumGauge.Limits = [-10 10];
            app.AngularMomentumGauge.MajorTicks = [-10 -5 0 5 10];
            app.AngularMomentumGauge.Position = [230 204 120 65];

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            app.UIAxes.DataAspectRatio = [1 1 1];
            app.UIAxes.XLim = [0 2];
            app.UIAxes.XColor = 'none';
            app.UIAxes.YColor = 'none';
            app.UIAxes.FontSize = 12;
            app.UIAxes.Position = [117 298 800 400];

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