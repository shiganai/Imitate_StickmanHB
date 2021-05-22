classdef Single_Stick_With_Collision_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                    matlab.ui.Figure
        PlayingButton               matlab.ui.control.StateButton
        Slider                      matlab.ui.control.Slider
        ResetButton                 matlab.ui.control.StateButton
        FrameButton                 matlab.ui.control.Button
        InitialThetaEditFieldLabel  matlab.ui.control.Label
        InitialThetaEditField       matlab.ui.control.NumericEditField
        Status_Label                matlab.ui.control.Label
        UIAxes                      matlab.ui.control.UIAxes
    end

    
    properties (Access = private)
        Lc = 1;
        M = 1;
        g = 9.8;
        I double
        
        myu = 0.65
        
        x double
        dx double
        y double
        dy double
        
        theta double
        dtheta double
        
        f_X double
        f_Y double
        
        tau = 0;
        time_step = 0.01;
        
        %         status char
        
        stick matlab.graphics.chart.primitive.Line
        
        quivers matlab.graphics.chart.primitive.Quiver
        quiver_Ratio double
    end
    
    methods (Access = private)
        function initialize_Data(app)
            app.x = 0;
            app.dx = 0;
            
            app.y = 0.5;
            app.dy = 0;
            
            app.theta = deg2rad(app.InitialThetaEditField.Value);
            app.dtheta = 0;
            
            app.f_X = 0;
            app.f_Y = 0;
            
            app.Status_Label.Text = 'Inair';
        end
        
        function refresh_Stick(app)
            app.stick.XData(1) = app.x;
            app.stick.YData(1) = app.y;
            app.stick.XData(2) = app.x + 2 * app.Lc * cos(app.theta);
            app.stick.YData(2) = app.y + 2 * app.Lc * sin(app.theta);
        end
        
        
        function run_Ode(app)
            if isequal(app.Status_Label.Text, 'Inair')
                run_Ode_Inair(app)
            elseif isequal(app.Status_Label.Text, 'Touch')
                run_Ode_Touch(app)
            elseif isequal(app.Status_Label.Text, 'Slides')
                run_Ode_Slides(app)
            end
            
        end
        
        function run_Ode_Inair(app)
            t = [0, app.time_step];
            q0 = [app.theta, app.dtheta, app.x, app.dx, app.y, app.dy]';
            ode_Option = odeset('Events', @(t,q) event_Inair(app, q));
            
            [~, q, ~, ~, ie] = ode45(@(t,q) ddt_Inair(app, q), t, q0, ode_Option);
            
            app.theta = q(end,1);
            app.dtheta = q(end,2);
            app.x = q(end,3);
            app.dx = q(end,4);
            app.y = q(end,5);
            app.dy = q(end,6);
            
            if ~isempty(ie)
                if ie(end) == 1
                    app.Status_Label.Text = 'Collision';
                    [dtheta_After,I_F_X,I_F_Y] = find_Status_After_Collision(app.Lc,app.M,app.dtheta,app.dx,app.dy,app.theta);
                    
                    app.Status_Label.Text = 'Touch';
                    
                    if abs(I_F_X / I_F_Y) > app.myu
                        if I_F_X / I_F_Y < 0
                            [dtheta_After,dx_After] = find_Status_After_Collision_Slides(app.Lc,app.M,app.dtheta,app.dx,app.dy,-app.myu,app.theta);
                        else
                            [dtheta_After,dx_After] = find_Status_After_Collision_Slides(app.Lc,app.M,app.dtheta,app.dx,app.dy,app.myu,app.theta);
                        end
                        app.dx = dx_After;
                        app.Status_Label.Text = 'Slides';
                    end
                    app.dtheta = dtheta_After;
                end
            end
        end
        
        function dotq = ddt_Inair(app, q)
            
            theta_Tmp = q(1);
            dtheta_Tmp = q(2);
            
            dx_Tmp = q(4);
            
            dy_Tmp = q(6);
            
            [ddtheta_eq,ddx_eq,ddy_eq] = find_dd_Inair(app.Lc,app.M,dtheta_Tmp,app.f_X,app.f_Y,app.g,app.tau,theta_Tmp);
            
            dotq = [dtheta_Tmp, ddtheta_eq, dx_Tmp, ddx_eq, dy_Tmp, ddy_eq]';
            
        end
        
        function [value, isterminal, direction] = event_Inair(app, q)
            y_Tmp = q(5);
            value = y_Tmp - (-0.5);
            direction = 0;
            isterminal = 1;
        end
        
        function run_Ode_Slides(app)
            t = [0, app.time_step];
            q0 = [app.theta, app.dtheta, app.x, app.dx]';
            ode_Option = odeset('Events', @(t,q) event_Slides(app, q));
            
            [~, q, ~, ~, ie] = ode45(@(t,q) ddt_Slides(app, q), t, q0, ode_Option);
            
            app.theta = q(end,1);
            app.dtheta = q(end,2);
            app.x = q(end,3);
            app.dx = q(end,4);
            
            if ~isempty(ie)
                if ie(end) == 1
                    app.Status_Label.Text = 'Touch';
                end
            end
        end
        
        function dotq = ddt_Slides(app, q)
            
            theta_Tmp = q(1);
            dtheta_Tmp = q(2);
            dx_Tmp = q(4);
            
            [ddtheta,ddx,f_X_Tmp] = find_dd_Slides(app.Lc,app.M,dtheta_Tmp,app.g,app.myu,app.tau,theta_Tmp);
            
            if sign(f_X_Tmp) == dx_Tmp
                [ddtheta,ddx] = find_dd_Slides(app.Lc,app.M,dtheta_Tmp,app.g,-app.myu,app.tau,theta_Tmp);
            end
            
            dotq = [dtheta_Tmp, ddtheta, dx_Tmp, ddx]';
            
        end
        
        function [value, isterminal, direction] = event_Slides(app, q)
            dx_Tmp = q(4);
            value = dx_Tmp;
            direction = 0;
            isterminal = 1;
        end
        
        function run_Ode_Touch(app)
            t = [0, app.time_step];
            q0 = [app.theta, app.dtheta]';
            [~, q] = ode45(@(t,q) ddt_Touch(app, q), t, q0);
            
            app.theta = q(end,1);
            app.dtheta = q(end,2);
        end
        
        function dotq = ddt_Touch(app, q)
            
            theta_Tmp = q(1);
            dtheta_Tmp = q(2);
            
            ddtheta = find_dd_Touch(app.Lc,app.M,app.g,app.tau,theta_Tmp);
            
            dotq = [dtheta_Tmp, ddtheta]';
            
        end
        
        function [dtheta_After,I_F_X,I_F_Y] = find_Status_After_Collision(Lc,M,dtheta_Before,dx_Before,dy_Before,theta)
            dtheta_After = (Lc.*dtheta_Before.*4.0+dy_Before.*cos(theta).*3.0-dx_Before.*sin(theta).*3.0)./(Lc.*4.0);
            if nargout > 1
                t2 = theta.*2.0;
                t3 = cos(t2);
                t4 = sin(t2);
                I_F_X = M.*(dx_Before.*5.0+dx_Before.*t3.*3.0+dy_Before.*t4.*3.0).*(-1.0./8.0);
            end
            if nargout > 2
                I_F_Y = M.*(dy_Before.*5.0+dx_Before.*t4.*3.0-dy_Before.*t3.*3.0).*(-1.0./8.0);
            end
        end
        
        function [dtheta_After,dx_After,I_F_Y] = find_Status_After_Collision_Slides(Lc,M,dtheta_Before,dx_Before,dy_Before,myu,theta)
            t2 = cos(theta);
            t3 = sin(theta);
            dtheta_After = dtheta_Before+(dy_Before.*(t2-myu.*t3).*6.0)./(Lc.*(t2.^2.*6.0-myu.*t2.*t3.*6.0+2.0));
            if nargout > 1
                t4 = theta.*2.0;
                t5 = cos(t4);
                t6 = sin(t4);
                t7 = t5.*3.0;
                t8 = myu.*t6.*3.0;
                t9 = -t8;
                t10 = t7+t9+5.0;
                t11 = 1.0./t10;
                dx_After = dx_Before+dy_Before.*t11.*(myu.*-5.0+t6.*3.0+myu.*t7);
            end
            if nargout > 2
                I_F_Y = M.*dy_Before.*t11.*-2.0;
            end
        end
        
        function [ddtheta_eq,ddx_eq,ddy_eq] = find_dd_Inair(Lc,M,dtheta,f_X,f_Y,g,tau,theta)
            t2 = cos(theta);
            t3 = sin(theta);
            t4 = Lc.^2;
            t5 = dtheta.^2;
            t7 = 1.0./Lc;
            t8 = 1.0./M;
            ddtheta_eq = (t8.*(tau+Lc.*f_X.*t3-Lc.*f_Y.*t2).*3.0)./t4;
            if nargout > 1
                t6 = t2.^2;
                ddx_eq = t7.*t8.*(Lc.*f_X.*4.0+t3.*tau.*3.0-Lc.*f_X.*t6.*3.0+M.*t2.*t4.*t5-Lc.*f_Y.*t2.*t3.*3.0);
            end
            if nargout > 2
                ddy_eq = t7.*t8.*(Lc.*f_Y-t2.*tau.*3.0-Lc.*M.*g+Lc.*f_Y.*t6.*3.0+M.*t3.*t4.*t5-Lc.*f_X.*t2.*t3.*3.0);
            end
        end
        
        function ddtheta = find_dd_Touch(Lc,M,g,tau,theta)
            ddtheta = (1.0./Lc.^2.*(tau.*3.0-Lc.*M.*g.*cos(theta).*3.0))./(M.*4.0);
        end
        
        function [ddtheta,ddx,f_X,f_Y] = find_dd_Slides(Lc,M,dtheta,g,myu,tau,theta)
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
            ddtheta = (t12.*t26.*t27.*2.0)./t5;
            if nargout > 1
                ddx = t11.*t12.*t26.*(t3.*t7+Lc.*myu.*t4.*(5.0./2.0)-Lc.*t4.*t10.*(3.0./2.0)+myu.*t2.*t7-Lc.*myu.*t4.*t9.*(3.0./2.0)+M.*t2.*t5.*t6.*4.0-M.*myu.*t3.*t5.*t6.*4.0).*2.0;
            end
            if nargout > 2
                t28 = t2.*t11.*t26.*t27.*2.0;
                t29 = t4+t20+t28;
                f_X = myu.*t29;
            end
            if nargout > 3
                f_Y = t29;
            end
        end
        
        
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            
            app.I = 1/12 * app.M * (2 * app.Lc)^2;
            
            initialize_Data(app)
            
            app.stick = plot(app.UIAxes, [0, 1], [0, 1], '-', 'LineWidth', 2);
            refresh_Stick(app)
            line(app.UIAxes, 2 * [-app.Lc, app.Lc], [-0.5, -0.5], 'Color', 'black')
            xlim(app.UIAxes, 2 * [-app.Lc, app.Lc])
            ylim(app.UIAxes, 2 * [-app.Lc, app.Lc])
        end

        % Value changed function: PlayingButton
        function PlayingButtonValueChanged(app, event)
            value = app.PlayingButton.Value;
            if value
                while true
                    
                    run_Ode(app)
                    refresh_Stick(app)
                    
                    pause(app.time_step/1.2)
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
            app.tau = changingValue;
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
                drawnow
            end
        end

        % Button pushed function: FrameButton
        function FrameButtonPushed(app, event)
            run_Ode(app)
            refresh_Stick(app)
            drawnow
        end

        % Value changed function: InitialThetaEditField
        function InitialThetaEditFieldValueChanged(app, event)
            app.PlayingButton.Value = false;
            initialize_Data(app)
            refresh_Stick(app)
            drawnow
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [400 150 639 480];
            app.UIFigure.Name = 'MATLAB App';

            % Create PlayingButton
            app.PlayingButton = uibutton(app.UIFigure, 'state');
            app.PlayingButton.ValueChangedFcn = createCallbackFcn(app, @PlayingButtonValueChanged, true);
            app.PlayingButton.Text = 'Playing';
            app.PlayingButton.Position = [271 72 100 22];

            % Create Slider
            app.Slider = uislider(app.UIFigure);
            app.Slider.Limits = [-3 3];
            app.Slider.ValueChangingFcn = createCallbackFcn(app, @SliderValueChanging, true);
            app.Slider.Position = [186 51 311 3];

            % Create ResetButton
            app.ResetButton = uibutton(app.UIFigure, 'state');
            app.ResetButton.ValueChangedFcn = createCallbackFcn(app, @ResetButtonValueChanged, true);
            app.ResetButton.Text = 'Reset';
            app.ResetButton.Position = [158 72 100 22];

            % Create FrameButton
            app.FrameButton = uibutton(app.UIFigure, 'push');
            app.FrameButton.ButtonPushedFcn = createCallbackFcn(app, @FrameButtonPushed, true);
            app.FrameButton.Position = [379 72 85 22];
            app.FrameButton.Text = '1 Frame';

            % Create InitialThetaEditFieldLabel
            app.InitialThetaEditFieldLabel = uilabel(app.UIFigure);
            app.InitialThetaEditFieldLabel.HorizontalAlignment = 'right';
            app.InitialThetaEditFieldLabel.Position = [220 414 68 22];
            app.InitialThetaEditFieldLabel.Text = 'Initial Theta';

            % Create InitialThetaEditField
            app.InitialThetaEditField = uieditfield(app.UIFigure, 'numeric');
            app.InitialThetaEditField.ValueChangedFcn = createCallbackFcn(app, @InitialThetaEditFieldValueChanged, true);
            app.InitialThetaEditField.Position = [303 414 100 22];
            app.InitialThetaEditField.Value = 200;

            % Create Status_Label
            app.Status_Label = uilabel(app.UIFigure);
            app.Status_Label.Position = [70 208 35 22];

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            app.UIAxes.DataAspectRatio = [1 1 1];
            app.UIAxes.XColor = 'none';
            app.UIAxes.YColor = 'none';
            app.UIAxes.FontSize = 12;
            app.UIAxes.Position = [171 106 300 300];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = Single_Stick_With_Collision_exported

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