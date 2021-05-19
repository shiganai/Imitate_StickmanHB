classdef Single_Stick_With_Slider_Torque_Controller_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure       matlab.ui.Figure
        PlayingButton  matlab.ui.control.StateButton
        Slider         matlab.ui.control.Slider
        ResetButton    matlab.ui.control.StateButton
        UIAxes         matlab.ui.control.UIAxes
    end

    
    properties (Access = private)
        Lc double
        M double
        g double
        time_step double
        stick matlab.graphics.chart.primitive.Line
        theta_0 double
        dtheta_0 double
        Mtheta double
        quivers matlab.graphics.chart.primitive.Quiver
        quiver_Ratio double
    end
    
    methods (Access = private)
        function initialize_Data(app)
            app.dtheta_0 = 0;
            app.theta_0 = 0/2 * pi;
        end
        
        function refresh_Stick(app)
            x = app.Lc * cos(app.theta_0 + 3/2 * pi);
            y = app.Lc * sin(app.theta_0 + 3/2 * pi);
            
            app.stick.XData(2) = x;
            app.stick.YData(2) = y;
        end
        
        function refresh_Quivers(app)
            Fx_out = find_Fx_out(app.Lc,app.M,app.Mtheta,app.dtheta_0,app.g,app.theta_0);
            Fy_out = find_Fy_out(app.Lc,app.M,app.Mtheta,app.dtheta_0,app.g,app.theta_0);
            Fx_out_G = find_Fx_out(app.Lc,app.M,0,app.dtheta_0,app.g,app.theta_0);
            Fy_out_G = find_Fy_out(app.Lc,app.M,0,app.dtheta_0,app.g,app.theta_0);
            Fx_out_Torque = Fx_out - Fx_out_G;
            Fy_out_Torque = Fy_out - Fy_out_G;
            
            app.quivers(1).UData = Fx_out / app.quiver_Ratio;
            app.quivers(1).VData = Fy_out / app.quiver_Ratio;
            app.quivers(2).UData = Fx_out_G / app.quiver_Ratio;
            app.quivers(2).VData = Fy_out_G / app.quiver_Ratio;
            app.quivers(3).UData = Fx_out_Torque / app.quiver_Ratio;
            app.quivers(3).VData = Fy_out_Torque / app.quiver_Ratio;
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.Lc = 1;
            app.M = 1;
            app.g = 0;
            app.time_step = 0.05;
            app.dtheta_0 = 0;
            app.theta_0 = 0/2 * pi;
            app.Mtheta = 0;
            app.quiver_Ratio = 2;
            
            app.stick = plot(app.UIAxes, [0, app.Lc * cos(app.theta_0 + 3/2 * pi)], [0, app.Lc * sin(app.theta_0 + 3/2 * pi)], '-', 'LineWidth', 2);
            xlim(app.UIAxes, [-app.Lc, app.Lc])
            ylim(app.UIAxes, [-app.Lc, app.Lc])
            
            Fx_out = find_Fx_out(app.Lc,app.M,app.Mtheta,app.dtheta_0,app.g,app.theta_0);
            Fy_out = find_Fy_out(app.Lc,app.M,app.Mtheta,app.dtheta_0,app.g,app.theta_0);
            Fx_out_G = find_Fx_out(app.Lc,app.M,0,app.dtheta_0,app.g,app.theta_0);
            Fy_out_G = find_Fy_out(app.Lc,app.M,0,app.dtheta_0,app.g,app.theta_0);
            Fx_out_Torque = Fx_out - Fx_out_G;
            Fy_out_Torque = Fy_out - Fy_out_G;
            
            hold(app.UIAxes, "on")
            app.quivers(1,1) = quiver(app.UIAxes, 0, 0, Fx_out / app.quiver_Ratio, Fy_out / app.quiver_Ratio);
            app.quivers(2,1) = quiver(app.UIAxes, 0, 0, Fx_out_G / app.quiver_Ratio, Fy_out_G / app.quiver_Ratio);
            app.quivers(3,1) = quiver(app.UIAxes, 0, 0, Fx_out_Torque / app.quiver_Ratio, Fy_out_Torque / app.quiver_Ratio);
            hold(app.UIAxes, "off")
        end

        % Value changed function: PlayingButton
        function PlayingButtonValueChanged(app, event)
            value = app.PlayingButton.Value;
            if value
                while true
                    
                    t = [0, app.time_step];
                    q0 = [app.theta_0, app.dtheta_0]';
                    
                    [~, q] = ode45(@(t,q) ddt(t, q,app.Lc, app.M, app.g, app.Mtheta), t, q0);
                    
                    app.theta_0 = q(end,1);
                    app.dtheta_0 = q(end,2);
                    
                    refresh_Stick(app)
                    refresh_Quivers(app)
                    
%                     pause(app.time_step/1.2)
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
            app.Mtheta = changingValue;
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
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 639 480];
            app.UIFigure.Name = 'MATLAB App';

            % Create PlayingButton
            app.PlayingButton = uibutton(app.UIFigure, 'state');
            app.PlayingButton.ValueChangedFcn = createCallbackFcn(app, @PlayingButtonValueChanged, true);
            app.PlayingButton.Text = 'Playing';
            app.PlayingButton.Position = [271 72 100 22];

            % Create Slider
            app.Slider = uislider(app.UIFigure);
            app.Slider.Limits = [-1 1];
            app.Slider.ValueChangingFcn = createCallbackFcn(app, @SliderValueChanging, true);
            app.Slider.Position = [186 51 311 3];

            % Create ResetButton
            app.ResetButton = uibutton(app.UIFigure, 'state');
            app.ResetButton.ValueChangedFcn = createCallbackFcn(app, @ResetButtonValueChanged, true);
            app.ResetButton.Text = 'Reset';
            app.ResetButton.Position = [158 72 100 22];

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            title(app.UIAxes, 'Title')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.DataAspectRatio = [1 1 1];
            app.UIAxes.FontSize = 12;
            app.UIAxes.Position = [171 106 300 300];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = Single_Stick_With_Slider_Torque_Controller_exported

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