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
    end
    
    methods (Access = private)
        
%         function dotq = ddt(app, t, q, Lc, M, g)
%             theta = q(1);
%             dtheta = q(2);
%             
%             Mtheta = 0;
%             
%             ddtheta = find_ddtheta(Lc,M,Mtheta,g,theta);
%             
%             dotq = [dtheta, ddtheta]';
%         end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.Lc = 1;
            app.M = 1;
            app.g = 9.8;
            app.time_step = 0.05;
            app.dtheta_0 = 0;
            app.theta_0 = 0/2 * pi;
%             app.theta_0 = 1/2 * pi;
            
            app.stick = plot(app.UIAxes, [0, app.Lc * cos(app.theta_0 + 3/2 * pi)], [0, app.Lc * sin(app.theta_0 + 3/2 * pi)], '-o');
            xlim(app.UIAxes, [-app.Lc, app.Lc])
            ylim(app.UIAxes, [-app.Lc, app.Lc])
        end

        % Value changed function: PlayingButton
        function PlayingButtonValueChanged(app, event)
            value = app.PlayingButton.Value;
            if value
                while true
                    
                    t = [0, app.time_step];
                    q0 = [app.theta_0, app.dtheta_0]';
                    
                    %                     Mtheta = app.EditField.Value;
                    Mtheta = app.Slider.Value;
                    
                    [~, q] = ode45(@(t,q) ddt(t, q,app.Lc, app.M, app.g, Mtheta), t, q0);
                    
                    theta = q(:,1);
                    dtheta = q(:,2);
                        
                    x = app.Lc * cos(theta + 3/2 * pi);
                    y = app.Lc * sin(theta + 3/2 * pi);
                    
                    app.stick.XData(2) = x(end);
                    app.stick.YData(2) = y(end);
                    
                    drawnow
                    
                    app.theta_0 = theta(end);
                    app.dtheta_0 = dtheta(end);
                    
                    if ~app.PlayingButton.Value
                        break
                    end
                    
                    if app.ResetButton.Value
                        dtheta = 0;
                        theta = 0/2 * pi;
                        
                        x = app.Lc * cos(theta + 3/2 * pi);
                        y = app.Lc * sin(theta + 3/2 * pi);
                        
                        app.stick.XData(2) = x(end);
                        app.stick.YData(2) = y(end);
                        
                        drawnow
                        
                        app.theta_0 = theta(end);
                        app.dtheta_0 = dtheta(end);
                        
                        app.PlayingButton.Value = false;
                        app.ResetButton.Value = false;
                        break
                    end
                end
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
            app.Slider.Limits = [-10 10];
            app.Slider.Position = [35 50 573 3];

            % Create ResetButton
            app.ResetButton = uibutton(app.UIFigure, 'state');
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