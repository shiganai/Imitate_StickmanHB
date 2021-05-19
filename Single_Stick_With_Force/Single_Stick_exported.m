classdef Single_Stick_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure          matlab.ui.Figure
        PlayingButton     matlab.ui.control.StateButton
        ResetButton       matlab.ui.control.StateButton
        Slider_Toruqe_01  matlab.ui.control.Slider
        Slider_Toruqe_02  matlab.ui.control.Slider
        Slider_Toruqe_03  matlab.ui.control.Slider
        Slider_Toruqe_04  matlab.ui.control.Slider
        Slider_Toruqe_05  matlab.ui.control.Slider
        LoopCheckBox      matlab.ui.control.CheckBox
        FrameButton       matlab.ui.control.Button
        Axes_Anime        matlab.ui.control.UIAxes
        Axes_Torque       matlab.ui.control.UIAxes
    end

    
    properties (Access = private)
        Lc double
        M double
        g double
        time_step double
        stick matlab.graphics.chart.primitive.Line
        theta_0 double
        dtheta_0 double
        torque_Makima struct
        time double
        time_Limit double
        slider_Positions double
        time_Line matlab.graphics.primitive.Line
        makima_Line matlab.graphics.chart.primitive.Line
        quiver_Force matlab.graphics.chart.primitive.Quiver
        quiver_Ratio double
        quiver_Torque matlab.graphics.chart.primitive.Quiver
        quiver_G matlab.graphics.chart.primitive.Quiver
        Mtheta double
    end
    
    methods (Access = private)
        
        function refresh_Makima(app, target_Y)
            target_X = linspace(0, app.time_Limit, 5);
            
            app.torque_Makima = makima(target_X, target_Y);
            
            plot_X = linspace(0, app.time_Limit, 20);
            plot_Y = fnval(app.torque_Makima, plot_X);
            if isempty(app.makima_Line)
                app.makima_Line = plot(app.Axes_Torque, plot_X, plot_Y);
            else
                app.makima_Line.XData = plot_X;
                app.makima_Line.YData = plot_Y;
            end
            ylim(app.Axes_Torque, [-5, 5])
            
            app.Mtheta = fnval(app.torque_Makima, app.time);
            Fx_out = find_Fx_out(app.Lc,app.M,app.Mtheta,app.dtheta_0,app.g,app.theta_0);
            Fy_out = find_Fy_out(app.Lc,app.M,app.Mtheta,app.dtheta_0,app.g,app.theta_0);
            Fx_out_G = find_Fx_out(app.Lc,app.M,0,app.dtheta_0,app.g,app.theta_0);
            Fy_out_G = find_Fy_out(app.Lc,app.M,0,app.dtheta_0,app.g,app.theta_0);
            Fx_out_Torque = Fx_out - Fx_out_G;
            Fy_out_Torque = Fy_out - Fy_out_G;
            
            
            if isempty(app.quiver_Force)
                hold(app.Axes_Anime, 'on');
                app.quiver_Force = quiver(app.Axes_Anime, 0, 0, Fx_out / app.quiver_Ratio, Fy_out / app.quiver_Ratio, 'LineWidth', 2);
                app.quiver_G = quiver(app.Axes_Anime, 0, 0, Fx_out_G / app.quiver_Ratio, Fy_out_G / app.quiver_Ratio, 'LineWidth', 2);
                app.quiver_Torque = quiver(app.Axes_Anime, 0, 0, Fx_out_Torque / app.quiver_Ratio, Fy_out_Torque / app.quiver_Ratio, 'LineWidth', 2);
                hold(app.Axes_Anime, 'off');
            else
                app.quiver_Force.UData = Fx_out(end) / app.quiver_Ratio;
                app.quiver_Force.VData = Fy_out(end) / app.quiver_Ratio;
                app.quiver_G.UData = Fx_out_G(end) / app.quiver_Ratio;
                app.quiver_G.VData = Fy_out_G(end) / app.quiver_Ratio;
                app.quiver_Torque.UData = Fx_out_Torque(end) / app.quiver_Ratio;
                app.quiver_Torque.VData = Fy_out_Torque(end) / app.quiver_Ratio;
            end
            
            drawnow
        end
        
        function refresh_Anime_Axes(app)
            x = app.Lc * cos(app.theta_0 + 3/2 * pi) * 2;
            y = app.Lc * sin(app.theta_0 + 3/2 * pi) * 2;
            
            app.stick.XData(2) = x(end);
            app.stick.YData(2) = y(end);
            
            app.time_Line.XData(1) = app.time;
            app.time_Line.XData(2) = app.time;
            
            
            Fx_out = find_Fx_out(app.Lc,app.M,app.Mtheta,app.dtheta_0,app.g,app.theta_0);
            Fy_out = find_Fy_out(app.Lc,app.M,app.Mtheta,app.dtheta_0,app.g,app.theta_0);
            Fx_out_G = find_Fx_out(app.Lc,app.M,0,app.dtheta_0,app.g,app.theta_0);
            Fy_out_G = find_Fy_out(app.Lc,app.M,0,app.dtheta_0,app.g,app.theta_0);
            Fx_out_Torque = Fx_out - Fx_out_G;
            Fy_out_Torque = Fy_out - Fy_out_G;
            
            app.quiver_Force.UData = Fx_out(end) / app.quiver_Ratio;
            app.quiver_Force.VData = Fy_out(end) / app.quiver_Ratio;
            app.quiver_G.UData = Fx_out_G(end) / app.quiver_Ratio;
            app.quiver_G.VData = Fy_out_G(end) / app.quiver_Ratio;
            app.quiver_Torque.UData = Fx_out_Torque(end) / app.quiver_Ratio;
            app.quiver_Torque.VData = Fy_out_Torque(end) / app.quiver_Ratio;
            
        end
        
        function run_Ode(app)
            t = [0, app.time_step];
            q0 = [app.theta_0, app.dtheta_0]';
            
            app.Mtheta = fnval(app.torque_Makima, app.time);
            
            [time_Ode, q] = ode45(@(t,q) ddt(t, q,app.Lc, app.M, app.g, app.Mtheta), t, q0);
            
            app.time = app.time + time_Ode(end);
            
            app.theta_0 = q(end,1);
            app.dtheta_0 = q(end,2);
        end
        
        function initialize_Data(app)
            app.dtheta_0 = 0;
            app.theta_0 = 0/2 * pi;
            app.time = 0;
            
            app.Mtheta = fnval(app.torque_Makima, app.time);
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.Lc = 1;
            app.M = 1;
            app.g = 2;
            app.time_step = 0.05;
            app.time_Limit = 4;
            app.quiver_Ratio = 3;
            app.torque_Makima = makima([0,1], [0,1]);
            initialize_Data(app)
            
            app.stick = plot(app.Axes_Anime, [0, app.Lc * 2 * cos(app.theta_0 + 3/2 * pi)], [0, app.Lc * 2 * sin(app.theta_0 + 3/2 * pi)], ...
                '-', 'LineWidth', 2);
            xlim(app.Axes_Anime, [-app.Lc * 2, app.Lc * 2])
            ylim(app.Axes_Anime, [-app.Lc * 2, app.Lc * 2])
            
            refresh_Makima(app, zeros(1,5))
            app.time_Line = line(app.Axes_Torque, [app.time; app.time], [-5; 5], 'Color', 'red');
            
            id_Warning = 'MATLAB:ui:Slider:fixedWidth';
            warning('off', id_Warning)
            
            app.Slider_Toruqe_01.Position(1) = app.Axes_Torque.InnerPosition(1) + app.Axes_Torque.InnerPosition(3) / 4 * 0;
            app.Slider_Toruqe_02.Position(1) = app.Axes_Torque.InnerPosition(1) + app.Axes_Torque.InnerPosition(3) / 4 * 1;
            app.Slider_Toruqe_03.Position(1) = app.Axes_Torque.InnerPosition(1) + app.Axes_Torque.InnerPosition(3) / 4 * 2;
            app.Slider_Toruqe_04.Position(1) = app.Axes_Torque.InnerPosition(1) + app.Axes_Torque.InnerPosition(3) / 4 * 3;
            app.Slider_Toruqe_05.Position(1) = app.Axes_Torque.InnerPosition(1) + app.Axes_Torque.InnerPosition(3) / 4 * 4;
            
            warning('on', id_Warning)
            
            app.slider_Positions = [
                app.Slider_Toruqe_01.Position;
                app.Slider_Toruqe_02.Position;
                app.Slider_Toruqe_03.Position;
                app.Slider_Toruqe_04.Position;
                app.Slider_Toruqe_05.Position;
                ];
            
        end

        % Value changed function: PlayingButton
        function PlayingButtonValueChanged(app, event)
            value = app.PlayingButton.Value;
            if value
                if_Shokai = true;
                while true
                    if app.time + app.time_step >= app.time_Limit
                        if app.PlayingButton.Value
                            
                            if app.LoopCheckBox.Value
                                initialize_Data(app)
                                refresh_Anime_Axes(app)
                                drawnow
                            else
                                if if_Shokai
                                    initialize_Data(app)
                                else
                                    app.PlayingButton.Value = false;
                                    break
                                end
                            end
                        else
                            warning('app.time + app.time_step >= app.time_Limit && ~app.PlayingButton.Value && No command for this route')
                        end
                    elseif app.time < app.time_Limit
                        
                        run_Ode(app)
                        refresh_Anime_Axes(app)
                        
                        pause(app.time_step/1.1)
                        drawnow
                    end
                    
                    if ~app.PlayingButton.Value
                        if app.ResetButton.Value
                            
                            app.ResetButton.Value = false;
                            
                            initialize_Data(app)
                            refresh_Anime_Axes(app)
                            drawnow
                        end
                        break
                    end
                    if_Shokai = false;
                end
            end
        end

        % Value changed function: ResetButton
        function ResetButtonValueChanged(app, event)
            if app.ResetButton.Value
                if app.PlayingButton.Value
                    app.PlayingButton.Value = false;
                else
                    app.ResetButton.Value = false;
                    
                    initialize_Data(app)
                    refresh_Anime_Axes(app)
                    drawnow
                end
            end
        end

        % Value changing function: Slider_Toruqe_01, 
        % Slider_Toruqe_02, Slider_Toruqe_03, Slider_Toruqe_04, 
        % Slider_Toruqe_05
        function Slider_Toruqe_ValueChanging(app, event)
            changingValue = event.Value;
            
            target_Index = prod(event.Source.Position == app.slider_Positions, 2);
            
            target_Y = [...
                app.Slider_Toruqe_01.Value, ...
                app.Slider_Toruqe_02.Value, ...
                app.Slider_Toruqe_03.Value, ...
                app.Slider_Toruqe_04.Value, ...
                app.Slider_Toruqe_05.Value, ...
                ];
            
            target_Y(logical(target_Index)) = changingValue;
            
            refresh_Makima(app,target_Y)
        end

        % Button pushed function: FrameButton
        function FrameButtonPushed(app, event)
            
            if app.time + app.time_step >= app.time_Limit
                initialize_Data(app)
                refresh_Anime_Axes(app)
                drawnow
            else
                run_Ode(app)
                refresh_Anime_Axes(app)
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
            app.UIFigure.Position = [100 100 884 480];
            app.UIFigure.Name = 'MATLAB App';

            % Create PlayingButton
            app.PlayingButton = uibutton(app.UIFigure, 'state');
            app.PlayingButton.ValueChangedFcn = createCallbackFcn(app, @PlayingButtonValueChanged, true);
            app.PlayingButton.Text = 'Playing';
            app.PlayingButton.Position = [204 49 100 22];

            % Create ResetButton
            app.ResetButton = uibutton(app.UIFigure, 'state');
            app.ResetButton.ValueChangedFcn = createCallbackFcn(app, @ResetButtonValueChanged, true);
            app.ResetButton.Text = 'Reset';
            app.ResetButton.Position = [91 49 100 22];

            % Create Slider_Toruqe_01
            app.Slider_Toruqe_01 = uislider(app.UIFigure);
            app.Slider_Toruqe_01.Limits = [-3 3];
            app.Slider_Toruqe_01.Orientation = 'vertical';
            app.Slider_Toruqe_01.ValueChangingFcn = createCallbackFcn(app, @Slider_Toruqe_ValueChanging, true);
            app.Slider_Toruqe_01.Position = [433 34 3 109];

            % Create Slider_Toruqe_02
            app.Slider_Toruqe_02 = uislider(app.UIFigure);
            app.Slider_Toruqe_02.Limits = [-3 3];
            app.Slider_Toruqe_02.Orientation = 'vertical';
            app.Slider_Toruqe_02.ValueChangingFcn = createCallbackFcn(app, @Slider_Toruqe_ValueChanging, true);
            app.Slider_Toruqe_02.Position = [505 34 3 109];

            % Create Slider_Toruqe_03
            app.Slider_Toruqe_03 = uislider(app.UIFigure);
            app.Slider_Toruqe_03.Limits = [-3 3];
            app.Slider_Toruqe_03.Orientation = 'vertical';
            app.Slider_Toruqe_03.ValueChangingFcn = createCallbackFcn(app, @Slider_Toruqe_ValueChanging, true);
            app.Slider_Toruqe_03.Position = [573 34 3 109];

            % Create Slider_Toruqe_04
            app.Slider_Toruqe_04 = uislider(app.UIFigure);
            app.Slider_Toruqe_04.Limits = [-3 3];
            app.Slider_Toruqe_04.Orientation = 'vertical';
            app.Slider_Toruqe_04.ValueChangingFcn = createCallbackFcn(app, @Slider_Toruqe_ValueChanging, true);
            app.Slider_Toruqe_04.Position = [647 34 3 109];

            % Create Slider_Toruqe_05
            app.Slider_Toruqe_05 = uislider(app.UIFigure);
            app.Slider_Toruqe_05.Limits = [-3 3];
            app.Slider_Toruqe_05.Orientation = 'vertical';
            app.Slider_Toruqe_05.ValueChangingFcn = createCallbackFcn(app, @Slider_Toruqe_ValueChanging, true);
            app.Slider_Toruqe_05.Position = [718 34 3 109];

            % Create LoopCheckBox
            app.LoopCheckBox = uicheckbox(app.UIFigure);
            app.LoopCheckBox.Text = 'Loop';
            app.LoopCheckBox.Position = [116 13 49 22];

            % Create FrameButton
            app.FrameButton = uibutton(app.UIFigure, 'push');
            app.FrameButton.ButtonPushedFcn = createCallbackFcn(app, @FrameButtonPushed, true);
            app.FrameButton.Position = [204 13 100 22];
            app.FrameButton.Text = '1 Frame';

            % Create Axes_Anime
            app.Axes_Anime = uiaxes(app.UIFigure);
            app.Axes_Anime.DataAspectRatio = [1 1 1];
            app.Axes_Anime.FontSize = 12;
            app.Axes_Anime.Position = [42 91 300 300];

            % Create Axes_Torque
            app.Axes_Torque = uiaxes(app.UIFigure);
            title(app.Axes_Torque, 'Title')
            xlabel(app.Axes_Torque, 'X')
            ylabel(app.Axes_Torque, 'Y')
            zlabel(app.Axes_Torque, 'Z')
            app.Axes_Torque.Position = [394 168 362 223];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = Single_Stick_exported

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