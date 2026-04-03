classdef SCARARobotApp1 < matlab.apps.AppBase

    properties (Access = public)
        UIFigure matlab.ui.Figure
        Axes3D matlab.ui.control.UIAxes

        % Controls
        Theta1Slider matlab.ui.control.Slider
        Theta2Slider matlab.ui.control.Slider
        ZSlider matlab.ui.control.Slider
        XEditField matlab.ui.control.NumericEditField
        YEditField matlab.ui.control.NumericEditField
        ZEditField matlab.ui.control.NumericEditField
        RunButton matlab.ui.control.Button
        WriteAButton matlab.ui.control.Button

        % Text Area
        ServoAngleTextArea matlab.ui.control.TextArea
    end

    properties (Access = private)
        L1 = 0.14;
        L2 = 0.15;
        Zmax = 0.10;
        currentTheta1 = 0;
        currentTheta2 = 0;
        currentZ = 0;
    end

    methods (Access = private)

        function update3DModel(app)
            cla(app.Axes3D);
            hold(app.Axes3D, 'on');
            view(app.Axes3D, 3);
            axis(app.Axes3D, [-0.3, 0.3, -0.3, 0.3, 0, 0.15]);
            grid(app.Axes3D, 'on');

            % Forward Kinematics
            x1 = app.L1 * cos(app.currentTheta1);
            y1 = app.L1 * sin(app.currentTheta1);
            z1 = 0;

            x2 = x1 + app.L2 * cos(app.currentTheta1 + app.currentTheta2);
            y2 = y1 + app.L2 * sin(app.currentTheta1 + app.currentTheta2);
            z2 = 0;
            z3 = app.currentZ;

            % Plot
            plot3(app.Axes3D, [0 x1], [0 y1], [0 z1], 'r', 'LineWidth', 3);
            plot3(app.Axes3D, [x1 x2], [y1 y2], [z1 z2], 'b', 'LineWidth', 3);
            plot3(app.Axes3D, [x2 x2], [y2 y2], [z2 z3], 'k', 'LineWidth', 3);

            updateServoAngles(app);
        end

        function solveIK(app)
            x = app.XEditField.Value;
            y = app.YEditField.Value;
            z = app.ZEditField.Value;

            r = sqrt(x^2 + y^2);
            cosTheta2 = (r^2 - app.L1^2 - app.L2^2) / (2 * app.L1 * app.L2);
            theta2 = acos(cosTheta2);
            theta1 = atan2(y, x) - atan2(app.L2 * sin(theta2), app.L1 + app.L2 * cos(theta2));

            % Clamp to [-270, 270] degrees in radians
            maxAngle = deg2rad(270);
            minAngle = -maxAngle;
            theta1 = max(min(theta1, maxAngle), minAngle);
            theta2 = max(min(theta2, maxAngle), minAngle);

            app.currentTheta1 = theta1;
            app.currentTheta2 = theta2;
            app.currentZ = min(max(z, 0), app.Zmax);
        end

        function updateServoAngles(app)
            app.ServoAngleTextArea.Value = {
                sprintf('Theta 1: %.2f deg', rad2deg(app.currentTheta1)),
                sprintf('Theta 2: %.2f deg', rad2deg(app.currentTheta2)),
                sprintf('Z Height: %.2f cm', app.currentZ * 100)
            };
        end

        function runSimulation(app)
            solveIK(app);
            update3DModel(app);
        end

        function writeLetterA(app)
            z = 0.02; % Lift
            pts = [
                -0.05, 0, z;
                 0,    0.1, z;
                 0.05, 0, z;
                -0.025, 0.05, z;
                 0.025, 0.05, z
            ];

            for i = 1:size(pts,1)
                app.XEditField.Value = pts(i,1);
                app.YEditField.Value = pts(i,2);
                app.ZEditField.Value = pts(i,3);
                runSimulation(app);
                pause(0.3);
            end
        end
    end

    methods (Access = public)
        function app = SCARARobotApp1
            app.UIFigure = uifigure('Name', 'SCARA 3D Robot Simulator');
            app.UIFigure.Position = [100 100 900 600];

            % Axes
            app.Axes3D = uiaxes(app.UIFigure, 'Position', [400, 150, 470, 400]);

            % Sliders with ±270° in radians
            angleLimit = deg2rad(270);
            app.Theta1Slider = uislider(app.UIFigure, 'Position', [50, 500, 300, 3], 'Limits', [-angleLimit angleLimit], 'ValueChangedFcn', @(s,e) updateSliders(app));
            app.Theta2Slider = uislider(app.UIFigure, 'Position', [50, 470, 300, 3], 'Limits', [-angleLimit angleLimit], 'ValueChangedFcn', @(s,e) updateSliders(app));
            app.ZSlider = uislider(app.UIFigure, 'Position', [50, 440, 300, 3], 'Limits', [0 app.Zmax], 'ValueChangedFcn', @(s,e) updateSliders(app));

            % IK Inputs
            app.XEditField = uieditfield(app.UIFigure, 'numeric', 'Position', [50, 350, 100, 22], 'Value', 0);
            app.YEditField = uieditfield(app.UIFigure, 'numeric', 'Position', [50, 310, 100, 22], 'Value', 0);
            app.ZEditField = uieditfield(app.UIFigure, 'numeric', 'Position', [50, 270, 100, 22], 'Value', 0);

            % Run Button
            app.RunButton = uibutton(app.UIFigure, 'push', 'Text', 'Run', 'Position', [50, 220, 100, 30]);
            app.RunButton.ButtonPushedFcn = @(btn,event) runSimulation(app);

            % Write A Button
            app.WriteAButton = uibutton(app.UIFigure, 'push', 'Text', 'Write A', 'Position', [160, 220, 100, 30]);
            app.WriteAButton.ButtonPushedFcn = @(btn,event) writeLetterA(app);

            % Text Area
            app.ServoAngleTextArea = uitextarea(app.UIFigure, 'Position', [50, 120, 300, 80]);

            update3DModel(app);
        end

        function updateSliders(app)
            app.currentTheta1 = app.Theta1Slider.Value;
            app.currentTheta2 = app.Theta2Slider.Value;
            app.currentZ = app.ZSlider.Value;
            update3DModel(app);
        end
    end
end