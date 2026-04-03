classdef SCARARobotApp < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure matlab.ui.Figure
        TabGroup matlab.ui.container.TabGroup
        DirectControlTab matlab.ui.container.Tab
        InverseKinematicsTab matlab.ui.container.Tab
        TrajectoryTab matlab.ui.container.Tab
        ServoUpdateTab matlab.ui.container.Tab
        Axes3D matlab.ui.control.UIAxes
        % Direct Control Components
        Theta1Slider matlab.ui.control.Slider
        Theta2Slider matlab.ui.control.Slider
        ZSlider matlab.ui.control.Slider
        % Inverse Kinematics Components
        XEditField matlab.ui.control.NumericEditField
        YEditField matlab.ui.control.NumericEditField
        ZEditField matlab.ui.control.NumericEditField
        SolveIKButton matlab.ui.control.Button
        % Trajectory Planning Components
        TrajAxes matlab.ui.control.UIAxes
        % Servo Status Components
        ServoAngleTextArea matlab.ui.control.TextArea
    end

    properties (Access = private)
        % SCARA Robot Properties
        L1 = 0.14; % First arm length (m)
        L2 = 0.15; % Second arm length (m)
        Zmax = 0.10; % Max vertical rack travel (m)
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
            
            % Base position
            base = [0, 0, 0];

            % Joint 1
            x1 = app.L1 * cos(app.currentTheta1);
            y1 = app.L1 * sin(app.currentTheta1);
            z1 = 0;

            % Joint 2
            x2 = x1 + app.L2 * cos(app.currentTheta1 + app.currentTheta2);
            y2 = y1 + app.L2 * sin(app.currentTheta1 + app.currentTheta2);
            z2 = 0;

            % Rack vertical position
            z3 = app.currentZ;

            % Plot arms
            plot3(app.Axes3D, [0 x1], [0 y1], [0 z1], 'r', 'LineWidth', 3);
            plot3(app.Axes3D, [x1 x2], [y1 y2], [z1 z2], 'b', 'LineWidth', 3);
            plot3(app.Axes3D, [x2 x2], [y2 y2], [z2 z3], 'k', 'LineWidth', 3);
        end

        function solveIK(app)
            x = app.XEditField.Value;
            y = app.YEditField.Value;
            z = app.ZEditField.Value;

            % Inverse Kinematics for planar SCARA
            r = sqrt(x^2 + y^2);
            cosTheta2 = (r^2 - app.L1^2 - app.L2^2) / (2 * app.L1 * app.L2);
            theta2 = acos(cosTheta2);
            theta1 = atan2(y, x) - atan2(app.L2 * sin(theta2), app.L1 + app.L2 * cos(theta2));

            % Update
            app.currentTheta1 = theta1;
            app.currentTheta2 = theta2;
            app.currentZ = min(max(z, 0), app.Zmax);

            update3DModel(app);
            updateServoAngles(app);
        end

        function updateServoAngles(app)
            app.ServoAngleTextArea.Value = {
                sprintf('Theta 1: %.2f deg', rad2deg(app.currentTheta1)),
                sprintf('Theta 2: %.2f deg', rad2deg(app.currentTheta2)),
                sprintf('Z Height: %.2f cm', app.currentZ * 100)
            };
        end

        function planTrajectory(app)
            cla(app.TrajAxes);
            hold(app.TrajAxes, 'on');
            view(app.TrajAxes, 3);
            axis(app.TrajAxes, [-0.3, 0.3, -0.3, 0.3, 0, 0.15]);
            grid(app.TrajAxes, 'on');

            t = linspace(0, 1, 100);
            x = 0.1 * cos(2*pi*t);
            y = 0.1 * sin(2*pi*t);
            z = 0.05 + 0.01 * sin(4*pi*t);

            plot3(app.TrajAxes, x, y, z, 'm');
        end
    end

    methods (Access = public)

        function startupFcn(app)
            update3DModel(app);
            updateServoAngles(app);
            planTrajectory(app);
        end

        function createComponents(app)
            % UIFigure and layout
            app.UIFigure = uifigure('Name', 'SCARA 3D Robot Simulator');
            app.UIFigure.Position = [100 100 900 600];

            app.TabGroup = uitabgroup(app.UIFigure);
            app.TabGroup.Position = [20 20 860 560];

            % Tab 1: Direct Control
            app.DirectControlTab = uitab(app.TabGroup, 'Title', 'Direct Control');
            app.Theta1Slider = uislider(app.DirectControlTab, 'Position', [100, 50, 300, 3], 'Limits', [-pi pi]);
            app.Theta2Slider = uislider(app.DirectControlTab, 'Position', [100, 100, 300, 3], 'Limits', [-pi pi]);
            app.ZSlider = uislider(app.DirectControlTab, 'Position', [100, 150, 300, 3], 'Limits', [0, app.Zmax]);
            app.Axes3D = uiaxes(app.DirectControlTab, 'Position', [450, 50, 370, 400]);

            % Tab 2: Inverse Kinematics
            app.InverseKinematicsTab = uitab(app.TabGroup, 'Title', 'Inverse Kinematics');
            app.XEditField = uieditfield(app.InverseKinematicsTab, 'numeric', 'Position', [100, 350, 100, 22]);
            app.YEditField = uieditfield(app.InverseKinematicsTab, 'numeric', 'Position', [100, 300, 100, 22]);
            app.ZEditField = uieditfield(app.InverseKinematicsTab, 'numeric', 'Position', [100, 250, 100, 22]);
            app.SolveIKButton = uibutton(app.InverseKinematicsTab, 'push', 'Text', 'Solve IK', 'Position', [100, 200, 100, 30]);
            app.SolveIKButton.ButtonPushedFcn = @(btn,event) solveIK(app);

            % Tab 3: Trajectory Planning
            app.TrajectoryTab = uitab(app.TabGroup, 'Title', 'Trajectory Planning');
            app.TrajAxes = uiaxes(app.TrajectoryTab, 'Position', [100, 100, 600, 400]);

            % Tab 4: Servo Update
            app.ServoUpdateTab = uitab(app.TabGroup, 'Title', 'Servo Angles');
            app.ServoAngleTextArea = uitextarea(app.ServoUpdateTab, 'Position', [100, 300, 300, 100]);

            % Connect Sliders to Model Update
            app.Theta1Slider.ValueChangedFcn = @(src,event) (app.currentTheta1 == src.Value); app.Theta1Slider.ValueChangingFcn = @(src,event) update3DModel(app);
            app.Theta2Slider.ValueChangedFcn = @(src,event) (app.currentTheta2 == src.Value); app.Theta2Slider.ValueChangingFcn = @(src,event) update3DModel(app);
            app.ZSlider.ValueChangedFcn = @(src,event) (app.currentZ == src.Value); app.ZSlider.ValueChangingFcn = @(src,event) update3DModel(app);
        end
    end

    methods (Access = public)
        function app = SCARARobotApp
            createComponents(app);
            startupFcn(app);
        end
    end
end