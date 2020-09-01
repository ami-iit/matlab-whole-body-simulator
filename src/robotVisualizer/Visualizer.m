classdef Visualizer < matlab.System
    % matlab.System handling the robot visualization
    % go in app/robots/iCub*/initVisualizer.m to change the setup config

    %@author: Giuseppe L'Erario

    properties (Nontunable)
        config
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
        KinDynModel, visualizer;
        g = [0, 0, -9.81]; % gravity vector
        pov = [92.9356 22.4635]; % view vector for the visualizer
    end

    methods (Access = protected)

        function setupImpl(obj)

            if obj.config.visualizeRobot
                % Perform one-time calculations, such as computing constants
                obj.prepareRobot()
            end

        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = ["Robot", "Visualizer"];
        end

        function stepImpl(obj, world_H_base, base_velocity, joints_positions, joints_velocity)

            if obj.config.visualizeRobot
                iDynTreeWrappers.setRobotState(obj.KinDynModel, world_H_base, joints_positions, base_velocity, joints_velocity, obj.g);
                iDynTreeWrappers.updateVisualization(obj.KinDynModel, obj.visualizer);
                obj.followTheRobot();
            end

        end

        function prepareRobot(obj)
            % Main variable of iDyntreeWrappers used for many things including updating
            % robot position and getting world to frame transforms
            obj.KinDynModel = iDynTreeWrappers.loadReducedModel(obj.config.jointOrder, 'root_link', ...
                obj.config.modelPath, obj.config.fileName, false);

            % Set initial position of the robot
            initial_base_velocity = zeros(6, 1);
            initial_joints_velocity = zeros(length(obj.config.joints_positions));

            iDynTreeWrappers.setRobotState(obj.KinDynModel, obj.config.world_H_base, obj.config.joints_positions, ...
                initial_base_velocity, initial_joints_velocity, obj.g);

            % Prepare figure
            [obj.visualizer, ~] = iDynTreeWrappers.prepareVisualization(obj.KinDynModel, obj.config.meshFilePrefix, ...
                'color', [1, 1, 1], 'material', 'metal', 'transparency', 1, 'debug', true, 'view', obj.pov, ...
                'groundOn', true, 'groundColor', [0.5 0.5 0.5], 'groundTransparency', 0.5);
            
            x0=300;
            y0=300;
            width=1300;
            height=1300;
            set(gcf,'position',[x0,y0,width,height])
        end

        function followTheRobot(obj)
            % moves the window around the robot
            comPosition = toMatlab(obj.KinDynModel.kinDynComp.getCenterOfMassPosition());
            xlim([comPosition(1) - obj.config.aroundRobot, comPosition(1) + obj.config.aroundRobot]);
            ylim([comPosition(2) - obj.config.aroundRobot, comPosition(2) + obj.config.aroundRobot]);
            zlim([comPosition(3) - obj.config.aroundRobot, comPosition(3) + obj.config.aroundRobot]);
        end

    end

end
