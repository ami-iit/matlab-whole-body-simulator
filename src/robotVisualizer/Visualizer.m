classdef Visualizer < matlab.System
    % Visualizer matlab.System handling the robot visualization.
    % go in app/robots/iCub*/initVisualizer.m to change the setup config

    properties (Nontunable)
        config
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
        KinDynModel, visualizer, tauNorm;
        g = [0, 0, -9.81]; % gravity vector
        pov = [92.9356 22.4635]; % view vector for the visualizer
        sphere;
        X_sphere,Y_sphere,Z_sphere;
        max_tau, min_tau;
    end

    methods (Access = protected)

        function setupImpl(obj)

            if obj.config.visualizeRobot
                % Perform one-time calculations, such as computing constants
                obj.prepareRobot()
                
                obj.prepareTorques();
            end

        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = ["Robot", "Visualizer"];
        end

        function stepImpl(obj, world_H_base, base_velocity, joints_positions, joints_velocity, joints_torque)
            % stepImpl Specifies the algorithm to execute when you run the System object
            if obj.config.visualizeRobot
                % take the kinematic quantities and sets the robot state
                iDynTreeWrappers.setRobotState(obj.KinDynModel, world_H_base, joints_positions, base_velocity, joints_velocity, obj.g);
                % update the visualization accordingly
                iDynTreeWrappers.updateVisualization(obj.KinDynModel, obj.visualizer);
                % the visualizer follows the robot
                obj.followTheRobot();
                
                obj.updateTorques(joints_torque);
            end

        end

        function prepareRobot(obj)
            % prepareRobot Prepares the visualization loading the information
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

            % The size of the visualizer matlab figure
            x0 = 300;
            y0 = 300;
            width = 1300;
            height = 1300;
            set(gcf, 'position', [x0, y0, width, height])
        end

        function followTheRobot(obj)
            % moves the window around the robot
            comPosition = toMatlab(obj.KinDynModel.kinDynComp.getCenterOfMassPosition());
            xlim([comPosition(1) - obj.config.aroundRobot, comPosition(1) + obj.config.aroundRobot]);
            ylim([comPosition(2) - obj.config.aroundRobot, comPosition(2) + obj.config.aroundRobot]);
            zlim([comPosition(3) - obj.config.aroundRobot, comPosition(3) + obj.config.aroundRobot]);
        end
        
        function updateTorques(obj, joints_torque)

            for i=1:length(obj.visualizer.linkNames)
                if(not(isempty(obj.config.LinkToJointMap{i})))
                    % position of the link joint
                    pos = obj.visualizer.transforms(i).Matrix(1:3,4);
                    
                    % compute the torque norm and the colour
                    obj.tauNorm(i) = norm(joints_torque(obj.config.LinkToJointMap{i}));
                    color = [min(obj.tauNorm(i),obj.max_tau)/obj.max_tau, max(obj.max_tau-max(obj.tauNorm(i)-obj.min_tau,obj.min_tau), 0)/obj.max_tau 0];
        
                    set(obj.sphere{i}, 'XData', obj.X_sphere + pos(1), ...
                                       'YData', obj.Y_sphere + pos(2), ...
                                       'ZData', obj.Z_sphere + pos(3), ...
                                       'FaceColor', color);
                end
            end
        end

        function prepareTorques(obj)
            
            % create the sphere
            [obj.X_sphere,obj.Y_sphere,obj.Z_sphere] = sphere(obj.config.SphereConfig.SphereSize);
            obj.X_sphere = obj.X_sphere * obj.config.SphereConfig.radius;
            obj.Y_sphere = obj.Y_sphere * obj.config.SphereConfig.radius;
            obj.Z_sphere = obj.Z_sphere * obj.config.SphereConfig.radius;
            
            % Initialize tau_norm, max and min
            obj.tauNorm = zeros(length(obj.visualizer.linkNames),1);
            obj.max_tau = obj.config.SphereConfig.saturation.max;
            obj.min_tau = obj.config.SphereConfig.saturation.min;
            
            for i=1:length(obj.visualizer.linkNames)
                if(not(isempty(obj.config.LinkToJointMap{i})))
                    % position of the link joint
                    pos = obj.visualizer.transforms(i).Matrix(1:3,4);

                    color = [0, 0, 0];

                    obj.sphere{i} = surf(obj.X_sphere + pos(1), ...
                                         obj.Y_sphere + pos(2), ...
                                         obj.Z_sphere + pos(3), ...
                                         'FaceColor',color, ...
                                         'EdgeColor','none');
                end
            end

        end

    end

end
