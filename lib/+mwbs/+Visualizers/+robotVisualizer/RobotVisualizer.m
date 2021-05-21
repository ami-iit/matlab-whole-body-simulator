classdef RobotVisualizer < matlab.System
    % RobotVisualizer matlab.System handling the robot visualization.
    % go in app/robots/iCub*/initRobotVisualizer.m to change the setup config
    
    properties (Nontunable)
        config
        min_time_viz = 1/25;
    end
    
    properties (DiscreteState)
        
    end
    
    % Pre-computed constants
    properties (Access = private)
        KinDynModel, viz;
        g = [0, 0, -9.81]; % gravity vector
    end
    
    methods (Access = protected)
        
        function setupImpl(obj)
            
            if obj.config.visualizeRobot
                % Perform one-time calculations, such as computing constants
                obj.prepareRobot()
                tic;
            end
            
        end
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = ["Robot", "Visualizer"];
        end
        
        function stepImpl(obj, world_H_base, base_velocity, joints_positions, joints_velocity)
            % stepImpl Specifies the algorithm to execute when you run the System object
            if obj.config.visualizeRobot
                % take the kinematic quantities and set the robot state
                iDynTreeWrappers.setRobotState(obj.KinDynModel, world_H_base, joints_positions, base_velocity, joints_velocity, obj.g);
                if obj.viz.run()
                    time_interval = toc;
                    % check the maximum fps
                    if time_interval > obj.min_time_viz
                        obj.updateVisualization(world_H_base, joints_positions);
                        obj.viz.draw();
                        tic
                    end
                else
                    error('Closing visualizer.')
                end
            end
            
        end
        
        function prepareRobot(obj)
            % Prepare the robot model and the iDyntree visualization
            obj.KinDynModel = iDynTreeWrappers.loadReducedModel(obj.config.jointOrder, 'root_link', ...
                obj.config.modelPath, obj.config.fileName, false);
            % instantiate iDynTree visualizer
            obj.viz = iDynTree.Visualizer();
            obj.viz.init();
            % add the 'model' robot the the visualizer
            obj.viz.addModel(obj.KinDynModel.kinDynComp.model(), 'model');
            env = obj.viz.enviroment();
            env.setElementVisibility('floor_grid', true);
            env.setElementVisibility('world_frame', true);
            obj.viz.camera().animator().enableMouseControl(true);
            % adding lights
            obj.viz.enviroment().addLight('sun1');
            obj.viz.enviroment().lightViz('sun1').setType(iDynTree.DIRECTIONAL_LIGHT);
            obj.viz.enviroment().lightViz('sun1').setDirection(iDynTree.Direction(-1, 0, 0));
            obj.viz.enviroment().addLight('sun2');
            obj.viz.enviroment().lightViz('sun2').setType(iDynTree.DIRECTIONAL_LIGHT);
            obj.viz.enviroment().lightViz('sun2').setDirection(iDynTree.Direction(1, 0, 0));
        end
        
        function updateVisualization(obj, world_H_base, joints_positions)
            % Update the visualization using the incoming kinematic
            % quantities
            s = iDynTree.VectorDynSize(obj.KinDynModel.NDOF);
            for k = 0:length(joints_positions)-1
                s.setVal(k,joints_positions(k+1));
            end
            baseRotation_iDyntree = iDynTree.Rotation();
            baseOrigin_iDyntree   = iDynTree.Position();
            T = iDynTree.Transform();
            for k = 0:2
                baseOrigin_iDyntree.setVal(k,world_H_base(k+1,4));
                for j = 0:2
                    baseRotation_iDyntree.setVal(k,j,world_H_base(k+1,j+1));
                end
            end
            T.setRotation(baseRotation_iDyntree);
            T.setPosition(baseOrigin_iDyntree);
            obj.viz.modelViz('model').setPositions(T, s);
        end
    end
end
