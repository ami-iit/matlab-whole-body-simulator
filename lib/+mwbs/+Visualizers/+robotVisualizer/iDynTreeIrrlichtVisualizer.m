classdef iDynTreeIrrlichtVisualizer < matlab.System
    % iDynTreeIrrlichtVisualizer matlab.System handling the robot visualization, based on iDynTree-Irrlicht-Visualizer bindings. NOTE: this Visualizer does not work on macOS.
    % See https://github.com/dic-iit/matlab-whole-body-simulator/pull/50#issuecomment-849515071
    % go in app/robots/iCub*/initRobotVisualizer.m to change the setup config

    properties (Nontunable)
        config;
        minSampleTime = 1/25;
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
        KinDynModel, viz;
        g = [0, 0, -9.81]; % gravity vector
        world_H_base = eye(4);
        jointsPosition;
        t; 
    end

    methods (Access = protected)

        function setupImpl(obj)

            if obj.config.visualizeRobot
                % Perform one-time calculations, such as computing constants
                obj.prepareRobot()
            end

        end
        
        function resetImpl(obj)
            obj.t = timer('TimerFcn',@(x,y)obj.updateVisualization(), 'Period', obj.minSampleTime,  ...
                          'ExecutionMode', 'fixedRate', 'BusyMode','queue');  

            start(obj.t);
        end
        
        function releaseImpl(obj)
            R = get(obj.t, 'Running');
            if isequal(R ,'on')
                stop(obj.t);
            end
            obj.viz.close();
            delete(obj.viz);
        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = ["iDynTree-Irrlicht", "Visualizer"];
        end

        function stepImpl(obj, world_H_base, base_velocity, jointsPosition, jointsVelocity)
            % stepImpl Specifies the algorithm to execute when you run the System object
            if obj.config.visualizeRobot
                % take the kinematic quantities and set the robot state
                iDynTreeWrappers.setRobotState(obj.KinDynModel, world_H_base, jointsPosition, base_velocity, jointsVelocity, obj.g);
                obj.world_H_base = world_H_base;
                obj.jointsPosition = jointsPosition;
            end

        end

        function prepareRobot(obj)
		
		% Definition of Plane Characteristics
            
            rot_matrix = mwbs.Utils.compute_rotation_matrix_of_contact_surface(obj.config.normalContactAxis);
            
            PlaneJointsPosition = zeros(1);
            PlaneBaseVelocity = zeros(6,1);
            PlaneJointsVelocity = zeros(1);
                
            JointOrder_Plane=cell({'Fixing_Joint'});
            Plane_Path=strcat(getenv('ISAAC_PATH'),'/isaac-wp1/WP1-4_GeneratedURDFModels');
            Plane_Name='/Plane.urdf';
            Plane_Base_Link='plane_left_link';
            
            % Loading the plane
            
            Plane = iDynTreeWrappers.loadReducedModel(JointOrder_Plane, Plane_Base_Link, Plane_Path, Plane_Name, false); % 'plane_link', '/home/fabiodinatale/Private/ISAAC/isaac-wp1/WP1-4_GeneratedURDFModels', '/Plane2.urdf', false);
            iDynTreeWrappers.setRobotState(Plane,rot_matrix,PlaneJointsPosition,PlaneBaseVelocity,PlaneJointsVelocity,obj.g);
			
            % Prepare the robot model and the iDyntree visualization
            obj.KinDynModel = iDynTreeWrappers.loadReducedModel(obj.config.jointOrder, obj.config.robotFrames.BASE, ...
                obj.config.modelPath, obj.config.fileName, false);
            % instantiate iDynTree visualizer
            obj.viz = iDynTree.Visualizer();
            obj.viz.init();
            % add the 'model' robot the the visualizer
            obj.viz.addModel(obj.KinDynModel.kinDynComp.model(), 'model');
			% add the plane to the visualizer
            obj.viz.addModel(Plane.kinDynComp.model(), 'plane');
			
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
            
            obj.jointsPosition = zeros(obj.KinDynModel.NDOF,1);
        end

        function updateVisualization(obj)
            % Update the visualization using the incoming kinematic
            % quantities
            if obj.viz.run()
                s = iDynTree.VectorDynSize(obj.KinDynModel.NDOF);
                for k = 0:length(obj.jointsPosition)-1
                    s.setVal(k,obj.jointsPosition(k+1));
                end
                baseRotation_iDyntree = iDynTree.Rotation();
                baseOrigin_iDyntree   = iDynTree.Position();
                T = iDynTree.Transform();
                for k = 0:2
                    baseOrigin_iDyntree.setVal(k,obj.world_H_base(k+1,4));
                    for j = 0:2
                        baseRotation_iDyntree.setVal(k,j,obj.world_H_base(k+1,j+1));
                    end
                end
                T.setRotation(baseRotation_iDyntree);
                T.setPosition(baseOrigin_iDyntree);
                obj.viz.modelViz('model').setPositions(T, s);
                obj.viz.draw();
            else
                stop(obj.t);
                error('Closing visualizer.')
            end
        end
    end
end
