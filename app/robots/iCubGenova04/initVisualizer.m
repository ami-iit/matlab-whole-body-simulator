%% configuration for the matlab iDyntree visualizer

confVisualizer.robotName = robotName;
confVisualizer.fileName = fileName;
confVisualizer.meshFilePrefix = meshFilePrefix;
confVisualizer.modelPath = modelPath;
confVisualizer.jointOrder = jointOrder;
% initial infos specified in configRobot
confVisualizer.joints_positions = Config.initialConditions.joints;
confVisualizer.world_H_base = ones(4);
confVisualizer.world_H_base(1:3, 1:3) = Config.initialConditions.orientation;
confVisualizer.world_H_base(1:3, 4) = Config.initialConditions.base_position;

% size of the square you see around the robot
confVisualizer.aroundRobot = 1; % mt

% refresh rate of the picure
confVisualizer.tStep = Config.tStep; % here equal to the time step used in the simulink model
