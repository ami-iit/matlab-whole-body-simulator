%% configuration for the matlab iDyntree visualizer
% Different from Config since matlab.System can handle structures only if
% nontunable (Config in all the functions of the controller is used as
% tunable parameter).

confVisualizer.robotName = robotName;

confVisualizer.fileName = fileName;

confVisualizer.meshFilePrefix = meshFilePrefix;
confVisualizer.modelPath = modelPath;

confVisualizer.jointOrder = jointOrder;

% create vector of positions
confVisualizer.joints_positions = Config.initialConditions.joints;

% % add a world to base
confVisualizer.world_H_base = ones(4);
confVisualizer.world_H_base(1:3,1:3) = Config.initialConditions.orientation;
confVisualizer.world_H_base(1:3,4) = Config.initialConditions.base_position;

% size of the square you see around the robot
confVisualizer.aroundRobot = 1; % mt

% refresh rate of the picure
confVisualizer.tStep = 0.001;
