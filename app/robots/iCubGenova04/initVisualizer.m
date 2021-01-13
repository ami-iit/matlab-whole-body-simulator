%% configuration for the matlab iDyntree visualizer

confVisualizer.robotName = robotName;
confVisualizer.fileName = robot_config.fileName;
confVisualizer.meshFilePrefix = robot_config.meshFilePrefix;
confVisualizer.modelPath = robot_config.modelPath;
confVisualizer.jointOrder = robot_config.jointOrder;
% initial infos specified in configRobot
confVisualizer.joints_positions = robot_config.initialConditions.s;
confVisualizer.world_H_base = robot_config.initialConditions.w_H_b;

% size of the square you see around the robot
confVisualizer.aroundRobot = 1; % mt

% refresh rate of the picure
confVisualizer.tStep = Config.tStep; % here equal to the time step used in the simulink model
