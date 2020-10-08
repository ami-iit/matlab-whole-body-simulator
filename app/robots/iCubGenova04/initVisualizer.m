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


confVisualizer.LinkToJointMap = { [],           ... % root_link
                               [],       ... % torso_1
                               [],       ... % r_hip_1
                               [],       ... % l_hip_1
                               [12,13,14],     ... % l_hip_2
                               [],         ... % l_upper_leg
                               [15],         ... % l_lower_leg
                               [], ... % l_ankle_1
                               [16,17],     ... % l_ankle_2
                               [18,19,20],     ... % r_hip_2
                               [], ... % r_upper_leg
                               [21],     ... % r_lower_leg
                               [],     ... % r_ankle_1
                               [22,23], ... % r_ankle_2
                               [], ... % torso_2
                               [1,2,3], ... % chest
                               [], ... % r_shoulder_1
                               [], ... % l_shoulder_1
                               [], ... % l_shoulder_2
                               [4,5,6],     ... % l_shoulder_3
                               [],     ... % l_elbow_1
                               [7], ... % l_forearm
                               [], ... % r_shoulder_2
                               [8,9,10], ... % r_shoulder_3
                               [], ... % r_elbow_1
                               [11]};... % r_forearm
                           
confVisualizer.SphereConfig.saturation.max=5.0;
confVisualizer.SphereConfig.saturation.min=0.0;
confVisualizer.SphereConfig.radius=0.05;
confVisualizer.SphereConfig.SphereSize = 4;