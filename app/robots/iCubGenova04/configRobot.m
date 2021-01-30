%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON ROBOT CONFIGURATION PARAMETERS                      %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Robot configuration for WBToolbox
WBTConfigRobotSim = WBToolbox.Configuration;
WBTConfigRobotSim.RobotName = 'icubSim';
WBTConfigRobotSim.UrdfFile = 'model.urdf';
WBTConfigRobotSim.LocalName = 'WBT';
WBTConfigRobotSim.GravityVector = [0,0,Config.GRAVITY_ACC];

% Controlboards and joints list. Each joint is associated to the corresponding controlboard
WBTConfigRobotSim.ControlBoardsNames = {'torso', 'left_arm', 'right_arm', 'left_leg', 'right_leg'};
WBTConfigRobotSim.ControlledJoints = [];
numOfJointsForEachControlboard = [];

ControlBoards = struct();
ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{1}) = {'torso_pitch', 'torso_roll', 'torso_yaw'};
ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{2}) = {'l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow'};
ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{3}) = {'r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', 'r_elbow'};
ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{4}) = {'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw', 'l_knee', 'l_ankle_pitch', 'l_ankle_roll'};
ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{5}) = {'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_ankle_pitch', 'r_ankle_roll'};

for n = 1:length(WBTConfigRobotSim.ControlBoardsNames)
    WBTConfigRobotSim.ControlledJoints = [WBTConfigRobotSim.ControlledJoints, ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{n})];
    numOfJointsForEachControlboard = [numOfJointsForEachControlboard; length(ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{n}))];
end

% structure used to configure the Robot class
% 
robot_config.jointOrder = WBTConfigRobotSim.ControlledJoints;
robot_config.numOfJointsForEachControlboard = numOfJointsForEachControlboard;
robot_config.meshFilePrefix = [icubModelsInstallPrefix '/share'];
robot_config.modelPath = [icubModelsInstallPrefix '/share/iCub/robots/' robotName '/'];
robot_config.fileName = WBTConfigRobotSim.UrdfFile;
robot_config.N_DOF = numel(WBTConfigRobotSim.ControlledJoints);
robot_config.N_DOF_MATRIX = eye(robot_config.N_DOF);

% Initial condition of iCub and for the integrators.
initialConditions.base_position = [0; 0; 0.65];
initialConditions.orientation = diag([-1, -1, 1]);
initialConditions.w_H_b = wbs.State.Rp2H(initialConditions.orientation, initialConditions.base_position);
% joint (inital) position
initialConditions.s = [0.1744; 0.0007; 0.0001; ...
                                    -0.1745; 0.4363; 0.6981; 0.2618; ...
                                    -0.1745; 0.4363; 0.6981; 0.2618; ...
                                    1; 0.0001; -0.0001; -0.0002; -0.0004; 0.0003; ...
                                    0.0002; 0.0001; -0.0002; 0.0004; -0.5; 0.0003];
% velocty initial conditions
initialConditions.base_linear_velocity = [0; 0; 0];
initialConditions.base_angular_velocity = [0; 0; 0];
initialConditions.base_pose_dot = [initialConditions.base_linear_velocity; initialConditions.base_angular_velocity];
initialConditions.s_dot = zeros(robot_config.N_DOF, 1);

robot_config.initialConditions = initialConditions;

% Reflected inertia
robot_config.SIMULATE_MOTOR_REFLECTED_INERTIA = true;

% Robot frames list
Frames.BASE = 'root_link';
Frames.COM = 'com';
Frames.LEFT_FOOT = 'l_sole';
Frames.RIGHT_FOOT = 'r_sole';

robot_config.robotFrames = Frames;

% structure used to configure the Contacts class
% 

% foot print of the feet (iCub)
vertex = zeros(3, 4);
vertex(:, 1) = [-0.06; 0.04; 0];
vertex(:, 2) = [0.11; 0.04; 0];
vertex(:, 3) = [0.11; -0.035; 0];
vertex(:, 4) = [-0.06; -0.035; 0];

contact_config.foot_print = vertex;
contact_config.total_num_vertices = size(vertex,2)*2;

% friction coefficient for the feet
contact_config.friction_coefficient = 0.1;

clear Frames initialConditions vertex
