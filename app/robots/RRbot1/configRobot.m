%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON ROBOT CONFIGURATION PARAMETERS                      %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Robot configuration for WBToolbox
WBTConfigRobotSim = WBToolbox.Configuration;
WBTConfigRobotSim.RobotName = 'RRbotSim';
WBTConfigRobotSim.UrdfFile = 'model.urdf';
WBTConfigRobotSim.LocalName = 'WBT';
WBTConfigRobotSim.GravityVector = Config.GRAVITY_ACC;

% Controlboards and joints list. Each joint is associated to the corresponding controlboard
WBTConfigRobotSim.ControlBoardsNames = {'rrbot_joint'};
WBTConfigRobotSim.ControlledJoints = [];
numOfJointsForEachControlboard = [];

ControlBoards = struct();
ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{1}) = {'RRbot_joint1','RRbot_joint2'};

for n = 1:length(WBTConfigRobotSim.ControlBoardsNames)
    WBTConfigRobotSim.ControlledJoints = [WBTConfigRobotSim.ControlledJoints, ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{n})];
    numOfJointsForEachControlboard = [numOfJointsForEachControlboard; length(ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{n}))];
end

% structure used to configure the Robot class
% 
robot_config.jointOrder = WBTConfigRobotSim.ControlledJoints;
robot_config.numOfJointsForEachControlboard = numOfJointsForEachControlboard;
% Note: Since iDynTree 3.0.0, if meshFilePrefix='', the standard iDynTree workflow of locating the
% mesh via the ExternalMesh.getFileLocationOnLocalFileSystem method is used. The iCub model meshes
% file tree is compatible with this workflow.
robot_config.meshFilePrefix = [fileparts(wbs.getModelPathFromFileNameAndYarpFinder(WBTConfigRobotSim.UrdfFile)),'/../../..'];
robot_config.fileName = WBTConfigRobotSim.UrdfFile;
robot_config.N_DOF = numel(WBTConfigRobotSim.ControlledJoints);
robot_config.N_DOF_MATRIX = eye(robot_config.N_DOF);

% Initial condition of iCub and for the integrators.
initialConditions.base_position = [0; 0; 0];
initialConditions.orientation = eye(3);
initialConditions.w_H_b = wbs.State.Rp2H(initialConditions.orientation, initialConditions.base_position);
% joint inital position (radians)
initialConditions.s = [0; 1]*pi/180;
% velocty initial conditions
initialConditions.base_linear_velocity = [0; 0; 0];
initialConditions.base_angular_velocity = [0; 0; 0];
initialConditions.base_pose_dot = [initialConditions.base_linear_velocity; initialConditions.base_angular_velocity];
initialConditions.s_dot = zeros(robot_config.N_DOF, 1);

robot_config.initialConditions = initialConditions;

% Reflected inertia
robot_config.SIMULATE_MOTOR_REFLECTED_INERTIA = true;

% Robot frames list
Frames.BASE = 'world';
Frames.COM = 'com';
Frames.LEFT_FOOT = Frames.BASE;
Frames.RIGHT_FOOT = Frames.BASE;

robot_config.robotFrames = Frames;

% structure used to configure the Contacts class
% 

% foot print of the feet (iCub)
vertex = zeros(3, 4);
vertex(:, 1) = [-2; 2; 0];
vertex(:, 2) = [2; 2; 0];
vertex(:, 3) = [2; -2; 0];
vertex(:, 4) = [-2; -2; 0];

contact_config.foot_print = vertex;
contact_config.total_num_vertices = size(vertex,2)*2;

% friction coefficient for the feet
contact_config.friction_coefficient = 10;

% size of the square you see around the robot
visualizerAroundRobot = [-2 2; -2 2; -0.1 4]; % mt

clear Frames initialConditions vertex
