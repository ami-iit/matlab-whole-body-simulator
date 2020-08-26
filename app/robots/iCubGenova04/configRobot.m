%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON ROBOT CONFIGURATION PARAMETERS                      %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


jointOrder={'torso_pitch','torso_roll','torso_yaw',...
    'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
    'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
    'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
    'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};

KinDynModel = iDynTreeWrappers.loadReducedModel(jointOrder,'root_link', modelPath, fileName, false);

% General robot model information
Config.N_DOF         = KinDynModel.NDOF;
Config.N_DOF         = 23;
Config.N_DOF_MATRIX  = eye(Config.N_DOF);
Config.ON_GAZEBO     = true;
Config.GRAVITY_ACC   = 9.81;


% Robot configuration for WBToolbox
WBTConfigRobot           = WBToolbox.Configuration;
WBTConfigRobot.RobotName = 'icubSim';
WBTConfigRobot.UrdfFile  = 'model.urdf';
WBTConfigRobot.LocalName = 'WBT';

% Controlboards and joints list. Each joint is associated to the corresponding controlboard 
WBTConfigRobot.ControlBoardsNames     = {'torso','left_arm','right_arm','left_leg','right_leg'};
WBTConfigRobot.ControlledJoints       = [];
Config.numOfJointsForEachControlboard = [];

ControlBoards                                        = struct();
ControlBoards.(WBTConfigRobot.ControlBoardsNames{1}) = {'torso_pitch','torso_roll','torso_yaw'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{2}) = {'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{3}) = {'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{4}) = {'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{5}) = {'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};

for n = 1:length(WBTConfigRobot.ControlBoardsNames)

    WBTConfigRobot.ControlledJoints       = [WBTConfigRobot.ControlledJoints, ControlBoards.(WBTConfigRobot.ControlBoardsNames{n})];
    Config.numOfJointsForEachControlboard = [Config.numOfJointsForEachControlboard; length(ControlBoards.(WBTConfigRobot.ControlBoardsNames{n}))];
end


% Initial condition of iCub and for the integrators.
Config.initialConditions.base_position = [0;0;0.65];
Config.initialConditions.orientation = diag([-1,-1,1]);
Config.initialConditions.world_H_base = Rp2Hom(Config.initialConditions.orientation, Config.initialConditions.base_position);
% generate decent position
Config.initialConditions.joints = [0.1744; 0.0007; 0.0001; -0.1745; ...
    0.4363; 0.6981; 0.2618; -0.1745; ...
    0.4363; 0.6981; 0.2618; 0.0003; ...
    0.0000; -0.0001; 0.0004; -0.0004; ...
    0.0003; 0.0002; 0.0001; -0.0002; ...
    0.0004; -0.0005; 0.0003];

Config.initialConditions.base_linear_velocity = [0;0;0];
Config.initialConditions.base_angular_velocity = [0;0;0];
Config.initialConditions.base_velocity = [Config.initialConditions.base_linear_velocity; Config.initialConditions.base_angular_velocity];
Config.initialConditions.joints_velocity = zeros(Config.N_DOF,1);


% foot print of the feet
Config.vertex = zeros(3,4);
Config.vertex(:,1) = [-0.06;   0.04; 0];
Config.vertex(:,2) = [ 0.11;   0.04; 0];
Config.vertex(:,3) = [ 0.11; -0.035; 0];
Config.vertex(:,4) = [-0.06; -0.035; 0];

% friction coefficient for the feet
Config.friction_coefficient = 0.1;

% Robot frames list
Frames.BASE_LINK        = 'root_link';
Frames.COM_FRAME        = 'com';
Frames.LFOOT_FRAME      = 'l_sole';
Frames.RFOOT_FRAME      = 'r_sole';