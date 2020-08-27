addpath(genpath('../src'))

icubModelsInstallPrefix = getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX');

meshFilePrefix = [icubModelsInstallPrefix '/share'];
% Select the robot using the folder name
% Example
config.robotName = 'iCubGenova04';
config.modelPath = [icubModelsInstallPrefix '/share/iCub/robots/' config.robotName '/'];
config.fileName = 'model.urdf';

config.jointOrder = {'torso_pitch', 'torso_roll', 'torso_yaw', ...
                    'l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow', ...
                    'r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', 'r_elbow', ...
                    'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw', 'l_knee', 'l_ankle_pitch', 'l_ankle_roll', ...
                    'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_ankle_pitch', 'r_ankle_roll'};
config.N_DOF = 23;

config.initialConditions.base_position = [0; 0; 0.65];
config.initialConditions.orientation = diag([-1, -1, 1]);
config.initialConditions.w_H_b = Rp2Hom(config.initialConditions.orientation, config.initialConditions.base_position);
% generate decent position
config.initialConditions.s = [0.1744; 0.0007; 0.0001; -0.1745; ...
                            0.4363; 0.6981; 0.2618; -0.1745; ...
                            0.4363; 0.6981; 0.2618; 0.0003; ...
                            0.0000; -0.0001; 0.0004; -0.0004; ...
                            0.0003; 0.0002; 0.0001; -0.0002; ...
                            0.0004; -0.0005; 0.0003];

config.initialConditions.base_linear_velocity = [0; 0; 0];
config.initialConditions.base_angular_velocity = [0; 0; 0];
config.initialConditions.base_pose_dot = [config.initialConditions.base_linear_velocity; config.initialConditions.base_angular_velocity];
config.initialConditions.s_dot = zeros(config.N_DOF, 1);

robot = Robot(config)

[base_dd, s_dd] = robot.forward_dynamics(zeros(23, 1), zeros(29, 1));

vertex(:, 1) = [-0.06; 0.04; 0];
vertex(:, 2) = [0.11; 0.04; 0];
vertex(:, 3) = [0.11; -0.035; 0];
vertex(:, 4) = [-0.06; -0.035; 0];

contacts = Contacts(vertex, robot, 0.1)
[generalized_total_wrench, wrench_left_foot, wrench_right_foot, contact_detected] = contacts.compute_contact_forces(robot, zeros(23, 1), zeros(29, 1))

state = State(0.01)
