%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2020
%  * @author: Giuseppe L'Erario
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear variables
clear functions
close all
clc

% REMARK : If you have installed the URDF models by https://github.com/robotology/icub-models
% You could fill the required variables as follows:
% % Substitute in the following the location of the install prefix of icub-models
% if installed with robotology superbuild you can directly use
icubModelsInstallPrefix = getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX');

%% GENERAL SIMULATION INFO
% Simulation time and delta_t [s]
Config.simulationTime = 1;
Config.GRAVITY_ACC = [0,0,-9.81];
Config.tStep = 0.001;

% Since we cannot assume that the optimization toolbox license is available, we use another QP
% solver in place of the MATLAB native solver 'quadprog'.
% The alternate optional solver was the OSQP, but after the changes to support the code generation in
% the Matlab System block 'step_block', only the QPOASES solver (Simulink block) is supported.
% So the option below is DEPRECATED.
Config.USE_OSQP = false;
Config.USE_QPOASES = true;

% Do you want to enable the Visualizer?
confVisualizer.visualizeRobot = true;

%% ADD CONFIGURATION FILES
% Select the robot name from YARP_ROBOT_NAME
robotName = getenv('YARP_ROBOT_NAME');

% Run robot-specific and controller-specific configuration parameters
run(strcat('app/robots/', robotName, '/configRobot.m'));
run(strcat('app/robots/', robotName, '/initVisualizer.m'));

%% Init simulator core physics paramaters
physics_config.GRAVITY_ACC = Config.GRAVITY_ACC;
physics_config.TIME_STEP = Config.tStep;
