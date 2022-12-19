%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2021 CoDyCo
%  * @author: Venus Pasandi (venus.pasandi@iit.it)
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * A copy of the license can be found at
%  * http://www.robotcub.org/icub/license/gpl.txt
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  *
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTE: THIS SCRIPT IS RUN AUTOMATICALLY WHEN THE USER STARTS THE ASSOCIATED
% SIMULINK MODEL. NO NEED TO RUN THIS SCRIPT EVERY TIME.

mwbs.RobotDynamicsWithContacts.initRobotDynamicsWithContactsCB_closedChains;

% Get the Workspace/Mask menu
propMotorReflectedInertiaFormat = get_param(gcb,'motorReflectedInertiaFormat');

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the simulink functions for having the jacobian, forward
% kinematic and bias acceleration for the links that interact with
% the ground

if isfield (robot_config.robotFrames,'IN_CONTACT_WITH_GROUND')
    robotInContactFrames = robot_config.robotFrames.IN_CONTACT_WITH_GROUND;
    if isempty(robot_config.robotFrames.IN_CONTACT_WITH_GROUND)
        numRobotInContactFrames = 0;
        numInContactFramesSlc = 1;
    else
        numRobotInContactFrames = length(robotInContactFrames);
        numInContactFramesSlc = length(robotInContactFrames);
    end
else
    robotInContactFrames = {};
    numRobotInContactFrames = 0;
    numInContactFramesSlc = 1;
end
    
if (numRobotInContactFrames == 0)
    H_jacobianBLK = find_system(gcb,'FindAll','On','LookUnderMasks','on','FollowLinks','on','Type','Block','Name','First Jacobian');
    H_forwardKinBLK = find_system(gcb,'FindAll','On','LookUnderMasks','on','FollowLinks','on','Type','Block','Name','First ForwardKinematics');
    H_biasAccBLK = find_system(gcb,'FindAll','On','LookUnderMasks','on','FollowLinks','on','Type','Block','Name','First DotJNu');
    set_param(H_jacobianBLK,'FrameName','robot_config.robotFrames.BASE');
    set_param(H_forwardKinBLK,'FrameName','robot_config.robotFrames.BASE');
    set_param(H_biasAccBLK,'FrameName','robot_config.robotFrames.BASE');
else
    H_jacobianBLK = find_system(gcb,'FindAll','On','LookUnderMasks','on','FollowLinks','on','Type','Block','Name','First Jacobian');
    H_forwardKinBLK = find_system(gcb,'FindAll','On','LookUnderMasks','on','FollowLinks','on','Type','Block','Name','First ForwardKinematics');
    H_biasAccBLK = find_system(gcb,'FindAll','On','LookUnderMasks','on','FollowLinks','on','Type','Block','Name','First DotJNu');
    set_param(H_jacobianBLK,'FrameName','robot_config.robotFrames.IN_CONTACT_WITH_GROUND{1}');
    set_param(H_forwardKinBLK,'FrameName','robot_config.robotFrames.IN_CONTACT_WITH_GROUND{1}');
    set_param(H_biasAccBLK,'FrameName','robot_config.robotFrames.IN_CONTACT_WITH_GROUND{1}');
end

% Check if the current block structure matches the robot model
blockPath = gcbp;
H_simFuncJConFramesBLK = find_system(strcat(blockPath.getBlock(1),'/simulinkFunction_dyn_jacobian_inContactFrames'),'FindAll','On','LookUnderMasks','on','FollowLinks','on','BlockType','Concatenate');
if isempty(H_simFuncJConFramesBLK)
    numInContactFrames = 1;
else
    numInContactFrames = eval(get_param(H_simFuncJConFramesBLK,'NumInputs'));
end

if (numInContactFramesSlc == numInContactFrames)
    % the current structure perfectly matches the robot model so there is
    % no need for further modifications.
elseif numInContactFramesSlc > numInContactFrames
    disBLK = 5;
    pose_jacobianBLK = get_param(H_jacobianBLK,'position');
    pose_forwardKinBLK = get_param(H_forwardKinBLK,'position');
    pose_biasAccBLK = get_param(H_biasAccBLK,'position');
    height_jacobianBLK = abs(pose_jacobianBLK(4)-pose_jacobianBLK(2));
    height_forwardKinBLK = abs(pose_forwardKinBLK(4)-pose_forwardKinBLK(2));
    height_biasAccBLK = abs(pose_biasAccBLK(4)-pose_biasAccBLK(2));
    if (numInContactFrames == 1)
        delete_line(get_param(H_jacobianBLK,'parent'),'First Jacobian/1','jacobian/1');
        delete_line(get_param(H_forwardKinBLK,'parent'),'First ForwardKinematics/1','w_H_sole/1');
        delete_line(get_param(H_biasAccBLK,'parent'),'First DotJNu/1','dotJnu/1');
        add_block('simulink/Math Operations/Matrix Concatenate',strcat(get_param(H_jacobianBLK,'parent'),'/myMux'),'NumInputs',num2str(numRobotInContactFrames),'position',[pose_jacobianBLK(3)+50,pose_jacobianBLK(2),pose_jacobianBLK(3)+70,pose_jacobianBLK(4)],'ConcatenateDimension','1');
        add_block('simulink/Math Operations/Matrix Concatenate',strcat(get_param(H_forwardKinBLK,'parent'),'/myMux'),'NumInputs',num2str(numRobotInContactFrames),'position',[pose_forwardKinBLK(3)+50,pose_forwardKinBLK(2),pose_forwardKinBLK(3)+70,pose_forwardKinBLK(4)],'ConcatenateDimension','1');
        add_block('simulink/Math Operations/Matrix Concatenate',strcat(get_param(H_biasAccBLK,'parent'),'/myMux'),'NumInputs',num2str(numRobotInContactFrames),'position',[pose_biasAccBLK(3)+50,pose_biasAccBLK(2),pose_biasAccBLK(3)+70,pose_biasAccBLK(4)],'ConcatenateDimension','1');
        add_line(get_param(H_jacobianBLK,'parent'),'First Jacobian/1','myMux/1');
        add_line(get_param(H_forwardKinBLK,'parent'),'First ForwardKinematics/1','myMux/1');
        add_line(get_param(H_biasAccBLK,'parent'),'First DotJNu/1','myMux/1');
        add_line(get_param(H_jacobianBLK,'parent'),'myMux/1','jacobian/1');
        add_line(get_param(H_forwardKinBLK,'parent'),'myMux/1','w_H_sole/1');
        add_line(get_param(H_biasAccBLK,'parent'),'myMux/1','dotJnu/1');
    else
        set_param(strcat(get_param(H_jacobianBLK,'parent'),'/myMux'),'NumInputs',num2str(numRobotInContactFrames));
        set_param(strcat(get_param(H_forwardKinBLK,'parent'),'/myMux'),'NumInputs',num2str(numRobotInContactFrames));
        set_param(strcat(get_param(H_biasAccBLK,'parent'),'/myMux'),'NumInputs',num2str(numRobotInContactFrames));
    end
    for counter = numInContactFrames+1 : numRobotInContactFrames
        pose_BLK = [0,(counter-1)*(disBLK+height_jacobianBLK),0,(counter-1)*(disBLK+height_jacobianBLK)];
        add_block(strcat(get_param(H_jacobianBLK,'parent'),'/First Jacobian'),strcat(get_param(H_jacobianBLK,'parent'),'/Jacobian_',num2str(counter)),'position',pose_jacobianBLK + pose_BLK,'FrameName',strcat('robot_config.robotFrames.IN_CONTACT_WITH_GROUND{',num2str(counter),'}'));
        add_line(get_param(H_jacobianBLK,'parent'),'basePose/1',strcat('Jacobian_',num2str(counter),'/1'));
        add_line(get_param(H_jacobianBLK,'parent'),'jointPosition/1',strcat('Jacobian_',num2str(counter),'/2'));
        add_line(get_param(H_jacobianBLK,'parent'),strcat('Jacobian_',num2str(counter),'/1'),strcat('myMux/',num2str(counter)));
    
        pose_BLK = [0,(counter-1)*(disBLK+height_forwardKinBLK),0,(counter-1)*(disBLK+height_forwardKinBLK)];
        add_block(strcat(get_param(H_forwardKinBLK,'parent'),'/First ForwardKinematics'),strcat(get_param(H_forwardKinBLK,'parent'),'/ForwardKinematics_',num2str(counter)),'position',pose_forwardKinBLK + pose_BLK,'FrameName',strcat('robot_config.robotFrames.IN_CONTACT_WITH_GROUND{',num2str(counter),'}'));
        add_line(get_param(H_forwardKinBLK,'parent'),'basePose/1',strcat('ForwardKinematics_',num2str(counter),'/1'));
        add_line(get_param(H_forwardKinBLK,'parent'),'jointPosition/1',strcat('ForwardKinematics_',num2str(counter),'/2'));
        add_line(get_param(H_forwardKinBLK,'parent'),strcat('ForwardKinematics_',num2str(counter),'/1'),strcat('myMux/',num2str(counter)));
    
        pose_BLK = [0,(counter-1)*(disBLK+height_biasAccBLK),0,(counter-1)*(disBLK+height_biasAccBLK)];
        add_block(strcat(get_param(H_biasAccBLK,'parent'),'/First DotJNu'),strcat(get_param(H_biasAccBLK,'parent'),'/DotJNu_',num2str(counter)),'position',pose_biasAccBLK + pose_BLK,'FrameName',strcat('robot_config.robotFrames.IN_CONTACT_WITH_GROUND{',num2str(counter),'}'));
        add_line(get_param(H_biasAccBLK,'parent'),'basePose/1',strcat('DotJNu_',num2str(counter),'/1'));
        add_line(get_param(H_biasAccBLK,'parent'),'jointPosition/1',strcat('DotJNu_',num2str(counter),'/2'));
        add_line(get_param(H_biasAccBLK,'parent'),'baseVel/1',strcat('DotJNu_',num2str(counter),'/3'));
        add_line(get_param(H_biasAccBLK,'parent'),'jointVel/1',strcat('DotJNu_',num2str(counter),'/4'));
        add_line(get_param(H_biasAccBLK,'parent'),strcat('DotJNu_',num2str(counter),'/1'),strcat('myMux/',num2str(counter)));
    end
else
    % Deleting the extra blocks
    for counter = numInContactFramesSlc+1 : numInContactFrames
        delete_line(get_param(H_jacobianBLK,'parent'),'basePose/1',strcat('Jacobian_',num2str(counter),'/1'));
        delete_line(get_param(H_jacobianBLK,'parent'),'jointPosition/1',strcat('Jacobian_',num2str(counter),'/2'));
        delete_line(get_param(H_jacobianBLK,'parent'),strcat('Jacobian_',num2str(counter),'/1'),strcat('myMux/',num2str(counter)));
        delete_block(strcat(get_param(H_jacobianBLK,'parent'),'/Jacobian_',num2str(counter)));
    
        delete_line(get_param(H_forwardKinBLK,'parent'),'basePose/1',strcat('ForwardKinematics_',num2str(counter),'/1'));
        delete_line(get_param(H_forwardKinBLK,'parent'),'jointPosition/1',strcat('ForwardKinematics_',num2str(counter),'/2'));
        delete_line(get_param(H_forwardKinBLK,'parent'),strcat('ForwardKinematics_',num2str(counter),'/1'),strcat('myMux/',num2str(counter)));
        delete_block(strcat(get_param(H_forwardKinBLK,'parent'),'/ForwardKinematics_',num2str(counter)));
    
        delete_line(get_param(H_biasAccBLK,'parent'),'basePose/1',strcat('DotJNu_',num2str(counter),'/1'));
        delete_line(get_param(H_biasAccBLK,'parent'),'jointPosition/1',strcat('DotJNu_',num2str(counter),'/2'));
        delete_line(get_param(H_biasAccBLK,'parent'),'baseVel/1',strcat('DotJNu_',num2str(counter),'/3'));
        delete_line(get_param(H_biasAccBLK,'parent'),'jointVel/1',strcat('DotJNu_',num2str(counter),'/4'));
        delete_line(get_param(H_biasAccBLK,'parent'),strcat('DotJNu_',num2str(counter),'/1'),strcat('myMux/',num2str(counter)));
        delete_block(strcat(get_param(H_biasAccBLK,'parent'),'/DotJNu_',num2str(counter)));
    end
    if (numInContactFramesSlc == 1)
        delete_line(get_param(H_jacobianBLK,'parent'),'First Jacobian/1','myMux/1');
        delete_line(get_param(H_forwardKinBLK,'parent'),'First ForwardKinematics/1','myMux/1');
        delete_line(get_param(H_biasAccBLK,'parent'),'First DotJNu/1','myMux/1');
        delete_line(get_param(H_jacobianBLK,'parent'),'myMux/1','jacobian/1');
        delete_line(get_param(H_forwardKinBLK,'parent'),'myMux/1','w_H_sole/1');
        delete_line(get_param(H_biasAccBLK,'parent'),'myMux/1','dotJnu/1');
        delete_block(strcat(get_param(H_jacobianBLK,'parent'),'/myMux'));
        delete_block(strcat(get_param(H_forwardKinBLK,'parent'),'/myMux'));
        delete_block(strcat(get_param(H_biasAccBLK,'parent'),'/myMux'));
        add_line(get_param(H_jacobianBLK,'parent'),'First Jacobian/1','jacobian/1');
        add_line(get_param(H_forwardKinBLK,'parent'),'First ForwardKinematics/1','w_H_sole/1');
        add_line(get_param(H_biasAccBLK,'parent'),'First DotJNu/1','dotJnu/1');
    else
        set_param(strcat(get_param(H_jacobianBLK,'parent'),'/myMux'),'NumInputs',num2str(numRobotInContactFrames));
        set_param(strcat(get_param(H_forwardKinBLK,'parent'),'/myMux'),'NumInputs',num2str(numRobotInContactFrames));
        set_param(strcat(get_param(H_biasAccBLK,'parent'),'/myMux'),'NumInputs',num2str(numRobotInContactFrames));
    end
end

clear H_jacobianBLK H_forwardKinBLK H_biasAccBLK 
clear H_simFuncJConFramesBLK numInContactFrames

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the simulink functions for having the jacobian, forward
% kinematic and bias acceleration for the frames of the prints of the 
% spilit points in the closed chains

% ----------------------------- INITIALIZATION ---------------------------
% Determine the number of split points that required according to the 
% desired robot model
if isfield(robot_config.robotFrames,'BREAK')
    spilitPointsFramesReq = robot_config.robotFrames.BREAK;
    if (numel(fieldnames(robot_config.robotFrames.BREAK)) == 0)
        numRobotSplitPoints = 0;
        numRobotSplitPointsSlc = 1; % Not to pass zero dimension
    else
        numRobotSplitPoints = numel(fieldnames(robot_config.robotFrames.BREAK));
        numRobotSplitPointsSlc = numel(fieldnames(robot_config.robotFrames.BREAK));
    end
else
    spilitPointsFramesReq = {};
    numRobotSplitPoints = 0;
    numRobotSplitPointsSlc = 1; % Not to pass zero dimension
end

% ----------------------------- MAIN -------------------------------------
H_jacobianBLK = find_system(gcb,'FindAll','On','LookUnderMasks','on','FollowLinks','on','Type','Block','Name','First Diff Jacobian');
H_biasAccBLK = find_system(gcb,'FindAll','On','LookUnderMasks','on','FollowLinks','on','Type','Block','Name','First Diff DotJNu');

if (numRobotSplitPoints == 0)
    set_param(getfullname(H_jacobianBLK),'firstFrameName','robot_config.robotFrames.BASE','secondFrameName','robot_config.robotFrames.BASE');
    set_param(getfullname(H_biasAccBLK),'firstFrameName','robot_config.robotFrames.BASE','secondFrameName','robot_config.robotFrames.BASE');
else
    set_param(getfullname(H_jacobianBLK),'firstFrameName','robot_config.robotFrames.BREAK.P1{1}','secondFrameName','robot_config.robotFrames.BREAK.P1{2}');
    set_param(getfullname(H_biasAccBLK),'firstFrameName','robot_config.robotFrames.BREAK.P1{1}','secondFrameName','robot_config.robotFrames.BREAK.P1{2}');
end

% Determine the number of split points considered currently in the block
H_simFuncJSPsBLK = find_system(strcat(blockPath.getBlock(1),'/simulinkFunction_dyn_jacobian_splitPoints'),'FindAll','On','LookUnderMasks','on','FollowLinks','on','BlockType','Concatenate');
if isempty(H_simFuncJSPsBLK)
    numSplitPoints = 1;
else
    numSplitPoints = eval(get_param(H_simFuncJSPsBLK,'NumInputs'));
end

% Check if the current block structure matches the robot model
if (numSplitPoints == numRobotSplitPointsSlc)
    % the current structure perfectly matches the robot model so there is
    % no need for further modifications.
elseif (numRobotSplitPointsSlc > numSplitPoints)
    disBLK = 5;
    pose_jacobianBLK = get_param(H_jacobianBLK,'position');
    pose_biasAccBLK = get_param(H_biasAccBLK,'position');
    height_jacobianBLK = abs(pose_jacobianBLK(4)-pose_jacobianBLK(2));
    height_biasAccBLK = abs(pose_biasAccBLK(4)-pose_biasAccBLK(2));
    if (numSplitPoints == 1)
        delete_line(get_param(H_jacobianBLK,'parent'),'First Diff Jacobian/1','jacobianDiffSplitPoint/1');
        delete_line(get_param(H_biasAccBLK,'parent'),'First Diff DotJNu/1','dotJNuDiffSplitPoint/1');
        add_block('simulink/Math Operations/Matrix Concatenate',strcat(get_param(H_jacobianBLK,'parent'),'/myMux'),'NumInputs',num2str(numRobotSplitPointsSlc),'position',[pose_jacobianBLK(3)+50,pose_jacobianBLK(2),pose_jacobianBLK(3)+70,pose_jacobianBLK(4)],'ConcatenateDimension','1');
        add_block('simulink/Math Operations/Matrix Concatenate',strcat(get_param(H_biasAccBLK,'parent'),'/myMux'),'NumInputs',num2str(numRobotSplitPointsSlc),'position',[pose_biasAccBLK(3)+50,pose_biasAccBLK(2),pose_biasAccBLK(3)+70,pose_biasAccBLK(4)],'ConcatenateDimension','1');
        add_line(get_param(H_jacobianBLK,'parent'),'First Diff Jacobian/1','myMux/1');
        add_line(get_param(H_biasAccBLK,'parent'),'First Diff DotJNu/1','myMux/1');
        add_line(get_param(H_jacobianBLK,'parent'),'myMux/1','jacobianDiffSplitPoint/1');
        add_line(get_param(H_biasAccBLK,'parent'),'myMux/1','dotJNuDiffSplitPoint/1');
    else
        set_param(strcat(get_param(H_jacobianBLK,'parent'),'/myMux'),'NumInputs',num2str(numRobotSplitPointsSlc));
        set_param(strcat(get_param(H_biasAccBLK,'parent'),'/myMux'),'NumInputs',num2str(numRobotSplitPointsSlc));
    end
    
    for counter = numSplitPoints+1 : numRobotSplitPointsSlc
        pose_BLK = [0,(counter-1)*(disBLK+height_jacobianBLK),0,(counter-1)*(disBLK+height_jacobianBLK)];
        add_block(strcat(get_param(H_jacobianBLK,'parent'),'/First Diff Jacobian'),strcat(get_param(H_jacobianBLK,'parent'),'/DiffJacobian_',num2str(counter)),'position',pose_jacobianBLK + pose_BLK,'firstFrameName',strcat('robot_config.robotFrames.BREAK.P',num2str(counter),'{1}'),'secondFrameName',strcat('robot_config.robotFrames.BREAK.P',num2str(counter),'{2}'));
        add_line(get_param(H_jacobianBLK,'parent'),'basePose/1',strcat('DiffJacobian_',num2str(counter),'/1'));
        add_line(get_param(H_jacobianBLK,'parent'),'jointPosition/1',strcat('DiffJacobian_',num2str(counter),'/2'));
        add_line(get_param(H_jacobianBLK,'parent'),strcat('DiffJacobian_',num2str(counter),'/1'),strcat('myMux/',num2str(counter)));
        
        pose_BLK = [0,(counter-1)*(disBLK+height_biasAccBLK),0,(counter-1)*(disBLK+height_biasAccBLK)];
        add_block(strcat(get_param(H_biasAccBLK,'parent'),'/First Diff DotJNu'),strcat(get_param(H_biasAccBLK,'parent'),'/DiffDotJNu_',num2str(counter)),'position',pose_biasAccBLK + pose_BLK,'firstFrameName',strcat('robot_config.robotFrames.BREAK.P',num2str(counter),'{1}'),'secondFrameName',strcat('robot_config.robotFrames.BREAK.P',num2str(counter),'{2}'));
        add_line(get_param(H_biasAccBLK,'parent'),'basePose/1',strcat('DiffDotJNu_',num2str(counter),'/1'));
        add_line(get_param(H_biasAccBLK,'parent'),'jointPosition/1',strcat('DiffDotJNu_',num2str(counter),'/2'));
        add_line(get_param(H_biasAccBLK,'parent'),'baseVel/1',strcat('DiffDotJNu_',num2str(counter),'/3'));
        add_line(get_param(H_biasAccBLK,'parent'),'jointVel/1',strcat('DiffDotJNu_',num2str(counter),'/4'));
        add_line(get_param(H_biasAccBLK,'parent'),strcat('DiffDotJNu_',num2str(counter),'/1'),strcat('myMux/',num2str(counter)));
    end
  else
    % Deleting the extra blocks
    for counter = numRobotSplitPointsSlc+1 : numSplitPoints
        delete_line(get_param(H_jacobianBLK,'parent'),'basePose/1',strcat('DiffJacobian_',num2str(counter),'/1'));
        delete_line(get_param(H_jacobianBLK,'parent'),'jointPosition/1',strcat('DiffJacobian_',num2str(counter),'/2'));
        delete_line(get_param(H_jacobianBLK,'parent'),strcat('DiffJacobian_',num2str(counter),'/1'),strcat('myMux/',num2str(counter)));
        delete_block(strcat(get_param(H_jacobianBLK,'parent'),'/DiffJacobian_',num2str(counter)));
       
        delete_line(get_param(H_biasAccBLK,'parent'),'basePose/1',strcat('DiffDotJNu_',num2str(counter),'/1'));
        delete_line(get_param(H_biasAccBLK,'parent'),'jointPosition/1',strcat('DiffDotJNu_',num2str(counter),'/2'));
        delete_line(get_param(H_biasAccBLK,'parent'),'baseVel/1',strcat('DiffDotJNu_',num2str(counter),'/3'));
        delete_line(get_param(H_biasAccBLK,'parent'),'jointVel/1',strcat('DiffDotJNu_',num2str(counter),'/4'));
        delete_line(get_param(H_biasAccBLK,'parent'),strcat('DiffDotJNu_',num2str(counter),'/1'),strcat('myMux/',num2str(counter)));
        delete_block(strcat(get_param(H_biasAccBLK,'parent'),'/DiffDotJNu_',num2str(counter)));
    end
    if (numRobotSplitPointsSlc == 1)
        delete_line(get_param(H_jacobianBLK,'parent'),'First Diff Jacobian/1','myMux/1');
        delete_line(get_param(H_biasAccBLK,'parent'),'First Diff DotJNu/1','myMux/1');
        delete_line(get_param(H_jacobianBLK,'parent'),'myMux/1','jacobianDiffSplitPoint/1');
        delete_line(get_param(H_biasAccBLK,'parent'),'myMux/1','dotJNuDiffSplitPoint/1');
        delete_block(strcat(get_param(H_jacobianBLK,'parent'),'/myMux'));
        delete_block(strcat(get_param(H_biasAccBLK,'parent'),'/myMux'));
        add_line(get_param(H_jacobianBLK,'parent'),'First Diff Jacobian/1','jacobianDiffSplitPoint/1');
        add_line(get_param(H_biasAccBLK,'parent'),'First Diff DotJNu/1','dotJNuDiffSplitPoint/1');
    else
        set_param(strcat(get_param(H_jacobianBLK,'parent'),'/myMux'),'NumInputs',num2str(numRobotSplitPointsSlc));
        set_param(strcat(get_param(H_biasAccBLK,'parent'),'/myMux'),'NumInputs',num2str(numRobotSplitPointsSlc));
    end
end

%% Prepare the parameters for the step block

contact_config.num_in_contact_frames = numRobotInContactFrames;
contact_config.num_closed_chains = numRobotSplitPoints;
contact_config.num_vertices = compute_number_of_vertices(contact_config.foot_print, contact_config.num_in_contact_frames);

%% Local functions
function num_vertices = compute_number_of_vertices(foot_print, num_in_contact_frames)
    if iscell(foot_print) && (length(foot_print) == num_in_contact_frames)

        num_vertices = size(foot_print{1},2);

    elseif iscell(foot_print)

        num_vertices = size(foot_print{1},2);

    else

        num_vertices = size(foot_print,2);

    end
end