%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2021 CoDyCo
%  * @author: Venus Pasandi
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
%  * venus.pasandi@iit.it
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTE: THIS SCRIPT IS RUN AUTOMATICALLY WHEN THE USER STARTS THE ASSOCIATED
% SIMULINK MODEL. NO NEED TO RUN THIS SCRIPT EVERY TIME.

% Get the Workspace/Mask menu
propMotorReflectedInertiaFormat = get_param(gcb,'motorReflectedInertiaFormat');

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the simulink functions for having the jacobian, forward
% kinematic and bias acceleration for the links that interact with
% the ground

if isempty(robot_config.robotFrames.IN_CONTACT_WITH_GROUND)
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

if (numInContactFrames == length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND))
    % the current structure perfectly matches the robot model so there is
    % no need for further modifications.
elseif length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND) > numInContactFrames
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
        add_block('simulink/Math Operations/Matrix Concatenate',strcat(get_param(H_jacobianBLK,'parent'),'/myMux'),'NumInputs',num2str(length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND)),'position',[pose_jacobianBLK(3)+50,pose_jacobianBLK(2),pose_jacobianBLK(3)+70,pose_jacobianBLK(4)],'ConcatenateDimension','1');
        add_block('simulink/Math Operations/Matrix Concatenate',strcat(get_param(H_forwardKinBLK,'parent'),'/myMux'),'NumInputs',num2str(length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND)),'position',[pose_forwardKinBLK(3)+50,pose_forwardKinBLK(2),pose_forwardKinBLK(3)+70,pose_forwardKinBLK(4)],'ConcatenateDimension','1');
        add_block('simulink/Math Operations/Matrix Concatenate',strcat(get_param(H_biasAccBLK,'parent'),'/myMux'),'NumInputs',num2str(length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND)),'position',[pose_biasAccBLK(3)+50,pose_biasAccBLK(2),pose_biasAccBLK(3)+70,pose_biasAccBLK(4)],'ConcatenateDimension','1');
        add_line(get_param(H_jacobianBLK,'parent'),'First Jacobian/1','myMux/1');
        add_line(get_param(H_forwardKinBLK,'parent'),'First ForwardKinematics/1','myMux/1');
        add_line(get_param(H_biasAccBLK,'parent'),'First DotJNu/1','myMux/1');
        add_line(get_param(H_jacobianBLK,'parent'),'myMux/1','jacobian/1');
        add_line(get_param(H_forwardKinBLK,'parent'),'myMux/1','w_H_sole/1');
        add_line(get_param(H_biasAccBLK,'parent'),'myMux/1','dotJnu/1');
    else
        set_param(strcat(get_param(H_jacobianBLK,'parent'),'/myMux'),'NumInputs',num2str(length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND)));
        set_param(strcat(get_param(H_forwardKinBLK,'parent'),'/myMux'),'NumInputs',num2str(length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND)));
        set_param(strcat(get_param(H_biasAccBLK,'parent'),'/myMux'),'NumInputs',num2str(length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND)));
    end
    for counter = numInContactFrames+1 : length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND)
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
    for counter = length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND)+1 : numInContactFrames
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
    if (length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND) == 1)
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
        set_param(strcat(get_param(H_jacobianBLK,'parent'),'/myMux'),'NumInputs',num2str(length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND)));
        set_param(strcat(get_param(H_forwardKinBLK,'parent'),'/myMux'),'NumInputs',num2str(length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND)));
        set_param(strcat(get_param(H_biasAccBLK,'parent'),'/myMux'),'NumInputs',num2str(length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND)));
    end
end

clear H_jacobianBLK H_forwardKinBLK H_biasAccBLK 
clear blockPath H_simFuncJConFramesBLK numInContactFrames