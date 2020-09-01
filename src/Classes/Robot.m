classdef Robot < handle
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here

    properties
        KinDynModel% kynDyn robot model
        g = [0, 0, -9.81]% gravity vector
        M_iDyn = iDynTree.MatrixDynSize(); % mass matrix iDynTree
        J_LFoot_iDyntree; % Jacobian relative to left foot
        J_RFoot_iDyntree; % Jacobian relative to right foot
        JDot_nu_LFoot_iDyntree; % \dot{J} \nu relative to left foot
        JDot_nu_RFoot_iDyntree; % \dot{J} \nu relative to right foot
        LFoot_frameID; RFoot_frameID; % framesID relative to left and right foot
        h_iDyn; % bias forces iDynTree
        NDOF; % DOF of the robot
        S; % selector matrix
    end

    methods

        function obj = Robot(config)
            % loading the model
            obj.KinDynModel = iDynTreeWrappers.loadReducedModel(config.jointOrder, 'root_link', ...
                config.modelPath, config.fileName, false);

            %initialize robot state
            obj.set_robot_state(config.initialConditions.w_H_b, config.initialConditions.s, ...
                config.initialConditions.base_pose_dot, config.initialConditions.s_dot);

            % initialize general quantites and iDynTree objects
            obj.J_LFoot_iDyntree = iDynTree.MatrixDynSize(6, obj.KinDynModel.NDOF + 6);
            obj.J_RFoot_iDyntree = iDynTree.MatrixDynSize(6, obj.KinDynModel.NDOF + 6);
            obj.LFoot_frameID = obj.KinDynModel.kinDynComp.getFrameIndex('l_sole');
            obj.RFoot_frameID = obj.KinDynModel.kinDynComp.getFrameIndex('r_sole');
            obj.h_iDyn = iDynTree.FreeFloatingGeneralizedTorques(obj.KinDynModel.kinDynComp.model);
            obj.M_iDyn = iDynTree.MatrixDynSize();
            obj.NDOF = obj.KinDynModel.NDOF;
            obj.S = [zeros(6, obj.KinDynModel.NDOF); eye(obj.KinDynModel.NDOF)];
        end

        function set_robot_state(obj, w_H_b, s, base_pose_dot, s_dot)
            iDynTreeWrappers.setRobotState(obj.KinDynModel, w_H_b, s, ...
                base_pose_dot, s_dot, obj.g);
        end

        function M = get_mass_matrix(obj)
            obj.KinDynModel.kinDynComp.getFreeFloatingMassMatrix(obj.M_iDyn);
            M = obj.M_iDyn.toMatlab;
        end

        function h = get_bias_forces(obj)
            obj.KinDynModel.kinDynComp.generalizedBiasForces(obj.h_iDyn);
            h_b = obj.h_iDyn.baseWrench.toMatlab;
            h_s = obj.h_iDyn.jointTorques.toMatlab;
            h = [h_b; h_s];
        end

        function [J_LFoot, J_RFoot] = get_feet_jacobians(obj)
            obj.KinDynModel.kinDynComp.getFrameFreeFloatingJacobian(obj.LFoot_frameID, obj.J_LFoot_iDyntree);
            obj.KinDynModel.kinDynComp.getFrameFreeFloatingJacobian(obj.RFoot_frameID, obj.J_RFoot_iDyntree);
            J_LFoot = obj.J_LFoot_iDyntree.toMatlab;
            J_RFoot = obj.J_RFoot_iDyntree.toMatlab;
        end

        function [JDot_nu_LFOOT, JDot_nu_RFOOT] = get_feet_JDot_nu(obj)
            obj.JDot_nu_LFoot_iDyntree = obj.KinDynModel.kinDynComp.getFrameBiasAcc('l_sole');
            obj.JDot_nu_RFoot_iDyntree = obj.KinDynModel.kinDynComp.getFrameBiasAcc('r_sole');
            JDot_nu_LFOOT = obj.JDot_nu_LFoot_iDyntree.toMatlab;
            JDot_nu_RFOOT = obj.JDot_nu_RFoot_iDyntree.toMatlab;
        end

        function [H_LFOOT, H_RFOOT] = get_feet_H(obj)
            H_LFOOT = iDynTreeWrappers.getWorldTransform(obj.KinDynModel, 'l_sole');
            H_RFOOT = iDynTreeWrappers.getWorldTransform(obj.KinDynModel, 'r_sole');
        end

        function [base_pose_ddot, s_ddot] = forward_dynamics(obj, torques, generalized_total_wrench)
            % compute forward dynamics as a first order system
            % M \dot{v} + h = S*tau + external_forces
            % state x = [x1, x2]
            % dot{x} = [x2;...
            %           dot{v}];
            % dot{v} = inv{M}(S*tau + external_forces - h)
            M = obj.get_mass_matrix();
            h = obj.get_bias_forces();
            ddot = M \ (obj.S * torques + generalized_total_wrench - h);
            base_pose_ddot = ddot(1:6);
            s_ddot = ddot(7:end);
        end

    end

end
