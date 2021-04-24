classdef Robot < handle
    %ROBOT The Robot class exploits the iDynTree wrappers to compute Kinematic and Dynamic quantities.
    % Robot Methods:
    %    set_robot_state - Sets the robot state with kinematic information
    %    get_mass_matrix - Returns the mass matrix with or without the motor reflected inertias
    %    get_bias_forces - Returns the bias force
    %    get_feet_jacobians - Returns the Jacobians of the feet
    %    get_feet_JDot_nu - Returns the Jacobian derivative of the feet multiplied by the configuration velocity
    %    get_feet_H - Returns the Homogenous transform of the feet in the world frame
    %    get_frame_jacobian - Returns the Jacobian of a specified frame
    %    get_frame_JDot_nu - Returns the Jacobian derivative of a specified frame multiplied by the configuration velocity
    %    forward_dynamics - Compute forward dynamics

    properties
        NDOF; % DOF of the robot
    end

    properties (Access = private)
        KinDynModel; % kynDyn robot model
        g; % gravity vector
        useMotorReflectedInertias; % Adds the reflected inetias to the mass matrix
        JDot_nu_LFoot_iDyntree; % \dot{J} \nu relative to left foot
        JDot_nu_RFoot_iDyntree; % \dot{J} \nu relative to right foot
        LFoot_frameName; RFoot_frameName; % frame names relative to left and right foot
        S; % selector matrix
    end

    methods

        function obj = Robot(config,gravityAcc)
            % ROBOT Sets up the object. Takes as input a config file
            % loading the model
            obj.KinDynModel = mwbs.RobotDynamicsWithContacts.iDynTree2WBTmappers.loadReducedModel(config.jointOrder, config.robotFrames.BASE, ...
                config.modelPath, config.fileName, false);

            %initialize robot state
            obj.g = gravityAcc;
            obj.set_robot_state(config.initialConditions.w_H_b, config.initialConditions.s, ...
                config.initialConditions.base_pose_dot, config.initialConditions.s_dot);

            % initialize general quantites and iDynTree objects
            obj.LFoot_frameName = config.robotFrames.LEFT_FOOT;
            obj.RFoot_frameName = config.robotFrames.RIGHT_FOOT;
            obj.useMotorReflectedInertias = config.SIMULATE_MOTOR_REFLECTED_INERTIA;
            obj.NDOF = obj.KinDynModel.NDOF;
            obj.S = [zeros(6, obj.KinDynModel.NDOF); eye(obj.KinDynModel.NDOF)];
        end

        function set_robot_state(obj, w_H_b, s, base_pose_dot, s_dot)
            % set_robot_state Sets the robot state with kinematic information
            % INPUT: - w_H_b: [4,4] Homogeneous transformation body to world frame
            %        - s = [NDOF, 1] Joints position vector
            %        - base_pose_dot = [6,1] linear and angular velocity of the base
            %        - s_dot = [NDOF, 1] Joints velocity vector
            obj.KinDynModel.kinDynComp = obj.KinDynModel.kinDynComp.setRobotState(w_H_b, s, base_pose_dot, s_dot);
        end

        function M = get_mass_matrix(obj,motorInertias)
            % get_mass_matrix Returns the mass matrix
            % OUTPUT: - M: mass matrix
            [ack,M] = obj.KinDynModel.kinDynComp.getFreeFloatingMassMatrix();
            if (~ack)
                error('[Robot: get_mass_matrix] Unable to retrieve the mass matrix');
            end

            % Add the reflected inertia if the feature is activated
            if obj.useMotorReflectedInertias
                M = M + diag([zeros(6,1);motorInertias]);
            end
        end

        function h = get_bias_forces(obj)
            % get_bias_forces Returns the bias force
            % OUTPUT: - h: bias force
            [ack,h] = obj.KinDynModel.kinDynComp.generalizedBiasForces();
            if (~ack)
                error('[Robot: get_bias_forces] Unable to retrieve the bias force')
            end
        end

        function [J_LFoot, J_RFoot] = get_feet_jacobians(obj)
            % get_feet_jacobians Returns the Jacobians of the feet
            % OUTPUT: - J_left_foot: Jacobian of the left foot
            %         - J_right_foot: Jacobian of the right foot
            [ack,J_LFoot] = obj.KinDynModel.kinDynComp.getFrameFreeFloatingJacobianLRfoot('LFoot');
            if (~ack)
                error('[Robot: get_feet_jacobians] Unable to retrieve the left foot jacobian');
            end

            [ack,J_RFoot] = obj.KinDynModel.kinDynComp.getFrameFreeFloatingJacobianLRfoot('RFoot');
            if (~ack)
                error('[Robot: get_feet_jacobians] Unable to retrieve the right foot jacobian');
            end
        end

        function [JDot_nu_LFOOT, JDot_nu_RFOOT] = get_feet_JDot_nu(obj)
            % get_feet_JDot_nu Returns the Jacobian derivative of the feet multiplied by the configuration velocity
            % OUTPUT: - JDot_nu_LFOOT: \dot{J} nu relative to the left foot
            %         - JDot_nu_RFOOT: \dot{J} nu relative to the right foot
            JDot_nu_LFOOT = obj.KinDynModel.kinDynComp.getFrameBiasAccLRfoot('LFoot');
            JDot_nu_RFOOT = obj.KinDynModel.kinDynComp.getFrameBiasAccLRfoot('RFoot');
        end

        function [H_LFOOT, H_RFOOT] = get_feet_H(obj)
            % get_feet_H Returns the Homogenous transform of the feet in the world frame
            % OUTPUT: - H_LFOOT: w_H_b of relative to the left feet
            %         - H_RFOOT: w_H_b of relative to the right feet
            H_LFOOT = obj.KinDynModel.kinDynComp.getWorldTransformLRfoot('LFoot');
            H_RFOOT = obj.KinDynModel.kinDynComp.getWorldTransformLRfoot('RFoot');
        end

        function [base_pose_ddot, s_ddot] = forward_dynamics(obj, torque, generalized_total_wrench,motorInertias)
            % forward_dynamics Compute forward dynamics
            % \dot{v} = inv{M}(S*tau + generalized_external_forces - h)
            % INPUT: - torque: the joints torque
            %        - generalized_total_wrench: the sum of the external wrenches in the configuration space
            % OUTPUT: - base_pose_ddot: the linear and angular acceleration of the base
            %         - s_ddot: the joints acceleration
            M = obj.get_mass_matrix(motorInertias);
            h = obj.get_bias_forces();
            ddot = M \ (obj.S * torque + generalized_total_wrench - h);
            base_pose_ddot = ddot(1:6);
            s_ddot = ddot(7:end);
        end

    end

end
