classdef step_block_with_closed_chain < matlab.System & matlab.system.mixin.Propagates
    % step_block This block takes as input the joint torques and the
    % applied external forces and evolves the state of the robot

    properties (Nontunable)
        robot_config;
        contact_config;
        physics_config;
        motorReflectedInertiaFormat;
        OutputBusName = 'bus_name';
    end

    properties (DiscreteState)

    end

    properties (Access = private)
        robot; contacts; state;
    end

    methods (Access = protected)

        function setupImpl(obj)
            obj.robot = mwbs.Robot(obj.robot_config,obj.physics_config.GRAVITY_ACC);
            obj.contacts = mwbs.Contacts(obj.contact_config.foot_print, obj.robot, obj.contact_config.friction_coefficient,obj.robot_config.robotFrames.IN_CONTACT_WITH_GROUND);
            obj.state = mwbs.State(obj.physics_config.TIME_STEP);
            obj.state.set(obj.robot_config.initialConditions.w_H_b, obj.robot_config.initialConditions.s, ...
                obj.robot_config.initialConditions.base_pose_dot, obj.robot_config.initialConditions.s_dot);
        end

        function [w_H_b, s, base_pose_dot, s_dot, wrench_inContact_frames, kinDynOut] = stepImpl(obj, generalized_ext_wrench, torque, motorInertias)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.

            % computes the contact quantites and the velocity after a possible impact
            [generalized_total_wrench, wrench_inContact_frames, base_pose_dot, s_dot] = ...
                obj.contacts.compute_contact_closedChain(obj.robot, torque, generalized_ext_wrench, motorInertias, obj.state.base_pose_dot, obj.state.s_dot,obj);
            % sets the velocity in the state
            obj.state.set_velocity(base_pose_dot, s_dot);
            % compute the robot acceleration
            [base_pose_ddot, s_ddot] = obj.robot.forward_dynamics(torque, generalized_total_wrench, motorInertias, obj);
            % integrate the dynamics
            [w_H_b, s, base_pose_dot, s_dot] = obj.state.ode_step(base_pose_ddot, s_ddot);
            % update the robot state
            obj.robot.set_robot_state(w_H_b, s, base_pose_dot, s_dot);
            % Get feet contact state
            links_in_contact = obj.contacts.getFeetContactState(length(obj.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
            
            % output the kinematic and dynamic variables
            kinDynOut.w_H_b = w_H_b;
            kinDynOut.s = s;
            kinDynOut.nu = [base_pose_dot;s_dot];
            kinDynOut.w_H_frames_sole = obj.robot.get_inContactWithGround_H();
            kinDynOut.J_frames_sole = obj.robot.get_inContactWithGround_jacobians();
            kinDynOut.JDot_frames_sole_nu = obj.robot.get_inContactWithGround_JDot_nu();
            kinDynOut.M = obj.robot.get_mass_matrix(motorInertias,obj);
            kinDynOut.h = obj.robot.get_bias_forces();
            kinDynOut.motorGrpI = zeros(size(s));
            kinDynOut.fc = wrench_inContact_frames;
            kinDynOut.nuDot = [base_pose_ddot;s_ddot];
            kinDynOut.frames_in_contact = links_in_contact;
        end

        function resetImpl(obj)

        end

        function [out, out2, out3, out4, out5, out6] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [4 4]; % homogeneous matrix dim
            out2 = [double(obj.robot_config.N_DOF),1]; % joints position vector dim
            out3 = [6 1]; % base velocity vector dim
            out4 = [double(obj.robot_config.N_DOF),1]; % joints velocity vector dim
            out5 = [6*length(obj.robot_config.robotFrames.IN_CONTACT_WITH_GROUND) 1]; % wrench of the frames interacting with the ground vector dim
            out6 = 1;
        end

        function [out, out2, out3, out4, out5, out6] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "double";
            out2 = "double";
            out3 = "double";
            out4 = "double";
            out5 = "double";
            out6 = obj.OutputBusName;
        end

        function [out, out2, out3, out4, out5, out6] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;
        end

        function [out, out2, out3, out4, out5, out6] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
            out6 = true;
        end

        function names = getSimulinkFunctionNamesImpl(~)
            names = {...
                'simFunc_getFrameFreeFloatingJacobian_inContactFrames', ...
                'simFunc_getFreeFloatingMassMatrix', ...
                'simFunc_generalizedBiasForces', ...
                'simFunc_getFrameBiasAcc_inContactFrames', ...
                'simFunc_getWorldTransform_inContactFrames', ...
                'simFunc_qpOASES',...
                'simFunc_getFrameFreeFloatingJacobianSpilitPoints',...
                'simFunc_getFrameBiasAccSpilitPoints'};
        end
    end

    methods (Access = ?mwbs.Robot)
        function Mout = getFormattedReflectedInertia(obj,M,motorInertias)
            switch obj.motorReflectedInertiaFormat
                case 'matrix'
                    Mout = M + blkdiag(zeros(6),motorInertias);
                case 'vector'
                    Mout = M + diag([zeros(6,1);motorInertias]);
                otherwise
                    Mout = M;
            end
        end
    end

end
