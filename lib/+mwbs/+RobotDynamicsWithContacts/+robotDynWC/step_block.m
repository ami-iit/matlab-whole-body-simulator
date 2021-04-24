classdef step_block < matlab.System & matlab.system.mixin.Propagates
    % step_block This block takes as input the joint torques and the
    % applied external forces and evolves the state of the robot

    properties (Nontunable)
        robot_config;
        contact_config;
        physics_config;
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
            obj.contacts = mwbs.Contacts(obj.contact_config.foot_print, obj.robot, obj.contact_config.friction_coefficient);
            obj.state = mwbs.State(obj.physics_config.TIME_STEP);
            obj.state.set(obj.robot_config.initialConditions.w_H_b, obj.robot_config.initialConditions.s, ...
                obj.robot_config.initialConditions.base_pose_dot, obj.robot_config.initialConditions.s_dot);
        end

        function [w_H_b, s, base_pose_dot, s_dot, wrench_left_foot, wrench_right_foot, kinDynOut] = stepImpl(obj, generalized_ext_wrench, torque, motorInertias)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.

            % computes the contact quantites and the velocity after a possible impact
            [generalized_total_wrench, wrench_left_foot, wrench_right_foot, base_pose_dot, s_dot] = ...
                obj.contacts.compute_contact(obj.robot, torque, generalized_ext_wrench, motorInertias, obj.state.base_pose_dot, obj.state.s_dot);
            % sets the velocity in the state
            obj.state.set_velocity(base_pose_dot, s_dot);
            % compute the robot acceleration
            [base_pose_ddot, s_ddot] = obj.robot.forward_dynamics(torque, generalized_total_wrench,motorInertias);
            % integrate the dynamics
            [w_H_b, s, base_pose_dot, s_dot] = obj.state.ode_step(base_pose_ddot, s_ddot);
            % update the robot state
            obj.robot.set_robot_state(w_H_b, s, base_pose_dot, s_dot);
            % Get feet contact state
            [left_foot_in_contact, right_foot_in_contact] = obj.contacts.getFeetContactState();
            
            % output the kinematic and dynamic variables
            kinDynOut.w_H_b = w_H_b;
            kinDynOut.s = s;
            kinDynOut.nu = [base_pose_dot;s_dot];
            [kinDynOut.w_H_l_sole    , kinDynOut.w_H_r_sole    ] = obj.robot.get_feet_H();
            [kinDynOut.J_l_sole      , kinDynOut.J_r_sole      ] = obj.robot.get_feet_jacobians();
            [kinDynOut.JDot_l_sole_nu, kinDynOut.JDot_r_sole_nu] = obj.robot.get_feet_JDot_nu();
            kinDynOut.M = obj.robot.get_mass_matrix(motorInertias);
            kinDynOut.h = obj.robot.get_bias_forces();
            kinDynOut.motorGrpI = zeros(size(s));
            kinDynOut.fc = [wrench_left_foot;wrench_right_foot];
            kinDynOut.nuDot = [base_pose_ddot;s_ddot];
            kinDynOut.left_right_foot_in_contact = [left_foot_in_contact,right_foot_in_contact];
        end

        function resetImpl(obj)

        end

        function [out, out2, out3, out4, out5, out6, out7] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [4 4]; % homogeneous matrix dim
            out2 = [double(obj.robot_config.N_DOF),1]; % joints position vector dim
            out3 = [6 1]; % base velocity vector dim
            out4 = [double(obj.robot_config.N_DOF),1]; % joints velocity vector dim
            out5 = [6 1]; % wrench left foot vector dim
            out6 = [6 1]; % wrench right foot vector dim
            out7 = 1;
        end

        function [out, out2, out3, out4, out5, out6, out7] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "double";
            out2 = "double";
            out3 = "double";
            out4 = "double";
            out5 = "double";
            out6 = "double";
            out7 = obj.OutputBusName;
        end

        function [out, out2, out3, out4, out5, out6, out7] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;
            out7 = false;
        end

        function [out, out2, out3, out4, out5, out6, out7] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
            out6 = true;
            out7 = true;
        end

        function names = getSimulinkFunctionNamesImpl(~)
            names = {...
                'simFunc_getFrameFreeFloatingJacobianLFoot', ...
                'simFunc_getFrameFreeFloatingJacobianRFoot', ...
                'simFunc_getFreeFloatingMassMatrix', ...
                'simFunc_generalizedBiasForces', ...
                'simFunc_getFrameBiasAccLFoot', ...
                'simFunc_getFrameBiasAccRFoot', ...
                'simFunc_getWorldTransformLFoot', ...
                'simFunc_getWorldTransformRFoot', ...
                'simFunc_qpOASES'};
        end

    end

end
