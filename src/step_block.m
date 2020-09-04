classdef step_block < matlab.System & matlab.system.mixin.Propagates
    % step_block This block takes as input the joint torques and the
    % applied external forces and evolves the state of the robot

    % Public, tunable properties
    properties (Nontunable)
        robot_config;
        contact_config;
        tStep;
    end

    properties (DiscreteState)

    end

    properties (Access = private)
        robot; contacts; state;
    end

    methods (Access = protected)

        function setupImpl(obj)
            obj.robot = Robot(obj.robot_config);
            obj.contacts = Contacts(obj.contact_config.foot_print, obj.robot, obj.contact_config.friction_coefficient);
            obj.state = State(obj.tStep);
            obj.state.set(obj.robot_config.initialConditions.w_H_b, obj.robot_config.initialConditions.s, ...
                obj.robot_config.initialConditions.base_pose_dot, obj.robot_config.initialConditions.s_dot);
        end

        function [w_H_b, s, base_pose_dot, s_dot, wrench_left_foot, wrench_right_foot] = stepImpl(obj, generalized_ext_wrench, torque)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.

            % computes the contact quantites and the velocity after a possible impact
            [generalized_total_wrench, wrench_left_foot, wrench_right_foot, base_pose_dot, s_dot] = ...
                obj.contacts.compute_contact(obj.robot, torque, generalized_ext_wrench, obj.state.base_pose_dot, obj.state.s_dot);
            % sets the velocity in the state
            obj.state.set_velocity(base_pose_dot, s_dot);
            % compute the robot acceleration
            [base_pose_ddot, s_ddot] = obj.robot.forward_dynamics(torque, generalized_total_wrench);
            % integrate the dynamics
            [w_H_b, s, base_pose_dot, s_dot] = obj.state.ode_step(base_pose_ddot, s_ddot);
            % update the robot state
            obj.robot.set_robot_state(w_H_b, s, base_pose_dot, s_dot)
        end

        function resetImpl(obj)

        end

        function [out, out2, out3, out4, out5, out6] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [4 4]; % homogeneous matrix dim
            out2 = [23 1]; % joints position vector dim
            out3 = [6 1]; % base velocity vector dim
            out4 = [23 1]; % joints velocity vector dim
            out5 = [6 1]; % wrench left foot vector dim
            out6 = [6 1]; % wrench right foot vector dim
        end

        function [out, out2, out3, out4, out5, out6] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = "double";
            out2 = "double";
            out3 = "double";
            out4 = "double";
            out5 = "double";
            out6 = "double";
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

    end

end
