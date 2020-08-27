classdef step_block < matlab.System & matlab.system.mixin.Propagates
    % untitled2 Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties (Nontunable)
        robot_config;
        contact_config;
        tStep;
    end

    properties
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
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
            initial_torque = zeros(23, 1);
            initial_generalized_ext_wrench = zeros(29, 1);
            [generalized_total_wrench, wrench_left_foot, wrench_right_foot, contact_detected] = ...
                obj.contacts.compute_contact_forces(obj.robot, initial_torque, initial_generalized_ext_wrench);
        end

        function [w_H_b, s, base_pose_dot, s_dot, wrench_left_foot, wrench_right_foot] = stepImpl(obj, generalized_ext_wrench, torque)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            disp(nargin)

            if nargin == 2
                disp('OLE')
                torque = zeros(obj.robot.NDOF, 1);
            end

            [generalized_total_wrench, wrench_left_foot, wrench_right_foot, contact_detected] = ...
                obj.contacts.compute_contact_forces(obj.robot, torque, generalized_ext_wrench);
            M = obj.robot.get_mass_matrix();
            J_feet = obj.robot.get_feet_jacobians();

            if contact_detected
                N = (eye(obj.robot.NDOF + 6) - M \ (J_feet' * ((J_feet * (M \ J_feet')) \ J_feet)));
                x = N * [obj.state.base_pose_dot; obj.state.s_dot];
                base_pose_dot = x(1:6);
                s_dot = x(7:end);
                obj.state.set_velocity(base_pose_dot, s_dot);
            end

            disp('eee')
            [base_pose_ddot, s_ddot] = obj.robot.forward_dynamics(torque, generalized_total_wrench);
            [w_H_b, s, base_pose_dot, s_dot] = obj.state.euler_step(base_pose_ddot, s_ddot);
            obj.robot.set_robot_state(w_H_b, s, base_pose_dot, s_dot)
        end

        function [w_H_b, s, base_pose_dot, s_dot, wrench_left_foot, wrench_right_foot] = resetImpl(obj)
            w_H_b = obj.robot_config.initialConditions.w_H_b;
            s = obj.robot_config.initialConditions.s;
            base_pose_dot = obj.robot_config.initialConditions.base_pose_dot;
            s_dot = obj.robot_config.initialConditions.s_dot;
            initial_torque = zeros(23, 1);
            initial_generalized_ext_wrench = zeros(29, 1);
            [generalized_total_wrench, wrench_left_foot, wrench_right_foot, contact_detected] = ...
                obj.contacts.compute_contact_forces(obj.robot, initial_torque, initial_generalized_ext_wrench);
        end

        function [out, out2, out3, out4, out5, out6] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [4 4];
            out2 = [23 1];
            out3 = [6 1];
            out4 = [23 1];
            out5 = [6 1];
            out6 = [6 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out, out2, out3, out4, out5, out6] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "double";
            out2 = "double";
            out3 = "double";
            out4 = "double";
            out5 = "double";
            out6 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out, out2, out3, out4, out5, out6] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out, out2, out3, out4, out5, out6] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
            out6 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end

        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(obj, name)
            % Return size, data type, and complexity of discrete-state
            % specified in name
            sz = [1 1];
            dt = "double";
            cp = false;
        end

    end

end
