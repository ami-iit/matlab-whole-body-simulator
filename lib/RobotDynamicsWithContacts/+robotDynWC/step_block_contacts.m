classdef step_block_contacts < matlab.System & matlab.system.mixin.Propagates
    % step_block_contacts This block takes as input the joint torques and the
    % applied external forces and evolves the state of the robot

    properties (Nontunable)
        robot_config;
        contact_config;
        physics_config;
        OutputBusName = 'bus_name';
    end

    properties (DiscreteState)

    end

    properties (Access = {?wbs.StepBlockInit})
        robot; contacts; state;
    end

    methods (Access = protected)

        function setupImpl(obj)
            obj.robot = wbs.Robot(obj.robot_config,obj.physics_config.GRAVITY_ACC);
            obj.contacts = wbs.Contacts(obj.contact_config.foot_print, obj.robot, obj.contact_config.friction_coefficient, obj.physics_config.USE_QPOASES);
            obj.state = wbs.State(obj.physics_config.TIME_STEP);
            obj.state.set(obj.robot_config.initialConditions.w_H_b, obj.robot_config.initialConditions.s, ...
                obj.robot_config.initialConditions.base_pose_dot, obj.robot_config.initialConditions.s_dot);
            wbs.StepBlockInit.setSharedConfig(obj);
        end

        function [J_feet, M, h, JDot_nu_feet, contact_points] = stepImpl(obj, motorInertias, torque, generalized_ext_wrench)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.

            % computes the inputs for estimating the contact quantites and the velocity after a possible impact
            [J_feet, M, h, JDot_nu_feet, contact_points] = ...
                obj.contacts.compute_inputs_for_unilateral_linear_contact_est(obj.robot, torque, generalized_ext_wrench, motorInertias, obj.state.base_pose_dot, obj.state.s_dot);
        end

        function resetImpl(obj)

        end

        function [out, out2, out3, out4, out5] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [6,2*(6+double(obj.robot_config.N_DOF))]; % [Jacobian,Jacobian] dim
            out2 = [6+double(obj.robot_config.N_DOF),1]; % h dim
            out3 = [6+double(obj.robot_config.N_DOF),6+double(obj.robot_config.N_DOF)]; % M dim
            out4 = [6,2]; % [JDot_nu_feet,JDot_nu_feet] dim
            out5 = [24 1]; % wrench left foot vector dim
        end

        function [out, out2, out3, out4, out5] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "double";
            out2 = "double";
            out3 = "double";
            out4 = "double";
            out5 = "double";
        end

        function [out, out2, out3, out4, out5] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
        end

        function [out, out2, out3, out4, out5] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
        end

    end

end
