classdef step_block_with_closed_chain < matlab.System & matlab.system.mixin.Propagates

%     STEP_BLOCK_WITH_CLOSED_CHAIN: This block evolves the state of the robot 
% 
%     **PROCEDURE**: For modeling the dynamics of a robot having some closed chains, 
%                    Each chain is opened by breaking a link in the chain. Then, a
%                    constraint wrench is applied to the broken point such that the
%                    two parts of the broken link is connected. Thus, the equation
%                    of motion for a floating base robot having some possible
%                    closed chains can be written as
% 
%                    M vDot + h = B u + Fe + Jc' Fc + Ji Fi
% 
%                    where
% 
%                    u is the joint torques, Fe is the external wrenches, Fc is the
%                    pure forces applied to the contact points, and Fi is the 
%                    internal wrenches in the spilit/broken points.
% 
%                    Considering the equation of motion, for computing the
%                    evolution of the states of the robot,
% 
%                    1. The contact quantites (i.e. Fc and Fi) and the velocity
%                       vector of the robot after a possible impact is computed,
% 
%                    2. The robot velocity vector is updated,
% 
%                    3. The robot acceleration vector is computed using the forward
%                       dynamics,
% 
%                    4. The states of the robot are computed by integrating the 
%                       robot acceleration vector.
% 
%     **FORMAT**: [w_H_b, s, base_pose_dot, s_dot, wrench_inContact_frames, kinDynOut] = stepImpl(obj, generalized_ext_wrench, torque, motorInertias)
% 
%     **INPUT:**
%                 - generalized_ext_wrench: [(N+6)x1] The external wrenches applied to the robot,
%                 - torque:                 [N x 1] The joint torques,
%                 - motorInertias:          [N x 1] or [N x N] The motors reflected inertia
% 
%     **OUTPUT:**
%                 - w_H_b:                   [4 x 4] The homogenous transformation matrix from the base link to the world frame,
%                 - s:                       [N x 1] The position vector of the joints,
%                 - base_pose_dot:           [6 x 1] The velocity vector of the base link,
%                 - s_dot:                   [N x 1] The velocity vector of the joints,   
%                 - wrench_inContact_frames: [(6m) x 1] The vector of the wrenches applied to the sole of the links that are in contact with the ground,
%                 - kinDynOut:               [BUS] Some kinematic and dynamic variables. 
% 
%     **AUTHORS:** Venus Pasandi, Nuno Guedelha
% 
%     all authors are with the Italian Istitute of Technology (IIT)
%     email: name.surname@iit.it
% 
%     PLACE AND DATE: <Genoa, March 2022>

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
            obj.contacts = mwbs.Contacts(obj.contact_config.foot_print, obj.robot.NDOF, obj.contact_config.friction_coefficient, obj.contact_config.num_in_contact_frames, obj.physics_config.TIME_STEP, obj.ifFieldExists('contact_config','max_consecuitive_fail'), obj.ifFieldExists('contact_config','useFrictionalImpact'), obj.ifFieldExists('contact_config','useDiscreteContact'), obj.ifFieldExists('contact_config','useQPOASES'));
            obj.state = mwbs.State(obj.physics_config.TIME_STEP);
            obj.state.set(obj.robot_config.initialConditions.w_H_b, obj.robot_config.initialConditions.s, ...
                obj.robot_config.initialConditions.base_pose_dot, obj.robot_config.initialConditions.s_dot);
        end

        function [w_H_b, s, base_pose_dot, s_dot, wrench_in_contact_frames, kinDynOut] = stepImpl(obj, generalized_ext_wrench, torque, motor_inertias)
            % Implement algorithm.

            % Compute the contact quantites and the velocity after a possible impact
            [generalized_total_wrench, wrench_in_contact_frames, base_pose_dot, s_dot] = ...
                obj.contacts.compute_contact_closed_chain(obj.robot, torque, generalized_ext_wrench, motor_inertias, obj.state.base_pose_dot, obj.state.s_dot,obj);
            
            % Update the velocity vector of the robot
            obj.state.set_velocity(base_pose_dot, s_dot);
            obj.robot.set_robot_velocity(base_pose_dot, s_dot);
            
            % Compute the robot acceleration vector
            [base_pose_ddot, s_ddot] = obj.robot.forward_dynamics(torque, generalized_total_wrench, motor_inertias, obj);
            
            % Integrate the robot acceleration vector
            [w_H_b, s, base_pose_dot, s_dot] = obj.state.ode_step(base_pose_ddot, s_ddot);
            
            % Update the robot states
            obj.robot.set_robot_state(w_H_b, s, base_pose_dot, s_dot);
            
            % Get feet contact state
            links_in_contact = obj.contacts.get_feet_contact_state(obj.contact_config.num_in_contact_frames);
            
            % Compute some kinematic and dynamic variables
            kinDynOut.w_H_b = w_H_b;
            kinDynOut.s = s;
            kinDynOut.nu = [base_pose_dot;s_dot];
            kinDynOut.w_H_feet_sole = obj.robot.get_inContactWithGround_H();
            kinDynOut.J_feet_sole = obj.robot.get_inContactWithGround_jacobians();
            kinDynOut.JDot_feet_sole_nu = obj.robot.get_inContactWithGround_JDot_nu();
            kinDynOut.M = obj.robot.get_mass_matrix(motor_inertias,obj);
            kinDynOut.h = obj.robot.get_bias_forces();
            kinDynOut.motorGrpI = zeros(size(s));
            kinDynOut.fc = wrench_in_contact_frames;
            kinDynOut.nuDot = [base_pose_ddot;s_ddot];
            if (obj.contact_config.num_in_contact_frames == 0)
                kinDynOut.feet_in_contact  = false;
            else
                kinDynOut.feet_in_contact = links_in_contact;
            end
        end

        function resetImpl(obj)

        end

        function field_value = ifFieldExists(obj, struct_name, field_name)
            if isfield(obj.(struct_name),field_name)
                field_value = obj.(struct_name).(field_name);
            else
                field_value = [];
            end
        end
        
        function [out, out2, out3, out4, out5, out6] = getOutputSizeImpl(obj)
            % Return size for each output port
            out  = [4 4];                              % homogeneous matrix dim
            out2 = [double(obj.robot_config.N_DOF),1]; % joints position vector dim
            out3 = [6 1];                              % base velocity vector dim
            out4 = [double(obj.robot_config.N_DOF),1]; % joints velocity vector dim
            out5 = [6*length(obj.robot_config.robotFrames.IN_CONTACT_WITH_GROUND) 1]; % wrench of the frames interacting with the ground vector dim
            out6 = 1;
        end

        function [out, out2, out3, out4, out5, out6] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out  = "double";
            out2 = "double";
            out3 = "double";
            out4 = "double";
            out5 = "double";
            out6 = obj.OutputBusName;
        end

        function [out, out2, out3, out4, out5, out6] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out  = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;
        end

        function [out, out2, out3, out4, out5, out6] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out  = true;
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
                'simFunc_qpOASES_impact_phase_I',...
                'simFunc_qpOASES_impact_phase_II',...
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
