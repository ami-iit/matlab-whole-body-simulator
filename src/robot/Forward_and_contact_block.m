classdef Forward_and_contact_block < matlab.System & matlab.system.mixin.Propagates
    % untitled2 Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.
    
    % Public, tunable properties
    properties (Nontunable)
        config
    end
    
    properties(DiscreteState)
        
    end
    
    % Pre-computed constants
    properties(Access = private)
        KinDynModel;
        g = [0,0,-9.81]; % gravity vector
        mass_matrix_iDyn = iDynTree.MatrixDynSize();
        JLeft_Jac_iDyntree;
        JRight_Jac_iDyntree;
        JDot_nu_LFOOT_iDyntree;
        JDot_nu_RFOOT_iDyntree;
        LFOOT_frameID, RFOOT_frameID;
        free_floating_generalized_torque;
        vertex;
        is_in_contact = ones(8,2);
        jointOrder={'torso_pitch','torso_roll','torso_yaw',...
            'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
            'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
            'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
            'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
        generalized_contact_wrench
    end
    
    methods(Access = protected)
        function setupImpl(obj, A)
            obj.prepareRobot();
            obj.build_foot_print();
            [~, obj.generalized_contact_wrench] = obj.compute_contact_forces(zeros(23,1), zeros(29,1));
            disp(obj.KynDynModel3.kinDynComp.getFrameIndex('l_sole'))
            disp(A)
            
        end
        
        function [w_H_b, joints_position, basePose_dot, joints_velocity, wrench_left_foot, wrench_right_foot] = stepImpl(obj, joints_torque, generalized_ext_wrench)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            [contact_forces, obj.generalized_contact_wrench] = obj.compute_contact_forces(joints_torque, generalized_ext_wrench);
            generalized_total_wrench = obj.generalized_contact_wrench + generalized_ext_wrench;
            
            [H_LFOOT, H_RFOOT] = obj.get_feet_hom();
            mass_matrix = obj.get_mass_matrix();
            bias_forces = obj.get_bias_forces;
            [wrench_left_foot, wrench_right_foot] = compute_contact_wrench_in_sole_frames(contact_forces, H_LFOOT, H_RFOOT, obj.vertex);
            
            [dot_dot_basePose, dot_dot_s] = forward_dynamics(mass_matrix, bias_forces, joints_torque, generalized_total_wrench, obj.config);
            
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function num = getNumOutputsImpl(~)
            % Define total number of outputs for system with optional
            % outputs
            num = 6;
        end
        
        function [out,out2,out3,out4,out5,out6] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [4 4];
            out2 = [obj.config.N_DOF, 1];
            out3 = [6 1];
            out4 = [obj.config.N_DOF, 1];
            out5 = [6 1];
            out6 = [6 1];
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out,out2,out3,out4,out5,out6] = getOutputDataTypeImpl(~)
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

        function [out,out2,out3,out4,out5,out6] = isOutputComplexImpl(~)
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

        function [out,out2,out3,out4,out5,out6] = isOutputFixedSizeImpl(~)
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
        
        function prepareRobot(obj)
            % Main variable of iDyntreeWrappers used for many things including updating
            % robot position and getting world to frame transforms
            obj.KinDynModel = iDynTreeWrappers.loadReducedModel(obj.jointOrder, 'root_link', ...
                obj.config.modelPath, obj.config.fileName, false);
            
            % Set initial position of the robot
            iDynTreeWrappers.setRobotState(obj.KinDynModel, obj.config.initialConditions.world_H_base, obj.config.initialConditions.joints, ...
                obj.config.initialConditions.base_velocity, obj.config.initialConditions.joints_velocity, obj.g);
            obj.JLeft_Jac_iDyntree  = iDynTree.MatrixDynSize(6,obj.KinDynModel.NDOF+6);
            obj.JRight_Jac_iDyntree = iDynTree.MatrixDynSize(6,obj.KinDynModel.NDOF+6);
            obj.LFOOT_frameID = obj.KinDynModel.kinDynComp.getFrameIndex('l_sole');
            obj.RFOOT_frameID = obj.KinDynModel.kinDynComp.getFrameIndex('r_sole');
            obj.free_floating_generalized_torque = iDynTree.FreeFloatingGeneralizedTorques(obj.KinDynModel.kinDynComp.model);
            
        end
        
        function mass_matrix = get_mass_matrix(obj)
            obj.KinDynModel.kinDynComp.getFreeFloatingMassMatrix(obj.mass_matrix_iDyn);
            mass_matrix = obj.mass_matrix_iDyn.toMatlab;
        end
        
        function bias_forces = get_bias_forces(obj)
            obj.KinDynModel.kinDynComp.generalizedBiasForces(obj.free_floating_generalized_torque);
            h_b = obj.free_floating_generalized_torque.baseWrench.toMatlab;
            h_s = obj.free_floating_generalized_torque.jointTorques.toMatlab;
            bias_forces = [h_b; h_s];
        end
        
        function [JLeft_frameJac, JRight_frameJac] = get_feet_jacobians(obj)
            obj.KinDynModel.kinDynComp.getFrameFreeFloatingJacobian(obj.LFOOT_frameID, obj.JLeft_Jac_iDyntree);
            obj.KinDynModel.kinDynComp.getFrameFreeFloatingJacobian(obj.RFOOT_frameID, obj.JRight_Jac_iDyntree);
            JLeft_frameJac  = obj.JLeft_Jac_iDyntree.toMatlab;
            JRight_frameJac = obj.JRight_Jac_iDyntree.toMatlab;
        end
        
        function [JDot_nu_LFOOT, JDot_nu_RFOOT] = get_feet_JDot_nu(obj)
            obj.JDot_nu_LFOOT_iDyntree = obj.KinDynModel.kinDynComp.getFrameBiasAcc('l_sole');
            obj.JDot_nu_RFOOT_iDyntree = obj.KinDynModel.kinDynComp.getFrameBiasAcc('r_sole');
            JDot_nu_LFOOT = obj.JDot_nu_LFOOT_iDyntree.toMatlab;
            JDot_nu_RFOOT = obj.JDot_nu_RFOOT_iDyntree.toMatlab;
        end
        
        function [H_LFOOT, H_RFOOT] = get_feet_hom(obj)
            H_LFOOT = iDynTreeWrappers.getWorldTransform(obj.KinDynModel,'l_sole');
            H_RFOOT = iDynTreeWrappers.getWorldTransform(obj.KinDynModel,'r_sole');
        end
        
        function build_foot_print(obj)
            obj.vertex(:,1) = [-0.06;   0.04; 0];
            obj.vertex(:,2) = [ 0.11;   0.04; 0];
            obj.vertex(:,3) = [ 0.11; -0.035; 0];
            obj.vertex(:,4) = [-0.06; -0.035; 0];
        end
        
        function [contact_forces, generalized_contact_wrench] = compute_contact_forces(obj, joints_torque, generalized_ext_wrench)
            bias_forces = obj.get_bias_forces();
            mass_matrix = obj.get_mass_matrix();
            [H_LFOOT, H_RFOOT] = obj.get_feet_hom();
            [JLeft_frameJac, JRight_frameJac] = obj.get_feet_jacobians();
            [JDot_nu_LFOOT, JDot_nu_RFOOT] = obj.get_feet_JDot_nu();
            
            for ii=1:4
                j = (ii-1)*3 + 1;
                J_left_vertex(j:j+2, :) =  JLeft_frameJac(1:3,:) - wbc.skew(H_LFOOT(1:3,1:3)*obj.vertex(:,ii))*JLeft_frameJac(4:6,:);
                JDot_nu_left_vertex(j:j+2, :) = JDot_nu_LFOOT(1:3,:) - wbc.skew(H_LFOOT(1:3,1:3)*obj.vertex(:,ii))*JDot_nu_LFOOT(4:6,:);
                J_right_vertex(j:j+2, :) =  JRight_frameJac(1:3,:) - wbc.skew(H_RFOOT(1:3,1:3)*obj.vertex(:,ii))*JRight_frameJac(4:6,:);
                JDot_nu_right_vertex(j:j+2, :) = JDot_nu_RFOOT(1:3,:) - wbc.skew(H_RFOOT(1:3,1:3)*obj.vertex(:,ii))*JDot_nu_RFOOT(4:6,:);
            end
            
            J_feet = [J_left_vertex; J_right_vertex];
            JDot_nu_feet = [JDot_nu_left_vertex; JDot_nu_right_vertex];
            
            left_z_vertex = zeros(4,1);
            right_z_vertex = zeros(4,1);
            
            for ii=1:4
                left_z = H_LFOOT*[obj.vertex(:,ii);1];
                left_z_vertex(ii) = left_z(3);
                right_z = H_RFOOT*[obj.vertex(:,ii);1];
                right_z_vertex(ii) = right_z(3);
                obj.is_in_contact(ii, 2) = left_z_vertex(ii) <= 0;
                obj.is_in_contact(ii + 4, 2) = right_z_vertex(ii) <= 0;
            end
            contact_points = [left_z_vertex', right_z_vertex']';
            
            free_acceleration = compute_free_acceleration(mass_matrix, bias_forces, joints_torque, generalized_ext_wrench, obj.config);
            
            contact_forces = compute_unilateral_linear_contact(J_feet, mass_matrix, free_acceleration, JDot_nu_feet, contact_points, obj.config);
            
            generalized_contact_wrench = J_feet'*contact_forces;
            
        end
        
        function free_acceleration = compute_free_acceleration(mass_matrix, bias_forces, joints_torque, generalized_jet_wrench, Config)
            % returns the system acceleration with NO contact forces
            % dot{v} = inv{M}(S*tau + external_forces - h)
            
            S = [zeros(6, Config.N_DOF);...
                eye(Config.N_DOF)];
            free_acceleration = mass_matrix\(S*joints_torque + generalized_jet_wrench - bias_forces);
        end
        
        function forces  = compute_unilateral_linear_contact(J_feet, mass_matrix, free_acceleration, JDot_nu_feet, contact_point, Config)
            % returns the pure forces acting on the feet vertices
            free_contact_acceleration = compute_free_contact_acceleration(J_feet, free_acceleration, JDot_nu_feet);
            H = J_feet*(mass_matrix\J_feet');
            f = free_contact_acceleration;
            mu = Config.friction_coefficient;
            A = zeros(24+16,24);
            b = zeros(24+16,1);
            Aeq = zeros(8, 24);
            beq = zeros(8,1);
            for i=1:8
                j = (i-1)*3 + 1;
                A(j:j+2, j:j+2) = [1, 0, -mu;...
                    0, 1, -mu;...
                    0, 0, -1];
                Aeq(i, i*3) = contact_point(i)>=0;
            end
            
            for i=9:16
                j = (i-1)*2 + 9;
                jj = (i-8-1)*3 + 1;
                A(j:j+1, jj:jj+2) = [-1, 0, -mu;...
                    0, -1, -mu];
                %     Aeq(i, i*3) = contact_point(i)>=0
            end
            options = optimoptions('quadprog','Algorithm','active-set');
            %             forces = quadprog(H,f,A,b,Aeq,beq, [], [], 100*ones(24,1), options);
            forces = quadprog(H,f,A,b,Aeq,beq);
        end
        
        function free_contact_acceleration = compute_free_contact_acceleration(J_feet, free_acceleration, JDot_nu_feet)
            free_contact_acceleration = J_feet*free_acceleration + JDot_nu_feet;
        end
        
        function [dot_dot_basePose, dot_dot_s] =  forward_dynamics(mass_matrix, bias_forces, torques, generalized_total_wrench, Config)
            % compute forward dynamics as a first order system
            % M \dot{v} + h = S*tau + external_forces
            % state x = [x1, x2]
            % dot{x} = [x2;...
            %           dot{v}];
            % dot{v} = inv{M}(S*tau + external_forces - h)
            
            S = [zeros(6,Config.N_DOF); eye(Config.N_DOF)];
            ddot = mass_matrix\(S*torques + generalized_total_wrench - bias_forces);
            dot_dot_basePose = ddot(1:6);
            dot_dot_s = ddot(7:end);
        end
        
        function [wrench_left_foot, wrench_right_foot] = compute_contact_wrench_in_sole_frames(contact_forces, H_LFOOT, H_RFOOT, vertex)
            % trasforms the pure forces on foot vertices in wrench in sole frames
            R_LFOOT = H_LFOOT(1:3,1:3);
            R_RFOOT = H_RFOOT(1:3,1:3);
            
            wrench_left_foot = zeros(6,1);
            wrench_right_foot = zeros(6,1);
            
            contact_forces_left = contact_forces(1:12);
            contact_forces_right = contact_forces(13:24);
            
            for i=1:4
                j = (i-1)*3 + 1;
                wrench_left_foot(1:3)  = wrench_left_foot(1:3) + R_LFOOT'*contact_forces_left(j:j+2);
                wrench_left_foot(4:6)  = wrench_left_foot(4:6) - wbc.skew(vertex(:,i))*(R_LFOOT'*contact_forces_left(j:j+2));
                wrench_right_foot(1:3) = wrench_right_foot(1:3) + R_RFOOT'*contact_forces_right(j:j+2);
                wrench_right_foot(4:6) = wrench_right_foot(4:6) - wbc.skew(vertex(:,i))*(R_RFOOT'*contact_forces_left(j:j+2));
            end
        end
        
    end
end
