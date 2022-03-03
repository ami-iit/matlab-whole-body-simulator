classdef Contacts < handle
    % CONTACTS The Contact class handles the computation of the contact forces and the impact.
    %
    % Contacts Methods:
    %   compute_contact - computes the wrench and the state velocity after a (possible) impact

    properties (Constant)
        num_vertices = 4;
        max_consecuitive_fail = 40;
        useOSQP=false; % Use the OSQP solver instead of quadprog for the optim. prob. computing the reaction forces at the feet
        useQPOASES=true;
    end
    
    properties (SetAccess = immutable)
        foot_print; % the coordinates of the vertices
        S; % selector matrix for the robot torque
        mu; % friction coefficient
    end
    
    properties (Access = private)
        was_in_contact; % this vector says if the vertex was in contact (1) or not (0)
        is_in_contact; % this vector says if the vertex is in contact (1) or not (0)
        A; Ax_Lb; Ax_Ub; Aeq; beq; ulb; % matrices used in the optimization problem
        osqpProb; % OSQP solver object
        firstSolverIter; % For handing osqp.setp and osqp.update
        fail_counter = 0; % For counting the consecuitive fails of the solver
    end

    methods

        function obj = Contacts(foot_print, robot, friction_coefficient, in_contact_frames)
            %CONTACTS The Contact class needs the coordinates of the vertices of the foot
            % Arguments
            %   foot_print - the coordinates of every vertex in xyz
            %   robot - the robot model
            %   frinction coefficient - the coefficient that defines the (simplified) friction cone
            if (~isequal(length(foot_print), length(in_contact_frames)))
                error('The foot print is a cell array composed of the foot print matrix of all the frames that are in contact with the ground');
            else
                for counter = 1 : length(in_contact_frames)
                    if (~isequal(size(foot_print{1}),[3,4]))
                        error('The foot print matrix for each contact frame is represented with a matrix composed by 4 columns in which every column is the set of xyz coordinates')
                    end
                end
            end

            obj.foot_print = foot_print;
            obj.S = [zeros(6, robot.NDOF); ...
                     eye(robot.NDOF)];
            obj.mu = friction_coefficient;
            
            obj.is_in_contact = ones(4*length(in_contact_frames),1);
            obj.was_in_contact = ones(4*length(in_contact_frames),1);
            
            % initialize the setup/update step of the osqp solver
            obj.firstSolverIter = true;
            
            obj.prepare_optimization_matrix(length(in_contact_frames));
            
        end

        [generalized_total_wrench, wrench_left_foot, wrench_right_foot, base_pose_dot, s_dot] = ...
                compute_contact(obj, robot, torque, generalized_ext_wrench, motorInertias, base_pose_dot, s_dot,obj_step_block);
        
        [generalized_total_wrench, wrench_inContact_frames, base_pose_dot, s_dot] = ...
                compute_contact_closedChain(obj, robot, torque, generalized_ext_wrench, motorInertias, base_pose_dot, s_dot,obj_step_block);
        
        links_in_contact = getFeetContactState(obj, num_inContact_frames);

    end

    methods (Access = private)

        [J_print, JDot_nu_print] = compute_J_and_JDot_nu_inContact_frames(obj, robot, num_inContact_frames);
        
        [J_diff_splitPoint, JDot_diff_nu_splitPoint] = compute_J_and_JDot_nu_splitPoint(obj, robot);

        contact_points = compute_contact_points(obj, robot, num_inContact_frames);

        [base_pose_dot, s_dot] = compute_velocity(obj, M, G, robot, base_pose_dot, s_dot, closed_chains, num_inContact_frames);
        
        P_damped_psudo_inverse = compute_damped_psudo_inverse(obj,P,damped_coefficient);

        free_acceleration = compute_free_acceleration(obj, M, h, torque, generalized_ext_wrench);

        free_contact_diff_acceleration = compute_free_contact_diff_acceleration(obj, G, free_acceleration, P);

        forces = compute_unilateral_linear_contact(obj, M, h, J_inContact, J_diff_splitPoint, JDot_nu_inContact, JDot_diff_nu_splitPoint, torque, contact_point, closedChains, generalized_ext_wrench, num_inContact_frames);

        wrench_inContactFrames = compute_contact_wrench_in_sole_frames(obj, contact_forces, robot, num_inContact_frames);

        prepare_optimization_matrix(obj,num_inContact_frames);
        
    end

end
