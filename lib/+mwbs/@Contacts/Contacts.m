classdef Contacts < handle
    % CONTACTS: The Contact class handles the computation of the contact forces and the impact.
    %
    % METHODS:
    %         - Contacts: initializes the Contact class
    %         - compute_contact: computes the wrench and the robot velocity vector after a (possible) impact for an open chain robot
    %         - compute_contact_closed_chain_: computes the wrench and the state velocity after a (possible) impact for a robot with (possibile) closed chains
    %         - get_feet_contact_state: determines if each foot is in contact with the ground
    %         - compute_J_and_JDot_nu_in_contact_frames: computes J and JDot x nu for the vertices of the feet
    %         - compute_J_and_JDot_nu_split_point: computes JTilde and JTildeDot for the splited points in the (possible) closed chains
    %         - compute_contact_points: determines if each vertex of the feet are in contact with the ground or not
    %         - compute_velocity: computes the robot velocity vector after a (possible) impact
    %         - compute_damped_psudo_inverse: computes the inverse of a matrix using the damped psudo inverse method
    %         - compute_free_acceleration: computes the free acceleration vector of the robot
    %         - compute_free_contact_diff_acceleration: computes the acceleration of the feet and the difference of the acceleration of the split points with NO contact forces
    %         - compute_unilateral_linear_contact: computes the pure forces acting on the feet vertices and the internal force/torques applied to the split points the (possible) closed chains
    %         - compute_unilateral_linear_impact: computes the pure impulsive forces acting on the feet vertices and the impulsive internal force/torques applied to the split points the (possible) closed chains in a (possible) impact
    %         - compute_contact_wrench_in_sole_frames: trasforms the pure forces on foot vertices in wrench in sole frames
    %         - prepare_optimization_matrix: prepares the matrices used by the optimization problem solver.
    %
    % AUTHORS: Venus Pasandi, Nuno Guedelha
    % 
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    % 
    %          PLACE AND DATE: <Genoa, March 2022>

    properties (Constant)
        num_vertices = 4;            % The number of vertices in each foot
        max_consecuitive_fail = 40;  % The maximum allowable consecuitive fail in computing the reaction forces in the feet
        useOSQP = false;             % Use the OSQP solver instead of quadprog for the optim. prob. computing the reaction forces at the feet
        useQPOASES = true;           % Use the QPOASES solver instead of quadprog for the optim. prob. computing the reaction forces at the feet
        useFrictionalImpact = false  % Use the frictional impact model instead of the frictionless one
        useDiscreteContact = false   % Use the discrete contact model instead of the continuous one
    end
    
    properties (SetAccess = immutable)
        foot_print;  % The coordinates of the vertices
        S;           % The selector matrix for the robot torque
        mu;          % The friction coefficient
        dt;          % The time step for the discrete contact model
    end
    
    properties (Access = private)
        was_in_contact;                  % This vector says if the vertex was in contact (1) or not (0)
        is_in_contact;                   % This vector says if the vertex is in contact (1) or not (0)
        A; Ax_Lb; Ax_Ub; Aeq; beq; ulb;  % The matrices used in the optimization problem
        osqpProb;                        % The OSQP solver object
        firstSolverIter;                 % For handing osqp.setp and osqp.update
        fail_counter = 0;                % For counting the consecuitive fails of the solver
    end

    methods

        function obj = Contacts(foot_print, NDOF, friction_coefficient, num_in_contact_frames, dt)
            % CONTACTS: This function initializes the Contact class
            % INPUTE:
            %         - foot_print:            [(3m) x k] The coordinates of every vertex in xyz
            %         - NDOF:                  [SCALAR]   The number of the joints of the robot
            %         - friction_coefficient:  [SCALAR] The friction coefficient of the ground
            %         - num_in_contact_frames: [SCALAR] The number of the frames that can interact with the ground
            %         - dt:                    [SCALAR] The time step for the discrete contact model
        
            if (~isequal(length(foot_print), num_in_contact_frames))
                error('The foot print is a cell array composed of the foot print matrix of all the frames that are in contact with the ground');
            else
                for counter = 1 : num_in_contact_frames
                    if (~isequal(size(foot_print{1}),[3,4]))
                        error('The foot print matrix for each contact frame is represented with a matrix composed by 4 columns in which every column is the set of xyz coordinates')
                    end
                end
            end

            obj.foot_print = foot_print;
            obj.S = [zeros(6, NDOF); ...
                     eye(NDOF)];
            obj.mu = friction_coefficient;
            obj.dt = dt;
            
            obj.is_in_contact = ones(4*num_in_contact_frames,1);
            obj.was_in_contact = ones(4*num_in_contact_frames,1);
            
            % initialize the setup/update step of the osqp solver
            obj.firstSolverIter = true;
            
            obj.prepare_optimization_matrix(num_in_contact_frames);
            
        end

        function [generalized_total_wrench, wrench_left_foot, wrench_right_foot, base_pose_dot, s_dot] = ...
                compute_contact(obj, robot, torque, generalized_ext_wrench, motor_inertias, base_pose_dot, s_dot,obj_step_block)
            
                        % compute_contact Computes the contact forces and the configuration velocity after a (possible) impact
            % INPUTS: - robot: instance of the Robot object
            %         - torque: joint torques
            %         - generalized_ext_wrench: wrench transformed using the Jacobian relative to the application point
            %         - base_pose_dot, s_dot: configuration velocity
            % OUTPUTS: - generalized_total_wrench: the sum of the generalized_ext_wrench and the generalized contact wrench
            %          - wrench_left_foot, wrench_right_foot: the wrench in sole frames
            %          - base_pose_dot, s_dot: configuration velocity, changed in the case of an impact with the ground

            % collecting robot quantities
            h = robot.get_bias_forces();
            M = robot.get_mass_matrix(motor_inertias,obj_step_block);
            [J_feet, JDot_nu_feet] = obj.compute_J_and_JDot_nu_inContact_frames(robot,length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
            % compute the vertical distance of every vertex from the ground
            contact_points = obj.compute_contact_points(robot, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
            % computes a 3 * num_total_vertices vector containing the pure forces acting on every vertes
            contact_forces = obj.compute_unilateral_linear_contact(M, h, J_feet, [], JDot_nu_feet, [], torque, contact_points, 0, generalized_ext_wrench, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
            % transform the contact in a wrench acting on the robot
            generalized_contact_wrench = J_feet' * contact_forces;
            % sum the contact wrench to the external one
            generalized_total_wrench = generalized_ext_wrench + generalized_contact_wrench;
            % compute the wrench in the sole frames, in order to simulate a sensor mounted onto the sole frame
            [wrench_left_foot, wrench_right_foot] = obj.compute_contact_wrench_in_sole_frames(contact_forces, robot, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
            % compute the configuration velocity - same, if no impact - discontinuous in case of impact
            [base_pose_dot, s_dot] = obj.compute_velocity(M, J_feet, robot, base_pose_dot, s_dot, obj_step_block.robot_config.closedChains, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));

            % update the contact log
            obj.was_in_contact = obj.is_in_contact;

        end
        
        function [generalized_total_wrench, wrench_inContact_frames, base_pose_dot, s_dot] = ...
                compute_contact_closed_chain_(obj, robot, torque, generalized_ext_wrench, motor_inertias, base_pose_dot, s_dot, obj_step_block)
            
            [generalized_total_wrench, wrench_inContact_frames, base_pose_dot, s_dot] = compute_contact_closed_chain(obj, robot, torque, generalized_ext_wrench, motor_inertias, base_pose_dot, s_dot, obj_step_block);
        end
        
        links_in_contact = get_feet_contact_state(obj, num_in_contact_frames);

    end

    methods (Access = private)

        % computes the Jacobian and J_dot_nu relative to the vertices (Not the sole frames!)
        [J_print, JDot_nu_print] = compute_J_and_JDot_nu_in_contact_frames(obj, robot, num_inContact_frames);
        
        % computes the JTilde and JDOTTilde * nu fot the spilit points in the (possible) closed chains
        [J_diff_splitPoint, JDot_diff_nu_splitPoint] = compute_J_and_JDot_nu_split_points(obj, robot);

        % computes the vertical position of every vertex and determine if each vertex is in contact with the ground or not
        contact_points = compute_contact_points(obj, robot, num_inContact_frames);

        % computes the robot velocity vector after a (possible) impact
        [base_pose_dot, s_dot] = compute_velocity(obj, M, G, base_pose_dot, s_dot, closed_chains, num_inContact_frames, contact_point)
        
        % computes the damped pseudo inverse of the matrix P
        P_damped_psudo_inverse = compute_damped_psudo_inverse(obj,P,damped_coefficient);

        % computes the robot acceleration with NO contact forces
        free_acceleration = compute_free_acceleration(obj, M, h, torque, generalized_ext_wrench);

        % computes the acceleration of the feet and the difference of the acceleration of the split points in the (possible) closed chains with NO contact forces
        free_contact_acceleration = compute_free_contact_acceleration(obj, G, free_acceleration, P);

        % computes the pure forces acting on the feet vertices and the
        % internal wrenches applied to the split points in the (possible) closed chains
        forces = compute_unilateral_linear_contact(obj, M, h, J_in_contact, J_diff_split_points, JDot_nu_in_contact, JDot_diff_nu_split_points, torque, contact_point, num_closed_chains, generalized_ext_wrench, num_in_contact_frames);
        
        % computes the impulsive pure forces acting on the feet vertices and the impulsive internal wrenches applied to the split points in the (possible) closed chain
        impulsive_forces = compute_unilateral_linear_impact(obj, M, nu, J_in_contact, J_diff_split_points, contact_point, num_closed_chains, num_in_contact_frames, map_vertices_new_contact);
        
        % trasforms the pure forces on foot vertices in wrench in sole frames
        wrench_in_contact_frames = compute_contact_wrench_in_sole_frames(obj, contact_forces, robot, num_in_contact_frames);

        % Fills the matrix relating to the unilateral and friction cone constraints used by the optimization problem solver.
        prepare_optimization_matrix(obj,num_in_contact_frames);
        
    end

end
