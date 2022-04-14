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
        useOSQP = false;                % Use the OSQP solver instead of quadprog for the optim. prob. computing the reaction forces at the feet
        useQPOASES = true;              % Use the QPOASES solver instead of quadprog for the optim. prob. computing the reaction forces at the feet
    end
    
    properties (SetAccess = immutable)
        S;                           % The selector matrix for the robot torque
        mu;                          % The friction coefficient
        dt;                          % The time step for the discrete contact model
        max_consecutive_failures = 10;  % The maximum allowable consecuitive fail in computing the reaction forces in the feet
        useFrictionalImpact = false;    % Use the frictional impact model instead of the frictionless one
        useDiscreteContact = false;     % Use the discrete contact model instead of the continuous one
        useCircularFeet = false;        % Determines of the feet are circular
    end
    
    properties (Access = private)
        was_in_contact;                  % This vector says if the vertex was in contact (1) or not (0)
        is_in_contact;                   % This vector says if the vertex is in contact (1) or not (0)
        A; Ax_Lb; Ax_Ub; Aeq; beq; ulb;  % The matrices used in the optimization problem
        osqpProb;                        % The OSQP solver object
        firstSolverIter;                 % For handing osqp.setp and osqp.update
        fail_counter = 0;                % For counting the consecuitive fails of the solver
        foot_print;                      % The coordinates of the vertices
    end
    
    methods

        function obj = Contacts(foot_print, NDOF, friction_coefficient, num_in_contact_frames, num_vertices, dt, max_consecutive_failures, useFrictionalImpact, useDiscreteContact, useCircularFeet)
            % CONTACTS: This function initializes the Contact class
            % INPUTE:
            %         - foot_print:            [(3m) x k] The coordinates of every vertex in xyz
            %         - NDOF:                  [SCALAR]   The number of the joints of the robot
            %         - friction_coefficient:  [SCALAR]   The friction coefficient of the ground
            %         - num_in_contact_frames: [SCALAR]   The number of the frames that can interact with the ground
            %         - dt:                    [SCALAR]   The time step for the discrete contact model
            %         - max_consecutive_failures: [SCALAR]   The maximum allowable fail in the computation of the reaction forces
            %         - useFrictionalImpact:   [BOOLEAN]  Determine if the frictional impact model is used instead of the frictionless unpact model
            %         - useDiscreteContact:    [BOOLEAN]  Determine if the discrete contact model is used instead of the continuous model
            %         - useQPOASES:            [BOOLEAN]  Determine if QPOQSES solver is used for the contact and impact models   
                
            obj.S = [zeros(6, NDOF); ...
                eye(NDOF)];
            obj.mu = friction_coefficient;
            obj.dt = dt;
            
            if ~isempty(max_consecutive_failures)
               obj.max_consecutive_failures = max_consecutive_failures;
            end
            if ~isempty(useFrictionalImpact)
                obj.useFrictionalImpact = useFrictionalImpact;
            end
            if ~isempty(useDiscreteContact)
                obj.useDiscreteContact = useDiscreteContact;
            end
            if ~isempty(useCircularFeet)
                obj.useCircularFeet = useCircularFeet;
            end
            
            obj.is_in_contact = ones(num_vertices * num_in_contact_frames,1);
            obj.was_in_contact = ones(num_vertices * num_in_contact_frames,1);
            
            % initialize the setup/update step of the osqp solver
            obj.firstSolverIter = true;
            
            obj.prepare_foot_print (num_in_contact_frames, num_vertices, foot_print);
            obj.prepare_optimization_matrix(num_in_contact_frames, num_vertices);
            
        end
        
        function [generalized_total_wrench, wrench_in_contact_frames, base_pose_dot, s_dot] = ...
                compute_contact(obj, robot, torque, generalized_ext_wrench, motor_inertias, base_pose_dot, s_dot, obj_step_block)
            
            % compute_contact Computes the contact forces and the configuration velocity after a (possible) impact
            % INPUTS: - robot: instance of the Robot object
            %         - torque: joint torques
            %         - generalized_ext_wrench: wrench transformed using the Jacobian relative to the application point
            %         - base_pose_dot, s_dot: configuration velocity
            % OUTPUTS: - generalized_total_wrench: the sum of the generalized_ext_wrench and the generalized contact wrench
            %          - wrench_left_foot, wrench_right_foot: the wrench in sole frames
            %          - base_pose_dot, s_dot: configuration velocity, changed in the case of an impact with the ground

            num_vertices = obj_step_block.num_vertices;
            num_in_contact_frames = obj_step_block.num_in_contact_frames; % The number of the links interacting with the ground
            
            % collecting robot quantities
            h = robot.get_bias_forces();
            M = robot.get_mass_matrix(motor_inertias,obj_step_block);
            [J_feet, JDot_nu_feet] = obj.compute_J_and_JDot_nu_in_contact_frames(robot, num_in_contact_frames, num_vertices);
            
            % compute the vertical distance of every vertex from the ground
            contact_points = obj.compute_contact_points(robot, num_in_contact_frames, num_vertices);
            
            % computes a 3 * num_total_vertices vector containing the pure forces acting on every vertes
            contact_forces = obj.compute_unilateral_linear_contact(M, h, J_feet, [], JDot_nu_feet, [], torque, contact_points, 0, generalized_ext_wrench, num_in_contact_frames, num_vertices, base_pose_dot, s_dot);
            
            % transform the contact in a wrench acting on the robot
            generalized_contact_wrench = J_feet' * contact_forces;
            
            % sum the contact wrench to the external one
            generalized_total_wrench = generalized_ext_wrench + generalized_contact_wrench;
            
            % compute the wrench in the sole frames, in order to simulate a sensor mounted onto the sole frame
            wrench_in_contact_frames = obj.compute_contact_wrench_in_sole_frames(contact_forces, robot, num_in_contact_frames, num_vertices);
            
            % compute the configuration velocity - same, if no impact - discontinuous in case of impact
            [base_pose_dot, s_dot, ~] = obj.compute_velocity(M, J_feet, base_pose_dot, s_dot, 0, num_in_contact_frames, contact_points, num_vertices);
            
            % update the contact log
            obj.was_in_contact = obj.is_in_contact;
            
        end
        
        function [generalized_total_wrench, wrench_in_contact_frames, base_pose_dot, s_dot] = ...
                compute_contact_closed_chain(obj, robot, torque, generalized_ext_wrench, motor_inertias, base_pose_dot, s_dot, obj_step_block)
            
            %     COMPUTE_CONTACT_CLOSED_CHAIN: Computes the contact forces, the internal wrenches of the spilit points in the (possible) closed chains, and the configuration velocity after a (possible) impact
            %
            %     PROCEDURE: For modeling the dynamics of a robot having some closed
            %             chains, we open each chain by breaking a link in the chain.
            %             Then, we apply a constraint wrench to the broken point that
            %             the two parts of the broken link become connected. In this
            %             way, the equation of motion for a floating base robot having
            %             some closed chains can be written as
            %
            %             M vDot + h = B u + Fe + Jc' Fc + Ji Fi
            %
            %             where
            %
            %             u is the joint torques, Fe is the external wrenches, Fc is
            %             the pure forces applied to the contact points, and Fi is the
            %             internal wrenches in the spilit/broken points.
            %
            %             To compute Fc, Fi and the effects of a (possible) impact,
            %
            %             1. Determine if each vertex of the feet is in contact with
            %             the ground or not by analysing their vertical height,
            %
            %             2. Compute the robot velocity vector if an impact happens,
            %
            %             3. Compute Fc and Fi by using the minimum dissipation
            %                principle.
            %
            %             4. Computes the resultant wrench applied to the sole of feet
            %
            %
            %     INPUTS:
            %             - robot:                                  instance of the Robot object
            %             - torque:                 [N x 1]         The joint torques
            %             - generalized_ext_wrench: [(N+6) x 1]     The external wrench transformed using the Jacobian relative to the application point
            %             - motor_inertias:         [(N+6) x (N+6)] The inertia matrix of the motors
            %             - base_pose_dot:          [6 x 1]         The velocity vector of the base link
            %             - s_dot:                  [N x 1]         The configuration velocity vector
            %             - obj_step_block:
            %
            %     OUTPUTS:
            %             - generalized_total_wrench: [(N+6) x (N+6)] The sum of the generalized_ext_wrench and the generalized contact wrench and the internal wrench on the (possible) split points
            %             - wrench_in_contact_frames: [(6m) x 1]      The resultant wrenches in sole of the contact frames
            %             - base_pose_dot:            [6 x 1]         The base velocity vector, changed in the case of an impact with the ground
            %             - s_dot:                    [N x 1]         The configuration velocity vector, changed in the case of an impact with the ground
            %
            %     **AUTHORS:** Venus Pasandi, Nuno Guedelha
            %
            %     all authors are with the Italian Istitute of Technology (IIT)
            %     email: name.surname@iit.it
            %
            %     PLACE AND DATE: <Genoa, March 2022>
            %
            
            num_vertices = obj_step_block.contact_config.num_vertices;  % The number of the contact vertices for each foot
            num_in_contact_frames = obj_step_block.contact_config.num_in_contact_frames;  % The number of the links interacting with the ground
            num_closed_chains = obj_step_block.contact_config.num_closed_chains;  % The number of the closed chains of the robot
            
            % collect the open-chain kinematic dynamic quantities
            M = robot.get_mass_matrix(motor_inertias,obj_step_block);
            [J_in_contact, JDot_nu_in_contact] = obj.compute_J_and_JDot_nu_in_contact_frames(robot, num_in_contact_frames, num_vertices);
            if ( num_closed_chains == 0)
                J_diff_split_points = [];
                JDot_diff_nu_split_points = [];
                G_forces = J_in_contact;
            else
                [J_diff_split_points, JDot_diff_nu_split_points] = obj.compute_J_and_JDot_nu_split_points(robot);
                G_forces = [J_in_contact;J_diff_split_points];
            end
            
            % compute the vertical distance of every vertex from the ground
            contact_points = obj.compute_contact_points(robot, num_in_contact_frames, num_vertices);
            
            % compute the configuration velocity - same, if no impact - discontinuous in case of impact
            [base_pose_dot, s_dot, impact_flag] = obj.compute_velocity(M, G_forces, base_pose_dot, s_dot, num_closed_chains, num_in_contact_frames, contact_points, num_vertices);
            
            % update the robot velocity in the case of impact
            if impact_flag
                % sets the velocity in the state
                robot.set_robot_velocity(base_pose_dot, s_dot);
            end
            
            % collect the open-chain kinematic dynamic quantities
            h = robot.get_bias_forces();
            
            % computes a 3 * num_total_vertices + 6 * num_closed_chains vector
            % containing the pure contact forces acting on every vertex and the
            % internal wrenches in the spilit points
            forces = obj.compute_unilateral_linear_contact(M, h, J_in_contact, J_diff_split_points, JDot_nu_in_contact, JDot_diff_nu_split_points, torque, contact_points, num_closed_chains, generalized_ext_wrench, num_in_contact_frames, num_vertices, base_pose_dot, s_dot);
            
            % transform the contact and the internal wrenches of the spilit points in the closed chains in a wrench acting on the robot
            generalized_contact_wrench = G_forces' * forces;
            
            % sum the contact wrench and the internal wrenches of the spilit points to the external one
            generalized_total_wrench = generalized_ext_wrench + generalized_contact_wrench;
            
            % compute the wrench in the sole frames, in order to simulate a sensor mounted onto the sole frame
            ground_forces = forces(1:end-6*num_closed_chains);
            wrench_in_contact_frames = obj.compute_contact_wrench_in_sole_frames(ground_forces, robot, num_in_contact_frames, num_vertices);
            
            % update the contact log
            obj.was_in_contact = obj.is_in_contact;
            
        end
        
        links_in_contact = get_feet_contact_state(obj, num_in_contact_frames, num_vertices);

    end

    methods (Access = private)

        % computes the Jacobian and J_dot_nu relative to the vertices (Not the sole frames!)
        [J_print, JDot_nu_print] = compute_J_and_JDot_nu_in_contact_frames(obj, robot, num_inContact_frames, num_vertices);
        
        % computes the JTilde and JDOTTilde * nu for the spilit points in the (possible) closed chains
        [J_diff_splitPoint, JDot_diff_nu_splitPoint] = compute_J_and_JDot_nu_split_points(obj, robot);

        % computes the vertical position of every vertex and determine if each vertex is in contact with the ground or not
        contact_points = compute_contact_points(obj, robot, num_inContact_frames, num_vertices);

        % computes the robot velocity vector after a (possible) impact
        [base_pose_dot, s_dot] = compute_velocity(obj, M, G, base_pose_dot, s_dot, closed_chains, num_inContact_frames, contact_point, num_vertices)
        
        % computes the damped pseudo inverse of the matrix P
        P_damped_psudo_inverse = compute_damped_psudo_inverse(obj,P,damped_coefficient);

        % computes the robot acceleration with NO contact forces
        free_acceleration = compute_free_acceleration(obj, M, h, torque, generalized_ext_wrench);

        % computes the acceleration of the feet and the difference of the acceleration of the split points in the (possible) closed chains with NO contact forces
        free_contact_acceleration = compute_free_contact_acceleration(obj, G, free_acceleration, P);

        % computes the pure forces acting on the feet vertices and the
        % internal wrenches applied to the split points in the (possible) closed chains
        forces = compute_unilateral_linear_contact(obj, M, h, J_in_contact, J_diff_split_points, JDot_nu_in_contact, JDot_diff_nu_split_points, torque, contact_point, num_closed_chains, generalized_ext_wrench, num_in_contact_frames, num_vertices, base_pose_dot, s_dot);
        
        % computes the impulsive pure forces acting on the feet vertices and the impulsive internal wrenches applied to the split points in the (possible) closed chain
        impulsive_forces = compute_unilateral_linear_impact(obj, M, nu, J_in_contact, J_diff_split_points, contact_point, num_closed_chains, num_in_contact_frames, map_vertices_new_contact, num_vertices);
        
        % trasforms the pure forces on foot vertices in wrench in sole frames
        wrench_in_contact_frames = compute_contact_wrench_in_sole_frames(obj, contact_forces, robot, num_in_contact_frames, num_vertices);

        % Fills the matrix relating to the unilateral and friction cone constraints used by the optimization problem solver.
        prepare_optimization_matrix(obj, num_in_contact_frames, num_vertices);
        
        % This function writes the local coordinates of the foot vertices in a cell array format
        prepare_foot_print (obj, num_in_contact_frames, num_vertices, foot_print);
    end

end
