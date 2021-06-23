classdef Contacts < handle
    %CONTACTS The Contact class handles the computation of the contact forces and the impact.
    %
    % Contacts Methods:
    %   compute_contact - computes the wrench and the state velocity after a (possible) impact

    properties (Constant)
        num_vertices = 4;
        useOSQP=false; % Use the OSQP solver instead of quadprog for the optim. prob. computing the reaction forces at the feet
        useQPOASES=true;
    end
    
    properties (Access = private)
        foot_print; % the coordinates of the vertices
        was_in_contact = ones(8, 1); % this vector says if the vertex was in contact (1) or not (0)
        is_in_contact = ones(8, 1); % this vector says if the vertex is in contact (1) or not (0)
        S; % selector matrix for the robot torque
        mu; % friction coefficient
        A; Ax_Lb; Ax_Ub; Aeq; beq; ulb; % matrices used in the optimization problem
        osqpProb; % OSQP solver object
        firstSolverIter; % For handing osqp.setp and osqp.update
    end

    methods

        function obj = Contacts(foot_print, robot, friction_coefficient)
            %CONTACTS The Contact class needs the coordinates of the vertices of the foot
            % Arguments
            %   foot_print - the coordinates of every vertex in xyz
            %   robot - the robot model
            %   frinction coefficient - the coefficient that defines the (simplified) friction cone
            if (~isequal(size(foot_print), [3, 4]))
                error('The foot print is represented with a matrix composed by 4 columns in which every column is the set of xyz coordinates')
            end

            obj.foot_print = foot_print;
            obj.S = [zeros(6, robot.NDOF); ...
                    eye(robot.NDOF)];
            obj.mu = friction_coefficient;
            obj.prepare_optimization_matrix();
            
            % initialize the setup/update step of the osqp solver
            obj.firstSolverIter = true;
        end

        function [generalized_total_wrench, wrench_left_foot, wrench_right_foot, base_pose_dot, s_dot] = ...
                compute_contact(obj, robot, torque, generalized_ext_wrench, motorInertias, base_pose_dot, s_dot,obj_step_block)
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
            M = robot.get_mass_matrix(motorInertias,obj_step_block);
            [J_feet, JDot_nu_feet] = obj.compute_J_and_JDot_nu(robot);
            % compute the vertical distance of every vertex from the ground
            contact_points = obj.compute_contact_points(robot);
            % computes a 3 * num_total_vertices vector containing the pure forces acting on every vertes
            contact_forces = obj.compute_unilateral_linear_contact(J_feet, M, h, torque, JDot_nu_feet, contact_points, generalized_ext_wrench);
            % transform the contact in a wrench acting on the robot
            generalized_contact_wrench = J_feet' * contact_forces;
            % sum the contact wrench to the external one
            generalized_total_wrench = generalized_ext_wrench + generalized_contact_wrench;
            % compute the wrench in the sole frames, in order to simulate a sensor mounted onto the sole frame
            [wrench_left_foot, wrench_right_foot] = obj.compute_contact_wrench_in_sole_frames(contact_forces, robot);
            % compute the configuration velocity - same, if no impact - discontinuous in case of impact
            [base_pose_dot, s_dot] = obj.compute_velocity(M, J_feet, robot, base_pose_dot, s_dot);
            % update the contact log
            obj.was_in_contact = obj.is_in_contact;
        end
        
        function [left_foot_in_contact, right_foot_in_contact] = getFeetContactState(obj)
            left_foot_in_contact = any(obj.is_in_contact(1:4));
            right_foot_in_contact = any(obj.is_in_contact(5:8));
        end

    end

    methods (Access = private)

        function [J_feet, JDot_nu_feet] = compute_J_and_JDot_nu(obj, robot)
            % compute_J_and_JDot_nu returns the Jacobian and J_dot_nu relative to the vertices (Not the sole frames!)
            [H_LFOOT, H_RFOOT] = robot.get_feet_H();
            [J_LFoot, J_RFoot] = robot.get_feet_jacobians();
            [JDot_nu_LFOOT, JDot_nu_RFOOT] = robot.get_feet_JDot_nu();
            J_L_lin = J_LFoot(1:3, :);
            J_L_ang = J_LFoot(4:6, :);
            J_R_lin = J_RFoot(1:3, :);
            J_R_ang = J_RFoot(4:6, :);
            JDot_nu_L_lin = JDot_nu_LFOOT(1:3, :);
            JDot_nu_L_ang = JDot_nu_LFOOT(4:6, :);
            JDot_nu_R_lin = JDot_nu_RFOOT(1:3, :);
            JDot_nu_R_ang = JDot_nu_RFOOT(4:6, :);
            R_LFOOT = H_LFOOT(1:3, 1:3);
            R_RFOOT = H_RFOOT(1:3, 1:3);

            % the vertices are affected by pure forces. We need only the linear Jacobians
            % for a vertex i:
            % Ji = J_linear - S(R*pi) * J_angular
            % JDot_nui = JDot_nu_linear - S(R*pi) * JDot_nu_angular
            J_left_foot_print = zeros(3*obj.num_vertices,size(J_L_lin,2));
            J_right_foot_print = zeros(3*obj.num_vertices,size(J_R_lin,2));
            JDot_nu_left_foot_print = zeros(3*obj.num_vertices,size(JDot_nu_L_lin,2));
            JDot_nu_right_foot_print = zeros(3*obj.num_vertices,size(JDot_nu_R_lin,2));
            for ii = 1:obj.num_vertices
                j = (ii - 1) * 3 + 1;
                v_coords = obj.foot_print(:, ii);
                J_left_foot_print(j:j + 2, :) = J_L_lin - mwbs.Utils.skew(R_LFOOT * v_coords) * J_L_ang;
                JDot_nu_left_foot_print(j:j + 2, :) = JDot_nu_L_lin - mwbs.Utils.skew(R_LFOOT * v_coords) * JDot_nu_L_ang;
                J_right_foot_print(j:j + 2, :) = J_R_lin - mwbs.Utils.skew(R_RFOOT * v_coords) * J_R_ang;
                JDot_nu_right_foot_print(j:j + 2, :) = JDot_nu_R_lin - mwbs.Utils.skew(R_RFOOT * v_coords) * JDot_nu_R_ang;
            end

            % stack the matrices
            J_feet = [J_left_foot_print; J_right_foot_print];
            JDot_nu_feet = [JDot_nu_left_foot_print; JDot_nu_right_foot_print];
        end

        function contact_points = compute_contact_points(obj, robot)
            % contact_points returns the vertical coordinate of every vertex

            % checks if the vertex is in contact with the ground
            [H_LFOOT, H_RFOOT] = robot.get_feet_H();
            left_z_foot_print = zeros(4, 1);
            right_z_foot_print = zeros(4, 1);

            for ii = 1:obj.num_vertices
                % transforms the coordinates of the vertex (in sole frame) in the world frame
                left_z = H_LFOOT * [obj.foot_print(:, ii); 1];
                left_z_foot_print(ii) = left_z(3);
                right_z = H_RFOOT * [obj.foot_print(:, ii); 1];
                right_z_foot_print(ii) = right_z(3);
                % the vertex is in contact if its z <= 0
                obj.is_in_contact(ii) = left_z_foot_print(ii) <= 0;
                obj.is_in_contact(ii + 4) = right_z_foot_print(ii) <= 0;
            end

            contact_points = [left_z_foot_print; right_z_foot_print];
        end

        function [base_pose_dot, s_dot] = compute_velocity(obj, M, J_feet, robot, base_pose_dot, s_dot)
            % compute_velocity returns the configuration velocity
            % the velocity does not change if there is no impact
            % the velocity change if there is the impact

            % We shall stack the jacobians relative to the vertices that will be in contact. For
            % those vertices, the velocity after impact should be zero.
            mapVerticesNewContact = obj.is_in_contact & ~obj.was_in_contact;
            new_contact = any(mapVerticesNewContact);

            % If a new contact is detected we should make sure that the velocity of the vertices
            % previously in contact is zero.
            if new_contact
                mapVerticesAtZeroVel = mapVerticesNewContact | obj.was_in_contact;
            else
                mapVerticesAtZeroVel = mapVerticesNewContact;
            end

            allIndexes = 1:numel(mapVerticesAtZeroVel);
            indexesVerticesAtZeroVel = (allIndexes(mapVerticesAtZeroVel)-1)*3+1; % each vertex has 3 components
            expandedIdxesVerticesAtZeroVel = [indexesVerticesAtZeroVel;indexesVerticesAtZeroVel+1;indexesVerticesAtZeroVel+2];
            expandedIdxesVerticesAtZeroVel = expandedIdxesVerticesAtZeroVel(:);
            J = J_feet(expandedIdxesVerticesAtZeroVel,1:end);

            % compute the projection in the null space of the scaled Jacobian of the vertices if a
            % new contact is detected, otherwise just resuse the input variables (base_pose_dot,
            % s_dot).
            if new_contact
                N = (eye(robot.NDOF + 6) - M \ (J' * ((J * (M \ J')) \ J)));
                % the velocity after the impact is a function of the velocity before the impact
                % under the constraint that the vertex velocity is equal to zeros
                x = N * [base_pose_dot; s_dot];
                base_pose_dot = x(1:6);
                s_dot = x(7:end);
            end
        end

        function free_acceleration = compute_free_acceleration(obj, M, h, torque, generalized_ext_wrench)
            % compute_free_acceleration returns the system acceleration with NO contact forces
            % dot{v} = inv{M}(S*tau + external_forces - h)
            free_acceleration = M \ (obj.S * torque + generalized_ext_wrench - h);
        end

        function free_contact_acceleration = compute_free_contact_acceleration(obj, J_feet, free_acceleration, JDot_nu_feet)
            % compute_free_contact_acceleration returns the acceleration of the feet with NO contact forces
            free_contact_acceleration = J_feet * free_acceleration + JDot_nu_feet;
        end

        function forces = compute_unilateral_linear_contact(obj, J_feet, M, h, torque, JDot_nu_feet, contact_point, generalized_ext_wrench)
            % compute_unilateral_linear_contact returns the pure forces acting on the feet vertices
            
            free_acceleration = obj.compute_free_acceleration(M, h, torque, generalized_ext_wrench);
            free_contact_acceleration = obj.compute_free_contact_acceleration(J_feet, free_acceleration, JDot_nu_feet);
            H = J_feet * (M \ J_feet');
            
            if ~issymmetric(H)
                H = (H + H') / 2; % if non sym
            end
            
            if obj.useOSQP
            for i = 1:obj.num_vertices * 2
                obj.Aeq(i, i * 3) = contact_point(i) > 0;
            end
                if obj.firstSolverIter
                    % Setup workspace and change alpha parameter
                    obj.osqpProb = osqp;
                    obj.osqpProb.setup(sparse(H), free_contact_acceleration, sparse([obj.A;obj.Aeq]), [obj.Ax_Lb;obj.beq], [obj.Ax_Ub;obj.beq], 'alpha', 1);
                    obj.firstSolverIter = true;
                else
                    % Update the problem
                    obj.osqpProb.update('Px', nonzeros(triu(sparse(H))), 'q', free_contact_acceleration, 'Ax', sparse([obj.A;obj.Aeq]));
                end
                % Solve problem
                res = obj.osqpProb.solve();
                forces = res.x;
            elseif obj.useQPOASES
                for i = 1:obj.num_vertices * 2
                    if (contact_point(i) > 0) % vertex NOT in contact with the ground
                        obj.ulb(3*i-2:3*i) = 0;
                    else % vertex in contact with the ground
                        obj.ulb(3*i-2:3*i) = 1/eps;
                    end
                end
                forces = simFunc_qpOASES(H, free_contact_acceleration, obj.A, obj.Ax_Lb, obj.Ax_Ub, -obj.ulb, obj.ulb);
            else
                for i = 1:obj.num_vertices * 2
                    obj.Aeq(i, i * 3) = contact_point(i) > 0;
                end
                options = optimoptions('quadprog', 'Algorithm', 'interior-point-convex', 'Display', 'off');
                forces = quadprog(H, free_contact_acceleration, obj.A, obj.Ax_Ub, obj.Aeq, obj.beq, [], [], 100 * ones(24, 1), options);
            end
        end

        function [wrench_left_foot, wrench_right_foot] = compute_contact_wrench_in_sole_frames(obj, contact_forces, robot)
            % compute_contact_wrench_in_sole_frames trasforms the pure forces on foot vertices in wrench in sole frames

            % Rotation matrix of sole w.r.t the world
            [H_LFOOT, H_RFOOT] = robot.get_feet_H();
            R_LFOOT = H_LFOOT(1:3, 1:3);
            R_RFOOT = H_RFOOT(1:3, 1:3);

            wrench_left_foot = zeros(6, 1);
            wrench_right_foot = zeros(6, 1);
            % computed contact forces on every vertex - split left and right
            contact_forces_left = contact_forces(1:12);
            contact_forces_right = contact_forces(13:24);

            for i = 1:obj.num_vertices
                j = (i - 1) * 3 + 1;
                wrench_left_foot(1:3) = wrench_left_foot(1:3) + R_LFOOT' * contact_forces_left(j:j + 2);
                wrench_left_foot(4:6) = wrench_left_foot(4:6) - mwbs.Utils.skew(obj.foot_print(:, i)) * (R_LFOOT' * contact_forces_left(j:j + 2));
                wrench_right_foot(1:3) = wrench_right_foot(1:3) + R_RFOOT' * contact_forces_right(j:j + 2);
                wrench_right_foot(4:6) = wrench_right_foot(4:6) - mwbs.Utils.skew(obj.foot_print(:, i)) * (R_RFOOT' * contact_forces_right(j:j + 2));
            end

        end

        function prepare_optimization_matrix(obj)
            % prepare_optimization_matrix Fills the matrix used by the optimization problem solver.
            % A being the matrix of friction cone constraints, for ...
            % - quadprog: Ax <= b (should include -x <= 0).
            % - OSQP    : l <= Ax <= u (should include -Inf <= -x <= 0).
            %
            % So in both cases we define: Ax_Lb <= Ax <= Ax_Ub where Ax_Lb = -Inf.
            
            total_num_vertices = obj.num_vertices * 2; % number of vertex per foot * number feet
            num_variables = 3 * total_num_vertices; % number of unknowns (3 force components per vertex)
            num_constr = 5 * total_num_vertices; % number of constraint: simplified friction cone + non negativity of vertical force
            % fill the optimization matrix
            obj.A = zeros(num_constr, num_variables);
            obj.Ax_Ub = zeros(num_constr, 1);
            obj.Ax_Lb = -1/eps+zeros(num_constr, 1);
            
            % Constraint "Fz=0 if no contact" formulated as Aeq x = 0.
            % Aeq shall be concatenated with A in the case of OSQP.
            obj.Aeq = zeros(total_num_vertices, num_variables);
            obj.beq = zeros(total_num_vertices, 1);
            obj.ulb = zeros(3*total_num_vertices, 1);
            
            constr_matrix = [...
                1, 0, -obj.mu; ...% first 4 rows: simplified friction cone
                0, 1, -obj.mu; ...
                -1, 0, -obj.mu; ...
                0, -1, -obj.mu; ...
                0, 0, -1]; ...% non negativity of vertical force
                
            % fill a block diagonal matrix with all the constraints
            Ac = repmat({constr_matrix}, 1, total_num_vertices); % Repeat Matrix for every vertex as a cell array
            obj.A = blkdiag(Ac{1:total_num_vertices});
            
            % Create an OSQP problem object
            if obj.useOSQP
                obj.osqpProb = osqp;
            end
        end
    end

end
