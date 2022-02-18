classdef Contacts < handle
    %CONTACTS The Contact class handles the computation of the contact forces and the impact.
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
                compute_contact_closedChain(obj, robot, torque, generalized_ext_wrench, motorInertias, base_pose_dot, s_dot,obj_step_block)
            % compute_contact_closedChains Computes the contact forces, the internal wrenches of the spilit points in the (possible) closed chains, and the configuration velocity after a (possible) impact
            % INPUTS: - robot: instance of the Robot object
            %         - closedChains_config: information about the closed
            %         chains
            %         - torque: joint torques
            %         - generalized_ext_wrench: wrench transformed using the Jacobian relative to the application point
            %         - base_pose_dot, s_dot: configuration velocity
            % OUTPUTS: - generalized_total_wrench: the sum of the
            % generalized_ext_wrench and the generalized contact wrench and
            % the internal wrench on the split points
            %          - wrench_left_foot, wrench_right_foot: the wrench in sole frames
            %          - base_pose_dot, s_dot: configuration velocity, changed in the case of an impact with the ground
            
            % collecting the open-chain robot quantities
            h = robot.get_bias_forces();
            M = robot.get_mass_matrix(motorInertias,obj_step_block);
            [J_inContact, JDot_nu_inContact] = obj.compute_J_and_JDot_nu_inContact_frames(robot,length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
            if (obj_step_block.robot_config.closedChains == 0)
                J_diff_splitPoint = [];
                JDot_diff_nu_splitPoint = [];
                G_forces = J_inContact;
            else
                [J_diff_splitPoint, JDot_diff_nu_splitPoint] = compute_J_and_JDot_nu_splitPoint(obj, robot);
                G_forces = [J_inContact;J_diff_splitPoint];
            end
            % compute the vertical distance of every vertex from the ground
            contact_points = obj.compute_contact_points(robot, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
            % computes a 3 * num_total_vertices vector containing the pure forces acting on every vertes
            forces = obj.compute_unilateral_linear_contact(M, h, J_inContact, J_diff_splitPoint, JDot_nu_inContact, JDot_diff_nu_splitPoint, torque, contact_points, obj_step_block.robot_config.closedChains, generalized_ext_wrench, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));            
            % transform the contact and the internal wrenches of the spilit points in the closed chains in a wrench acting on the robot
            generalized_contact_wrench = G_forces' * forces;
            % sum the contact wrench and the internal wrenches of the spilit points to the external one
            generalized_total_wrench = generalized_ext_wrench + generalized_contact_wrench;
            % compute the wrench in the sole frames, in order to simulate a sensor mounted onto the sole frame
            ground_forces = forces(1:end-6*obj_step_block.robot_config.closedChains);
            wrench_inContact_frames = obj.compute_contact_wrench_in_sole_frames(ground_forces, robot, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
            % compute the configuration velocity - same, if no impact - discontinuous in case of impact
            [base_pose_dot, s_dot] = obj.compute_velocity(M, G_forces, robot, base_pose_dot, s_dot,obj_step_block.robot_config.closedChains, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
            % update the contact log
            obj.was_in_contact = obj.is_in_contact;
        end
        
        function links_in_contact = getFeetContactState(obj, num_inContact_frames)
            links_in_contact = false(1,num_inContact_frames);
            for counter = 1 : num_inContact_frames
                links_in_contact(counter) = any(obj.is_in_contact(1+4*(counter-1):4*counter));
            end
        end

    end

    methods (Access = private)

        function [J_print, JDot_nu_print] = compute_J_and_JDot_nu_inContact_frames(obj, robot, num_inContact_frames)
            % compute_J_and_JDot_nu returns the Jacobian and J_dot_nu relative to the vertices (Not the sole frames!)
            H_inGroundContact = robot.get_inContactWithGround_H();
            J_inGroundContact = robot.get_inContactWithGround_jacobians();
            JDot_nu_inGroundContact = robot.get_inContactWithGround_JDot_nu();
            
            J_print = zeros(3*obj.num_vertices*num_inContact_frames,size(J_inGroundContact,2));
            JDot_nu_print = zeros(3*obj.num_vertices*num_inContact_frames,size(JDot_nu_inGroundContact,2));
            for counter = 1 : num_inContact_frames
                H_frame = H_inGroundContact(1+4*(counter-1):4*counter,:);
                J_frame = J_inGroundContact(1+6*(counter-1):6*counter,:);
                JDot_nu_frame = JDot_nu_inGroundContact(1+6*(counter-1):6*counter,:);
                
                J_lin = J_frame(1:3, :);
                J_ang = J_frame(4:6, :);
                JDot_nu_lin = JDot_nu_frame(1:3, :);
                JDot_nu_ang = JDot_nu_frame(4:6, :);
                R_frame = H_frame(1:3, 1:3);
            
                % the vertices are affected by pure forces. We need only the linear Jacobians
                % for a vertex i:
                % Ji = J_linear - S(R*pi) * J_angular
                % JDot_nui = JDot_nu_linear - S(R*pi) * JDot_nu_angular
                J_frame_print = zeros(3*obj.num_vertices,size(J_inGroundContact,2));
                JDot_nu_frame_print = zeros(3*obj.num_vertices,size(JDot_nu_inGroundContact,2));
                for ii = 1:obj.num_vertices
                    j = (ii - 1) * 3 + 1;
                    foot_print_frame = obj.foot_print{counter};
                    v_coords = foot_print_frame(:, ii);
                    J_frame_print(j:j + 2, :) = J_lin - wbc.skew(R_frame * v_coords) * J_ang;
                    JDot_nu_frame_print(j:j + 2, :) = JDot_nu_lin - wbc.skew(R_frame * v_coords) * JDot_nu_ang;
                end
                J_print(1+(counter-1)*3*obj.num_vertices:counter*3*obj.num_vertices,:) = J_frame_print;
                JDot_nu_print(1+(counter-1)*3*obj.num_vertices:counter*3*obj.num_vertices,:) = JDot_nu_frame_print;
            end
        end
        
        function [J_diff_splitPoint, JDot_diff_nu_splitPoint] = compute_J_and_JDot_nu_splitPoint(obj, robot)
            [JDiff_Lpoint,JDiff_Rpoint] = robot.get_spilitPoints_diff_jacobian();
            [JDotNuDiff_Lpoint,JDotNuDiff_Rpoint] = robot.get_SpilitPoints_diff_JDot_nu();
            J_diff_splitPoint = [JDiff_Lpoint ; JDiff_Rpoint];
            JDot_diff_nu_splitPoint = [JDotNuDiff_Lpoint ; JDotNuDiff_Rpoint];
        end

        function contact_points = compute_contact_points(obj, robot, num_inContact_frames)
            % contact_points returns the vertical coordinate of every vertex
            contact_points = zeros(num_inContact_frames*obj.num_vertices,1);
            
            % checks if the vertex is in contact with the ground
            H_IN_CONTACT_WITH_GROUND = robot.get_inContactWithGround_H();
            for counter = 1 : num_inContact_frames
                H_frame = H_IN_CONTACT_WITH_GROUND(1+4*(counter-1):4*counter,:);
                foot_print_frame = obj.foot_print{counter};
                z_frame_print = zeros(obj.num_vertices, 1);
                for ii = 1 : obj.num_vertices
                    % transforms the coordinates of the vertex (in sole frame) in the world frame
                    z = H_frame * [foot_print_frame(:, ii); 1];
                    z_frame_print(ii) = z(3);
                    % the vertex is in contact if its z <= 0
                    obj.is_in_contact(obj.num_vertices*(counter-1)+ii) = z_frame_print(ii) <= 0;
                end
                contact_points(1+obj.num_vertices*(counter-1):obj.num_vertices*counter,1) = z_frame_print;
            end
        end

        function [base_pose_dot, s_dot] = compute_velocity(obj, M, G, robot, base_pose_dot, s_dot, closed_chains, num_inContact_frames)
            % compute_velocity returns the configuration velocity
            % the velocity does not change if there is no impact
            % the velocity change if there is the impact
            if size(G,1) == (num_inContact_frames*3*obj.num_vertices)
                J_feet = G;
                J_split_points = [];
            elseif size(G,1) > (num_inContact_frames*3*obj.num_vertices)
                J_feet = G(1:(num_inContact_frames*3*obj.num_vertices),:);
                J_split_points = G(num_inContact_frames*3*obj.num_vertices+1:end,:);
            end
            
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
            G = [J;J_split_points];

            % compute the projection in the null space of the scaled Jacobian of the vertices if a
            % new contact is detected, otherwise just resuse the input variables (base_pose_dot,
            % s_dot).
            if new_contact 
                if(closed_chains==0)
                    N = (eye(robot.NDOF + 6) - M \ (G' * (G * (M \ G') \ G)));   
                else
                    N = (eye(robot.NDOF + 6) - M \ (G' * (obj.compute_damped_psudo_inverse(G * (M \ G'),0.001) * G)));
                end
                % the velocity after the impact is a function of the velocity before the impact
                % under the constraint that the vertex velocity is equal to zeros
                x = N * [base_pose_dot; s_dot];
                base_pose_dot = x(1:6);
                s_dot = x(7:end);
            end
        end
        
        function P_damped_psudo_inverse = compute_damped_psudo_inverse(obj,P,damped_coefficient)
            P_damped_psudo_inverse = P'/(P*P'+damped_coefficient^2*eye(size(P,1)));
        end

        function free_acceleration = compute_free_acceleration(obj, M, h, torque, generalized_ext_wrench)
            % compute_free_acceleration returns the system acceleration with NO contact forces
            % dot{v} = inv{M}(S*tau + external_forces - h)
            free_acceleration = M \ (obj.S * torque + generalized_ext_wrench - h);
        end

        function free_contact_diff_acceleration = compute_free_contact_diff_acceleration(obj, G, free_acceleration, P)
            % compute_free_contact_diff_acceleration returns the acceleration of the feet  and the difference of the acceleration of the split points with NO contact forces
            free_contact_diff_acceleration = G * free_acceleration + P;
        end

        function forces = compute_unilateral_linear_contact(obj, M, h, J_inContact, J_diff_splitPoint, JDot_nu_inContact, JDot_diff_nu_splitPoint, torque, contact_point, closedChains, generalized_ext_wrench, num_inContact_frames)
            % compute_unilateral_linear_contact returns the pure forces
            % acting on the feet vertices and the internal force/torques
            % applied to the split points if there is a closed chain
            % kinematic
            
            free_acceleration = obj.compute_free_acceleration(M, h, torque, generalized_ext_wrench);
            free_contact_diff_acceleration_contact = obj.compute_free_contact_diff_acceleration(J_inContact, free_acceleration, JDot_nu_inContact);
            
            if (closedChains == 0) % there is no closed chain
                H = J_inContact * (M \ J_inContact');
                g = free_contact_diff_acceleration_contact;
            else % there are some closed chains
                H = J_inContact * (M\J_inContact') - (J_inContact*(M\J_diff_splitPoint'))*(obj.compute_damped_psudo_inverse(J_diff_splitPoint *(M\J_diff_splitPoint'),0.001)*(J_diff_splitPoint*(M\J_inContact') ));
                free_contact_diff_acceleration_internal = obj.compute_free_contact_diff_acceleration(J_diff_splitPoint, free_acceleration, JDot_diff_nu_splitPoint);
                g = free_contact_diff_acceleration_contact - (J_inContact*(M\J_diff_splitPoint'))*(obj.compute_damped_psudo_inverse(J_diff_splitPoint *(M\J_diff_splitPoint') ,0.001 )*free_contact_diff_acceleration_internal);
            end
            
            if ~issymmetric(H)
                H = (H + H') / 2; % if non sym
            end
           
            if obj.useOSQP
                for i = 1:obj.num_vertices * num_inContact_frames
                    obj.Aeq(i, i * 3) = contact_point(i) > 0;
                end
                if obj.firstSolverIter
                    % Setup workspace and change alpha parameter
                    obj.osqpProb = osqp;
                    obj.osqpProb.setup(sparse(H), g, sparse([obj.A;obj.Aeq]), [obj.Ax_Lb;obj.beq], [obj.Ax_Ub;obj.beq], 'alpha', 1);
                    obj.firstSolverIter = true;
                else
                    % Update the problem
                    obj.osqpProb.update('Px', nonzeros(triu(sparse(H))), 'q', g, 'Ax', sparse([obj.A;obj.Aeq]));
                end
                % Solve problem
                res = obj.osqpProb.solve();
                % Counter the consecuitive failure of the solver
                if (res.info.status_val == 1)
                    obj.fail_counter = 0;
                else
                    obj.fail_counter = obj.fail_counter + 1;
                end
                contactForces = res.x;
            elseif obj.useQPOASES
                obj.ulb = 1e10 + zeros(obj.num_vertices*num_inContact_frames*3, 1);
                for i = 1 : obj.num_vertices * num_inContact_frames
                    if (contact_point(i) > 0) % vertex NOT in contact with the ground
                        obj.ulb(3*i-2:3*i) = 0;
                    end
                end
                [contactForces,status] = simFunc_qpOASES(H, g, obj.A, obj.Ax_Lb, obj.Ax_Ub, -obj.ulb, obj.ulb);
                % Counter the consecuitive failure of the solver
                if (status == 0)
                    obj.fail_counter = 0;
                else
                    obj.fail_counter = obj.fail_counter + 1;
                end
            else
                for i = 1:obj.num_vertices * num_inContact_frames
                    obj.Aeq(i, i * 3) = contact_point(i) > 0;
                end
                obj.ulb = 1e10 + zeros(obj.num_vertices*num_inContact_frames*3, 1);
                for i = 1 : obj.num_vertices * num_inContact_frames
                    if (contact_point(i) > 0) % vertex NOT in contact with the ground
                        obj.ulb(3*i-2:3*i) = 0;
                    end
                end
                options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');
                [contactForces,~,exitFlag,~] = quadprog(H, g, obj.A, obj.Ax_Ub, [], [], -obj.ulb, obj.ulb, 100 * ones(size(H,1), 1), options);
                % Counter the consecuitive failure of the solver
                if (exitFlag == 1)
                    obj.fail_counter = 0;
                else
                    obj.fail_counter = obj.fail_counter + 1;
                end
            end
            % generate an error if the optimization solver fails to solve
            % the optimization problem and obtain the contact forces.
            if (obj.fail_counter >= obj.max_consecuitive_fail)
                error(strjoin({'[RobotDynWithContacts] The solver fails to compute the contact forces for',num2str(obj.max_consecuitive_fail),'times'}));
            end
            % compute the internal wrenches of the spilit points if there
            % is closed chain
            if (closedChains == 0)
                internalWrenches = [];
            else
                internalWrenches = -obj.compute_damped_psudo_inverse(J_diff_splitPoint *(M\J_diff_splitPoint'),0.001 )*((J_diff_splitPoint*(M\J_inContact'))*contactForces + free_contact_diff_acceleration_internal);
            end
            forces = [contactForces;internalWrenches];
        end

        function wrench_inContactFrames = compute_contact_wrench_in_sole_frames(obj, contact_forces, robot, num_inContact_frames)
            % compute_contact_wrench_in_sole_frames trasforms the pure forces on foot vertices in wrench in sole frames
            
            wrench_inContactFrames = zeros(6*num_inContact_frames,1);
            for counter = 1 : num_inContact_frames
                % Rotation matrix of sole w.r.t the world
                H_frame = robot.get_inContactWithGround_H();
                R_frame = H_frame(1:3, 1:3);
                foot_print_frame = obj.foot_print{counter};
                
                wrench_frame = zeros(6, 1);
                % computed contact forces on every vertex
                contact_forces_frame = contact_forces(1+12*(counter-1):12*counter);
                
                for i = 1 : obj.num_vertices
                    j = (i - 1) * 3 + 1;
                    wrench_frame(1:3) = wrench_frame(1:3) + R_frame' * contact_forces_frame(j:j + 2);
                    wrench_frame(4:6) = wrench_frame(4:6) - wbc.skew(foot_print_frame(:, i)) * (R_frame' * contact_forces_frame(j:j + 2));
                end
                wrench_inContactFrames(1+6*(counter-1):6*counter) = wrench_frame;
            end

        end

        function prepare_optimization_matrix(obj,num_inContact_frames)
            % prepare_optimization_matrix Fills the matrix used by the optimization problem solver.
            % A being the matrix of friction cone constraints, for ...
            % - quadprog: Ax <= b (should include -x <= 0).
            % - OSQP    : l <= Ax <= u (should include -Inf <= -x <= 0).
            %
            % So in both cases we define: Ax_Lb <= Ax <= Ax_Ub where Ax_Lb = -Inf.
            
            total_num_vertices = obj.num_vertices * num_inContact_frames; % number of vertex per foot * number feet
            num_variables = 3 * total_num_vertices; % number of unknowns (3 force components per vertex)
            num_constr = 5 * total_num_vertices; % number of constraint: simplified friction cone + non negativity of vertical force
            % fill the optimization matrix
            obj.A = zeros(num_constr, num_variables);
            obj.Ax_Ub = zeros(num_constr, 1);
            obj.Ax_Lb = -1e20 + zeros(num_constr, 1);
            
            % Constraint "Fz=0 if no contact" formulated as Aeq x = 0.
            % Aeq shall be concatenated with A in the case of OSQP.
            obj.Aeq = zeros(total_num_vertices, num_variables);
            obj.beq = zeros(total_num_vertices, 1);
            obj.ulb = zeros(num_variables, 1);
            
            constr_matrix = [...
                1, 0, -obj.mu; ...% first 4 rows: simplified friction cone
                0, 1, -obj.mu; ...
                -1, 0, -obj.mu; ...
                0, -1, -obj.mu; ...
                0, 0, -1]; ...% non negativity of vertical force
                
            % fill a block diagonal matrix with all the constraints
            Ac = repmat({constr_matrix}, 1, total_num_vertices); % Repeat Matrix for every vertex as a cell array
            obj.A(:,1:num_variables) = blkdiag(Ac{1:total_num_vertices});
            
            % Create an OSQP problem object
            if obj.useOSQP
                obj.osqpProb = osqp;
            end
        end
        
    end

end
