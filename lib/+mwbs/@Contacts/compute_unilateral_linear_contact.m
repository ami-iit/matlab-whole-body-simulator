function forces = compute_unilateral_linear_contact(obj, M, h, J_in_contact, J_diff_split_points, JDot_nu_in_contact, JDot_diff_nu_split_points, torque, contact_point, num_closed_chains, generalized_ext_wrench, num_in_contact_frames)

    %     COMPUTE_UNILATERAL_LINEAR_CONTACT: computes the pure forces acting on the feet vertices and the internal wrenches applied to the split points in the (possible) closed chains
    % 
    %     PROCEDURE: The equation of motion for a N DOF robotic system can be written as
    % 
    %             M vDot + h = B u + Fe + Jc^T Fc + Ji^T Fi
    % 
    %             where u is the joint torque, Fe is the external wrenches, Fc is the contact forces, and Fi if the internal wrenches in the spilit points in the (possible) closed chains.
    % 
    %             The above dynamics is constrained by the topological constraint of the closed chains and the contact points as
    % 
    %             Ji v = 0
    %             Jc v = 0
    % 
    %             In the present function, there are two methods for computing Fc and Fi where the both methods are based on the maximum kinetic energy dissipation principle.
    % 
    %             1) Continuous contact model:
    %             Assuming that Ji v(0) = 0 and Jc v(0) = 0, we can rewrite the above constraint in the acceleration level as
    % 
    %             JiDot v + Ji vDot = 0
    %             JcDot v + Jc vDot = 0
    % 
    %             Using the maximum kinetic energy dissipation principle, vDot, fc, fi can be computed as
    % 
    %             Minimize 0.5 |vDot - vDot_free|_M
    %             subject to:
    %             JiDot v + Ji vDot = 0
    %             JcDot v + Jc vDot = 0
    % 
    %             where vDot_free is the free acceleration of the system.
    %             The KKT conditions of the above problem are as
    % 
    %             M vDot - M vDot_free - Jc^T lc - Ji^T li = 0
    %             JiDot v + Ji vDot = 0
    %             JcDot v + Jc vDot = 0
    % 
    %             It can be shown that lc = Fc and li = Fi.
    %             Substituting VDot from the first equation to the second and third one gives us
    % 
    %             JiDot v + Ji vDot_free + Ji M^-1 Jc^T Fc + Ji M^-1 Ji^T Fi = 0
    %             JcDot v + Jc vDot_free + Jc M^-1 Jc^T Fc + Jc M^-1 Ji^T Fi = 0
    % 
    %             From the first equation, we have
    % 
    %             Fi = - (Ji M^-1 Ji^T)^-1 (JiDot v + Ji vDot_free + Ji M^-1 Jc^T Fc)
    % 
    %             Substituting Fi in the second equation, we obtain
    % 
    %             JcDot v + Jc vDot_free + (Jc M^-1 Jc^T - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 Ji M^-1 Jc^T) Fc - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 (JiDot v + Ji vDot_free) = 0
    % 
    %             The above equation can be written as
    % 
    %             H Fc + g = 0
    % 
    %             where
    % 
    %             H = Jc M^-1 Jc^T - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 Ji M^-1 Jc^T
    %             g = JcDot v + Jc vDot_free - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 (JiDot v + Ji vDot_free);
    % 
    %             The above equation can be write as an optimisation problem like
    % 
    %             Minimize 0.5 Fc^T H Fc + g^T Fc
    % 
    %             Now we can add the unilateral and friction cone constraints to the above optimisation problem and write it as
    % 
    %             Minimize 0.5 Fc^T H Fc + g^T Fc
    %             subject to:
    %             Fc_z >= 0
    %             -mu Fc_z <= Fc_x, Fc_y <= mu Fc_z
    % 
    %             2) Discrete Contact model:
    %             Using the Euler integration, the topological constraints can be written as
    % 
    %             Vi(k) = Vi(k-1) + Dt * ViDot(k) = Ji v + Dt * (JiDot v + Ji vDot) = Ji v + Dt JiDot v + Dt Ji vDot
    %             Vc(k) = Vc(k-1) + Dt * VcDot(k) = Jc v + Dt * (JcDot v + Jc vDot) = Jc v + Dt JcDot v + Dt Jc vDot
    % 
    %             Using the maximum kinetic energy dissipation principle, vDot, fc, fi can be computed as
    % 
    %             Minimize 0.5 |vDot - vDot_free|_M
    %             subject to:
    %             Ji/Dt v + JiDot v + Ji vDot = 0
    %             Jc/Dt v + JcDot v + Jc vDot = 0
    % 
    %             where vDot_free is the free acceleration of the system.
    %             The KKT conditions of the above problem are as
    % 
    %             M vDot - M vDot_free - Jc^T lc - Ji^T li = 0
    %             Ji/Dt v + JiDot v + Ji vDot = 0
    %             Jc/Dt v + JcDot v + Jc vDot = 0
    % 
    %             It can be shown that lc = Fc and li = Fi.
    %             Substituting VDot from the first equation to the second and third one gives us
    % 
    %             Ji/Dt v + JiDot v + Ji vDot_free + Ji M^-1 (Jc^T Fc + Ji^T Fi) = 0
    %             Jc/Dt v + JcDot v + Jc vDot_free + Jc M^-1 (Jc^T Fc + Ji^T Fi) = 0
    % 
    %             From the first equation, we have
    % 
    %             Fi = - (Ji M^-1 Ji^T)^-1 (Ji/Dt v + JiDot v + Ji vDot_free + Ji M^-1 Jc^T Fc)
    % 
    %             Substituting Fi in the second equation, we obtain
    % 
    %             Jc/Dt v + JcDot v + Jc vDot_free + Jc M^-1 Jc^T Fc - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 (Ji/Dt v + JiDot v + Ji vDot_free + Ji M^-1 Jc^T Fc) = 0
    % 
    %             The above equation can be written as
    % 
    %             H Fc + g = 0
    % 
    %             where
    % 
    %             H = Jc M^-1 Jc^T - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 Ji M^-1 Jc^T
    %             g = Jc/Dt v + JcDot v + Jc vDot_free - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 (Ji/Dt v + JiDot v + Ji vDot_free)
    % 
    %             The above equation can be write as an optimisation problem like
    % 
    %             Minimize 0.5 Fc^T H Fc + g^T Fc
    % 
    %             Now we can add the unilateral and friction cone constraints to the above optimisation problem and write it as
    % 
    %             Minimize 0.5 Fc^T H Fc + g^T Fc
    %             subject to:
    %             Fc_z >= 0
    %             -mu Fc_z <= Fc_x, Fc_y <= mu Fc_z
    % 
    %     **FORMAT**: forces = Contact_object.compute_unilateral_linear_contact(M, h, J_in_contact, J_diff_split_points, JDot_nu_in_contact, JDot_diff_nu_split_points, torque, contact_point, num_closed_chains, generalized_ext_wrench, num_in_contact_frames)
    % 
    %     **INPUT:**
    %             - M:                          [(N+6) x (N+6)]  The inertia matrix
    %             - h:                          [(N+6) x 1]      The Coriolis, Centrifugal, and gravity effect vector
    %             - J_in_contact:               [3m x 1]         The linear Jacobian of the contact vertices
    %             - J_diff_split_points:        [6p x (N+6)]     The difference of the Jacobian of the two prints of the spilit points in the (possible) closed chains
    %             - JDot_nu_in_contact:         [3m x 1]         The linear part of JacobianDot * nu of the contact vertices
    %             - JDot_diff_nu_split_points:  [6p x 1]         The difference of the JacobianDot * nu of the two prints of the spilit points in the (possible) closed chains 
    %             - torque:                     [N x 1]          The joint torques vector
    %             - contact_point:              [m x 1]          The vertical position of the contact points
    %             - num_closed_chains:          [SCALAR]         The number of the closed chains
    %             - generalized_ext_wrench:     [(N+6) x 1]      The external wrenches applied to the robot projected to the state space of the robot
    %             - num_in_contact_frames:      [SCALAR]         The number of the links/frames interacting with the ground
    % 
    %     **OUTPUT:**
    %             - forces:  [(3m+6p) x 1] The contact pure forces and the internal wrenches in the spilit points
    % 
    %     **AUTHORS:**  Venus Pasandi, Nuno Guedelha
    % 
    %     all authors are with the Italian Istitute of Technology (IIT)
    %     email: name.surname@iit.it
    % 
    %     PLACE AND DATE: <Genoa, March 2022>

% ----------------------- INITIALIZATION ----------------------------------

% --------------------------- MAIN ----------------------------------------
free_acceleration = obj.compute_free_acceleration(M, h, torque, generalized_ext_wrench);
free_contact_acceleration_contact = obj.compute_free_contact_acceleration(J_in_contact, free_acceleration, JDot_nu_in_contact);

if useDiscreteContact && num_closed_chains == 0 % there is no closed chain
    
    H = J_in_contact * (M \ J_in_contact');
    g = J_in_contact * [base_pose_dot ; s_dot] / obj.dt + free_contact_acceleration_contact;
    
elseif useDiscreteContact % there are some closed chains
    free_contact_diff_acceleration_internal = obj.compute_free_contact_acceleration(J_diff_split_points, free_acceleration, JDot_diff_nu_split_points);
    JMJ_dmp_pseudo_inv = obj.compute_damped_psudo_inverse(J_diff_split_points * (M \ J_diff_split_points'),0.001);
    
    H = J_in_contact * (M \ J_in_contact') - (J_in_contact * (M \ J_diff_split_points')) * (JMJ_dmp_pseudo_inv * (J_diff_split_points * (M \ J_in_contact') ));
    g = J_in_contact * [base_pose_dot ; s_dot] / obj.dt + free_contact_acceleration_contact - (J_in_contact * (M \ J_diff_split_points')) * (JMJ_dmp_pseudo_inv * (J_diff_split_points * [base_pose_dot ; s_dot] / obj.dt + free_contact_diff_acceleration_internal));
    
elseif ~useDiscreteContact && num_closed_chains == 0 % there is no closed chain
      
    H = J_in_contact * (M \ J_in_contact');
    g = free_contact_acceleration_contact;
    
else % there are some closed chains
    free_contact_diff_acceleration_internal = obj.compute_free_contact_acceleration(J_diff_split_points, free_acceleration, JDot_diff_nu_split_points);
    JMJ_dmp_pseudo_inv = obj.compute_damped_psudo_inverse(J_diff_split_points * (M \ J_diff_split_points'),0.001);
    
    H = J_in_contact * (M \ J_in_contact') - (J_in_contact*(M \ J_diff_split_points')) * (JMJ_dmp_pseudo_inv * (J_diff_split_points * (M \ J_in_contact') ));
    g = free_contact_acceleration_contact - (J_in_contact*(M \ J_diff_split_points')) * (JMJ_dmp_pseudo_inv * free_contact_diff_acceleration_internal);
    
end

if ~issymmetric(H)
    H = (H + H') / 2; % if non sym
end

if obj.useOSQP
    
    for i = 1 : obj.num_vertices * num_in_contact_frames
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
    
    % Count the consecuitive failure of the solver
    if (res.info.status_val == 1)
        obj.fail_counter = 0;
    else
        obj.fail_counter = obj.fail_counter + 1;
    end
    contactForces = res.x;
    
elseif obj.useQPOASES
    
    obj.ulb = 1e10 + zeros(obj.num_vertices * num_in_contact_frames * 3, 1);
    for i = 1 : obj.num_vertices * num_in_contact_frames
        if (contact_point(i) > 0) % vertex NOT in contact with the ground
            obj.ulb(3*i-2:3*i) = 0;
        end
    end
    
    [contactForces,status] = simFunc_qpOASES(H, g, obj.A, obj.Ax_Lb, obj.Ax_Ub, -obj.ulb, obj.ulb);
    
    % Count the consecuitive failure of the solver
    if (status == 0)
        obj.fail_counter = 0;
    else
        obj.fail_counter = obj.fail_counter + 1;
    end
    
else % USE QUADPROG
    
    for i = 1:obj.num_vertices * num_in_contact_frames
        obj.Aeq(i, i * 3) = contact_point(i) > 0;
    end
    obj.ulb = 1e10 + zeros(obj.num_vertices*num_in_contact_frames*3, 1);
    for i = 1 : obj.num_vertices * num_in_contact_frames
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

% compute the internal wrenches of the spilit points in the (possible)
% closed chains
if num_closed_chains == 0 % there is no closed chain
    
    internalWrenches = [];
    
elseif useDiscreteContact % there are some closed chains
    
    JMJ_dmpd_pseudo_inv = obj.compute_damped_psudo_inverse(J_diff_split_points * (M \ J_diff_split_points'),0.001 );
    internalWrenches = -JMJ_dmpd_pseudo_inv * ((J_diff_split_points * (M \ J_in_contact')) * contactForces + free_contact_diff_acceleration_internal + J_diff_split_points * [base_pose_dot ; s_dot] / obj.dt);
    
else % there are some closed chains
    
    JMJ_dmpd_pseudo_inv = obj.compute_damped_psudo_inverse(J_diff_split_points * (M \ J_diff_split_points'),0.001 );
    internalWrenches = -JMJ_dmpd_pseudo_inv * ((J_diff_split_points * (M \ J_in_contact')) * contactForces + free_contact_diff_acceleration_internal);
    
end

forces = [contactForces;internalWrenches];

end