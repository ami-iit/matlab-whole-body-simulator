function forces = compute_unilateral_linear_contact(obj, M, h, J_in_contact, J_holonomic_cnstr, JDot_nu_in_contact, JDot_nu_holonomic_cnstr, torque, contact_point, generalized_ext_wrench, num_in_contact_frames, num_vertices, base_pose_dot, s_dot)

    %     COMPUTE_UNILATERAL_LINEAR_CONTACT: computes the pure forces acting on the feet vertices and the internal wrenches applied to the split points in the (possible) closed chains
    % 
    %     PROCEDURE: The equation of motion for a N DOF robotic system can be written as
    % 
    %             M vDot + h = B u + Fe + Jc^T R Fc + Ji^T Fi
    % 
    %             where u is the joint torque, Fe is the external wrenches, 
    %             Fc is the contact forces represented in the contact frame, 
    %             and Fi if the internal wrenches in the spilit points in 
    %             the (possible) closed chains.
    % 
    %             The above dynamics is constrained by the topological constraint of the closed chains and the contact points as
    % 
    %             Ji v = 0
    %             R' Jc v = 0
    % 
    %             In the present function, there are two methods for computing Fc and Fi where the both methods are based on the maximum kinetic energy dissipation principle.
    % 
    %             1) Continuous contact model:
    %             Assuming that Ji v(0) = 0 and Jc v(0) = 0, we can rewrite the above constraint in the acceleration level as
    % 
    %             JiDot v + Ji vDot = 0
    %             R' JcDot v + R' Jc vDot = 0
    % 
    %             Using the maximum kinetic energy dissipation principle, vDot, fc, fi can be computed as
    % 
    %             Minimize 0.5 |vDot - vDot_free|_M
    %             subject to:
    %             JiDot v + Ji vDot = 0
    %             R' JcDot v + R' Jc vDot = 0
    % 
    %             where vDot_free is the free acceleration of the system.
    %             The KKT conditions of the above problem are as
    % 
    %             M vDot - M vDot_free - Jc^T R lc - Ji^T li = 0
    %             JiDot v + Ji vDot = 0
    %             R' JcDot v + R' Jc vDot = 0
    % 
    %             It can be shown that lc = Fc and li = Fi.
    %             Substituting VDot from the first equation to the second and third one gives us
    % 
    %             JiDot v + Ji vDot_free + Ji M^-1 Jc^T R Fc + Ji M^-1 Ji^T Fi = 0
    %             R' JcDot v + R' Jc vDot_free + R' Jc M^-1 Jc^T R Fc + R' Jc M^-1 Ji^T Fi = 0
    % 
    %             From the first equation, we have
    % 
    %             Fi = - (Ji M^-1 Ji^T)^-1 (JiDot v + Ji vDot_free + Ji M^-1 Jc^T R Fc)
    % 
    %             Substituting Fi in the second equation, we obtain
    % 
    %             R' JcDot v + R' Jc vDot_free + (R' Jc M^-1 Jc^T R - R' Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 Ji M^-1 Jc^T R) Fc - R' Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 (JiDot v + Ji vDot_free) = 0
    % 
    %             The above equation can be written as
    % 
    %             H Fc + g = 0
    % 
    %             where
    % 
    %             H = R' (Jc M^-1 Jc^T - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 Ji M^-1 Jc^T) R
    %             g = R' (JcDot v + Jc vDot_free - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 (JiDot v + Ji vDot_free));
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
    %             Vc(k) = Vc(k-1) + Dt * VcDot(k) = R' Jc v + Dt * (R' JcDot v + R' Jc vDot) = R' Jc v + Dt R' JcDot v + Dt R' Jc vDot
    % 
    %             Using the maximum kinetic energy dissipation principle, vDot, fc, fi can be computed as
    % 
    %             Minimize 0.5 |vDot - vDot_free|_M
    %             subject to:
    %             Ji/Dt v + JiDot v + Ji vDot = 0
    %             R' Jc/Dt v + R' JcDot v + R' Jc vDot = 0
    % 
    %             where vDot_free is the free acceleration of the system.
    %             The KKT conditions of the above problem are as
    % 
    %             M vDot - M vDot_free - Jc^T R lc - Ji^T li = 0
    %             Ji/Dt v + JiDot v + Ji vDot = 0
    %             R' Jc/Dt v + R' JcDot v + R' Jc vDot = 0
    % 
    %             It can be shown that lc = Fc and li = Fi.
    %             Substituting VDot from the first equation to the second and third one gives us
    % 
    %             Ji/Dt v + JiDot v + Ji vDot_free + Ji M^-1 (Jc^T R Fc + Ji^T Fi) = 0
    %             R' Jc/Dt v + R' JcDot v + R' Jc vDot_free + R' Jc M^-1 (Jc^T R Fc + Ji^T Fi) = 0
    % 
    %             From the first equation, we have
    % 
    %             Fi = - (Ji M^-1 Ji^T)^-1 (Ji/Dt v + JiDot v + Ji vDot_free + Ji M^-1 Jc^T R Fc)
    % 
    %             Substituting Fi in the second equation, we obtain
    % 
    %             R' Jc/Dt v + R' JcDot v + R' Jc vDot_free + R' Jc M^-1 Jc^T R Fc - R' Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 (Ji/Dt v + JiDot v + Ji vDot_free + Ji M^-1 Jc^T R Fc) = 0
    % 
    %             The above equation can be written as
    % 
    %             H Fc + g = 0
    % 
    %             where
    % 
    %             H = R' ( Jc M^-1 Jc^T - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 Ji M^-1 Jc^T) R
    %             g = R' (Jc/Dt v + JcDot v + Jc vDot_free - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 (Ji/Dt v + JiDot v + Ji vDot_free))
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
    %             - J_holonomic_cnstr:          [6p x (N+6)]     The difference of the Jacobian of the two prints of the spilit points in the (possible) closed chains
    %             - JDot_nu_in_contact:         [3m x 1]         The linear part of JacobianDot * nu of the contact vertices
    %             - JDot_diff_nu_split_points:  [6p x 1]         The difference of the JacobianDot * nu of the two prints of the spilit points in the (possible) closed chains 
    %             - torque:                     [N x 1]          The joint torques vector
    %             - contact_point:              [m x 1]          The vertical position of the contact points
    %             - num_closed_chains:          [SCALAR]         The number of the closed chains
    %             - generalized_ext_wrench:     [(N+6) x 1]      The external wrenches applied to the robot projected to the state space of the robot
    %             - num_in_contact_frames:      [SCALAR]         The number of the links/frames interacting with the ground
    %             - base_pose_dot:              [6 x 1]          The velocity vector of the base link
    %             - s_dot:                      [N x 1]          The configuration velocity vector
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
R_cell = repmat({obj.w_R_c}, num_vertices * num_in_contact_frames,1);  % Repeat Matrix for every vertex as a cell array
R = blkdiag(R_cell{:});
num_holonomic_cnstr = size(J_holonomic_cnstr,1) / 6;
free_contact_diff_acceleration_internal = 1;

% --------------------------- MAIN ----------------------------------------
free_acceleration = obj.compute_free_acceleration(M, h, torque, generalized_ext_wrench);
free_contact_acceleration_contact = obj.compute_free_contact_acceleration(J_in_contact, free_acceleration, JDot_nu_in_contact);

if obj.useDiscreteContact && num_holonomic_cnstr ~= 0 % there are some holonomic constraints
    
    free_contact_diff_acceleration_internal = obj.compute_free_contact_acceleration(J_holonomic_cnstr, free_acceleration, JDot_nu_holonomic_cnstr);
    JMJ_dmp_pseudo_inv = obj.compute_damped_psudo_inverse(J_holonomic_cnstr * (M \ J_holonomic_cnstr'),0.001);
    
    H = J_in_contact * (M \ J_in_contact') - (J_in_contact * (M \ J_holonomic_cnstr')) * (JMJ_dmp_pseudo_inv * (J_holonomic_cnstr * (M \ J_in_contact') ));
    g = J_in_contact * [base_pose_dot ; s_dot] / obj.dt + free_contact_acceleration_contact - (J_in_contact * (M \ J_holonomic_cnstr')) * (JMJ_dmp_pseudo_inv * (J_holonomic_cnstr * [base_pose_dot ; s_dot] / obj.dt + free_contact_diff_acceleration_internal));
    
elseif obj.useDiscreteContact % there is no holonomic constraint
    
    H = J_in_contact * (M \ J_in_contact');
    g = J_in_contact * [base_pose_dot ; s_dot] / obj.dt + free_contact_acceleration_contact;
        
elseif ~obj.useDiscreteContact && num_holonomic_cnstr ~= 0 % there are some holonomic constraints
     
    free_contact_diff_acceleration_internal = obj.compute_free_contact_acceleration(J_holonomic_cnstr, free_acceleration, JDot_nu_holonomic_cnstr);
    JMJ_dmp_pseudo_inv = obj.compute_damped_psudo_inverse(J_holonomic_cnstr * (M \ J_holonomic_cnstr'),0.001);
    
    H = J_in_contact * (M \ J_in_contact') - (J_in_contact*(M \ J_holonomic_cnstr')) * (JMJ_dmp_pseudo_inv * (J_holonomic_cnstr * (M \ J_in_contact') ));
    g = free_contact_acceleration_contact - (J_in_contact*(M \ J_holonomic_cnstr')) * (JMJ_dmp_pseudo_inv * free_contact_diff_acceleration_internal);
    
else % there is no holonomic constraint
     
    H = J_in_contact * (M \ J_in_contact');
    g = free_contact_acceleration_contact;
    
end

% consider the rotation of the contact surface
H = R' * H * R;
g = R' * g;

if ~issymmetric(H)
    H = (H + H') / 2; % if non sym
end

if obj.useOSQP
    
    obj.ulb = 1e10 + zeros(num_vertices * num_in_contact_frames * 3, 1);
    for i = 1 : num_vertices * num_in_contact_frames
        if (contact_point(i) > 0) % vertex NOT in contact with the ground
            obj.ulb(3*i-2:3*i) = 0;
        end
    end
    
    [contactForces,status] = simFunc_OSQP(H, g, obj.A, obj.Ax_Lb, obj.Ax_Ub, -obj.ulb, obj.ulb);
    contactForces_world = R * contactForces;
    
    % Count the consecuitive failure of the solver
    if (status == 0)
        obj.fail_counter = 0;
    else
        obj.fail_counter = obj.fail_counter + 1;
    end
    
elseif obj.useQPOASES
    
    obj.ulb = 1e10 + zeros(num_vertices * num_in_contact_frames * 3, 1);
    for i = 1 : num_vertices * num_in_contact_frames
        if (contact_point(i) > 0) % vertex NOT in contact with the ground
            obj.ulb(3*i-2:3*i) = 0;
        end
    end
    
    [contactForces,status] = simFunc_qpOASES(H, g, obj.A, obj.Ax_Lb, obj.Ax_Ub, -obj.ulb, obj.ulb);
    contactForces_world = R * contactForces;
    
    % Count the consecuitive failure of the solver
    if (status == 0)
        obj.fail_counter = 0;
    else
        obj.fail_counter = obj.fail_counter + 1;
    end
    
else % USE QUADPROG
    
    for i = 1:num_vertices * num_in_contact_frames
        obj.Aeq(i, i * 3) = contact_point(i) > 0;
    end
    obj.ulb = 1e10 + zeros(num_vertices*num_in_contact_frames*3, 1);
    for i = 1 : num_vertices * num_in_contact_frames
        if (contact_point(i) > 0) % vertex NOT in contact with the ground
            obj.ulb(3*i-2:3*i) = 0;
        end
    end
    options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');
    [contactForces,~,exitFlag,~] = quadprog(H, g, obj.A, obj.Ax_Ub, [], [], -obj.ulb, obj.ulb, 100 * ones(size(H,1), 1), options);
    contactForces_world = R * contactForces;
    
    % Counter the consecuitive failure of the solver
    if (exitFlag == 1)
        obj.fail_counter = 0;
    else
        obj.fail_counter = obj.fail_counter + 1;
    end
end

% generate an error if the optimization solver fails to solve
% the optimization problem and obtain the contact forces.
if (obj.fail_counter >= obj.max_consecutive_failures)
    error(strjoin({'[RobotDynWithContacts] The solver fails to compute the contact forces for',sprintf('%d',int8(obj.max_consecutive_failures)),'times'}));
end

% compute the wrenches due to the holonomic constraints
if num_holonomic_cnstr == 0 % there is no holonomic constraint
    
    holonomicWrenches = [];
    
elseif obj.useDiscreteContact % there are some holonomic constraint
    
    JMJ_dmpd_pseudo_inv = obj.compute_damped_psudo_inverse(J_holonomic_cnstr * (M \ J_holonomic_cnstr'),0.001 );
    holonomicWrenches = -JMJ_dmpd_pseudo_inv * ((J_holonomic_cnstr * (M \ J_in_contact')) * contactForces_world + free_contact_diff_acceleration_internal + J_holonomic_cnstr * [base_pose_dot ; s_dot] / obj.dt);
    
else % there are some holonomic constraints
    
    JMJ_dmpd_pseudo_inv = obj.compute_damped_psudo_inverse(J_holonomic_cnstr * (M \ J_holonomic_cnstr'),0.001 );
    holonomicWrenches = -JMJ_dmpd_pseudo_inv * ((J_holonomic_cnstr * (M \ J_in_contact')) * contactForces_world + free_contact_diff_acceleration_internal);
    
end

forces = [contactForces_world;holonomicWrenches];

end
