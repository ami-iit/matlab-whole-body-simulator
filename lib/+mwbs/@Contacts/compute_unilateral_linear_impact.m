function impulsive_forces = compute_unilateral_linear_impact(obj, M, nu, J_in_contact, J_diff_split_points, contact_point, num_closed_chains, num_in_contact_frames, num_vertices)

    %     COMPUTE_UNILATERAL_LINEAR_IMPACT : computes the impulsive pure forces acting on the feet vertices and the impulsive internal wrenches applied to the split points in the (possible) closed chain
    % 
    %     PROCEDURE: The dynamic equation of the motion of the robot is
    % 
    %             M vDot + h = B u + Fe + Jc^T R Fc + Ji^T Fi
    % 
    %             where
    % 
    %             u is the joint torques, Fe is the external wrenches, Fc is 
    %             the pure forces applied to the contact points, and Fi is the
    %             internal wrenches in the spilit/broken points.
    % 
    %             Assuming that the impact time is short enough, the robot pose is
    %             contant during the impact. The velocity after the impact is a 
    %             function of the velocity before the impact under the constraint
    %             that the vertex velocity is equal to zero
    % 
    %             The integration of the equation of the motion during the impact is
    % 
    %             M (v^+ - v^-) = Jc^T R fc + Ji^T fi
    % 
    %             where v^- and v^+ are the robot velocity vector before and after 
    %             the impact, respectively. fc and fi are the impulsive reaction
    %             and internal forces.
    % 
    %             we have the following constraint for the closed chains
    % 
    %             Ji v^+ = 0
    % 
    %             Using the integration of the equation of the motion during the impact, we can rewrite the above equation as
    % 
    %             Ji v^- + Ji M^-1 Jc^T R fc + Ji M^-1 Ji^T fi = 0
    % 
    %             Thus, fi is as
    % 
    %             fi = -(Ji M^-1 Ji^T)^-1 (Ji v^- + Ji M^-1 Jc^T R fc)
    % 
    %             Substituting fi in the integration of the equation of the motion during the impact, we have
    % 
    %             M v^+ = ( M - Ji^T (Ji M^-1 Ji^T)^-1 Ji ) v^- + (Jc^T - Ji^T (Ji M^-1 Ji^T)^-1 Ji M^-1 Jc^T) R fc
    % 
    %             On the other hand, the velocity of the contact points after the impact is
    % 
    %             Vc^+ = R' Jc v^+
    % 
    %             Substituting v^+ in the above equation, we have
    % 
    %             Vc^+ = R' (Jc - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 Ji) v^- + R' (Jc M^-1 Jc^T - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 Ji M^-1 Jc^T) R fc
    % 
    %             The above equation is written usually in the A/b representation as
    % 
    %             Vc^+  = b + A fc
    % 
    %             where
    % 
    %             b = R' (Jc - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 Ji) v^-,
    %             A = R' (Jc M^-1 Jc^T - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 Ji M^-1 Jc^T) R,
    % 
    %             Using the maximum dissipation principle, the impulsive forces fc is as
    % 
    %                 Minimize 0.5 fc^T (A fc + 2 b)
    %                 subject to:
    %                 [A_z b]fc + bz >= 0,
    %                 fc_z >= 0,
    %                 1^T fc_z <= k,
    %                 -mu fc_z <= fc_x, fc_y <= mu fc_z
    % 
    %             References:
    %                 [1] Drumwright, Evan, and Dylan A. Shell. "Modeling contact friction and joint friction in dynamic robotic simulation using the principle of maximum dissipation." Algorithmic foundations of robotics IX. Springer, Berlin, Heidelberg, 2010. 249-266.
    % 
    % 
    %     **FORMAT**: impulsive_forces = Contact_object.compute_unilateral_linear_impact(M, nu, J_in_contact, J_diff_split_points, contact_point, num_closed_chains, num_in_contact_frames)
    % 
    %     **INPUT:**
    %             - M:                      [(N+6) x (N+6)]  The inertia matrix
    %             - nu:                     [(N+6) x 1]      The velocity vector of the system
    %             - J_in_contact:           [3m x (N+6)]     The group linear Jacobian of the contact vertices
    %             - J_diff_split_points:    [6p x (N+6)]     The difference between the Jacobian of the prints of the spilit points in the (possible) closed chains
    %             - contact_point:          [m x 1]          The vertical coordinate of the contact vertices
    %             - num_closed_chains:      [SCALAR]         The number of the closed chains
    %             - num_in_contact_frames:  [SCALAR]         The number of the links/frames interacting with the ground
    %             - num_vertices:           [SCALAR]         The number of the contact vertices for each foot
    % 
    %     **OUTPUT:**
    %             - impulsive_forces:  [(3m+6p) x 1]  The impulsive contact forces and the impulsive internal wrenches in the spilit points
    % 
    %     **AUTHORS:**  Venus Pasandi, Nuno Guedelha
    % 
    %     all authors are with the Italian Istitute of Technology (IIT)
    %     email: name.surname@iit.it
    % 
    %     PLACE AND DATE: <Genoa, March 2022>
    %

% ------------------------- INITIALIZATION -------------------------------
R_cell = repmat({obj.w_R_c}, num_vertices * num_in_contact_frames,1);  % Repeat Matrix for every vertex as a cell array
R = blkdiag(R_cell{:});

% ---------------------------- MAIN --------------------------------------
if num_closed_chains == 0 % there is no closed chain
    
    H = J_in_contact * (M \ J_in_contact');
    g = J_in_contact * nu;
    
else % there are some closed chains
    JMJ_dmpd_pseudo_inv = obj.compute_damped_psudo_inverse(J_diff_split_points * (M \ J_diff_split_points'),0.001);
    
    H = J_in_contact * (M \ J_in_contact') - (J_in_contact * (M \ J_diff_split_points')) * (JMJ_dmpd_pseudo_inv * (J_diff_split_points * (M \ J_in_contact') ));
    g = (J_in_contact - J_in_contact * ( M \ J_diff_split_points') * JMJ_dmpd_pseudo_inv * J_diff_split_points) * nu;
    
end

% consider the rotation of the contact surface
H = R' * H * R;
g = R' * g;

if ~issymmetric(H)
    H = (H + H') / 2; % if non sym
end

if obj.useOSQP
    for i = 1:num_vertices * num_in_contact_frames
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

    contactForces = res.x;
    contactForces_world = R * contactForces;
    
elseif obj.useQPOASES
    
    obj.ulb = 1e10 + zeros(3*num_vertices*num_in_contact_frames, 1);
    for i = 1 : num_vertices * num_in_contact_frames
        if (contact_point(i) > 0) % vertex NOT in contact with the ground
            obj.ulb(3*i-2:3*i) = 0;
        end
    end
    
    allIndexes = 1:numel(obj.is_in_contact);
    mapVertices = true(numel(obj.is_in_contact),1);
    indexesVertices = (allIndexes(mapVertices) - 1) * 3 + 1; % each vertex has 3 components
    indexesVerticesNormalDirection = indexesVertices + 2;
    indexesVerticesNormalDirection = indexesVerticesNormalDirection(:);
    
    normalForces = [0,0,1];
    expandedNormalForces = repmat(normalForces,[1,numel(obj.is_in_contact)]);
    
    H_nn = H(indexesVerticesNormalDirection,indexesVerticesNormalDirection);
    H_n = H(indexesVerticesNormalDirection,:);
    g_n = g(indexesVerticesNormalDirection);
    
    % The first phase of the impact model
    lb_phase_I = zeros(numel(obj.is_in_contact),1);
    ub_phase_I = obj.ulb(indexesVerticesNormalDirection);
    
    [contactForcesNormal,~] = simFunc_qpOASES_impact_phase_I(H_nn, g_n, lb_phase_I, ub_phase_I);
    
    % The second phase of the impact model
    maximumNormalForceMagnitude = sum(contactForcesNormal);
    
    A_phase_II = [obj.A; -diag(obj.is_in_contact)*H_n; expandedNormalForces];
    A_Ub_phase_II = [obj.Ax_Ub; diag(obj.is_in_contact)*g_n; maximumNormalForceMagnitude];
    A_Lb_phase_II = -1e10 + zeros(1+(5+1)* num_vertices * num_in_contact_frames,1);
    
    [contactForces, ~] = simFunc_qpOASES_impact_phase_II(H, g, A_phase_II, A_Lb_phase_II, A_Ub_phase_II, -obj.ulb, obj.ulb);
    contactForces_world = R * contactForces;
    
else
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
    [contactForces,~,~,~] = quadprog(H, g, obj.A, obj.Ax_Ub, [], [], -obj.ulb, obj.ulb, 100 * ones(size(H,1), 1), options);
    contactForces_world = R * contactForces;
    
end

% compute the internal wrenches of the spilit points if there
% is closed chain
if (num_closed_chains == 0)
    internalWrenches = [];
else
    internalWrenches = - JMJ_dmpd_pseudo_inv * ((J_diff_split_points * ( M \ J_in_contact')) * contactForces_world + J_diff_split_points * nu);
end

impulsive_forces = [contactForces_world;internalWrenches];

end
