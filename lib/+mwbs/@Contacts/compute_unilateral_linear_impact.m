function impulsive_forces = compute_unilateral_linear_impact(obj, M, nu, J_inContact, J_diff_splitPoint, contact_point, closedChains, num_inContact_frames)
% compute_unilateral_linear_contact returns the pure forces
% acting on the feet vertices and the internal force/torques
% applied to the split points if there is a closed chain
% kinematic

if (closedChains == 0) % there is no closed chain
    H = J_inContact * (M \ J_inContact');
    g = J_inContact * nu;
else % there are some closed chains
    H = J_inContact * (M \ J_inContact') - (J_inContact * (M \ J_diff_splitPoint')) * (obj.compute_damped_psudo_inverse(J_diff_splitPoint * (M \ J_diff_splitPoint'),0.001) * (J_diff_splitPoint * (M \ J_inContact') ));
    g = (J_inContact - J_inContact * ( M \ J_diff_splitPoint') * obj.compute_damped_psudo_inverse(J_diff_splitPoint * (M \ J_diff_splitPoint') ,0.001 ) * J_diff_splitPoint) * nu;
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
    
    obj.ulb = 1e10 + zeros(3*obj.num_vertices*num_inContact_frames, 1);
    for i = 1 : obj.num_vertices * num_inContact_frames
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
    
    A_phase_II = [obj.A;-diag(obj.is_in_contact)*H_n;expandedNormalForces];
    A_Ub_phase_II = [obj.Ax_Ub;diag(obj.is_in_contact)*g_n;maximumNormalForceMagnitude];
    A_Lb_phase_II = -1e10 + zeros(1+(5+1)*8,1);
    
    [contactForces,status] = simFunc_qpOASES_impact_phase_II(H, g, A_phase_II, A_Lb_phase_II, A_Ub_phase_II, -obj.ulb, obj.ulb);
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
    internalWrenches = -obj.compute_damped_psudo_inverse(J_diff_splitPoint * (M \ J_diff_splitPoint'),0.001) * ((J_diff_splitPoint * ( M \ J_inContact')) * contactForces + J_diff_splitPoint * nu);
end
impulsive_forces = [contactForces;internalWrenches];
end
