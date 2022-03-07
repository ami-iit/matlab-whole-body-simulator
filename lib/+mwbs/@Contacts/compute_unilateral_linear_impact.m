function impulsive_forces = compute_unilateral_linear_impact(obj, M, nu, J_inContact, J_diff_splitPoint, contact_point, closedChains, num_inContact_frames, mapVerticesNewContact)
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
    
    obj.ulb = 1e10 + zeros(obj.num_vertices*num_inContact_frames*3, 1);
    for i = 1 : obj.num_vertices * num_inContact_frames
        if (~obj.is_in_contact(i)) % vertex NOT in contact with the ground
            obj.ulb(3*i-2:3*i) = 0;
        end
    end
    
    allIndexes = 1:numel(obj.is_in_contact);
    mapVerticesAtZeroVel_logical = logical(obj.is_in_contact);
    indexesVerticesAtZeroVel = (allIndexes(mapVerticesAtZeroVel_logical)-1)*3+1; % each vertex has 3 components
    expandedIdxesVerticesAtZeroVel = [indexesVerticesAtZeroVel;indexesVerticesAtZeroVel+1;indexesVerticesAtZeroVel+2];
    expandedIdxesVerticesAtZeroVel_all = expandedIdxesVerticesAtZeroVel(:);
    H_active = H(expandedIdxesVerticesAtZeroVel_all,expandedIdxesVerticesAtZeroVel_all);
    g_active = g(expandedIdxesVerticesAtZeroVel_all);
    
    allIndexes = 1:numel(mapVerticesNewContact);
    mapVerticesAtZeroVel_logical = logical(mapVerticesNewContact);
    indexesVerticesAtZeroVel = (allIndexes(mapVerticesAtZeroVel_logical)-1)*3+1; % each vertex has 3 components
%     expandedIdxesVerticesAtZeroVel = [indexesVerticesAtZeroVel;indexesVerticesAtZeroVel+1;indexesVerticesAtZeroVel+2];
    expandedIdxesVerticesAtZeroVel = indexesVerticesAtZeroVel+2;
    expandedIdxesVerticesAtZeroVel_new = expandedIdxesVerticesAtZeroVel(:);

    allIndexes = 1:numel(obj.was_in_contact);
    mapVerticesAtZeroVel_logical = logical(obj.was_in_contact);
    indexesVerticesAtZeroVel = (allIndexes(mapVerticesAtZeroVel_logical)-1)*3+1; % each vertex has 3 components
%     expandedIdxesVerticesAtZeroVel = [indexesVerticesAtZeroVel;indexesVerticesAtZeroVel+1;indexesVerticesAtZeroVel+2];
    expandedIdxesVerticesAtZeroVel = indexesVerticesAtZeroVel+2;
    expandedIdxesVerticesAtZeroVel_c = expandedIdxesVerticesAtZeroVel(:);
%     A_more = zeros(24,24);
%     A_more(expandedIdxesVerticesAtZeroVel,:) = H(expandedIdxesVerticesAtZeroVel,:);
%     Ax_more = zeros(24,1);
%     Ax_more(expandedIdxesVerticesAtZeroVel) = -g(expandedIdxesVerticesAtZeroVel);
    A_more = zeros(24,24);
%     A_more(expandedIdxesVerticesAtZeroVel_c,:) = H(expandedIdxesVerticesAtZeroVel_c,:);
%     A_more(expandedIdxesVerticesAtZeroVel_new,:) = H(expandedIdxesVerticesAtZeroVel_new,:);
    Ax_more = zeros(24,1) - 1e20;
%     Ax_more(expandedIdxesVerticesAtZeroVel_c) = -g(expandedIdxesVerticesAtZeroVel_c);
%     Ax_more(expandedIdxesVerticesAtZeroVel_new) = -g(expandedIdxesVerticesAtZeroVel_new);
    Ax_Ub_more = zeros(24,1) + 1e20;
%     Ax_Ub_more(expandedIdxesVerticesAtZeroVel_c) = 1e12;
%     Ax_Ub_more(expandedIdxesVerticesAtZeroVel_new) = -g(expandedIdxesVerticesAtZeroVel_new);
%     Ax_Ub_more(expandedIdxesVerticesAtZeroVel_c) = -g(expandedIdxesVerticesAtZeroVel_c);


    allIndexes = 1:numel(obj.was_in_contact);
    mapVerticesAtZeroVel_logical = logical(obj.was_in_contact);
    indexesVerticesAtZeroVel = (allIndexes(mapVerticesAtZeroVel_logical)-1)*5+1; % each vertex has 5 friction constraint
    expandedIdxesVerticesAtZeroVel = [indexesVerticesAtZeroVel;indexesVerticesAtZeroVel+1;indexesVerticesAtZeroVel+2;indexesVerticesAtZeroVel+3;indexesVerticesAtZeroVel+4];
    expandedIdxesVerticesAtZeroVel_old = expandedIdxesVerticesAtZeroVel(:);
    A = zeros(5*2*4,24);
    A(expandedIdxesVerticesAtZeroVel_old,:) = obj.A(expandedIdxesVerticesAtZeroVel_old,:);

%     F_active = -(H_active'*H_active)\(H_active'*g_active) + null(H_active);
%     F = zeros(24,1);
%     F(expandedIdxesVerticesAtZeroVel_all) = F_active;
%     g = -F;
%     H = eye(24);
    [contactForces,status] = simFunc_qpOASES_impact(H' * H, H' * g, [obj.A;A_more], [obj.Ax_Lb;Ax_more], [obj.Ax_Ub;Ax_Ub_more], -obj.ulb, obj.ulb);
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
