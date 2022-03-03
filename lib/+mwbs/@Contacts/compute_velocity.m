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