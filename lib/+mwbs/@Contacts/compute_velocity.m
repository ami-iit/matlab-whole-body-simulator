function [base_pose_dot, s_dot] = compute_velocity(obj, M, G, base_pose_dot, s_dot, closed_chains, num_inContact_frames, contact_point)
% compute_velocity returns the configuration velocity
% the velocity does not change if there is no impact
% the velocity change if there is the impact

num_total_vertices = num_inContact_frames * obj.num_vertices;

if size(G,1) == (3 * num_total_vertices)
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

% compute the projection in the null space of the scaled Jacobian of the vertices if a
% new contact is detected, otherwise just resuse the input variables (base_pose_dot,
% s_dot).
if new_contact
    % the velocity after the impact is a function of the velocity before the impact
    % under the constraint that the vertex velocity is equal to zeros
    impulsive_forces = compute_unilateral_linear_impact(obj, M, [base_pose_dot; s_dot], J_feet, J_split_points, contact_point, closed_chains, num_inContact_frames, mapVerticesNewContact);
    nu_after_impact = [base_pose_dot; s_dot] + M \ (G' * impulsive_forces);
    base_pose_dot = nu_after_impact(1:6);
    s_dot = nu_after_impact(7:end);
end
end
