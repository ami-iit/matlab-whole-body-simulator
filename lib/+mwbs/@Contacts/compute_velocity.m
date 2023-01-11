function [base_pose_dot, s_dot, impact_flag] = compute_velocity(obj, M, G, base_pose_dot, s_dot, num_in_contact_frames, contact_point, num_vertices)

    %     COMPUTE_VELOCITY: computes the robot velocity vector after a (possible) impact
    % 
    %     PROCEDURE: The dynamic equation of the motion of the robot is
    %
    %         M vDot + h = B u + Fe + Jc^T R Fc + Ji^T Fi
    % 
    %         where
    % 
    %         u is the joint torques, Fe is the external wrenches, Fc is 
    %         the pure forces applied to the contact points, and Fi is the
    %         internal wrenches in the spilit/broken points.
    % 
    %         Assuming that the impact time is short enough, the robot pose is
    %         contant during the impact. The velocity after the impact is a 
    %         function of the velocity before the impact under the constraint
    %         that the vertex velocity is equal to zero
    % 
    %         There are two models for the impact:
    % 
    %         1) The frictionless impact model:
    % 
    %         The integration of the equation of the motion during the impact is
    % 
    %         M (v^+ - v^-) = Jc^T R fc + Ji^T fi
    % 
    %         where v^- and v^+ are the robot velocity vector before and after 
    %         the impact, respectively. fc and fi are the impulsive reaction
    %         and internal forces.
    % 
    %         Assuming that the velocity of the contact vertices and the
    %         spilit points are zero after the impact,
    % 
    %         R' Jc v^+ = 0
    %         Ji v^+ = 0
    % 
    %         Putting the above three equations together, we have
    % 
    %           M     -Jc^T R  -Ji^T      v^+      M v^-
    %         [ R' Jc  0        0    ]  [ fc ] = [ 0    ]
    %           Ji     0        0         fi       0
    % 
    %         Using the schur complement, the velocity vector after the impact is
    % 
    %         v^+ = (I - M^-1 G^T (G M^-1 G^T)^-1 G) v^-
    % 
    %         where G = [Jc;Ji].
    % 
    %         2) The frictional impact model:
    % 
    %         we have the following constraint for the closed chains
    % 
    %         Ji v^+ = 0
    % 
    %         Using the integration of the equation of the motion during the impact, we can rewrite the above equation as
    % 
    %         Ji v^- + Ji M^-1 Jc^T R fc + Ji M^-1 Ji^T fi = 0
    % 
    %         Thus, fi is as
    % 
    %         fi = - (Ji M^-1 Ji^T)^-1 (Ji v^- + Ji M^-1 Jc^T R fc)
    % 
    %         Substituting fi in the integration of the equation of the motion during the impact, we have
    % 
    %         M v^+ = ( M - Ji^T (Ji M^-1 Ji^T)^-1 Ji ) v^- + (Jc^T - Ji^T (Ji M^-1 Ji^T)^-1 Ji M^-1 Jc^T) R fc
    % 
    %         On the other hand, the velocity of the contact points after the impact is
    % 
    %         Vc^+ = R' Jc v^+
    % 
    %         Substituting v^+ in the above equation, we have
    % 
    %         Vc^+ = R' (Jc - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 Ji) v^- + (Jc M^-1 Jc^T - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 Ji M^-1 Jc^T) R fc
    % 
    %         The above equation is written usually in the A/b representation as
    % 
    %         Vc^+  = b + A fc
    % 
    %         where
    % 
    %         b = R' (Jc - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 Ji) v^-,
    %         A = R' (Jc M^-1 Jc^T - Jc M^-1 Ji^T (Ji M^-1 Ji^T)^-1 Ji M^-1 Jc^T) R,
    % 
    %         Using the maximum dissipation principle, the impulsive forces fc is as
    % 
    %             Minimize 0.5 fc^T (A fc + 2 b)
    %             subject to:
    %             [A_z b]fc + bz >= 0,
    %             fc_z >= 0,
    %             1^T fc_z <= k,
    %             -mu fc_z <= fc_x, fc_y <= mu fc_z
    %         
    %         references:
    %             - Drumwright, Evan, and Dylan A. Shell. "Modeling contact friction and joint friction in dynamic robotic simulation using the principle of maximum dissipation." Algorithmic foundations of robotics IX. Springer, Berlin, Heidelberg, 2010. 249-266.
    % 
    % 
    %     **FORMAT**: [base_pose_dot, s_dot, impact_flag] = Contact_object.compute_velocity(M, G, base_pose_dot, s_dot, num_closed_chains, num_in_contact_frames, contact_point)
    % 
    %     **INPUT:**
    %                 - M                      [(N+6) x (N+6)]               The inertia matrix of the robot
    %                 - G                      [(3 x m x k + 6 x p) x (N+6)] The group vector of the linear Jacobian of teh contact vertices and the Jacobian of the spilit points in the (possible) closed chains
    %                 - base_pose_dot          [6 x 1]                       The velocity vector of the base link
    %                 - s_dot:                 [N x 1]                       The configuration velocity vector
    %                 - num_closed_chains:     [SCALAR]                      The number of the closed chains
    %                 - num_in_contact_frames: [SCALAR]                      The number of the links interacting with the ground   
    %                 - contact_point:         [(m x k) x 1]                 The vertical position of the vertices
    %                 - num_vertices:          [SCALAR]                      The number of the contact vertices for each foot
    % 
    %     **OUTPUT:**
    %                 - base_pose_dot:         [6 x 1]   The velocity vector of the base link
    %                 - s_dot:                 [N x 1]   The configuration velocity vector
    %                 - impact_flag:           [Boolean] The variable determines if an impact happens
    % 
    %     **AUTHORS:** Venus Pasandi, Nuno Guedelha
    % 
    %     all authors are with the Italian Istitute of Technology (IIT)
    %     email: name.surname@iit.it
    % 
    %     PLACE AND DATE: <Genoa, March 2022>
    
% ---------------- INITIALIZAITON -------------------------------------
NDOF = size(s_dot,1);

num_total_vertices = num_in_contact_frames * num_vertices;
num_contact_forces = 3 * num_total_vertices;

if (size(G,1) > num_contact_forces)
    num_holonomic_cnstr = size(G,1) - num_contact_forces;
    J_feet = G(1 : num_contact_forces, :);
    J_holonomic_cnstr = G(num_contact_forces + 1 : end, :); % Group jacobian of the desired fixed frame + jacobian difference fir the spilit points
else
    num_holonomic_cnstr = 0;
    J_feet = G;
    J_holonomic_cnstr = [];
end

% ----------------- MAIN -----------------------------------------------
% An impact happens if a new vertex become in contact with the ground.
map_vertices_new_contact = obj.is_in_contact & ~obj.was_in_contact;
new_contact = any(map_vertices_new_contact);

if new_contact && ~obj.useFrictionalImpact % A frictionless impact
    impact_flag = true;
    
    mapVerticesAtZeroVel = map_vertices_new_contact | obj.was_in_contact;
    allIndexes = 1:numel(mapVerticesAtZeroVel);
    indexesVerticesAtZeroVel = (allIndexes(mapVerticesAtZeroVel)-1)*3+1; % each vertex has 3 components
    expandedIdxesVerticesAtZeroVel = [indexesVerticesAtZeroVel;indexesVerticesAtZeroVel+1;indexesVerticesAtZeroVel+2];
    expandedIdxesVerticesAtZeroVel = expandedIdxesVerticesAtZeroVel(:);
    J = J_feet(expandedIdxesVerticesAtZeroVel,1:end);
    G = [J;J_holonomic_cnstr];
    
    if (isempty(J_holonomic_cnstr))
        N = (eye(NDOF + 6) - M \ (G' * (G * (M \ G') \ G)));
    else
        N = (eye(NDOF + 6) - M \ (G' * (obj.compute_damped_psudo_inverse(G * (M \ G'),0.001) * G)));
    end
    nu_after_impact = N * [base_pose_dot; s_dot];
    base_pose_dot = nu_after_impact(1:6);
    s_dot = nu_after_impact(7:end);
    
elseif new_contact && obj.useFrictionalImpact % A frictional impact
    impact_flag = true;
    impulsive_forces = compute_unilateral_linear_impact(obj, M, [base_pose_dot; s_dot], J_feet, J_holonomic_cnstr, contact_point, num_holonomic_cnstr, num_in_contact_frames, num_vertices);
    nu_after_impact = [base_pose_dot; s_dot] + M \ (G' * impulsive_forces);
    base_pose_dot = nu_after_impact(1:6);
    s_dot = nu_after_impact(7:end);
    
else % no impact
    impact_flag = false;
    
end
end
