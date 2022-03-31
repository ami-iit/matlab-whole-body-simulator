function [J_print, JDot_nu_print] = compute_J_and_JDot_nu_in_contact_frames(obj, robot, num_in_contact_frames, num_vertices)

    %     COMPUTE_J_AND_JDOT_NU_IN_CONTACT_FRAMES : computes the Jacobian and J_dot_nu relative to the vertices (Not the sole frames!)
    % 
    %     PROCEDURE: The vertices are affected by pure forces. Thus, we need only the
    %                linear Jacobians. For a vertex i:
    % 
    %                     Ji = J_linear_sole - S(Pi) * J_angular_sole
    %                     JDot_nui = JDot_nu_linear_sole - S(Pi) *
    %                     JDot_nu_angular_sole + (w_f * w_f' - |w_f|^2) Pi
    %                 
    %                 where Pi is the absolute position of the contact
    %                 vertex i
    % 
    %     **FORMAT**: [J_print, JDot_nu_print] = Contact_obj.compute_J_and_JDot_nu_in_contact_frames(robot, num_inContact_frames)
    % 
    %     **INPUT:**
    %                 - robot:                 [ROBOT OBJECT] 
    %                 - num_in_contact_frames  [SCALAR]       The number of the links interacting with the ground
    %                 - num_vertices           [SCALAR]       The number of the contact vertices for each foot
    % 
    %     **OUTPUT:**
    %                 - J_print:        [(3*m*k) x (N+6)] The group of the linear part of the Jacobian matrix of the vertices 
    %                 - JDot_nu_print:  [(3*m*k) x 1]     The group of the linear part of JDot * nu vector of the vertices
    % 
    %     **AUTHORS:** Venus Pasandi, Nuno Guedelha
    % 
    %     all authors are with the Italian Istitute of Technology (IIT)
    %     email: name.surname@iit.it
    % 
    %     PLACE AND DATE: <Genoa, March 2022>

% -------------------- INITIALIZATION ----------------------------
NDOF = robot.NDOF + 6;

J_print = zeros(3 * num_vertices * num_in_contact_frames, NDOF);
JDot_nu_print = zeros(3 * num_vertices * num_in_contact_frames, 1);

H_in_ground_contact = robot.get_inContactWithGround_H();
J_in_ground_contact = robot.get_inContactWithGround_jacobians();
JDot_nu_in_ground_contact = robot.get_inContactWithGround_JDot_nu();
[base_pose_dot,s_dot] = robot.get_robot_velocity();
nu = [base_pose_dot;s_dot];

% ------------------- MAIN ----------------------------------------
for counter = 1 : num_in_contact_frames
    
    H_frame = H_in_ground_contact(1+4*(counter-1) : 4*counter,:); % The homogenous transformation matrix of the sole frame
    J_frame = J_in_ground_contact(1+6*(counter-1) : 6*counter,:); % The Jacobian matrix of the sole frame
    JDot_nu_frame = JDot_nu_in_ground_contact(1+6*(counter-1) : 6*counter,:); % The JDot * nu of the sole frame
    
    J_lin = J_frame(1:3, :); % The linear Jacobian of the sole frame 
    J_ang = J_frame(4:6, :); % The angular Jacobian of the sole frame
    
    JDot_nu_lin = JDot_nu_frame(1:3, :);
    JDot_nu_ang = JDot_nu_frame(4:6, :);
    
    R_frame = H_frame(1:3, 1:3);
    
    J_frame_print = zeros(3 * num_vertices,NDOF);
    JDot_nu_frame_print = zeros(3 * num_vertices,1);
    
    for ii = 1 : num_vertices
        j = (ii - 1) * 3 + 1;
        foot_print_frame = obj.foot_print{counter};
        v_coords = foot_print_frame(:, ii);
        w_f = J_ang * nu;
        if obj.useCircularFeet % the foot is circular
            v_coords_local = [v_coords(1:2);0];
            v_coords_global = [0;0;v_coords(3)];
            J_frame_print(j:j + 2, :) = J_lin - mwbs.Utils.skew(R_frame * v_coords_local + v_coords_global) * J_ang;
            JDot_nu_frame_print(j:j + 2, :) = JDot_nu_lin - mwbs.Utils.skew(R_frame * v_coords_local + v_coords_global) * JDot_nu_ang + (w_f * w_f' - w_f' * w_f * eye(3)) * (R_frame * v_coords_local + v_coords_global);
        else % the foot is flat
            J_frame_print(j:j + 2, :) = J_lin - mwbs.Utils.skew(R_frame * v_coords) * J_ang;
            JDot_nu_frame_print(j:j + 2, :) = JDot_nu_lin - mwbs.Utils.skew(R_frame * v_coords) * JDot_nu_ang + (w_f * w_f' - w_f' * w_f * eye(3)) * (R_frame * v_coords);
        end
    end
    
    j = 1 + (counter-1) * 3 * num_vertices : counter * 3 * num_vertices;
    J_print(j, :) = J_frame_print;
    JDot_nu_print(j, :) = JDot_nu_frame_print;
    
end
end