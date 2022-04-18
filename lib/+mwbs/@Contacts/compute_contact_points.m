function contact_points = compute_contact_points(obj, robot, num_in_contact_frames, num_vertices)

    %     COMPUTE_CONTACT_POINTS: computes the vertical position of every vertex
    %                             and determine if each vertex is in contact with
    %                             the ground or not
    % 
    %     PROCEDURE: The position of the vertex i in the world frame is as
    % 
    %                 P_i = H_sole_i x [p_i_sole ; 1]
    % 
    %                where H_sole_i is the homogenous transformation matrix from the 
    %                frame i to the world frame and p_i_sole is the position of the
    %                vertex i in the frame i. The position of the vertex i in the contact frame is as
    %
    %                 c_P_i = w_R_c x P_i(1:3)
    % 
    %                If the foot is circular, any points in the boundary of the foot
    %                can be in contact with the ground. However, the contact point is
    %                always the lower point of the circle. Assuming that the origin of 
    %                the frame of the sole foot is in the center of the circle, the
    %                position of the lower point of the circle is as
    % 
    %                 P_i = H_sole_i * [p_i_sole_x ; p_i_sole_y ; 0 ; 1]
    % 
    %                the position of the lower point of the circle in the contact frame is as
    %                 
    %                 c_P_i = w_R_c x P_i(1:3) - R
    %
    %                where R is the radius of the circle.
    %
    %                The vertical position of the vertex i is equal to c_P_i(3).
    %                The vertex i is in contact with the ground if
    % 
    %                 c_P_i(3) <= 0
    % 
    % 
    %     **FORMAT**: contact_points = Contact_object.compute_contact_points(robot, num_inContact_frames)
    % 
    %     **INPUT:**
    %                 - robot                  [ROBOT OBJECT]
    %                 - num_in_contact_frames: [SCALAR]       The number of the frames interacting with the ground
    %                 - flag_circular_feet:    [BOOLEAN]      Determines if the feet are circular 
    % 
    %     **OUTPUT:**
    %                 - contact_points:  [(m x k) x 1] The vertical coordinate of the vertices
    % 
    %     **AUTHORS:**  Venus Pasandi, Nuno Guedelha
    % 
    %     all authors are with the Italian Istitute of Technology (IIT)
    %     email: name.surname@iit.it
    % 
    %     PLACE AND DATE: <Genoa, March 2022>

% -------------------- INITIALIZATION --------------------------------
contact_points = zeros(num_in_contact_frames * num_vertices,1);
H_in_contact_with_ground = robot.get_inContactWithGround_H();

% -------------------- MAIN ------------------------------------------
for counter = 1 : num_in_contact_frames
    H_frame = H_in_contact_with_ground(1+4*(counter-1):4*counter,:);
    foot_print_frame = obj.foot_print{counter};
    z_frame_print = zeros(num_vertices, 1);
    for ii = 1 : num_vertices
        if obj.useCircularFeet % the foot is circular
            % transforms the coordinates of the center of the circle (in
            % sole frame) in the world frame - radius of the circle
            p_origin_w = H_frame * [foot_print_frame(1:2, ii); 0; 1];
            p_origin_c = obj.w_R_c' * p_origin_w(1:3);
            p_c = p_origin_c + [0;0;foot_print_frame(3, ii)];
        else % the foot is rectangular
            % transforms the coordinates of the vertex (in sole frame) in the world frame
            p_c_w = H_frame * [foot_print_frame(:, ii); 1];
            p_c = obj.w_R_c' * p_c_w(1:3);
        end
        z_frame_print(ii) = p_c(3);
        
        % the vertex is in contact if its z <= 0
        obj.is_in_contact(num_vertices*(counter-1)+ii) = z_frame_print(ii) <= 0;
    end
    jj = 1 + num_vertices * (counter - 1) : num_vertices * counter;
    contact_points(jj,1) = z_frame_print;
end
end