function contact_points = compute_contact_points(obj, robot, num_in_contact_frames, num_vertices)

    %     COMPUTE_CONTACT_POINTS: computes the vertical position of every vertex
    %                             and determine if each vertex is in contact with
    %                             the ground or not
    % 
    %     PROCEDURE: The position of the vertex i in the world frame is as
    % 
    %                 P_i = H_sole_i * [p_i_sole ; 1]
    % 
    %                where H_sole_i is the homogenous transformation matrix from the 
    %                frame i to the world frame and p_i_sole is the position of the
    %                vertex i in the frame i.
    % 
    %                The vertical position of the vertex i is equal to P_i(3).
    %                The vertex i is in contact with the ground if
    % 
    %                 P_i(3) <= 0
    % 
    % 
    %     **FORMAT**: contact_points = Contact_object.compute_contact_points(robot, num_inContact_frames)
    % 
    %     **INPUT:**
    %                 - robot                  [ROBOT OBJECT]
    %                 - num_in_contact_frames: [SCALAR]       The number of the frames interacting with the ground
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
        % transforms the coordinates of the vertex (in sole frame) in the world frame
        z = H_frame * [foot_print_frame(:, ii); 1];
        z_frame_print(ii) = z(3);
        
        % the vertex is in contact if its z <= 0
        obj.is_in_contact(num_vertices*(counter-1)+ii) = z_frame_print(ii) <= 0;
    end
    jj = 1 + num_vertices * (counter - 1) : num_vertices * counter;
    contact_points(jj,1) = z_frame_print;
end
end