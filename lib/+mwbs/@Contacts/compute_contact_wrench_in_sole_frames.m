function wrench_in_contact_frames = compute_contact_wrench_in_sole_frames(obj, contact_forces, robot, num_in_contact_frames, num_vertices)

    %     COMPUTE_CONTACT_WRENCHE_IN_SOLE_FRAMES : trasforms the pure forces on foot vertices in wrench in sole frames
    % 
    %     PROCEDURE: The resultant wrench applied to the sole link is equal to the summation of the forces applied to its vertices that is transformed to the link frame, i.e.
    % 
    %             resultant force = i_R_w * (Fi_1 + Fi_2 + .. )
    %             resultant torque = Skew(i_pi_1) i_R_w * Fi_1 + Skew(i_pi_2) i_R_w * Fi_2 + ...
    % 
    %             where 
    %                - i_pi_k is the position corrdinate of the vertex k of link i in the frame of the link i.
    %                - Fi_k is the pure forces applied to the vertex k of the link i
    %                - i_R_w is the rotation matrix from the link i to the world frame
    % 
    %     **FORMAT**: wrench_in_contact_frames = Contact_object.compute_contact_wrench_in_sole_frames(contact_forces, robot, num_in_contact_frames)
    % 
    %     **INPUT:**
    %             - contact_forces:        [(3mk) x 1]     The pure forces applied to the vertices
    %             - robot:                 [ROBOT OBJECT]  The robot object
    %             - num_in_contact_frames: [SCALAR]        The number of the links/frames interacting with the ground
    %             - num_vertices:          [SCALAR]        The number of the contact vertices for each foot
    % 
    %     **OUTPUT:**
    %             - wrench_in_contact_frames:  [(6m) x 1]  The resultant wrench applied to the sole of the links interacting with the ground represented in the corresponding link frame
    % 
    %     **AUTHORS:**  Venus Pasandi, Nuno Guedelha
    % 
    %     all authors are with the Italian Istitute of Technology (IIT)
    %     email: name.surname@iit.it
    % 
    %     PLACE AND DATE: <Genoa, March 2022>


% ---------------------- INITIALIZATION ----------------------------------
wrench_in_contact_frames = zeros(6*num_in_contact_frames,1);


% -------------------------- MAIN -----------------------------------------
for counter = 1 : num_in_contact_frames
    % Rotation matrix of sole w.r.t the world
    H_frame = robot.get_inContactWithGround_H();
    R_frame = H_frame(1:3, 1:3);
    foot_print_frame = obj.foot_print{counter};
    
    wrench_frame = zeros(6, 1);
    % computed contact forces on every vertex
    contact_forces_frame = contact_forces(1+12*(counter-1):12*counter);
    
    for i = 1 : num_vertices
        j = (i - 1) * 3 + 1;
        wrench_frame(1:3) = wrench_frame(1:3) + R_frame' * contact_forces_frame(j:j + 2);
        wrench_frame(4:6) = wrench_frame(4:6) - mwbs.Utils.skew(foot_print_frame(:, i)) * (R_frame' * contact_forces_frame(j:j + 2));
    end
    wrench_in_contact_frames(1+6*(counter-1):6*counter) = wrench_frame;
end

end