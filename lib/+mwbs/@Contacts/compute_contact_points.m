function contact_points = compute_contact_points(obj, robot, num_inContact_frames)
% contact_points returns the vertical coordinate of every vertex
contact_points = zeros(num_inContact_frames*obj.num_vertices,1);

% checks if the vertex is in contact with the ground
H_IN_CONTACT_WITH_GROUND = robot.get_inContactWithGround_H();
for counter = 1 : num_inContact_frames
    H_frame = H_IN_CONTACT_WITH_GROUND(1+4*(counter-1):4*counter,:);
    foot_print_frame = obj.foot_print{counter};
    z_frame_print = zeros(obj.num_vertices, 1);
    for ii = 1 : obj.num_vertices
        % transforms the coordinates of the vertex (in sole frame) in the world frame
        z = H_frame * [foot_print_frame(:, ii); 1];
        z_frame_print(ii) = z(3);
        % the vertex is in contact if its z <= 0
        obj.is_in_contact(obj.num_vertices*(counter-1)+ii) = z_frame_print(ii) <= 0;
    end
    contact_points(1+obj.num_vertices*(counter-1):obj.num_vertices*counter,1) = z_frame_print;
end
end