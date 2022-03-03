function wrench_inContactFrames = compute_contact_wrench_in_sole_frames(obj, contact_forces, robot, num_inContact_frames)
% compute_contact_wrench_in_sole_frames trasforms the pure forces on foot vertices in wrench in sole frames

wrench_inContactFrames = zeros(6*num_inContact_frames,1);
for counter = 1 : num_inContact_frames
    % Rotation matrix of sole w.r.t the world
    H_frame = robot.get_inContactWithGround_H();
    R_frame = H_frame(1:3, 1:3);
    foot_print_frame = obj.foot_print{counter};
    
    wrench_frame = zeros(6, 1);
    % computed contact forces on every vertex
    contact_forces_frame = contact_forces(1+12*(counter-1):12*counter);
    
    for i = 1 : obj.num_vertices
        j = (i - 1) * 3 + 1;
        wrench_frame(1:3) = wrench_frame(1:3) + R_frame' * contact_forces_frame(j:j + 2);
        wrench_frame(4:6) = wrench_frame(4:6) - mwbs.Utils.skew(foot_print_frame(:, i)) * (R_frame' * contact_forces_frame(j:j + 2));
    end
    wrench_inContactFrames(1+6*(counter-1):6*counter) = wrench_frame;
end

end