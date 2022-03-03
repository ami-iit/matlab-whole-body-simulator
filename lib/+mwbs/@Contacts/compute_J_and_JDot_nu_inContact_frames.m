function [J_print, JDot_nu_print] = compute_J_and_JDot_nu_inContact_frames(obj, robot, num_inContact_frames)
% compute_J_and_JDot_nu returns the Jacobian and J_dot_nu relative to the vertices (Not the sole frames!)

H_inGroundContact = robot.get_inContactWithGround_H();
J_inGroundContact = robot.get_inContactWithGround_jacobians();
JDot_nu_inGroundContact = robot.get_inContactWithGround_JDot_nu();

J_print = zeros(3*obj.num_vertices*num_inContact_frames,size(J_inGroundContact,2));
JDot_nu_print = zeros(3*obj.num_vertices*num_inContact_frames,size(JDot_nu_inGroundContact,2));
for counter = 1 : num_inContact_frames
    H_frame = H_inGroundContact(1+4*(counter-1):4*counter,:);
    J_frame = J_inGroundContact(1+6*(counter-1):6*counter,:);
    JDot_nu_frame = JDot_nu_inGroundContact(1+6*(counter-1):6*counter,:);
    
    J_lin = J_frame(1:3, :);
    J_ang = J_frame(4:6, :);
    JDot_nu_lin = JDot_nu_frame(1:3, :);
    JDot_nu_ang = JDot_nu_frame(4:6, :);
    R_frame = H_frame(1:3, 1:3);
    
    % the vertices are affected by pure forces. We need only the linear Jacobians
    % for a vertex i:
    % Ji = J_linear - S(R*pi) * J_angular
    % JDot_nui = JDot_nu_linear - S(R*pi) * JDot_nu_angular
    J_frame_print = zeros(3*obj.num_vertices,size(J_inGroundContact,2));
    JDot_nu_frame_print = zeros(3*obj.num_vertices,size(JDot_nu_inGroundContact,2));
    for ii = 1:obj.num_vertices
        j = (ii - 1) * 3 + 1;
        foot_print_frame = obj.foot_print{counter};
        v_coords = foot_print_frame(:, ii);
        J_frame_print(j:j + 2, :) = J_lin - mwbs.Utils.skew(R_frame * v_coords) * J_ang;
        JDot_nu_frame_print(j:j + 2, :) = JDot_nu_lin - mwbs.Utils.skew(R_frame * v_coords) * JDot_nu_ang;
    end
    J_print(1+(counter-1)*3*obj.num_vertices:counter*3*obj.num_vertices,:) = J_frame_print;
    JDot_nu_print(1+(counter-1)*3*obj.num_vertices:counter*3*obj.num_vertices,:) = JDot_nu_frame_print;
    
end
end