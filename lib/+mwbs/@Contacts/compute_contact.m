function [generalized_total_wrench, wrench_left_foot, wrench_right_foot, base_pose_dot, s_dot] = ...
    compute_contact(obj, robot, torque, generalized_ext_wrench, motorInertias, base_pose_dot, s_dot,obj_step_block)
% compute_contact Computes the contact forces and the configuration velocity after a (possible) impact
% INPUTS: - robot: instance of the Robot object
%         - torque: joint torques
%         - generalized_ext_wrench: wrench transformed using the Jacobian relative to the application point
%         - base_pose_dot, s_dot: configuration velocity
% OUTPUTS: - generalized_total_wrench: the sum of the generalized_ext_wrench and the generalized contact wrench
%          - wrench_left_foot, wrench_right_foot: the wrench in sole frames
%          - base_pose_dot, s_dot: configuration velocity, changed in the case of an impact with the ground

% collecting robot quantities
h = robot.get_bias_forces();
M = robot.get_mass_matrix(motorInertias,obj_step_block);
[J_feet, JDot_nu_feet] = obj.compute_J_and_JDot_nu_inContact_frames(robot,length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
% compute the vertical distance of every vertex from the ground
contact_points = obj.compute_contact_points(robot, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
% computes a 3 * num_total_vertices vector containing the pure forces acting on every vertes
contact_forces = obj.compute_unilateral_linear_contact(M, h, J_feet, [], JDot_nu_feet, [], torque, contact_points, 0, generalized_ext_wrench, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
% transform the contact in a wrench acting on the robot
generalized_contact_wrench = J_feet' * contact_forces;
% sum the contact wrench to the external one
generalized_total_wrench = generalized_ext_wrench + generalized_contact_wrench;
% compute the wrench in the sole frames, in order to simulate a sensor mounted onto the sole frame
[wrench_left_foot, wrench_right_foot] = obj.compute_contact_wrench_in_sole_frames(contact_forces, robot, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
% compute the configuration velocity - same, if no impact - discontinuous in case of impact
[base_pose_dot, s_dot] = obj.compute_velocity(M, J_feet, robot, base_pose_dot, s_dot, obj_step_block.robot_config.closedChains, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));

% update the contact log
obj.was_in_contact = obj.is_in_contact;

end