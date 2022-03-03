function [generalized_total_wrench, wrench_inContact_frames, base_pose_dot, s_dot] = ...
    compute_contact_closedChain(obj, robot, torque, generalized_ext_wrench, motorInertias, base_pose_dot, s_dot,obj_step_block)
% compute_contact_closedChains Computes the contact forces, the internal wrenches of the spilit points in the (possible) closed chains, and the configuration velocity after a (possible) impact
% INPUTS: - robot: instance of the Robot object
%         - closedChains_config: information about the closed
%         chains
%         - torque: joint torques
%         - generalized_ext_wrench: wrench transformed using the Jacobian relative to the application point
%         - base_pose_dot, s_dot: configuration velocity
% OUTPUTS: - generalized_total_wrench: the sum of the
% generalized_ext_wrench and the generalized contact wrench and
% the internal wrench on the split points
%          - wrench_left_foot, wrench_right_foot: the wrench in sole frames
%          - base_pose_dot, s_dot: configuration velocity, changed in the case of an impact with the ground

% collecting the open-chain robot quantities
h = robot.get_bias_forces();
M = robot.get_mass_matrix(motorInertias,obj_step_block);
[J_inContact, JDot_nu_inContact] = obj.compute_J_and_JDot_nu_inContact_frames(robot,length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
if (obj_step_block.robot_config.closedChains == 0)
    J_diff_splitPoint = [];
    JDot_diff_nu_splitPoint = [];
    G_forces = J_inContact;
else
    [J_diff_splitPoint, JDot_diff_nu_splitPoint] = compute_J_and_JDot_nu_splitPoint(obj, robot);
    G_forces = [J_inContact;J_diff_splitPoint];
end
% compute the vertical distance of every vertex from the ground
contact_points = obj.compute_contact_points(robot, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
% computes a 3 * num_total_vertices vector containing the pure forces acting on every vertes
forces = obj.compute_unilateral_linear_contact(M, h, J_inContact, J_diff_splitPoint, JDot_nu_inContact, JDot_diff_nu_splitPoint, torque, contact_points, obj_step_block.robot_config.closedChains, generalized_ext_wrench, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
% transform the contact and the internal wrenches of the spilit points in the closed chains in a wrench acting on the robot
generalized_contact_wrench = G_forces' * forces;
% sum the contact wrench and the internal wrenches of the spilit points to the external one
generalized_total_wrench = generalized_ext_wrench + generalized_contact_wrench;
% compute the wrench in the sole frames, in order to simulate a sensor mounted onto the sole frame
ground_forces = forces(1:end-6*obj_step_block.robot_config.closedChains);
wrench_inContact_frames = obj.compute_contact_wrench_in_sole_frames(ground_forces, robot, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
% compute the configuration velocity - same, if no impact - discontinuous in case of impact
[base_pose_dot, s_dot] = obj.compute_velocity(M, G_forces, robot, base_pose_dot, s_dot,obj_step_block.robot_config.closedChains, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));
% update the contact log
obj.was_in_contact = obj.is_in_contact;
end