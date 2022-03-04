function [generalized_total_wrench, wrench_inContact_frames, base_pose_dot, s_dot] = ...
    compute_contact_closedChain(obj, robot, torque, generalized_ext_wrench, motorInertias, base_pose_dot, s_dot,obj_step_block)
%
%     COMPUTE_CONTACT_CLOSED_CHAIN: Computes the contact forces, the internal wrenches of the spilit points in the (possible) closed chains, and the configuration velocity after a (possible) impact
%
%     PROCEDURE: For modeling the dynamics of a robot having some closed chains, we open each chain by breaking a link in the chain. Then, we apply a constraint wrench to the broken point that the two parts of the broken link become connected.
%             Thus, the equation of motion for a floating base robot having some closed chains can be written as
% 
%             M vDot + h = B u + Fe + Jc' Fc + Ji Fi
% 
%             where
% 
%             u is the joint torques, Fe is the external wrenches, Fc is the pure forces applied to the contact points, and Fi is the internal wrenches in the spilit/broken points.
% 
%             We use the minimum dissipation principle for computing Fc and Fi. In details, we first use a Quadratic Programming for computing Fc as
% 
%             Fc = argmin 0.5 Fc' H Fa + g' Fc
%             subject to: A Fc > 0
% 
%             where A Fc > 0 representes the simplified friction cone constraint, and
% 
%             H = Jc M^-1 Jc' - (Jc M^-1 Ji') (Ji M^-1 Ji')^-1 (Ji vDot + JiDot v)
%             g = Jc vDot + JcDot v - (Jc M^-1 Ji') (Ji M^-1 Ji')^-1 (Ji vDot + JiDot v)
% 
%             Then, we compute Fi through an equation resulted from the minimum dissipation principle as
% 
%             Fi = -(Ji M^-1 Ji')^-1 ( (Ji M^-1 Jc') Fc + Ji vDot + JiDot v )
% 
%             In the case that an impact happens, we also update the velocity vector of the system by
% 
%             v^+ = (I - M^-1 G' (G M-1 G')^-1 G) v^-
% 
%             where G = [Jc;Ji].
%
% 
%     INPUTS:
%             - robot: instance of the Robot object
%             - torque: joint torques
%             - generalized_ext_wrench: wrench transformed using the Jacobian relative to the application point
%             - motorInertias: inertia matrix of the motors
%             - base_pose_dot: 
%             - s_dot: configuration velocity
%             - obj_step_block: 
% 
%     OUTPUTS: 
%             - generalized_total_wrench: the sum of the generalized_ext_wrench and the generalized contact wrench and the internal wrench on the (possible) split points
%             - wrench_inContact_frames: the wrench in sole of the contact frames
%             - base_pose_dot: base velocity vector, changed in the case of an impact with the ground
%             - s_dot: joint velocity vector, changed in the case of an impact with the ground
%
%     **AUTHORS:** Venus Pasandi, Nuno Guedelha
% 
%     all authors are with the Italian Istitute of Technology (IIT)
%     email: name.surname@iit.it
% 
%     PLACE AND DATE: <Genoa, March 2022>
%

% collect the open-chain kinematic dynamic quantities
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

% compute the configuration velocity - same, if no impact - discontinuous in case of impact
[base_pose_dot, s_dot] = compute_velocity(obj, M, G_forces, base_pose_dot, s_dot, obj_step_block.robot_config.closedChains, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND), contact_points);

% computes a 3 * num_total_vertices + 6 * num_closed_chains vector
% containing the pure contact forces acting on every vertex and the
% internal wrenches in the spilit points
forces = obj.compute_unilateral_linear_contact(M, h, J_inContact, J_diff_splitPoint, JDot_nu_inContact, JDot_diff_nu_splitPoint, torque, contact_points, obj_step_block.robot_config.closedChains, generalized_ext_wrench, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));

% transform the contact and the internal wrenches of the spilit points in the closed chains in a wrench acting on the robot
generalized_contact_wrench = G_forces' * forces;

% sum the contact wrench and the internal wrenches of the spilit points to the external one
generalized_total_wrench = generalized_ext_wrench + generalized_contact_wrench;

% compute the wrench in the sole frames, in order to simulate a sensor mounted onto the sole frame
ground_forces = forces(1:end-6*obj_step_block.robot_config.closedChains);
wrench_inContact_frames = obj.compute_contact_wrench_in_sole_frames(ground_forces, robot, length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND));

% update the contact log
obj.was_in_contact = obj.is_in_contact;

end