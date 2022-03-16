function [generalized_total_wrench, wrench_in_contact_frames, base_pose_dot, s_dot] = compute_contact_closed_chain(obj, robot, torque, generalized_ext_wrench, motor_inertias, base_pose_dot, s_dot, obj_step_block)

%     COMPUTE_CONTACT_CLOSED_CHAIN: Computes the contact forces, the internal wrenches of the spilit points in the (possible) closed chains, and the configuration velocity after a (possible) impact
%
%     PROCEDURE: For modeling the dynamics of a robot having some closed 
%             chains, we open each chain by breaking a link in the chain.
%             Then, we apply a constraint wrench to the broken point that 
%             the two parts of the broken link become connected. In this 
%             way, the equation of motion for a floating base robot having
%             some closed chains can be written as
%
%             M vDot + h = B u + Fe + Jc' Fc + Ji Fi
%
%             where
%
%             u is the joint torques, Fe is the external wrenches, Fc is 
%             the pure forces applied to the contact points, and Fi is the
%             internal wrenches in the spilit/broken points.
%
%             To compute Fc, Fi and the effects of a (possible) impact,
%
%             1. Determine if each vertex of the feet is in contact with
%             the ground or not by analysing their vertical height,
%
%             2. Compute the robot velocity vector if an impact happens,
%
%             3. Compute Fc and Fi by using the minimum dissipation
%                principle.
%
%             4. Computes the resultant wrench applied to the sole of feet 
%
%
%     INPUTS:
%             - robot:                                  instance of the Robot object
%             - torque:                 [N x 1]         The joint torques
%             - generalized_ext_wrench: [(N+6) x 1]     The external wrench transformed using the Jacobian relative to the application point
%             - motor_inertias:         [(N+6) x (N+6)] The inertia matrix of the motors
%             - base_pose_dot:          [6 x 1]         The velocity vector of the base link 
%             - s_dot:                  [N x 1]         The configuration velocity vector
%             - obj_step_block:
%
%     OUTPUTS:
%             - generalized_total_wrench: [(N+6) x (N+6)] The sum of the generalized_ext_wrench and the generalized contact wrench and the internal wrench on the (possible) split points
%             - wrench_in_contact_frames: [(6m) x 1]      The resultant wrenches in sole of the contact frames
%             - base_pose_dot:            [6 x 1]         The base velocity vector, changed in the case of an impact with the ground
%             - s_dot:                    [N x 1]         The configuration velocity vector, changed in the case of an impact with the ground
%
%     **AUTHORS:** Venus Pasandi, Nuno Guedelha
%
%     all authors are with the Italian Istitute of Technology (IIT)
%     email: name.surname@iit.it
%
%     PLACE AND DATE: <Genoa, March 2022>
%

num_in_contact_frames = length(obj_step_block.robot_config.robotFrames.IN_CONTACT_WITH_GROUND);  % The number of the links interacting with the ground
num_closed_chains = obj_step_block.robot_config.closed_chains;  % The number of the closed chains of the robot

% collect the open-chain kinematic dynamic quantities
M = robot.get_mass_matrix(motor_inertias,obj_step_block);
[J_in_contact, JDot_nu_in_contact] = obj.compute_J_and_JDot_nu_in_contact_frames(robot, num_in_contact_frames);
if ( num_closed_chains == 0)
    J_diff_split_points = [];
    JDot_diff_nu_split_points = [];
    G_forces = J_in_contact;
else
    [J_diff_split_points, JDot_diff_nu_split_points] = obj.compute_J_and_JDot_nu_split_points(robot);
    G_forces = [J_in_contact;J_diff_split_points];
end

% compute the vertical distance of every vertex from the ground
contact_points = obj.compute_contact_points(robot, num_in_contact_frames);

% compute the configuration velocity - same, if no impact - discontinuous in case of impact
[base_pose_dot, s_dot, impact_flag] = compute_velocity(obj, M, G_forces, base_pose_dot, s_dot, num_closed_chains, num_in_contact_frames, contact_points);

% update the robot velocity in the case of impact
if impact_flag
    % sets the velocity in the state
    robot.set_robot_velocity(base_pose_dot, s_dot);
end

% collect the open-chain kinematic dynamic quantities
h = robot.get_bias_forces();

% computes a 3 * num_total_vertices + 6 * num_closed_chains vector
% containing the pure contact forces acting on every vertex and the
% internal wrenches in the spilit points
forces = obj.compute_unilateral_linear_contact(M, h, J_in_contact, J_diff_split_points, JDot_nu_in_contact, JDot_diff_nu_split_points, torque, contact_points, num_closed_chains, generalized_ext_wrench, num_in_contact_frames);

% transform the contact and the internal wrenches of the spilit points in the closed chains in a wrench acting on the robot
generalized_contact_wrench = G_forces' * forces;

% sum the contact wrench and the internal wrenches of the spilit points to the external one
generalized_total_wrench = generalized_ext_wrench + generalized_contact_wrench;

% compute the wrench in the sole frames, in order to simulate a sensor mounted onto the sole frame
ground_forces = forces(1:end-6*num_closed_chains);
wrench_in_contact_frames = obj.compute_contact_wrench_in_sole_frames(ground_forces, robot, num_in_contact_frames);

% update the contact log
obj.was_in_contact = obj.is_in_contact;

end