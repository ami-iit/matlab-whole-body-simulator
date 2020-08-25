function [base_linear_velocity, omega_b, joints_velocity, dot_v_b, dot_omega_b, joint_acceleration, ...
    contact_detected, wrench_left_foot, wrench_right_foot] = ...
    forward_and_contact_dynamics(mass_matrix, bias_forces, joints_torque, generalized_ext_wrench, ...
    base_velocity, joints_velocity, JLeft_frameJac, JRight_frameJac, JDot_nu_LFOOT, JDot_nu_RFOOT, H_LFOOT, H_RFOOT, Config)
% returns the state derivative under the joint torques and external
% wrenches (external wrenches + contact wrenches)

persistent is_in_contact

if isempty(is_in_contact)
    is_in_contact = [ones(8,1), zeros(8,1)];
end

% vertex = zeros(3,4);
% vertex(:,1) = [-0.06;   0.04; 0];
% vertex(:,2) = [ 0.11;   0.04; 0];
% vertex(:,3) = [ 0.11; -0.035; 0];
% vertex(:,4) = [-0.06; -0.035; 0];

vertex = Config.vertex;

% N_DOF + base DOF
J_left_vertex = zeros(12,Config.N_DOF + 6);
JDot_nu_left_vertex = zeros(12,1);

J_right_vertex = zeros(12,Config.N_DOF + 6);
JDot_nu_right_vertex = zeros(12,1);

for ii=1:4
    j = (ii-1)*3 + 1;
    J_left_vertex(j:j+2, :) =  JLeft_frameJac(1:3,:) - wbc.skew(H_LFOOT(1:3,1:3)*vertex(:,ii))*JLeft_frameJac(4:6,:);
    JDot_nu_left_vertex(j:j+2, :) = JDot_nu_LFOOT(1:3,:) - wbc.skew(H_LFOOT(1:3,1:3)*vertex(:,ii))*JDot_nu_LFOOT(4:6,:);
    J_right_vertex(j:j+2, :) =  JRight_frameJac(1:3,:) - wbc.skew(H_RFOOT(1:3,1:3)*vertex(:,ii))*JRight_frameJac(4:6,:);
    JDot_nu_right_vertex(j:j+2, :) = JDot_nu_RFOOT(1:3,:) - wbc.skew(H_RFOOT(1:3,1:3)*vertex(:,ii))*JDot_nu_RFOOT(4:6,:);
end

JDot_nu_feet = [JDot_nu_left_vertex; JDot_nu_right_vertex];

J_feet = [J_left_vertex; J_right_vertex];

left_z_vertex = zeros(4,1);
right_z_vertex = zeros(4,1);

for ii=1:4
    left_z = H_LFOOT*[vertex(:,ii);1];
    left_z_vertex(ii) = left_z(3);
    right_z = H_RFOOT*[vertex(:,ii);1];
    right_z_vertex(ii) = right_z(3);
    is_in_contact(ii, 2) = left_z_vertex(ii) <= 0;
    is_in_contact(ii + 4, 2) = right_z_vertex(ii) <= 0;
end

contact_points = [left_z_vertex', right_z_vertex']';

free_acceleration = compute_free_acceleration(mass_matrix, bias_forces, joints_torque, generalized_ext_wrench, Config);

contact_forces = compute_unilateral_linear_contact(J_feet, mass_matrix, free_acceleration, JDot_nu_feet, contact_points, Config);

generalized_contact_wrench = J_feet'*contact_forces;

base_linear_velocity = base_velocity(1:3);
omega_b = base_velocity(4:6);

contact_detected = 0;

for ii=1:8
    if is_in_contact(ii,2) == 1 && is_in_contact(ii, 1) == 0
        N = (eye(Config.N_DOF + 6) - mass_matrix\(J_feet'*((J_feet*(mass_matrix\J_feet'))\J_feet)));
        x = N*[base_velocity; joints_velocity];
        base_linear_velocity = x(1:3);
        omega_b = x(4:6);
        joints_velocity = x(7:end);
        contact_detected = 1;
        break
    end
end
generalized_total_wrench = generalized_ext_wrench + generalized_contact_wrench;
[wrench_left_foot, wrench_right_foot] = compute_contact_wrench_in_sole_frames(contact_forces, H_LFOOT, H_RFOOT, vertex);
[dot_v_b, dot_omega_b, joint_acceleration] = forward_dynamics(mass_matrix, bias_forces, joints_torque, generalized_total_wrench, Config);

is_in_contact(:,1) = is_in_contact(:,2);
end
