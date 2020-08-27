function [dot_v_b, dot_omega_b, joint_acceleration] = forward_dynamics(mass_matrix, bias_forces, torques, generalized_external_wrenches, Config)
    % compute forward dynamics as a first order system
    % M \dot{v} + h = S*tau + external_forces
    % state x = [x1, x2]
    % dot{x} = [x2;...
    %           dot{v}];
    % dot{v} = inv{M}(S*tau + external_forces - h)

    S = [zeros(6, Config.N_DOF); eye(Config.N_DOF)];
    ddot = mass_matrix \ (S * torques + generalized_external_wrenches - bias_forces);
    dot_v_b = ddot(1:3);
    dot_omega_b = ddot(4:6);
    joint_acceleration = ddot(7:end);
end
