function free_acceleration = compute_free_acceleration(mass_matrix, bias_forces, joints_torque, generalized_jet_wrench, Config)
% returns the system acceleration with NO contact forces
% dot{v} = inv{M}(S*tau + external_forces - h)

S = [zeros(6, Config.N_DOF);...
    eye(Config.N_DOF)];

free_acceleration = mass_matrix\(S*joints_torque + generalized_jet_wrench - bias_forces);
end
