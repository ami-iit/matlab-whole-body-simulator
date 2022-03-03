function free_acceleration = compute_free_acceleration(obj, M, h, torque, generalized_ext_wrench)
% compute_free_acceleration returns the system acceleration with NO contact forces
% dot{v} = inv{M}(S*tau + external_forces - h)
free_acceleration = M \ (obj.S * torque + generalized_ext_wrench - h);
end