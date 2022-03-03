function P_damped_psudo_inverse = compute_damped_psudo_inverse(obj,P,damped_coefficient)
P_damped_psudo_inverse = P'/(P*P'+damped_coefficient^2*eye(size(P,1)));
end