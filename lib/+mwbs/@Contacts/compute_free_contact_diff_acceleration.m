function free_contact_diff_acceleration = compute_free_contact_diff_acceleration(obj, G, free_acceleration, P)
% compute_free_contact_diff_acceleration returns the acceleration of the feet  and the difference of the acceleration of the split points with NO contact forces
free_contact_diff_acceleration = G * free_acceleration + P;
end