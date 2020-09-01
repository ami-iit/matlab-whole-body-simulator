function dot_R = get_R_dot(R, omega)
    % Takes the rotation matrix R and the
    % angular velocity omega to compute the
    % time derivative of R

    % \dot{A_R_B} = S(omega_A)*A_R_B
    % dot_R = wbc.skew(omega)*R;

    % apply Baumgardte stabilization
    gain = 0.001;
    A = gain * ((R' * R)' - eye(3));

    dot_R = (wbc.skew(omega) + A) * R;
