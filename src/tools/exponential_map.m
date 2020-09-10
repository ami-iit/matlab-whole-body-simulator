function angle2R = exponential_map(theta)
    % exponential_map Returns the rotation matrix given the angle theta
    n = norm(theta);

    if n < 1e-6
        angle2R = eye(3);
        return
    end

    theta_norm = theta / n;
    angle2R = eye(3) + sin(n) * wbc.skew(theta_norm) + (1 - cos(n)) * wbc.skew(theta_norm) * wbc.skew(theta_norm);
end
