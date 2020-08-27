function forces = compute_unilateral_linear_contact(J_feet, mass_matrix, free_acceleration, JDot_nu_feet, contact_point, Config)
    % returns the pure forces acting on the feet vertices
    free_contact_acceleration = compute_free_contact_acceleration(J_feet, free_acceleration, JDot_nu_feet);
    H = J_feet * (mass_matrix \ J_feet');
    f = free_contact_acceleration;
    mu = Config.friction_coefficient;
    A = zeros(24 + 16, 24);
    b = zeros(24 + 16, 1);
    Aeq = zeros(8, 24);
    beq = zeros(8, 1);

    for i = 1:8
        j = (i - 1) * 3 + 1;
        A(j:j + 2, j:j + 2) = [1, 0, -mu; ...
                            0, 1, -mu; ...
                            0, 0, -1];
        Aeq(i, i * 3) = contact_point(i) >= 0;
    end

    for i = 9:16
        j = (i - 1) * 2 + 9;
        jj = (i - 8 - 1) * 3 + 1;
        A(j:j + 1, jj:jj + 2) = [-1, 0, -mu; ...
                            0, -1, -mu];
        %     Aeq(i, i*3) = contact_point(i)>=0
    end

    options = optimoptions('quadprog', 'Algorithm', 'active-set');
    forces = quadprog(H, f, A, b, Aeq, beq, [], [], 100 * ones(24, 1), options);
end

function free_contact_acceleration = compute_free_contact_acceleration(J_feet, free_acceleration, JDot_nu_feet)
    free_contact_acceleration = J_feet * free_acceleration + JDot_nu_feet;
end
