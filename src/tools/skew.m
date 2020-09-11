function X = skew(x)

    % SKEW function to generate a [3 * 3] skew-symmetric matrix out of a
    %      [3 * 1] vector.
    %
    % INPUT:  - x = [3 * 1] vector
    %
    % OUTPUT: - X = [3 * 3] skew symmetric matrix

    X = [0 -x(3) x(2);
        x(3) 0 -x(1);
        -x(2) x(1) 0];
end
