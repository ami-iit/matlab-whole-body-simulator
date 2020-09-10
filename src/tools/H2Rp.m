function [R, p] = H2Rp(H)
    % H2Rp Returns the Rotation matrix R and the position vector p from the Homogenous matrix H
    R = H(1:3, 1:3);
    p = H(1:3, 4);
end
