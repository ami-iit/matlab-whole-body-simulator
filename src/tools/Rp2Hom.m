function H = Rp2Hom(p, R)
% Takes position p and rotation matrix R.
% Returns the corresponding homogeneous transformation matrix T

% check if det(R) = 1, if not use the SVD to correct the rotation matrix
% if abs(det(R) - 1) > 0.05
if norm(R'*R - eye(3)) > 0.05
    %     error('The rotation matrix is not a anymore a rotation matrix');
    disp('Making the matrix a rotation one using SVD');
    [U,~,V] = svd(R);
    R = U*V';
end
H = [R, p; 0, 0, 0, 1];
end
