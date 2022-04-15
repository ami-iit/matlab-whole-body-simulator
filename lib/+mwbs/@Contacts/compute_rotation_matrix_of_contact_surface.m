function w_R_c = compute_rotation_matrix_of_contact_surface(obj,n)

    %     COMPUTE_ROTATION_MATRIX_OF_CONTACT_SURFACE : This function
    %           computes the rotation matrix of the contact frame to the world
    %           frame. The Contact frame is defined as a frame that its z axis is
    %           aligned to the normal axis of the contact surface.
    % 
    %     PROCEDURE: We want to rotate the world frame around an axis like
    %           "u" with the angle "p" such that its z axis become aligned with
    %           the normal axis of the contact surface. We could define "u" as
    %
    %               u = n x k
    %
    %           where k is the unit vector of the z axis of the world frame and n
    %           is the unit vector of the normal axis of the contact surface.
    %
    %           The rotation angle p is defined as
    %
    %               sin(p) = |n x k|
    %
    %           Considering p and n, the rotation matrix from the contact frame
    %           to the world frame is computed [1].
    %
    %           [1] "Rotation", Encyclopedia of Mathematics, EMS Press, 2001 [1994]
    %
    %     **FORMAT**: Contact_object.compute_rotation_matrix_of_contact_surface(n)
    % 
    %     **INPUT:**
    %             - n:  [3 x 1] The normal axis of the contact surface
    % 
    %     **OUTPUT:**
    %             - w_R_c:  [3 x 3] The rotation matrix from the Contact frame to the World frame
    % 
    %     **AUTHORS:**  Venus Pasandi
    % 
    %     all authors are with the Italian Istitute of Technology (IIT)
    %     email: name.surname@iit.it
    % 
    %     PLACE AND DATE: <Genoa, April 2022>
    %

% ---------------------- INITIALIZATION ----------------------------------
k = [0;0;1];  % The z axis of the world frame
n_normalized = n / norm(n);

% -------------------------- MAIN ----------------------------------------
u = mwbs.Utils.skew(k) * n_normalized;
u_norm = norm(u);

p = asin(u_norm);
cp = cos(p);
sp = sin(p);

if (u_norm == 0) % The cases that n is aligned with k
    u_norm = 1;
end
u_normalized = u / u_norm;

ux = u_normalized(1);
uy = u_normalized(2);
uz = u_normalized(3);

w_R_c = [ cp + ux^2 * (1 - cp)      ,  ux*uy * (1-cp) - uz * sp  ,  ux*uz * (1-cp) + uy * sp;...
          ux*uy * (1-cp) + uz * sp  ,  cp + uy^2 * (1 - cp)      ,  uy*uz * (1-cp) - ux * sp;...
          ux*uz * (1-cp) - uy * sp  ,  uy*uz * (1-cp) + ux * sp  ,  cp + uz^2 * (1 - cp)];
          
end