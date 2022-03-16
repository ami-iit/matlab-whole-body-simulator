function P_damped_psudo_inverse = compute_damped_psudo_inverse(obj, P, damped_coefficient)

    %     COMPUTE_DAMPED_PSUDO_INVERSE: computes the damped pseudo inverse of a matrix
    % 
    %     PROCEDURE: The damped pseudo-inverse addressess the problem of the possible discontinuity of the
    %     pseudo-inverse at a singular configuration. The damped pseudo inverse of A is defined as
    % 
    %         A = A^T (A A^T + k^2 I)^-1
    % 
    %         where k is called the damping factor/coefficient.
    % 
    %     **FORMAT**: P_damped_psudo_inverse = Contact_object.compute_damped_psudo_inverse(P, damped_coefficient)
    % 
    %     **INPUT:**
    %             - P:                   [n x m] the desired matrix
    %             - damped_coefficient:  [SCALAR] damping coefficient
    % 
    %     **OUTPUT:**
    %             - P_damped_psudo_inverse:   [m x m] the damped pseudo inverse matrix of P
    % 
    %     **AUTHORS:** Venus Pasandi
    % 
    %     all authors are with the Italian Istitute of Technology (IIT)
    %     email: name.surname@iit.it
    % 
    %     PLACE AND DATE: <Genoa, March 2022>

% -------------------- INITIALIZATION --------------------------------
n = size(P,1);

% -------------------- MAIN ------------------------------------------
P_damped_psudo_inverse = P' / ( P * P' + damped_coefficient ^ 2 * eye(n));

end