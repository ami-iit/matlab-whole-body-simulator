function free_contact_diff_acceleration = compute_free_contact_diff_acceleration(obj, G, free_acceleration, P)

    %     COMPUTE_FREE_CONTACT_DIFF_ACCELERATION: computes the acceleration of the feet and the difference of the acceleration of the split points in the (possible) closed chains with NO contact forces
    % 
    %     PROCEDURE: The linear velocity of a contact point and the relative velocity of the two prints of a spilit point is as
    % 
    %             Vc = Jc * v
    %             Vi = (Ji_1 - J_i_2) * v = Ji *v
    % 
    %             The above two equations can be written in a compact form as
    % 
    %             Vci = [Jc ; Ji] v = G v
    % 
    %             Differentiating the above equation, the accelerations are as
    % 
    %             VDotci = GDot v + G vDot = p + G vDot
    % 
    %             where p = GDot v. Using the free acceleration vector of the robot vDot_free in the above equation, the free acceleration of the contact points and the relative free acceleration of the prints of sprint points are as
    % 
    %             VDotci_free = GDot v + G vDot = p + G vDot_free
    % 
    %     **FORMAT**: free_contact_diff_acceleration = Contact_object.compute_free_contact_diff_acceleration(G, free_acceleration, P)
    % 
    %     **INPUT:**
    %             - G:                  [(3m + 6p) x (N+6)]  The group matrix of the Jacobian of m contact points and the difference of the Jacobian of the prints of p spilit points
    %             - free_acceleration:  [(N+6) x 1]          The free acceleration vector of the robot
    %             - P:                  [(3m + 6p) x 1]      The group matrix of the JacobianDot * nu of m contact points and the difference of the JacobianDot * nu of the prints of p spilit points
    % 
    %     **OUTPUT:**
    % 
    %     **AUTHORS:**  Venus Pasandi, Nuno Guedelha
    % 
    %     all authors are with the Italian Istitute of Technology (IIT)
    %     email: name.surname@iit.it
    % 
    %     PLACE AND DATE: <Genoa, March 2022>

% -------------------- INITIALIZATION -----------------------------------


% ----------------------- MAIN -------------------------------------------
free_contact_diff_acceleration = G * free_acceleration + P;

end