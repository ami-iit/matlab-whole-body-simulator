function free_acceleration = compute_free_acceleration(obj, M, h, torque, generalized_ext_wrench)

    %     COMPUTE_FREE_ACCELERATION: computes the robot acceleration with NO contact forces
    % 
    %     PROCEDURE: The equation of motion of a robot having no contact constraints can be written as
    % 
    %                 M vDot + h = S u + Fe
    % 
    %                where vDot is the acceleration vector of the robot, u is the joints torque vector and Fe is the external wrenches projected into the state spoace of the robot.
    %                Since there is no contact constraint, vDot is called the free acceleration of the robot.
    %                Assuming that M is positive definite,
    % 
    %                 vDot = M^-1 (-h + S u + Fe)
    % 
    %     **FORMAT**: free_acceleration = compute_free_acceleration(obj, M, h, torque, generalized_ext_wrench)
    % 
    %     **INPUT:**
    %             - M:                        [(N+6) x (N+6)]  The inertia matrix
    %             - h:                        [(N+6) x 1]      The Coriolis, Centrifugal, and gravity effects vector
    %             - torque:                   [N x 1]          The joint torques vector
    %             - generalized_ext_wrench:   [(N+6) x 1]      The generalized external wrenches transformed to the state space of the robot
    % 
    %     **OUTPUT:**
    %             - free_acceleration         [(N+6) x 1]      The free acceleration vector
    % 
    %     **AUTHORS:** Venus Pasandi, Nuno Guedelha
    % 
    %     all authors are with the Italian Istitute of Technology (IIT)
    %     email: name.surname@iit.it
    % 
    %     PLACE AND DATE: <Genoa, March 2022>

% -------------------- INITIALIZATION ---------------------------------


% -------------------- MAIN -------------------------------------------
free_acceleration = M \ (obj.S * torque + generalized_ext_wrench - h);

end