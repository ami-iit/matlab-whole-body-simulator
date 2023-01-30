function [J, JDot_nu] = compute_J_and_JDot_nu_fixed_frame(obj, robot)

    %     COMPUTE_J_AND_JDOT_NU_FIXED_FRAME: computes the JTilde and JDOTTilde * nu fot the fixed frame
    % 
    %     PROCEDURE:
    % 
    %     **FORMAT**: [J, JDot_nu] = compute_J_and_JDot_nu_fixed_frame(obj, robot)
    % 
    %     **INPUT:**
    %                 - robot  [ROBOT OBJECT]
    % 
    %     **OUTPUT:**
    %                 - J:       [6 x (N+6)] The Jacobian matrix of the fixed frame
    %                 - JDot_nu  [6 x 1]     The JDot * nu vector for the fixed frame
    % 
    %     **AUTHORS:** Venus Pasandi
    % 
    %     all authors are with the Italian Istitute of Technology (IIT)
    %     email: name.surname@iit.it
    % 
    %     PLACE AND DATE: <Genoa, March 2022>

% read the J and JDot_nu for the desired fixed frame
J = robot.get_fixedFrame_jacobian();
JDot_nu = robot.get_fixedFrame_JDot_nu();

end