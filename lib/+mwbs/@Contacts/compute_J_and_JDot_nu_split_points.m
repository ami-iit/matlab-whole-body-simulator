function [J_diff_split_points, JDot_diff_nu_split_points] = compute_J_and_JDot_nu_split_points(obj, robot)

    %     COMPUTE_J_AND_JDOT_NU_SPLIT_POINTS: computes the JTilde and JDOTTilde * nu fot the spilit points in the (possible) closed chains
    % 
    %     PROCEDURE: Assuming J_diff_i and JDot_nu_diff_i are the difference of the
    %                Jacobian matrix and JDot * nu vector between the two prints of
    %                the splite point i for i = 1 : p,
    % 
    %                J_diff_split_points = [J_diff_1 ; J_diff_2 ; ... ; J_diff_p],
    %                JDot_diff_nu_split_points= [JDot_nu_diff_1 ; JDot_nu_diff_2 ; ... ; JDot_nu_diff_p];
    % 
    %     **FORMAT**: [J_diff_split_points, JDot_diff_nu_split_points] = Contact_obj.compute_J_and_JDot_nu_split_points(robot)
    % 
    %     **INPUT:**
    %                 - robot  [ROBOT OBJECT]
    % 
    %     **OUTPUT:**
    %                 - J_diff_split_points:       [(p*6) x (N+6)] The group of the difference of the Jacobian matrix between the two prints of the spilit points
    %                 - JDot_diff_nu_split_points  [(p*6) x 1]     The group of the difference of the JDot * nu vector between the two prints of the spilit points
    % 
    %     **AUTHORS:** Venus Pasandi
    % 
    %     all authors are with the Italian Istitute of Technology (IIT)
    %     email: name.surname@iit.it
    % 
    %     PLACE AND DATE: <Genoa, March 2022>

% read the J_diff and JDot_nu_diff for each split point
J_diff_split_points = robot.get_spilitPoints_diff_jacobian();
JDot_diff_nu_split_points = robot.get_SpilitPoints_diff_JDot_nu();

end