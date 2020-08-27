classdef Contacts < handle
    %CONTACTS Summary of this class goes here
    %   Detailed explanation goes here

    properties
        num_vertices = 4;
        foot_print;
        is_in_contact = ones(8, 2);
        S; % selector matrix
        mu; % friction coefficient
        A; b; Aeq; beq;
    end

    methods

        function obj = Contacts(foot_print, robot, friction_coefficient)
            %CONTACTS Construct an instance of this class
            %   Detailed explanation goes here
            obj.foot_print = foot_print;
            obj.S = [zeros(6, robot.NDOF); ...
                    eye(robot.NDOF)];
            obj.mu = friction_coefficient;
            obj.prepare_optimization_matrix();
        end

        function [generalized_total_wrench, wrench_left_foot, wrench_right_foot, contact_detected] = compute_contact_forces(obj, robot, torque, generalized_ext_wrench)
            h = robot.get_bias_forces();
            M = robot.get_mass_matrix();
            [H_LFOOT, H_RFOOT] = robot.get_feet_H();
            [J_LFoot, J_RFoot] = robot.get_feet_jacobians();
            [JDot_nu_LFOOT, JDot_nu_RFOOT] = robot.get_feet_JDot_nu();

            for ii = 1:obj.num_vertices
                j = (ii - 1) * 3 + 1;
                J_left_foot_print(j:j + 2, :) = J_LFoot(1:3, :) - wbc.skew(H_LFOOT(1:3, 1:3) * obj.foot_print(:, ii)) * J_LFoot(4:6, :);
                JDot_nu_left_foot_print(j:j + 2, :) = JDot_nu_LFOOT(1:3, :) - wbc.skew(H_LFOOT(1:3, 1:3) * obj.foot_print(:, ii)) * JDot_nu_LFOOT(4:6, :);
                J_right_foot_print(j:j + 2, :) = J_RFoot(1:3, :) - wbc.skew(H_RFOOT(1:3, 1:3) * obj.foot_print(:, ii)) * J_RFoot(4:6, :);
                JDot_nu_right_foot_print(j:j + 2, :) = JDot_nu_RFOOT(1:3, :) - wbc.skew(H_RFOOT(1:3, 1:3) * obj.foot_print(:, ii)) * JDot_nu_RFOOT(4:6, :);
            end

            J_feet = [J_left_foot_print; J_right_foot_print];
            JDot_nu_feet = [JDot_nu_left_foot_print; JDot_nu_right_foot_print];
            left_z_foot_print = zeros(4, 1);
            right_z_foot_print = zeros(4, 1);

            for ii = 1:4
                left_z = H_LFOOT * [obj.foot_print(:, ii); 1];
                left_z_foot_print(ii) = left_z(3);
                right_z = H_RFOOT * [obj.foot_print(:, ii); 1];
                right_z_foot_print(ii) = right_z(3);
                obj.is_in_contact(ii, 2) = left_z_foot_print(ii) <= 0;
                obj.is_in_contact(ii + 4, 2) = right_z_foot_print(ii) <= 0;
            end

            contact_points = [left_z_foot_print', right_z_foot_print']';
            % free_acceleration = obj.compute_free_acceleration(M, h, torque, generalized_ext_wrench);
            contact_forces = obj.compute_unilateral_linear_contact(J_feet, M, h, torque, JDot_nu_feet, contact_points, generalized_ext_wrench);
            generalized_contact_wrench = J_feet' * contact_forces;
            generalized_total_wrench = generalized_ext_wrench + generalized_contact_wrench;
            [wrench_left_foot, wrench_right_foot] = compute_contact_wrench_in_sole_frames(contact_forces, H_LFOOT, H_RFOOT, obj.foot_print);
            contact_detected = false;

            for ii = 1:8

                if obj.is_in_contact(ii, 2) == 1 && obj.is_in_contact(ii, 1) == 0
                    contact_detected = true;
                    break
                end

            end

            obj.is_in_contact(:, 1) = obj.is_in_contact(:, 2);
        end

        function free_acceleration = compute_free_acceleration(obj, M, h, torque, generalized_jet_wrench)
            % returns the system acceleration with NO contact forces
            % dot{v} = inv{M}(S*tau + external_forces - h)
            free_acceleration = (M + 0.05) \ (obj.S * torque + generalized_jet_wrench - h);
        end

        function free_contact_acceleration = compute_free_contact_acceleration(obj, J_feet, free_acceleration, JDot_nu_feet)
            % acceleration of the feet with NO contact forces
            free_contact_acceleration = J_feet * free_acceleration + JDot_nu_feet;
        end

        function forces = compute_unilateral_linear_contact(obj, J_feet, M, h, torque, JDot_nu_feet, contact_point, generalized_ext_wrench)
            % returns the pure forces acting on the feet vertices

            free_acceleration = obj.compute_free_acceleration(M, h, torque, generalized_ext_wrench);
            free_contact_acceleration = obj.compute_free_contact_acceleration(J_feet, free_acceleration, JDot_nu_feet);
            H = J_feet * (M \ J_feet');
            f = free_contact_acceleration;

            for i = 1:8
                obj.Aeq(i, i * 3) = contact_point(i) > 0;
            end

            options = optimoptions('quadprog', 'Algorithm', 'active-set');
            forces = quadprog(H, f, obj.A, obj.b, obj.Aeq, obj.beq, [], [], 100 * ones(24, 1), options);
%             forces = quadprog(H, f, obj.A, obj.b, obj.Aeq, obj.beq);
        end

        function [wrench_left_foot, wrench_right_foot] = compute_contact_wrench_in_sole_frames(contact_forces, H_LFOOT, H_RFOOT, foot_print)
            % trasforms the pure forces on foot vertices in wrench in sole frames
            R_LFOOT = H_LFOOT(1:3, 1:3);
            R_RFOOT = H_RFOOT(1:3, 1:3);

            wrench_left_foot = zeros(6, 1);
            wrench_right_foot = zeros(6, 1);

            contact_forces_left = contact_forces(1:12);
            contact_forces_right = contact_forces(13:24);

            for i = 1:4
                j = (i - 1) * 3 + 1;
                wrench_left_foot(1:3) = wrench_left_foot(1:3) + R_LFOOT' * contact_forces_left(j:j + 2);
                wrench_left_foot(4:6) = wrench_left_foot(4:6) - wbc.skew(foot_print(:, i)) * (R_LFOOT' * contact_forces_left(j:j + 2));
                wrench_right_foot(1:3) = wrench_right_foot(1:3) + R_RFOOT' * contact_forces_right(j:j + 2);
                wrench_right_foot(4:6) = wrench_right_foot(4:6) - wbc.skew(foot_print(:, i)) * (R_RFOOT' * contact_forces_left(j:j + 2));
            end

        end

        function prepare_optimization_matrix(obj)
            % TODO: avoid to hard code the number of constraints
            % 16 + 16 = constraints on x - y forces on 8 vertices. 4 for every foot_print: upper and lower bounded
            % 8 = constraints on z (positive)
            obj.A = zeros(24 + 16, 24);
            obj.b = zeros(24 + 16, 1);
            obj.Aeq = zeros(8, 24);
            obj.beq = zeros(8, 1);

            for i = 1:8
                j = (i - 1) * 3 + 1;
                obj.A(j:j + 2, j:j + 2) = [1, 0, -obj.mu; ...
                                            0, 1, -obj.mu; ...
                                            0, 0, -1];
            end

            for i = 9:16
                j = (i - 1) * 2 + 9;
                jj = (i - 8 - 1) * 3 + 1;
                obj.A(j:j + 1, jj:jj + 2) = [-1, 0, -obj.mu; ...
                                            0, -1, -obj.mu];
            end

        end

    end

end
