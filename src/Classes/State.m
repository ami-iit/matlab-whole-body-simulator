classdef State < handle
    %SYSTEM_INTEGRATION Summary of this class goes here
    %   Detailed explanation goes here

    properties
        w_H_b; s;
        base_pose_dot; s_dot;
        dt;
    end

    methods

        function obj = State(dt)
            %SYSTEM_INTEGRATION Construct an instance of this class
            %   Detailed explanation goes here
            obj.dt = dt;
        end

        function set(obj, w_H_b, s, base_pose_dot, s_dot)
            obj.w_H_b = w_H_b;
            obj.s = s;
            obj.base_pose_dot = base_pose_dot;
            obj.s_dot = s_dot;
        end

        function set_velocity(obj, base_pose_dot, s_dot)
            obj.base_pose_dot = base_pose_dot;
            obj.s_dot = s_dot;
        end

        function [w_H_b, s, base_pose_dot, s_dot] = euler_step(obj, base_pose_ddot, s_ddot)
            [R, p] = obj.H2Rp(obj.w_H_b);
            p_dot = obj.base_pose_dot(1:3);
            omega = obj.base_pose_dot(4:6);
            p_ddot = base_pose_ddot(1:3);
            omega_dot = base_pose_ddot(4:6);

            p = p + p_dot * obj.dt + p_ddot * obj.dt^2/2;

            % TODO: check the exponential map approach
            theta = omega * obj.dt + omega_dot * obj.dt^2/2;
            R = obj.exponential_map(R, theta);

            %             gain = 0.001;
            %             A = gain*((R'*R)' - eye(3));
            %             omega =  omega_dot*obj.dt;
            %
            %             R_dot = (wbc.skew(omega) + A)*R;
            %             R = R + R_dot*obj.dt;

            w_H_b = obj.Rp2H(R, p);
            s = obj.s + obj.s_dot * obj.dt + s_ddot * obj.dt^2/2;
            base_pose_dot = obj.base_pose_dot + base_pose_ddot * obj.dt;
            s_dot = obj.s_dot + s_ddot;
            obj.set(w_H_b, s, base_pose_dot, s_dot);
        end

    end

    methods (Static)

        function R = exponential_map(R, theta)
            n = norm(theta);
            theta_norm = theta / n;
            angle2R = eye(3) + sin(n) * wbc.skew(theta_norm) + (1 - cos(n)) * wbc.skew(theta_norm) * wbc.skew(theta_norm);
            R = R * angle2R;
        end

        function [R, p] = H2Rp(H)
            R = H(1:3, 1:3);
            p = H(1:3, 4);
        end

        function H = Rp2H(R, p)
            H = [R, p; 0, 0, 0, 1];
        end

    end

end
