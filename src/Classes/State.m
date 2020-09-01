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
            angle2R = obj.exponential_map(theta);
            %             R = R * angle2R;
            R = angle2R * R;
            %             gain = 0.001;
            %             A = gain*((R'*R)' - eye(3));
            %             omega =  omega_dot*obj.dt;
            %
            %             R_dot = (wbc.skew(omega) + A)*R;
            %             R = R + R_dot*obj.dt;

            w_H_b = obj.Rp2H(R, p);
            s = obj.s + obj.s_dot * obj.dt + s_ddot * obj.dt^2/2;
            base_pose_dot = obj.base_pose_dot + base_pose_ddot * obj.dt;
            s_dot = obj.s_dot + s_ddot * obj.dt;
            obj.set(w_H_b, s, base_pose_dot, s_dot);
        end

        function [w_H_b, s, base_pose_dot, s_dot] = ode_step(obj, base_pose_ddot, s_ddot)
            [R, p] = obj.H2Rp(obj.w_H_b);
            p_dot = obj.base_pose_dot(1:3);
            omega = obj.base_pose_dot(4:6);
            p_ddot = base_pose_ddot(1:3);
            omega_dot = base_pose_ddot(4:6);

            [~, y] = ode45(@(t, y) p_dot, [0 obj.dt], p);
            p = y(end, :)';

            % TODO: check the exponential map approach
            [~, y] = ode45(@(t, y) omega, [0 obj.dt], zeros(3, 1));
            theta = y(end, :)';
            angle2R = obj.exponential_map(theta);
            R = angle2R * R;
            %             gain = 0.001;
            %             A = gain*((R'*R)' - eye(3));
            %             omega =  omega_dot*obj.dt;
            %
            %             R_dot = (wbc.skew(omega) + A)*R;
            %             R = R + R_dot*obj.dt;

            w_H_b = obj.Rp2H(R, p);
            [~, y] = ode45(@(t, y) obj.s_dot, [0 obj.dt], obj.s);
            s = y(end, :)';

            [~, y] = ode45(@(t, y) base_pose_ddot, [0 obj.dt], obj.base_pose_dot);
            base_pose_dot = y(end, :)';

            [~, y] = ode45(@(t, y) s_ddot, [0 obj.dt], obj.s_dot);
            s_dot = y(end, :)';

            obj.set(w_H_b, s, base_pose_dot, s_dot);
        end

    end

    methods (Static)

        function angle2R = exponential_map(theta)
            n = norm(theta);
            if n < 1e-6
                angle2R = eye(3);
                return
            end
            theta_norm = theta / n;
            angle2R = eye(3) + sin(n) * wbc.skew(theta_norm) + (1 - cos(n)) * wbc.skew(theta_norm) * wbc.skew(theta_norm);
        end

        function [R, p] = H2Rp(H)
            R = H(1:3, 1:3);
            p = H(1:3, 4);
        end

        function H = Rp2H(R, p)
            H = [R, p; ...
                 0, 0, 0, 1];
        end

    end

end
