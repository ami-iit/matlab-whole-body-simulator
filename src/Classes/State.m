classdef State < handle
    %STATE The State class takes care of the numerical integration of the system configuration.
    % State Properties:
    %   w_H_b - the Homogeneous transformation from the base frame to the world
    %   s - the joints position
    %   base_pose_dot - the base pose velocity
    %   s_dot - the joints velocity
    % State Methods:
    %   set - Sets the State quantities
    %   set_velocity - Sets only the configuration velocity
    %   euler_step - Integrates the system using forward Euler
    %   ode_step - Integrates the system using ODE45 method

    properties
        w_H_b (4, 4) double;
        s;
        base_pose_dot (6, 1) double;
        s_dot;
        dt double;
    end

    methods

        function obj = State(dt)
            %STATE Construct an instance of this class
            %   INPUT: dt - the timestep of the integrator
            obj.dt = dt;
        end

        function set(obj, w_H_b, s, base_pose_dot, s_dot)
            % set Sets the State quantities
            % INPUT: - w_H_b: the Homogeneous transformation from the base frame to the world
            %        - s: the joints position
            %        - base_pose_dot: the base pose velocity
            %        - s_dot: the joints velocity
            obj.w_H_b = w_H_b;
            obj.s = s;
            obj.base_pose_dot = base_pose_dot;
            obj.s_dot = s_dot;
        end

        function set_velocity(obj, base_pose_dot, s_dot)
            % set_velocity Sets only the configuration velocity
            % INPUT: - base_pose_dot: the base pose velocity
            %        - s_ddot: the joints velocity
            obj.base_pose_dot = base_pose_dot;
            obj.s_dot = s_dot;
        end

        function [w_H_b, s, base_pose_dot, s_dot] = euler_step(obj, base_pose_ddot, s_ddot)
            % euler_step Integrates the system using forward Euler
            % INPUT: - base_pose_dot: the base pose acceleration
            %        - s_ddot: the joints acceleration
            % OUTPUT: - w_H_b: the Homogeneous transformation from the base frame to the world
            %         - s: the joints position
            %         - base_pose_ddot: the base pose velocity
            %         - s_ddot: the joints velocity
            [R, p] = obj.H2Rp(obj.w_H_b);
            p_dot = obj.base_pose_dot(1:3);
            omega = obj.base_pose_dot(4:6);
            p_ddot = base_pose_ddot(1:3);
            omega_dot = base_pose_ddot(4:6);
            p = p + p_dot * obj.dt + p_ddot * obj.dt^2/2;
            % Compute the rotation matrix using the exponential map
            theta = omega * obj.dt + omega_dot * obj.dt^2/2;
            angle2R = obj.exponential_map(theta);
            R = angle2R * R;
            w_H_b = obj.Rp2H(R, p);
            s = obj.s + obj.s_dot * obj.dt + s_ddot * obj.dt^2/2;
            base_pose_dot = obj.base_pose_dot + base_pose_ddot * obj.dt;
            s_dot = obj.s_dot + s_ddot * obj.dt;
            obj.set(w_H_b, s, base_pose_dot, s_dot);
        end

        function [w_H_b, s, base_pose_dot, s_dot] = ode_step(obj, base_pose_ddot, s_ddot)
            % ode_step Integrates the system using ODE45 method
            % INPUT: - base_pose_dot: the base pose acceleration
            %        - s_ddot: the joints acceleration
            % OUTPUT: - w_H_b: the Homogeneous transformation from the base frame to the world
            %         - s: the joints position
            %         - base_pose_dot: the base pose velocity
            %         - s_ddot: the joints velocity
            [R, p] = obj.H2Rp(obj.w_H_b);
            p_dot = obj.base_pose_dot(1:3);
            omega = obj.base_pose_dot(4:6);
            % not useful for now...
            % p_ddot = base_pose_ddot(1:3);
            % omega_dot = base_pose_ddot(4:6);

            [~, y] = ode45(@(t, y) p_dot, [0 obj.dt], p);
            p = y(end, :)';

            % Compute the rotation matrix using the exponential map
            [~, y] = ode45(@(t, y) omega, [0 obj.dt], zeros(3, 1));
            theta = y(end, :)';
            angle2R = obj.exponential_map(theta);
            R = angle2R * R;

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
            % exponential_map Returns the rotation matrix given the angle theta
            n = norm(theta);

            if n < 1e-6
                angle2R = eye(3);
                return
            end

            theta_norm = theta / n;
            angle2R = eye(3) + sin(n) * wbc.skew(theta_norm) + (1 - cos(n)) * wbc.skew(theta_norm) * wbc.skew(theta_norm);
        end

        function [R, p] = H2Rp(H)
            % H2Rp Returns the Rotation matrix R and the position vector p from the Homogenous matrix H
            R = H(1:3, 1:3);
            p = H(1:3, 4);
        end

        function H = Rp2H(R, p)
            % Rp2H Returns the homogenous matrix H from the Rotation matrix R and the position vector p
            H = [R, p; ...
                    0, 0, 0, 1];
        end

    end

end
