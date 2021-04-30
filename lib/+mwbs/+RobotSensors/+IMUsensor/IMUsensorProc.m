classdef IMUsensorProc < matlab.System
    % IMUsensorProc This block computes the typical outputs of an IMU from the base pose and joints kinematics.
    % 
    % Assuming DoFs is the number of internal degrees of freedom of the robot:
    % 
    % Input:
    %   - w_H_b: Base pose, 4x4 matrix representing the homogenous transformation between the the
    %   base frame and the world frame.
    %   - nu: Base and joints velocities: Vector of size 6 representing the linear and angular
    %   velocity of the base frame, concatenated with a vector of size DoFs, representing the
    %   velocity of the joints.
    %   - nuDot: Base and joints accelerations: Vector of size 6 representing the linear and angular
    %   acceleration of the base frame, concatenated with a vector of size DoFs, representing the
    %   acceleration of the joints.
    %   - Jimu: Jacobian of the IMU frame.
    %   - JimuDotNu: Bias linear acceleration of the IMU frame with respect to the inertial (world)
    %   frame.
    % 
    % Output:
    %   The sensor frame orientation, linear acceleration and angular velocity w.r.t. the inertial
    %   (world) frame. Linear acceleration and angular velocity are expressed in the world frame.
    %   - w_rollPitchYaw: roll, pitch, yaw Euler angles in rad.
    %   - w_linAcc: Proper linear acceleration in m/s^2.
    %   - w_omega: Angular velocity in rad/s.
    %   - imuOut: A [12x1] 1-D vector concatenating the previous three outputs and a placeholder for
    %   other measurements (e.g. magnetometer).
    %   [roll,pitch,yaw,accx,accy,accz,omegax,omegay,omegaz,0,0,0]'.
    %   
    % Parameters:
    %   - Frame name: the name of the IMU frame. It should be specified in the URDF model.

    % Public, tunable properties
    properties

    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)

    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function [imuOut,w_omega,w_linAcc,w_rollPitchYaw] = stepImpl(obj,Jimu,JimuDotNu,w_H_imu,nu,nudot)
            % Implement algorithm. Calculate y as a function of inputs and discrete states.
            
            % Gyroscope measurements
            w_imuVel = Jimu*nu;
            w_omega = w_imuVel(4:6);
            
            % Accelerometer measurements
            w_imuAcc = Jimu*nudot + JimuDotNu;
            w_linAcc = w_imuAcc(1:3);
            
            % Euler angles estimation
            R = mwbs.State.H2Rp(w_H_imu);
            w_rollPitchYaw = wbc.rollPitchYawFromRotation(R);
            
            % composite sensor output
            imuOut = [w_rollPitchYaw; w_linAcc; w_omega; zeros(3,1)];
        end

        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
    end
end
