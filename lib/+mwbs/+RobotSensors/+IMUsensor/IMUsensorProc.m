classdef IMUsensorProc < matlab.System
    % IMUsensorProc This block computes the typical outputs of an IMU from the floating base kinematics
    % 
    % Floating base kinematics inputs:
    % - base pose, velocity and acceleration.
    % 
    % Outputs:
    % - base angular velocity, linear acceleration, and orientation (Euler angles).
    % 
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

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

        function [imuOut,w_omega,w_linAcc,w_rollPitchYaw] = stepImpl(obj,Jimu,dotJimuNu,w_H_imu,nu,nudot)
            % Implement algorithm. Calculate y as a function of inputs and discrete states.
            
            % Gyroscope measurements
            w_imuVel = Jimu*nu;
            w_omega = w_imuVel(4:6);
            
            % Accelerometer measurements
            w_imuAcc = Jimu*nudot + dotJimuNu;
            w_linAcc = w_imuAcc(1:3);
            
            % Euler angles estimation
            R = mwbs.State.H2Rp(w_H_imu);
            w_rollPitchYaw = wbc.rollPitchYawFromRotation(R);
            
            % composite sensor output
            imuOut = [w_rollPitchYaw; w_omega; w_linAcc; zeros(3,1)];
        end

        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
    end
end
