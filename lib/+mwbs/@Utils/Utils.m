classdef Utils
    %Utils This class provides utilities for kinematics and dynamics computations
    %   Detailed explanation goes here
    
    methods(Static)
        % Returns the rotation matrix given the angle theta
        angle2R = exponentialMap(theta);
        
        % Returns the Rotation matrix R and the position vector p from the Homogenous matrix H
        [R, p] = H2Rp(H);
        
        % Returns the homogenous matrix H from the Rotation matrix R and the position vector p.
        H = Rp2H(R,p);
        
        % Generates a [3x3] skew-symmetric matrix out of a [3x1] vector.
        X = skew(x);
        
        % converts a rotation matrix into Euler angles.
        % The Euler angles convention follows the one of iDyntree and is such that the rotation
        % matrix is:  R = Rz(yaw)*Ry(pitch)*Rx(roll).
        rollPitchYaw = rollPitchYawFromRotation(R);
        
        % compute the rotation matrix from a plane to the world frame using the normal axis of
        % the plane
        RotPlane2World = computeRotationMatrixFromNormal(normal_axis);
    end
end
