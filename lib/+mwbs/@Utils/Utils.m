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
    end
end
