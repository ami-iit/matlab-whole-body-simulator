classdef KinDynComputations < handle
    %KINDYNCOMPUTATIONS The calss wraps the iDynTree::KinDynComputations methods using Simulink function calls.
    %   Detailed explanation goes here
    
    properties (Access = private)
        w_H_b (4, 4) double;
        s;
        base_pose_dot (6, 1) double;
        s_dot;
        NDOF double;
    end
    
    methods
        function obj = KinDynComputations()
        end
        
        function setRobotState(obj,w_H_b,s,base_pose_dot,s_dot)
            % Sets the robot state the wrapper methods depend on
            [obj.w_H_b,obj.s,obj.base_pose_dot,obj.s_dot] = deal(w_H_b,s,base_pose_dot,s_dot);
            obj.NDOF = numel(s_dot);
        end
        
        function [w_H_b,s,base_pose_dot,s_dot,NDOF] = getRobotState(obj)
            % Gets the robot state the wrapper methods depend on
            [w_H_b,s,base_pose_dot,s_dot,NDOF] = deal(obj.w_H_b,obj.s,obj.base_pose_dot,obj.s_dot,obj.NDOF);
        end
        
        function [ack,J_LRfoot] = getFrameFreeFloatingJacobianLRfoot(obj,LRfoot)
            ack = true;
            switch LRfoot
                case 'LFoot'
                    J_LRfoot = simFunc_getFrameFreeFloatingJacobianLFoot(obj.w_H_b,obj.s);
                case 'RFoot'
                    J_LRfoot = simFunc_getFrameFreeFloatingJacobianRFoot(obj.w_H_b,obj.s);
                otherwise
                    ack = false;
            end
        end
        
        function [ack,M] = getFreeFloatingMassMatrix(obj)
            M = simFunc_getFreeFloatingMassMatrix(obj.w_H_b,obj.s);
            ack = true;
        end
        
        function [ack,h] = generalizedBiasForces(obj)
            h = simFunc_generalizedBiasForces(obj.w_H_b,obj.s,obj.base_pose_dot,obj.s_dot);
            ack = true;
        end
        
        function JDot_nu_LRfoot = getFrameBiasAccLRfoot(obj,LRfoot)
            switch LRfoot
                case 'LFoot'
                    JDot_nu_LRfoot = simFunc_getFrameBiasAccLFoot(obj.w_H_b,obj.s,obj.base_pose_dot,obj.s_dot);
                case 'RFoot'
                    JDot_nu_LRfoot = simFunc_getFrameBiasAccRFoot(obj.w_H_b,obj.s,obj.base_pose_dot,obj.s_dot);
                otherwise
                    error('Unsupported "getFrameBiasAccRFoot" input parameter.');
            end
        end
        
        function transform = getWorldTransformLRfoot(obj,LRfoot)
            switch LRfoot
                case 'LFoot'
                    transform = simFunc_getWorldTransformLFoot(obj.w_H_b,obj.s);
                case 'RFoot'
                    transform = simFunc_getWorldTransformRFoot(obj.w_H_b,obj.s);
                otherwise
                    error('Unsupported "getWorldTransform" input parameter.');
            end
        end
    end
end
