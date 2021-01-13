classdef KinDynComputations < handle
    %KINDYNCOMPUTATIONS The calss wraps the iDynTree::KinDynComputations methods using Simulink function calls.
    %   Detailed explanation goes here
    
    properties (Access = private)
        w_H_b;
        s;
        base_pose_dot;
        s_dot;
        NDOF;
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
        
        function [ack,J_LRfoot] = getFrameFreeFloatingJacobian(obj,LRfoot)
            ack = true;
            switch LRfoot
                case 'LFoot'
                    J_LRfoot = simFunc_getFrameFreeFloatingJacobianLFoot(obj.w_H_b,obj.s);
                case 'RFoot'
                    J_LRfoot = simFunc_getFrameFreeFloatingJacobianRFoot(obj.w_H_b,obj.s);
                otherwise
                    error('Unsupported "getFrameFreeFloatingJacobian" input parameter.');
                    ack = false;
            end
        end
        
        function M = getFreeFloatingMassMatrix(obj)
            M = simFunc_getFreeFloatingMassMatrix(obj.w_H_b,obj.s);
        end
    end
end
