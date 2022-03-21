classdef KinDynComputations
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
        
        function obj = setRobotState(obj,w_H_b,s,base_pose_dot,s_dot)
            % Sets the robot state the wrapper methods depend on
            [obj.w_H_b,obj.s,obj.base_pose_dot,obj.s_dot] = deal(w_H_b,s,base_pose_dot,s_dot);
            obj.NDOF = numel(s_dot);
        end
        
        function obj = setRobotVelocity(obj,base_pose_dot,s_dot)
            % Sets the robot velocity the wrapper methods depend on
            [obj.base_pose_dot,obj.s_dot] = deal(base_pose_dot,s_dot);
            obj.NDOF = numel(s_dot);
        end
        
        function [w_H_b,s,base_pose_dot,s_dot,NDOF] = getRobotState(obj)
            % Gets the robot state the wrapper methods depend on
            [w_H_b,s,base_pose_dot,s_dot,NDOF] = deal(obj.w_H_b,obj.s,obj.base_pose_dot,obj.s_dot,obj.NDOF);
        end
        
        function [ack,J_inGroundContact] = getFrameFreeFloatingJacobian_inContactFrames(obj)
            ack = true;
            J_inGroundContact = simFunc_getFrameFreeFloatingJacobian_inContactFrames(obj.w_H_b,obj.s);
        end
        
        function [ack,JDiff_splitPoint] = getFrameFreeFloatingJacobianSpilitPoints(obj)
            ack = true;
            JDiff_splitPoint = simFunc_getFrameFreeFloatingJacobianSpilitPoints(obj.w_H_b,obj.s);
        end
        
        function [ack,JDotNuDiff_splitPoint] = getFrameBiasACCSpilitPoints(obj)
            ack = true;
            JDotNuDiff_splitPoint = simFunc_getFrameBiasAccSpilitPoints(obj.w_H_b,obj.s,obj.base_pose_dot,obj.s_dot);
        end
        
        function [ack,M] = getFreeFloatingMassMatrix(obj)
            M = simFunc_getFreeFloatingMassMatrix(obj.w_H_b,obj.s);
            ack = true;
        end
        
        function [ack,h] = generalizedBiasForces(obj)
            h = simFunc_generalizedBiasForces(obj.w_H_b,obj.s,obj.base_pose_dot,obj.s_dot);
            ack = true;
        end
        
        function [ack,JDot_nu_inGroundContact] = getFrameBiasAcc_inContactFrames(obj)
            JDot_nu_inGroundContact = simFunc_getFrameBiasAcc_inContactFrames(obj.w_H_b,obj.s,obj.base_pose_dot,obj.s_dot);
            ack = true;
        end
        
        function [ack,transform] = getWorldTransform_inContactFrames(obj)
            transform = simFunc_getWorldTransform_inContactFrames(obj.w_H_b,obj.s);
            ack = true;
        end
    end
end
