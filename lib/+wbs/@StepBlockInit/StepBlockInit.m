classdef StepBlockInit < handle
    % step_block_contacts This block takes as input the joint torques and the
    % applied external forces and evolves the state of the robot

    properties (Access = private)
        robot; contacts; state;
    end
    
    properties (Constant = true)
        sharedConfig = wbs.StepBlockInit();
    end

    methods (Static = true)
        function setSharedConfig(obj)
            theSingleton = wbs.StepBlockInit.sharedConfig;
            theSingleton.robot = obj.robot;
            theSingleton.contacts = obj.contacts;
            theSingleton.state = obj.state;
        end

        function obj = getSharedConfig(obj)
            theSingleton = wbs.StepBlockInit.sharedConfig;
            obj.robot = theSingleton.robot;
            obj.contacts = theSingleton.contacts;
            obj.state = theSingleton.state;
        end
    end

    methods (Access = protected)
        function obj = StepBlockInit()
        end
    end
end
