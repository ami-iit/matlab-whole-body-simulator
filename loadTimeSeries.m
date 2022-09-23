clear all

%% Function list or the Matlab interpreter profiling

matlabProfilerFunctionNames = {
    'funcKey'      'robotClassMethod'                     'funcIndex' 'bindings'
    'MATLABsystem' 'step_block>step_block.stepImpl'       1           {}
    'MassMatrix'   'Robot.Robot>Robot.get_mass_matrix'    6           {'KinDynComputations>KinDynComputations.getFreeFloatingMassMatrix','toMatlab'}
    'BiasForces'   'Robot.Robot>Robot.get_bias_forces'    5           {'KinDynComputations>KinDynComputations.generalizedBiasForces','toMatlab'}
    'Jacobian'     'Robot.Robot>Robot.get_feet_jacobians' 9           {'KinDynComputations>KinDynComputations.getFrameFreeFloatingJacobian','toMatlab'}
    'DotJNu'       'Robot.Robot>Robot.get_feet_JDot_nu'   4           {'KinDynComputations>KinDynComputations.getFrameBiasAcc','toMatlab'}
    'FwrdKin'      'Robot.Robot>Robot.get_feet_H'         7           {'KinDynComputations>KinDynComputations.getWorldTransform','Transform>Transform.asHomogeneousTransform','Matrix4x4>Matrix4x4.toMatlab'}
%     'qpOASES'      ''                                     2           {}
};

[funcs2propsMap,orderedKeys] = mapFuncKeys2Properties(matlabProfilerFunctionNames);


%% Load the profiler results before optim (commit 6af23d0)

profileFiles = dir('profilerResults_before_optim/test_matlab_system*.mat');

execTimesBefOptim = procTotalTimesAndPlot(profileFiles,funcs2propsMap,'beforeOptim');

%% Load the profiler results after optim (266512a)

profileFiles = dir('profilerResults_after_optim/test_matlab_system*.mat');

execTimesAftOptim = procTotalTimesAndPlot(profileFiles,funcs2propsMap,'afterOptim');

%% Plot
orderedKeys = ['RobotDynWithContacts';orderedKeys];
X = categorical(orderedKeys);
X = reordercats(X,flip(orderedKeys));
Y = [cell2mat(execTimesAftOptim.values(orderedKeys)),cell2mat(execTimesBefOptim.values(orderedKeys))];
barh(X,Y,'grouped');

title('Function execution time before and after optimisation','Fontsize',18,'FontWeight','bold');
grid on;
xlabel('exec time (seconds)','Fontsize',18);
ylabel('function','Fontsize',18);
legend('after optim','before optim','Location','SouthEast');
set(gca,'FontSize',18);

%% Local functions
function modulesExecTimes = procTotalTimesAndPlot(filesList,funcs2propsMap,beforeOrAfterOptim)

% Load the profiles
% 
index = 1;
% functionRobotDynWithContacts = {};
for aFile = filesList(:)'
    load([aFile.folder filesep aFile.name],'profilerData','profilerData_interpreter');
    
    switch (beforeOrAfterOptim)
        case 'beforeOptim'
            functionArraySimulink(index,1) = profilerData.rootUINode.children(1).children;
            functionNames = {profilerData_interpreter.FunctionTable.FunctionName};
            funcIdxes = [];
            for func = funcs2propsMap.keys
                [~,funcIdx] = ismember(1,contains(functionNames,funcs2propsMap(cell2mat(func)).robotClassMethod));
                funcIdxes = [funcIdxes,funcIdx];
            end
            functionArrayMatlab(index,:) = profilerData_interpreter.FunctionTable(funcIdxes);
        case 'afterOptim'
            functionArraySimulink(index,:) = profilerData.rootUINode.children(1).children;
            functionArrayMatlab = [];
        otherwise
    end
    
    functionRobotDynWithContacts(index) = profilerData.rootUINode.children(1);
    index=index+1;
end

modulesExecTimes = containers.Map();

% Compute the total times
% 
switch (beforeOrAfterOptim)
    case 'beforeOptim'
        funcIndex = 1;
        for func = funcs2propsMap.keys
            modulesExecTimes(cell2mat(func)) = mean([functionArrayMatlab(:,funcIndex).TotalTime]);
            funcIndex = funcIndex+1;
        end
        % MATLABsystem and step_block>step_block.stepImpl blocks take approximately the same
        % time, so ovwrwrite it such that we have an accurate comparison btw before and after
        % the optimisation.
        MATLABsystemTotalTime = mean([functionArraySimulink(:,funcs2propsMap('MATLABsystem').funcIndex).totalTime]);
        if (abs(MATLABsystemTotalTime-modulesExecTimes('MATLABsystem'))/MATLABsystemTotalTime>0.05)
            warning('Total time of MATLABsystem and step_block>step_block.stepImpl mismatch');
        end
        modulesExecTimes('MATLABsystem') = MATLABsystemTotalTime;
    case 'afterOptim'
        for func = funcs2propsMap.keys
            modulesExecTimes(cell2mat(func)) = mean([functionArraySimulink(:,funcs2propsMap(cell2mat(func)).funcIndex).totalTime]);
        end
    otherwise
end

modulesExecTimes('RobotDynWithContacts') = mean([functionRobotDynWithContacts.totalTime]);

end

function [aMap,orderedKeys] = mapFuncKeys2Properties(inDescCellArray)

orderedKeys = inDescCellArray(2:end,1);
fieldNames = inDescCellArray(1,2:end);
valuesArray = cell2struct(inDescCellArray(2:end,2:end),fieldNames,2);
values = num2cell(valuesArray);

aMap = containers.Map(orderedKeys,values);

end
