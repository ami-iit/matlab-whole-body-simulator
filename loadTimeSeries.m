clear all

%% Function list or the Matlab interpreter profiling

matlabProfilerFunctionNames = {
    'funcKey'      'robotClassMethod'                     'funcIndex' 'bindings'
    'MATLABsystem' ''                                     1           {}
    'MassMatrix'   'Robot.Robot>Robot.get_mass_matrix'    6           {'KinDynComputations.m>KinDynComputations.getFreeFloatingMassMatrix',}
    'BiasForces'   'Robot.Robot>Robot.get_bias_forces'    5           {'KinDynComputations.m>KinDynComputations.generalizedBiasForces'}
    'Jacobian'     'Robot.Robot>Robot.get_feet_jacobians' 9           {'KinDynComputations.m>KinDynComputations.getFrameFreeFloatingJacobian'}
    'DotJNu'       'Robot.Robot>Robot.get_feet_JDot_nu'   4           {'KinDynComputations.m>KinDynComputations.getFrameBiasAcc'}
    'FwrdKin'      'Robot.Robot>Robot.get_feet_H'         7           {'iDynTreeWrappers.getWorldTransform'}
%     'qpOASES'      ''                                     2           {}
};

[funcs2propsMap,orderedKeys] = mapFuncKeys2Properties(matlabProfilerFunctionNames);


%% Load the profilerResults_6af23d0 files

profileFiles = dir('profilerResults_before_optim/test_matlab_system_19*.mat');

execTimesBefOptim = procTotalTimesAndPlot(profileFiles,funcs2propsMap);

%% Load the profilerResults_c760c0d files

profileFiles = dir('profilerResults_c760c0d/test_matlab_system*.mat');

execTimesAftOptim = procTotalTimesAndPlot(profileFiles,funcs2propsMap);

%% Plot
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
function modulesExecTimes = procTotalTimesAndPlot(filesList,funcs2propsMap)

% Load the profiles
% 
% profilerDataArray = struct([]);
index = 1;
for aFile = filesList(:)'
    load([aFile.folder filesep aFile.name],'profilerData','profilerData_interpreter');
    functionArray(index,:) = profilerData.rootUINode.children(1).children;
    index=index+1;
end

modulesExecTimes = containers.Map();

if size(functionArray,2) > 1
    for func = funcs2propsMap.keys
        modulesExecTimes(cell2mat(func)) = mean([functionArray(:,funcs2propsMap(cell2mat(func)).funcIndex).totalTime]);
    end
else
    for func = funcs2propsMap.keys
        modulesExecTimes(cell2mat(func)) = 0;
    end
    modulesExecTimes('MATLABsystem') = mean([functionArray(:,1).totalTime]);
end

end

function [aMap,orderedKeys] = mapFuncKeys2Properties(inDescCellArray)

orderedKeys = inDescCellArray(2:end,1);
fieldNames = inDescCellArray(1,2:end);
valuesArray = cell2struct(inDescCellArray(2:end,2:end),fieldNames,2);
values = num2cell(valuesArray);

aMap = containers.Map(orderedKeys,values);

end
