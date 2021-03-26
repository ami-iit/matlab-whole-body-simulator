function [modelPath,meshFilePrefix] = getModelInfoFromRobotCfgAndYarpFinder(robot_config)
%getModelInfoFromRobotCfgAndYarpFinder Returns the model path and mex path prefix
%   This function uses the Yarp Resource Finder for finding the model file.

[~,modelPathFromYarpFinder] = system(['yarp resource --find ',robot_config.fileName]);
modelPath = strcat(fileparts(char(eval(modelPathFromYarpFinder))),'/');
meshFilePrefixSplit = strsplit(modelPath,'share');
meshFilePrefix = strcat(strjoin(meshFilePrefixSplit(1:end-1),'share'),'share');

end
