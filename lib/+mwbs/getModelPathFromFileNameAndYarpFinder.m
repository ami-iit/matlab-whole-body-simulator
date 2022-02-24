function [modelPath,robotFileName] = getModelPathFromFileNameAndYarpFinder(robotFileName)
%getModelInfoFromRobotCfgAndYarpFinder Returns the model path and mex path prefix and robot file name
%   This function uses the Yarp Resource Finder for finding the model file if the robot file name does not contain the model path

% Extract the model path from the robot file name if it exists.
modelPath = fileparts(robotFileName);

if isempty(modelPath) % if the model path is not mentioned in the robot file name, find the path through the Yarp Resources Finder
	% Instanciate the YARP resource finder
	YarpRF = yarp.ResourceFinder;
	% Get the model file
	modelPathFromYarpFinder = YarpRF.findFileByName(robotFileName);
	% Just return the parent folder path
	modelPath = strcat(fileparts(modelPathFromYarpFinder),'/');
else
	modelPath = strcat(modelPath,'/');
	robotFileName = erase(robotFileName,modelPath);
end
