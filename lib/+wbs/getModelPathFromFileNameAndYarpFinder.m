function modelPath = getModelPathFromFileNameAndYarpFinder(robotFileName)
%getModelInfoFromRobotCfgAndYarpFinder Returns the model path and mex path prefix
%   This function uses the Yarp Resource Finder for finding the model file.

% Instanciate the YARP resource finder
YarpRF = yarp.ResourceFinder;
% Get the model file
modelPathFromYarpFinder = YarpRF.findFileByName(robotFileName);
% Just return the parent folder path
modelPath = strcat(fileparts(modelPathFromYarpFinder),'/');

end
