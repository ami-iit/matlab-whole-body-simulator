function modelPath = getModelPathFromFileNameAndYarpFinder(robotFileName)
%getModelInfoFromRobotCfgAndYarpFinder Returns the model path and mex path prefix
%   This function uses the Yarp Resource Finder for finding the model file.

[~,modelPathFromYarpFinder] = system(['yarp resource --find ',robotFileName]);
modelPath = strcat(fileparts(char(eval(modelPathFromYarpFinder))),'/');

end
