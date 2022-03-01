%% configuration for the matlab iDyntree visualizer


% Get the model path from the Urdf file name and the YARP resource finder
if ~isfield(confVisualizer,'UrdfFile') && isfield(confVisualizer,'fileName')
	confVisualizer.UrdfFile = confVisualizer.fileName;
elseif ~isfield(confVisualizer,'UrdfFile') && ~isfield(confVisualizer,'fileName')
	error('[initRobotVisualizer]: The urdf file is not provided in "confVisualizer".');
end

[confVisualizer.modelPath,confVisualizer.fileName] = mwbs.getModelPathFromFileNameAndYarpFinder(confVisualizer.UrdfFile);
