% Create in the workspace the bus required for having an output bus with the kynematic & dynamic
% variables.
kinDynOut_bus = initKinDynOut_bus(robot_config);

% Get the model path from the Urdf file name and the YARP resource finder
if ~isfield(robot_config,'UrdfFile') && isfield(robot_config,'fileName')
	robot_config.UrdfFile = robot_config.fileName;
elseif ~isfield(robot_config,'UrdfFile') && ~isfield(robot_config,'fileName')
	error('[initRobotDyanmicsWithContactsCB]: The urdf file is not provided in "robot_config".');
end
[robot_config.modelPath,robot_config.fileName] = mwbs.getModelPathFromFileNameAndYarpFinder(robot_config.UrdfFile);




%======== LOCAL FUNCTIONS ====================

function kinDynOut_bus = initKinDynOut_bus(robot_config)

% Variable properties to stream on the output bus
kinDynListSignals = {
'name','format','type'; ...
'w_H_b','Htrans','double'; ...
's','genJointVec','double'; ...
'nu','genBaseJointVec','double'; ...
'w_H_frames_sole','HtransGrp','double'; ...
'J_frames_sole','JcbianWGrp','double'; ...
'JDot_frames_sole_nu','wrenchGrp','double'; ...
'M','massMtx','double'; ...
'h','genBaseJointVec','double'; ...
'motorGrpI','genJointVec','double'; ...
'fc','wrenchGrp','double'; ...
'nuDot','genBaseJointVec','double'; ...
'frames_in_contact','BlGrp','boolean'; ...
};

% Convert to names and dimensions
kinDynListNames = kinDynListSignals(2:end,1);
kinDynListFormats = kinDynListSignals(2:end,2);
kinDynListTypes = kinDynListSignals(2:end,3);
kinDynListDimensions = cell(size(kinDynListNames));
for formatIdx = 1:numel(kinDynListFormats)
    switch kinDynListFormats{formatIdx}
        case 'Htrans'
            kinDynListDimensions{formatIdx} = [4,4];
        case 'HtransGrp'
            kinDynListDimensions{formatIdx} = [4*length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND),4];
        case 'genJointVec'
            kinDynListDimensions{formatIdx} = [double(robot_config.N_DOF),1];
        case 'genBaseJointVec'
            kinDynListDimensions{formatIdx} = [double(6+robot_config.N_DOF),1];
        case 'JcbianW'
            kinDynListDimensions{formatIdx} = [6,6+double(robot_config.N_DOF)];
        case 'JcbianWGrp'
            kinDynListDimensions{formatIdx} = [6*length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND),6+double(robot_config.N_DOF)];
        case 'massMtx'
            kinDynListDimensions{formatIdx} = 6+size(robot_config.N_DOF_MATRIX);
        case 'wrench'
            kinDynListDimensions{formatIdx} = [6,1];
        case 'wrenchGrp'
            kinDynListDimensions{formatIdx} = [6*length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND),1];
        case 'dbleWrench'
            kinDynListDimensions{formatIdx} = [12,1];
        case 'BlGrp'
            kinDynListDimensions{formatIdx} = [1,length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND)];
        otherwise
            eval(['kinDynListDimensions{formatIdx} = ',kinDynListFormats{formatIdx}]);
    end
end

elems(1:numel(kinDynListNames)) = Simulink.BusElement;
[elems.Name] = deal(kinDynListNames{:});
[elems.Dimensions] = deal(kinDynListDimensions{:});
[elems.DimensionsMode] = deal('Fixed');
[elems.DataType] = deal(kinDynListTypes{:});
[elems.SampleTime] = deal(-1);
[elems.Complexity] = deal('real');
 
kinDynOut_bus = Simulink.Bus;
kinDynOut_bus.Elements = elems;

end
