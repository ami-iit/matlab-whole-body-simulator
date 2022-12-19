% Create in the workspace the bus required for having an output bus with the kynematic & dynamic
% variables.
kinDynOut_bus = initKinDynOut_bus(robot_config);
assignin('base',"kinDynOut_bus",kinDynOut_bus);

% Get the model path from the Urdf file name and the YARP resource finder
if ~isfield(robot_config,'UrdfFile') && isfield(robot_config,'fileName')
	robot_config.UrdfFile = robot_config.fileName;
elseif ~isfield(robot_config,'UrdfFile') && ~isfield(robot_config,'fileName')
	error('[initRobotDyanmicsWithContactsCB]: The urdf file is not provided in "robot_config".');
end
[robot_config.modelPath,robot_config.fileName] = mwbs.getModelPathFromFileNameAndYarpFinder(robot_config.UrdfFile);




%======== LOCAL FUNCTIONS ====================

function kinDynOut_bus = initKinDynOut_bus(robot_config)

% --------------------- INITIALIZATION -----------------------------------
if ~isfield(robot_config.robotFrames,'IN_CONTACT_WITH_GROUND')
    num_in_contact_frames = 1;
elseif isempty(robot_config.robotFrames.IN_CONTACT_WITH_GROUND)
    num_in_contact_frames = 1;
else
    num_in_contact_frames = length(robot_config.robotFrames.IN_CONTACT_WITH_GROUND);
end 

% Variable properties to stream on the output bus
kinDynListSignals = {
'name','format','type'; ...
'w_H_b','Htrans','double'; ...
's','genJointVec','double'; ...
'nu','genBaseJointVec','double'; ...
'w_H_feet_sole','HtransGrp','double'; ...
'J_feet_sole','JcbianWGrp','double'; ...
'JDot_feet_sole_nu','wrenchGrp','double'; ...
'M','massMtx','double'; ...
'h','genBaseJointVec','double'; ...
'motorGrpI','genJointVec','double'; ...
'fc','wrenchGrp','double'; ...
'nuDot','genBaseJointVec','double'; ...
'feet_in_contact','BlGrp','boolean'; ...
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
            kinDynListDimensions{formatIdx} = [4*num_in_contact_frames,4];
        case 'genJointVec'
            kinDynListDimensions{formatIdx} = [double(robot_config.N_DOF),1];
        case 'genBaseJointVec'
            kinDynListDimensions{formatIdx} = [double(6+robot_config.N_DOF),1];
        case 'JcbianW'
            kinDynListDimensions{formatIdx} = [6,6+double(robot_config.N_DOF)];
        case 'JcbianWGrp'
            kinDynListDimensions{formatIdx} = [6*num_in_contact_frames,6+double(robot_config.N_DOF)];
        case 'massMtx'
            kinDynListDimensions{formatIdx} = 6+size(robot_config.N_DOF_MATRIX);
        case 'wrench'
            kinDynListDimensions{formatIdx} = [6,1];
        case 'wrenchGrp'
            kinDynListDimensions{formatIdx} = [6*num_in_contact_frames,1];
        case 'dbleWrench'
            kinDynListDimensions{formatIdx} = [12,1];
        case 'BlGrp'
            kinDynListDimensions{formatIdx} = [1,num_in_contact_frames];
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
