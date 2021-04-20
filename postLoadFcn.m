% Set path to the utility functions and to WBC library
import wbc.*
addpath(genpath('./src'));
if ~contains(path,'RobotDynamicsWithContacts')
    addpath('./lib/+mwbs/RobotDynamicsWithContacts');
end
