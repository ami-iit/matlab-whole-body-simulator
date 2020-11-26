% Add path to local source code, and parent folder for the making the dependencies classes package
% visible to MATLAB path.
addpath(strcat(fileparts(mfilename('fullpath')),'/..'));

% set paths to the osqp-matlab library
osqp_libraryPath=getenv('OSQP_MATLAB_PATH');
if exist(osqp_libraryPath, 'dir')==7
    addpath(genpath(osqp_libraryPath));
    disp('Added osqp-matlab library');
else
    error([osqp_libraryPath,': Path directory non existent!']);
end
