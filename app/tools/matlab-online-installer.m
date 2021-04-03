%% create dev folder
mkdir '/MATLAB Drive/dev'

%% Install the tools in that folder

cd '/MATLAB Drive/dev'

% If the miniforge3 tool is not yet installed, do it
if ~exist([getenv('HOME'),'/miniforge3'],'dir')
    % Download the miniforge3 tool installer
    system('curl -LO https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh');
    % Install with default options
    system('sh Miniforge3-Linux-x86_64.sh -b');
end

% Add the conda binary to the bashrc and the PATH
if ~contains(getenv('PATH'),'miniforge3')
    setenv('PATH',['~/miniforge3/bin',':',getenv('PATH')]);
end
system('conda info');

% Install Mamba
system('conda install -y mamba');

% Install Git
system('mamba install -y git');

%% Install the dependencies and add the paths to MATLAB path

% Install the conda binaries from the robotology channel.
% Using the `conda create/activate/deactivate` commands requires to run `conda init` and source the
% bashrc, which won't have any effect on MATLAB online. For that reason we have to install the conda
% binaries from the robotology channel on the **base** environment.
system('mamba install -y -c robotology iDynTree qpOASES icub-models wb-toolbox whole-body-controllers');
addpath('~/miniforge3/mex');
addpath('~/miniforge3/share/WBToolbox');
addpath('~/miniforge3/share/WBToolbox/images');

%% Get the repo files

cd '/MATLAB Drive/dev'

% Clone the repository: shallow clone with a single branch 'devel' and latest commit on that branch
if ~exist('matlab-whole-body-simulator','dir')
    system('git clone --depth 1 https://github.com/dic-iit/matlab-whole-body-simulator --branch devel --single-branch');
    cd matlab-whole-body-simulator
else
    cd matlab-whole-body-simulator
    system('git checkout devel');
    system('git pull origin devel');
end
