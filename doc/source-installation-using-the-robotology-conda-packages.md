# Source Installation using the Dependencies from the Conda Robotology Channel

1. If you are not already using the `conda` package manager, install the `conda` miniforge distribution following https://github.com/robotology/robotology-superbuild/blob/master/doc/install-miniforge.md#linux. Remember to restart your shell session or run `source ~/.bashrc` (`~/.bash_profile` on MacOS) for the `conda init` command to take effect.
2. Install Mamba, create a new environment and install the robotology dependency binaries:
    ```
    conda install mamba
    conda create -n robotologyenv
    conda activate robotologyenv
    mamba install -c robotology yarp-matlab-bindings idyntree qpoases icub-models wb-toolbox
    ```
    To read more about installing robotology-superbuild package binaries refer to https://github.com/robotology/robotology-superbuild/blob/master/doc/conda-forge.md#binary-installation.
3. Check the MATLABPATH environment variable. It should now have...
    ```
    <user-home-dir>/miniforge3/envs/robotologyenv/mex: <user-home-dir>/miniforge3/envs/robotologyenv/share/WBToolbox: <user-home-dir>/miniforge3/envs/robotologyenv/share/WBToolbox/images
    ```
    Check the mex and Simulink libraries in the folder `<user-home-dir>/miniforge3/envs/robotologyenv/mex`. It should contain:
    ```
    +iDynTree               BlockFactory.mexmaci64      mwbs_lib.slx
    +iDynTreeWrappers       BlockFactory.tlc            mwbs_robotDynamicsWithContacts_lib.slx
    +yarp                   iDynTreeMEX.mexmaci64       mwbs_robotSensors_lib.slx
                            yarpMEX.mexmaci64           mwbs_visualizers_lib.slx
    ```
4. Clone the repository  `matlab-whole-body-simulators`
    ```
    git clone https://github.com/dic-iit/matlab-whole-body-simulator.git
    ```
5. Open MATLAB and Add the path `<matlab-whole-body-simulators-dir>/lib` to the top of the MATLAB path (done automatically after you open and compile the test model `test_matlab_system_2020b.mdl`).
6. The `Matla Whole Body Simulator` library, along with the sub-libraries **robotDynamicsWithContacts**, **robotSensors** and **visualizers** should be visible in the Simulink Library Browser (press F5 to refresh the Library Browser if otherwise). They can be drag and dropped into any open Simulink model.
<img width="963" alt="image" src="https://user-images.githubusercontent.com/6848872/116485698-1ff57580-a88c-11eb-8856-c4527e00b401.png">
