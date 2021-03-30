# matlab-whole-body-simulator

**matlab-whole-body-simulator** is a simulator for the humanoid robots. It has been designed to work in Simulink.
In the simulator the ground is assumed to be flat and the contact forces are computed using the **Maximum dissipation principle** and under the linear approximation of the friction cones assumption.

## :hammer: Dependencies

- [Matlab/Simulink 2019b](https://it.mathworks.com/products/matlab.html)
- [YARP](https://github.com/robotology/yarp) & [yarp-matlab-bindings](https://github.com/robotology/yarp-matlab-bindings): Yarp Resource Finder.
- [WBToolbox](https://github.com/robotology/wb-toolbox): WB-Toolbox library Simulink blocks.
- [iDynTree](https://github.com/robotology/idyntree): Dynamic computations (through bindings) and WB-Toolbox library dependencies.
- [qpOASES](https://github.com/robotology-dependencies/qpOASES): QP solver.
- [icub-models](https://github.com/robotology/icub-models): access to the iCub models.
- [Whole-Body-Controllers](https://github.com/robotology/whole-body-controllers): `+wbc` package helpers for kinematics & dynamics computations.

It is recommended to install these dependencies using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild) resources:
- Either installing the full superbuild from source.
- Either installing the required binary packages derived from the above listed dependencies, using the conda package manager, from the robotology channel.

## :floppy_disk: Installing the dependencies through the robotology superbuild source

- Clone and build the robotology-superbuild following the steps in https://github.com/robotology/robotology-superbuild/blob/master/README.md.
- `icub-models`: set the profile option `ROBOTOLOGY_ENABLE_CORE`.
- `iDynTree`, `WBToolbox`, `Whole-Body-Controller`, `qpOASES` and `yarp-matlab-bindings`: set the profile option `ROBOTOLOGY_ENABLE_DYNAMICS` and `ROBOTOLOGY_USES_MATLAB`.

**Note 1:** In general, for selecting the profile CMake options according to the sub-projects to install, refer to the table in the section [Profile CMake Options](https://github.com/robotology/robotology-superbuild/blob/master/doc/profiles.md#profile-cmake-options).
**Note 2:** By setting the profile `ROBOTOLOGY_ENABLE_DYNAMICS`in the superbuild cmake options. The `qpOASES` library will then be installed along with `iDynTree` as an external dependency.
**Note 3:** Previously, two other optional QP solvers were available: OSQP and the native MATLAB solver `quadprog`. OSQP bindings and `quadprog` are not supported by the Simulink code generation build option in the `step_block` MATLAB System block. For this reason, the `step_block` MATLAB System block uses the qpOASES WBT block through a Simulink function call, and this choice is hardcoded in the class `Contacts`. As soon as the OSQP WBT block is created, it will replace the qpOASES Solver block.

## :floppy_disk: Installing the dependencies from the conda robotology channel

1. Install Mamba, create a new environment and install the robotology dependency binaries:
    ```
    $ conda install mamba
    $ conda create -n robotologyenv
    $ conda activate robotologyenv
    $ mamba install -c robotology iDynTree qpOASES icub-models wb-toolbox whole-body-controllers
    ```
5. Check the MATLABPATH environment variable. It should now have:
    ```
    <user-home-dir>/miniforge3/envs/robotologyenv/mex: <user-home-dir>/miniforge3/envs/robotologyenv/share/WBToolbox: <user-home-dir>/miniforge3/envs/robotologyenv/share/WBToolbox/images
    ``` 
    Mex libraries:
    ```
    $ ls <user-home-dir>/miniforge3/envs/robotologyenv/mex/
    ```
    +iDynTree
    +iDynTreeWrappers
    +wbc
    BlockFactory.mexmaci64
    BlockFactory.tlc
    SwigGet.m
    SwigMem.m
    SwigRef.m
    iDynTreeMEX.mexmaci64
    mesh2tri.m
    ```
6. Clone the repository  `matlab-whole-body-simulators`
    ```
    $ git clone https://github.com/dic-iit/matlab-whole-body-simulator.git
    ```
7. Run matlab in the same conda environment.
8. Change working directory to the root path of repository `matlab-whole-body-simulators`
9. Open and run the model `test_matlab_system_2020b.mdl`.

## :runner: How to use the simulator

Just connect your controller to the robot block. This block takes as imput the **joints torque** and the (possible) **generalized external forces** and outputs the **robot state** and **contact wrenches** in the left and right sole frames (in order to simulate a sensor on the feet).

![image](https://user-images.githubusercontent.com/29798643/92244565-617f7d80-eec3-11ea-95d0-2a15f1bdb54f.png)

To run and example open and launch `test_matlab_system.mdl`. It also contains a callback to the `init` file that retrieves the needed information.
