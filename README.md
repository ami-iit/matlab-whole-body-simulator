# matlab-whole-body-simulator

**matlab-whole-body-simulator** is a simulator for the humanoid robots. It has been designed to work in Simulink.
In the simulator the ground is assumed to be flat and the contact forces are computed using the **Maximum dissipation principle** and under the linear approximation of the friction cones assumption.

## :hammer: Dependencies

- [Matlab/Simulink 2019b](https://it.mathworks.com/products/matlab.html)
- [yarp-matlab-bindings](https://github.com/robotology/yarp-matlab-bindings): Yarp Resource Finder.
- [WBToolbox](https://github.com/robotology/wb-toolbox): WB-Toolbox library Simulink blocks.
- [iDynTree](https://github.com/robotology/idyntree): Dynamic computations (through bindings) and WB-Toolbox library dependency.
- [qpOASES](https://github.com/robotology-dependencies/qpOASES): QP solver for estimating the contact wrenches.
- [icub-models](https://github.com/robotology/icub-models): access to the iCub models.

## :floppy_disk: Binary Installation

This section is for users wishing to run an example showcase of the library or integrate the library in a wider Simulink model they are implementing., it is recommended to:
- either use the [Conda package manager](https://anaconda.org) for installing directly the `conda` package [`matlab-whole-body-simulator`](https://anaconda.org/robotology/matlab-whole-body-simulator), available since the `conda` build number 8 of the `conda` binaries hosted in the [robotology conda channel](https://anaconda.org/robotology),
- either use a [one line installer](https://github.com/robotology/robotology-superbuild/blob/master/doc/matlab-one-line-install.md#one-line-installation-of-robotology-matlabsimulink-packages), meant for users not familiar with the `conda` package manager nor [Git](https://git-scm.com/).

### One Line Installation

For users not familiar with the `conda` package manager nor Git version control system, a one line installer is available, that can be downloaded and run from the Matlab command line, without any access to a terminal:
1. Run Matlab.
1. In the Matlab command line change the current folder to a directory where you wish to download the one installer script and install all the packages.
1. Run the following commands:
```
websave('install_robotology_packages.m', 'https://raw.githubusercontent.com/robotology/robotology-superbuild/master/scripts/install_robotology_packages.m')
install_robotology_packages
robotology_setup
```

For testing the installation, run the example as described in [this section](#eyes-checking-the-installation).

### Binary Installation from the Conda Robotology Channel

If you're familiar with running shell commands in a terminal, handling environment variables, and have a home directory where you can freely install a package manager, you can run the following steps:
1. If you are not already using the `conda` package manager, install the `conda` miniforge distribution following https://github.com/robotology/robotology-superbuild/blob/master/doc/install-miniforge.md#linux. Remember to restart your shell session or run `source ~/.bashrc` (`~/.bash_profile` on MacOS) for the `conda init` command to take effect.
2. Install Mamba, create a new environment and install the robotology dependency binaries:
    ```
    conda install mamba
    conda create -n robotologyenv
    conda activate robotologyenv
    mamba install -c robotology matlab-whole-body-simulator
    ```
    To read more about installing `robotology` package binaries refer to https://github.com/robotology/robotology-superbuild/blob/master/doc/conda-forge.md#binary-installation.
3. Check the MATLABPATH environment variable. It should now have...
    ```
    <user-home-dir>/miniforge3/envs/robotologyenv/mex: <user-home-dir>/miniforge3/envs/robotologyenv/share/WBToolbox: <user-home-dir>/miniforge3/envs/robotologyenv/share/WBToolbox/images
    ```
    Check the mex and Simulink libraries in the folder `<user-home-dir>/miniforge3/envs/robotologyenv/mex`. It should contain:
    ```
    +iDynTree               BlockFactory.mexmaci64      mwbs_lib.slx
    +iDynTreeWrappers       BlockFactory.tlc            mwbs_robotDynamicsWithContacts_lib.slx
    +mwbs                   iDynTreeMEX.mexmaci64       mwbs_robotSensors_lib.slx
    +yarp.                  yarpMEX.mexmaci64           mwbs_visualizers_lib.slx
    ```
4. The `Matla Whole Body Simulator` library, along with the sub-libraries **robotDynamicsWithContacts**, **robotSensors** and **visualizers** should be visible in the Simulink Library Browser. They can be drag and dropped into any open Simulink model.
<img width="963" alt="image" src="https://user-images.githubusercontent.com/6848872/116485698-1ff57580-a88c-11eb-8856-c4527e00b401.png">

For testing the installation, run the example as described in [this section](#eyes-checking-the-installation).

### :cloud: One Line Installation in MATLAB Online Session

This use case is very convenient if a local host with installed MATLAB application and license is not available, or simply if the user wishes to leave his usual working environment unchanged by the package dependencies of this simulator framework.

With a MATLAB account, one can sign in and access to [MATLAB online](https://www.mathworks.com/products/matlab-online.html#connect-to-the-cloud), an online workspace that provides MATLAB and Simulink from any standard web browser. The GUI is practically identical to the one provided by the desktop application.

The procedure is similar to the [One Line Installation](#one-line-installation) one, except that you run it in the MATLAB online session and the command set is slightly different (refer to [robotology-superbuild/doc/matlab-one-line-install.md](https://github.com/robotology/robotology-superbuild/blob/master/doc/matlab-one-line-install.md#installation-on-matlab-online)).

The same example integrating the "YOGA++" controller and the **Matlab Whole-body Simulator simulator** library blocks can be run in MATLAB Online in the same way as described in section [One Line Installation](#one-line-installation).

For testing the installation, run the example as described in [this section](#eyes-checking-the-installation).

## :floppy_disk: Source Installation

This section is for developers wishing to implement new features or fixes in the library **Matlab Whole Body Simulator**. This repository has to be cloned and the modules listed in the [dependencies section](#hammer-dependencies) need to be installed.

It is recommended to follow one of the two procedures using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild) resources:
- Get the `matlab-whole-body-simulators` repository and the dependencies by installing the full superbuild from source.
- clone the `matlab-whole-body-simulators` repository, then install the dependency binary packages from the [conda robotology channel](https://anaconda.org/robotology)using the [miniforge conda distribution](https://github.com/conda-forge/miniforge) package manager.

### [Source Installation using the full superbuild installation from source](doc/source-installation-using-the-full-superbuild-installation-from-source.md).

### [Source Installation using the robotology conda packages](doc/source-installation-using-the-robotology-conda-packages.md).

For testing the installation, run the example as described in [this section](#eyes-checking-the-installation).


## :eyes: Checking The Installation

A [Simulink model example](https://github.com/robotology/whole-body-controllers/tree/master/controllers/%2BfloatingBaseBalancingTorqueControlWithSimulator), from the repository `whole-body-controllers`, integrates a controller model with the simulator library **RobotDynWithContacts** from this repository. The controller (labelled "YOGA++") controls a humanoid robot (iCub) in performing a dynamic trajectory while balancing.

For getting the `whole-body-controllers` repository, follow [this guide](doc/getting-whole-body-controllers.md).

You can then run the model from the MATLAB command line, from any directory, as follows:
```
floatingBaseBalancingTorqueControlWithSimulator.torqueControlBalancingWithSimu
```


## :runner: How to use the simulator

1. Connect your controller to the **RobotDynWithContacts** block. This block takes as imput the **joints torque**, **motor inertia** and an eventual **generalized external wrench**. It outputs the robot **state**, the contact wrenches **wrench_LFoot** and **wrench_RFoot**, respectively applied to the left and right foot (sole frames), and **kinDynOut**, an output bus exposing all the computed dynamics quantities relevant for debugging or the extension of dynamics computations in external blocks (emulation of an IMU sensor, of pressure sensors on the feet, etc).
    [A test model](https://dic-iit.github.io/matlab-whole-body-simulator/doc/webview/test_matlab_system_2020b/webview.html) was created for illustrating how the library can be integrated with the IMU sensor block and a dummy controller providing desired torques.
3. Select the robot model by setting the environment variable `YARP_ROBOT_NAME` (e.g. `setenv('YARP_ROBOT_NAME','iCubGenova04')`). The available models are:
    | Model description | iCub robot (iCubGenova04) | RRBot from Gazebo |
    | --- | --- | --- |
    | Preview | <img width="982" alt="iCubGenova04" src="https://user-images.githubusercontent.com/6848872/114422028-31652f00-9bb6-11eb-987b-62e9b5b38811.png"> | <img width="930" alt="RRbot1" src="https://user-images.githubusercontent.com/6848872/114421414-910f0a80-9bb5-11eb-8256-a1f8678fec5a.png"> |
    | $YARP_ROBOT_NAME | `'iCubGenova04'` | `'RRbot1'` |

3. Set the configuration parameters `robot_config`, `contact_config` and `physics_config`.
<img width="1035" alt="image" src="https://user-images.githubusercontent.com/6848872/117595197-742c0f80-b140-11eb-8b7c-85f3274760d9.png">

   **Details:**
|| Structure | Field name | Size/Type | Description |
| --- | --- | --- | --- | --- |
|<td rowspan="8">robot_config</td>| jointOrder | [1×N_DOF] / cell | List of N_DOF "controlled" joints (matches dimension of joint torques input) |
|                                 |                  meshFilePrefix  | char | Prefix to be concatenated to the mesh file path specified in the URDF model file (*) |
|                                 |                        fileName  | char | File name of the URDF model (typically: model.urdf) |
|                                 |                           N_DOF  | [1x1] double | Robot degrees of freedom |
|                                 |                    N_DOF_MATRIX  | [23×23] double] | `eye(robot_config.N_DOF)` |
|                                 |               initialConditions  | [1×1] struct | Base pose and velocity. Joint positions and velocities |
|                                 | SIMULATE_MOTOR_REFLECTED_INERTIA | [1x1] logical | Activate motor reflected inertia emulation |
|                                 |                     robotFrames  | [1×1] struct | Selected reference frames (base, left/right sole) |
|<td rowspan="3">contact_config</td>|                     foot_print | [3×4] double | 4 Contact points on one of the feet soles |
|                                   |             total_num_vertices | [1x1] double | Total number of contact points (Nv=8) |
|                                   |           friction_coefficient | [1x1] double | Ground/feet Coulomb friction coefficient $\mu _{xy}$ (F _{\bot} = \mu _{xy} N) |
|<td rowspan="3">physics_config</td>|                    GRAVITY_ACC | [1x3] double | Gravity vector (gz = -9.81) |
|                                   |                      TIME_STEP | [1x1] double | Simulator sampling time (recommended 1e-03) |

### Notes
(*) Since iDynTree 3.0.0, if meshFilePrefix='', the standard iDynTree workflow of locating the mesh via the `ExternalMesh.getFileLocationOnLocalFileSystem` method is used. The iCub model meshes file tree is compatible with this workflow.

4. Set to `vector` or `matrix` the parameter "input motor reflected inertia format" from the pop-up pick-list. The selected format should match the format of the input signal `motorInertias` and respective definition as follows:

<img width="470" alt="image" src="https://user-images.githubusercontent.com/6848872/117595272-adfd1600-b140-11eb-8481-698f7b1773d9.png">

| parameter value | `motorInertias` format | represented quantity |
| --- | --- | --- |
| vector | [NDOFx1] | Column vector composed of the diagonal terms of $\Gamma^{-\top} G^{-1} I _m G^{-1} \Gamma^{-1}$ |
| matrix | [NDOFxNDOF] | Matrix defined by $\Gamma^{-\top} G^{-1} I _m G^{-1} \Gamma^{-1}$ |

The motor inertia reflected on the mass matrix of the articulated system is given by $\Gamma^{-\top} G^{-1} I _m G^{-1} \Gamma^{-1}$, which results in a diagonal matrix in the case of a robot without coupled joints. In such case, we only need to pass to **RobotDynWithContacts** the diagonall terms as a [NDOFx1] column vector. Otherwise, in presence of coupled joints (as in the case of iCub), the coupling matrix $\Gamma$ is not diagonal (the gearbox ratio matrix $G$ is though). The same applies to the term $\Gamma^{-\top} G^{-1} I _m G^{-1} \Gamma^{-1}$, which now does not have only diagonal terms and has to be passed to the block **RobotDynWithContacts** in its full matrix form of dimension [N_DOFxN_DOF].


5. To run and example open and launch `test_matlab_system.mdl`. It also contains a callback to the `init` file that retrieves the needed information.
