# matlab-whole-body-simulator

**matlab-whole-body-simulator** is a simulator for the humanoid robots. It has been designed to work in Simulink.
In the simulator the ground is assumed to be flat and the contact forces are computed using the **Maximum dissipation principle** and under the linear approximation of the friction cones assumption.

## :hammer: Dependencies

- [Matlab/Simulink 2019b](https://it.mathworks.com/products/matlab.html)
- [YARP](https://github.com/robotology/yarp) & [yarp-matlab-bindings](https://github.com/robotology/yarp-matlab-bindings): Yarp Resource Finder.
- [WBToolbox](https://github.com/robotology/wb-toolbox): WB-Toolbox library Simulink blocks.
- [iDynTree](https://github.com/robotology/idyntree): Dynamic computations (through bindings) and WB-Toolbox library dependency.
- [qpOASES](https://github.com/robotology-dependencies/qpOASES): QP solver for estimating the contact wrenches.
- [icub-models](https://github.com/robotology/icub-models): access to the iCub models.
- [Whole-Body-Controllers](https://github.com/robotology/whole-body-controllers): `+wbc` package helpers for kinematics & dynamics computations.

It is recommended to install these dependencies using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild) resources:
- Either installing the full superbuild from source.
- Either installing the required binary packages derived from the above listed dependencies, using the [miniforge conda distribution](https://github.com/conda-forge/miniforge) package manager, from the robotology channel.

## :floppy_disk: Installing the dependencies through the robotology superbuild source

- Clone and build the robotology-superbuild following the steps in https://github.com/robotology/robotology-superbuild/blob/master/README.md.
- `icub-models`, `YARP`: set the profile option `ROBOTOLOGY_ENABLE_CORE`.
- `iDynTree`, `WBToolbox`, `Whole-Body-Controller`, `qpOASES` and `yarp-matlab-bindings`: set the profile option `ROBOTOLOGY_ENABLE_DYNAMICS` and CMake option  `ROBOTOLOGY_USES_MATLAB`.

**Note 1:** In general, for selecting the profile CMake options according to the sub-projects to install, refer to the table in the section [Profile CMake Options](https://github.com/robotology/robotology-superbuild/blob/master/doc/profiles.md#profile-cmake-options).
**Note 2:** By setting the profile `ROBOTOLOGY_ENABLE_DYNAMICS`in the superbuild cmake options. The `qpOASES` library will then be installed along with `iDynTree` as an external dependency.
**Note 3:** Previously, two other optional QP solvers were available: OSQP and the native MATLAB solver `quadprog`. OSQP bindings and `quadprog` are not supported by the Simulink code generation build option in the `step_block` MATLAB System block. For this reason, the `step_block` MATLAB System block uses the qpOASES WBT block through a Simulink function call, and this choice is hardcoded in the class `Contacts`. As soon as the OSQP WBT block is created, it will replace the qpOASES Solver block.

## :floppy_disk: Installing the dependencies from the conda robotology channel

1. Install the conda miniforge distribution following https://github.com/robotology/robotology-superbuild/blob/master/doc/install-miniforge.md#linux. Remember to restart your shell session or run `source ~/.bashrc` (`~/.bash_profile` on MacOS) for the `conda init` command to take effect.
2. Install Mamba, create a new environment and install the robotology dependency binaries:
    ```
    $ conda install mamba
    $ conda create -n robotologyenv
    $ conda activate robotologyenv
    $ mamba install -c robotology yarp yarp-matlab-bindings idyntree qpoases icub-models wb-toolbox whole-body-controllers
    ```
    To read more about installing robotology-superbuild package binaries refer to https://github.com/robotology/robotology-superbuild/blob/master/doc/conda-forge.md#binary-installation.
3. Check the MATLABPATH environment variable. It should now have:
    ```
    <user-home-dir>/miniforge3/envs/robotologyenv/mex: <user-home-dir>/miniforge3/envs/robotologyenv/share/WBToolbox: <user-home-dir>/miniforge3/envs/robotologyenv/share/WBToolbox/images
    ```
    Mex libraries:
    ```
    $ ls <user-home-dir>/miniforge3/envs/robotologyenv/mex/

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
4. Clone the repository  `matlab-whole-body-simulators`
    ```
    $ git clone https://github.com/dic-iit/matlab-whole-body-simulator.git
    ```
5. Run matlab in the same conda environment.
6. Change working directory to the root path of repository `matlab-whole-body-simulators`.
7. Open and run the model `test_matlab_system_2020b.mdl`.

## :floppy_disk: Installing the framework in MATLAB Online environment

This use case is very convenient if a local host with installed MATLAB application and license is not available, or simply if the user wishes to leave his usual working environment unchanged by the package dependencies of this simulator framework.

With a MATLAB account, one can sign in and access to [MATLAB online](https://www.mathworks.com/products/matlab-online.html#connect-to-the-cloud), an online workspace that provides MATLAB and Simulink from any standard web browser. The GUI is practically identical to the one provided by the desktop application.

Once connected, the goal is to follow a procedure similar to the one from [the previous section](https://github.com/dic-iit/matlab-whole-body-simulator/tree/devel#floppy_disk-installing-the-dependencies-from-the-conda-robotology-channel) extensively using the [Robotology](https://anaconda.org/robotology) and the [conda-forge](https://anaconda.org/conda-forge) conda channels.

The steps will slightly differ from the ones in the referenced procedure because of the limitations of the bash provided by the MATLAB online session, as listed below:
- The session `~/.bashrc` cannot be sourced (or sourcing it won't have any effect), so the **PATH** environment variable shall be set directly.
- No **Git** tool is available by default, so it has to be installed through **mamba**.
- any command run on terminal in the original procedure shall be run through the `system` instruction (refer to https://github.com/robotology/robotology-superbuild/pull/652#issuecomment-794027294).
- Any prompt resulting from those commands is not accessible from MATLAB, so we have to use the automatic "yes" user input (command option `-y`).

The required commands have been sequenced in a MATLAB script, `app/tools/matlabOnlineInstaller.m`. Follow the below steps once logged in the MATLAB Online session.
1. Download and run the installer:
  ```
  system('curl -LO https://raw.githubusercontent.com/dic-iit/matlab-whole-body-simulator/featue/install-matlab-online/app/tools/matlabOnlineInstaller.m');
  run matlabOnlineInstaller.
  ```
2. After step 1, you find the `matlab-whole-body-simulator` repository has been downloaded to `/MATLAB Drive/dev/matlab-whole-body-simulator`, further designated as <MATLAB_WB_SIM_SRC>, and its dependencies installed in `$HOME/miniforge3`. Select the robot model among the available ones in `<MATLAB_WB_SIM_SRC>/app/robots` and as described in the next section. For instance, selecting `iCubGenova04` would be done as follows:
  ```
  setenv('YARP_ROBOT_NAME','iCubGenova04');
  ```
3. Open the test model `<MATLAB_WB_SIM_SRC>/test_matlab_system_2020b.mdl`, and run it. After the compilation is complete, a figure will appear, displaying the robot performing the task driven by the controller used in the test model.

## :runner: How to use the simulator

1. Connect your controller to the **RobotDynWithContacts** block. This block takes as imput the **joints torque**, **motor inertia** and an eventual **generalized external wrench**. It outputs the robot **state**, the contact wrenches **wrench_LFoot** and **wrench_RFoot**, respectively applied to the left and right foot (sole frames), and **kinDynOut**, an output bus exposing all the computed dynamics quantities relevant for debugging or the extension of dynamics computations in external blocks (emulation of an IMU sensor, of pressure sensors on the feet, etc).
2. Select the robot model by setting the environment variable `YARP_ROBOT_NAME` (e.g. `setenv('YARP_ROBOT_NAME','iCubGenova04')`). The available models are:
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
