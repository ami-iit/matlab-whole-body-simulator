# matlab-whole-body-simulator

**matlab-whole-body-simulator** is a simulator for the humanoid robots. It has been designed to work in Simulink.
In the simulator the ground is assumed to be flat and the contact forces are computed using the **Maximum dissipation principle** and under the linear approximation of the friction cones assumption.

## :hammer: Dependencies

- [Matlab/Simulink 2019b](https://it.mathworks.com/products/matlab.html)
- [iDynTree](https://github.com/robotology/idyntree)
- [OSQP](https://github.com/oxfordcontrol/osqp.git)
- [osqp-matlab](https://github.com/oxfordcontrol/osqp-matlab) (the OSQP MATLAB bindings).
- [qpOASES](https://github.com/robotology-dependencies/qpOASES)
- [icub-models](https://github.com/robotology/icub-models) to access iCub models.

It is recommended to install `iDynTree`, `icub-models`, `OSQP` and `qpOASES` using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild):
- `icub-models`: set the profile option `ROBOTOLOGY_ENABLE_CORE`.
- `iDynTree` and `OSQP`: set the profile option `ROBOTOLOGY_ENABLE_DYNAMICS`.

## :floppy_disk: Installing the OSQP MATLAB bindings

- `OSQP` library:<br/>
    It is recommended to install the `OSQP` library through the [robotology-superbuild](https://github.com/robotology/robotology-superbuild), by setting the profile `ROBOTOLOGY_ENABLE_DYNAMICS`in the superbuild cmake options. The `OSQP` library will then be installed along with `iDynTree` as an external dependency.

- `OSQP` MATLAB bindings (https://osqp.org/docs/get_started/matlab.html):<br/>
    - Clone the repository [osqp-matlab](https://github.com/oxfordcontrol/osqp-matlab) (the OSQP MATLAB bindings) in the location of your choice.
    ```
    cd <some-path>
    !git clone --recurse-submodules https://github.com/oxfordcontrol/osqp-matlab
    cd osqp-matlab
    make_osqp
    ```
    - Set accordingly the environment variable `OSQP_MATLAB_PATH` in the bash profile (on Linux: /home/<user>/.bashrc, on MacOS: /home/\<user\>/.bash_profile):
    ```
    export OSQP_MATLAB_PATH=<some-path>/osqp-matlab
    ```
    - The option `Config.USE_OSQP` in the main [init](https://github.com/dic-iit/matlab-whole-body-simulators/blob/devel/init.m) is deprecated, as SWIG bindings, OSQP bindings and the native MATLAB solver `quadprog` are not supported by the Simulink code generation build option in the `step_block` MATLAB System block. For this reason, the `step_block` MATLAB System block uses the qpOASES WBT block through a Simulink function call, and this choice is hardcoded in the class `Contacts`. As soon as the OSQP WBT block is created, it will replace the qpOASES Solver block.
    
    For your information, you can find further information on the OSQP library setup and use of the MATLAB interface (bindings) in https://osqp.org/docs:
    
    https://osqp.org/docs/index.html<br/>
    https://osqp.org/docs/get_started/matlab.html<br/>
    https://osqp.org/docs/interfaces/matlab.html#matlab-interface

## :runner: How to use the simulator

Just connect your controller to the robot block. This block takes as imput the **joints torque** and the (possible) **generalized external forces** and outputs the **robot state** and **contact wrenches** in the left and right sole frames (in order to simulate a sensor on the feet).

![image](https://user-images.githubusercontent.com/29798643/92244565-617f7d80-eec3-11ea-95d0-2a15f1bdb54f.png)

To run and example open and launch `test_matlab_system.mdl`. It also contains a callback to the `init` file that retrieves the needed information.
