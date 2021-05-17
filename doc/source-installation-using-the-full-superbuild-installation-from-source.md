# Source Installation using the full Superbuild Installation from Source

1. Clone and build the robotology-superbuild following the steps in https://github.com/robotology/robotology-superbuild/blob/master/README.md.
2. For cloning `icub-models`, `YARP` (dependency of `yarp-matlab-bindings`): set the profile option `ROBOTOLOGY_ENABLE_CORE`.
3. For cloning `iDynTree`, `WBToolbox`, `qpOASES`, `yarp-matlab-bindings` and `matlab-whole-body-simulator`(*): set the profile option `ROBOTOLOGY_ENABLE_DYNAMICS` and CMake option  `ROBOTOLOGY_USES_MATLAB`. The repository `matlab-whole-body-simulator` is cloned in `<ROBOTOLOGY_SUPERBUILD_SOURCE_DIR>/src/matlab-whole-body-simulator`.
4. Open MATLAB and Add the path `<ROBOTOLOGY_SUPERBUILD_SOURCE_DIR>/src/matlab-whole-body-simulator/lib` to the top of the MATLAB path. This allows MATLAB to find the source library instead of the installed one.

**Note 1:** In general, for selecting the profile CMake options according to the sub-projects to install, refer to the table in the section [Profile CMake Options](https://github.com/robotology/robotology-superbuild/blob/master/doc/profiles.md#profile-cmake-options).
**Note 2:** By setting the profile `ROBOTOLOGY_ENABLE_DYNAMICS`in the superbuild cmake options, the `qpOASES` library will then be installed along with `iDynTree` as an external dependency.
**Note 3:** Previously, two other optional QP solvers were available: OSQP and the native MATLAB solver `quadprog`. OSQP bindings and `quadprog` are not supported by the Simulink code generation build option in the `step_block` MATLAB System block. For this reason, the `step_block` MATLAB System block uses the qpOASES WBT block through a Simulink function call, and this choice is hardcoded in the class `Contacts`. As soon as the OSQP WBT block is created, it will replace the qpOASES Solver block.
