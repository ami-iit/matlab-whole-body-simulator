# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [3.1.0] - 2023-07-04

### Added
- Added the option to model the motion on an inclined surface (https://github.com/ami-iit/matlab-whole-body-simulator/pull/78).

### Fixed
- Fixed compatibility with Windows (https://github.com/ami-iit/matlab-whole-body-simulator/pull/88).

## [3.0.0] - 2022-04-14

### Added
- Added the possibility of simulating robots with cylindrical or spherical feet (https://github.com/ami-iit/matlab-whole-body-simulator/pull/71)
- Added a new simulator block called `RobotDynWithContacts_closedChain` that is capable of simulating a robot with multiple closed kinematic chains and of defining multiple links of the robot as the links interacting with the ground (https://github.com/ami-iit/matlab-whole-body-simulator/pull/64).
- Added the possibility of simulating the sliding contact model.

### Changed
- Added the option to load model with the absolute path, without using the YARP resource management at all (https://github.com/ami-iit/matlab-whole-body-simulator/pull/61).
- Added the possibility of interacting with the robot visualizer while the simulation is paused (https://github.com/ami-iit/matlab-whole-body-simulator/pull/63)

## [2.0.0] - 2021-05-12
### Added

- Added an alternative repository installation method, implementing fixes for allowing an easier installation of the dependencies from superbuild conda binaries (#20).
- Add double pendulum model to the simulation framework (#22).
- Add an installer script for running the simulator in a MATLAB online session (#26).
- Added another alternative installation method, Making a CMake build system out of this repository (#36), thus enabling its installation with the **robotology-superbuild**.
- Add visualizers (Robot Visualizer) and sensors libraries (IMUsensor) and integrate them in the test model (#41).

### Changed

- Make a library out of the "Robot" simulator sub-system (#3).
- Add bus output "KynDynOut" to the simulator block "Robot" interface for exporting the kinematic & dynamic variables (#6).
- Further refactored the interface and made it more generic with respect to the robot model (#8). This allowed to use the simulator with models other than iCub.
- Allowed the use of the standard iDynTree workflow for locating the mesh via the `ExternalMesh.getFileLocationOnLocalFileSystem` method (#22), dismissing its definition in the input parameters.
- Changed the simulator implementation and interface by adding the motors reflected inertia (#9).
- Add optional format alternative to motor reflected inertia (#44).
- Replaced the Matlab native QP (quadratic programming) solver `quadprog` by the open source solver qpOASES in the contacts computation (#10 and #15).
- Replaced the bindings by WBT blocks through Simulink function calls (#15). this allowed to improve significantly the execution speed of the simulator.
- Improved the documentation (#33).


## [1.0.0] - 2020-09-11
### Added
- This is the first release of the matlab-whole-body-simulators project. A changelog has just been introduced in this release.
- The project implements a simulator for humanoid robots, designed to work in Simulink. In the simulator the ground is assumed to be flat and the contact forces are computed using the Maximum dissipation principle and under the linear approximation of the friction cones assumption.
- For more details on how to use the simulator, check the [README](https://github.com/dic-iit/matlab-whole-body-simulators/blob/master/README.md).
