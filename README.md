# matlab-whole-body-simulator

**matlab-whole-body-simulator** is a simulator for the humanoid robots. It has been designed to work in Simulink.
In the simulator the ground is assumed to be flat and the contact forces are computed using the **Maximum dissipation principle** and under the linear approximation of the friction cones assumption.

## :hammer: Dependencies

- [Matlab/Simulink](https://it.mathworks.com/products/matlab.html)
- [iDynTree](https://github.com/robotology/idyntree)
- [icub-models](https://github.com/robotology/icub-models) to access iCub models.

It is to suggest to install `iDynTree`, `icub-models` using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild).

## :runner: How to use the simulator

Just connect your controller to the robot block. This block takes as imput the **joints torque** and the (possible) **generalized external forces** and outputs the **robot state** and **contact wrenches** in the sole frames (in order to simulate a sensor on the feet).
