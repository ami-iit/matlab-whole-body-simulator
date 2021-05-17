# Getting the `whole-body-controllers` repository

## Local and Online One Line Installation
With the [local one line installation](../README.md#one-line-installation) and [one line installation in MATLAB Online](../README.md#cloud-one-line-installation-in-matlab-online-session), the script `install_robotology_packages` installs all the Simulink/Matlab packages available in the [robotology-superbuild](https://github.com/robotology/robotology-superbuild) resources. This includes the package `matlab-whole-body-simulator` and all its dependencies, and also the package `whole-body-controllers`, which includes the Simulink model example mentioned above.

## Binary Installation Using Conda
If you followed the guide [Binary Installation from the Conda Robotology Channel](../README.md#binary-installation-from-the-conda-robotology-channel), install the repository `whole-body-controllers` as follows:
```
mamba install -c robotology whole-body-controllers
```
An additional library folder, `+wbc`, appears in `<user-home-dir>/miniforge3/envs/robotologyenv/mex`, and the MATLABPATH environment variable was appended with...
```
<user-home-dir>/miniforge3/envs/robotologyenv/mex/+wbc/simulink: <user-home-dir>/miniforge3/envs/robotologyenv/mex/+wbc/examples:
```

## Source Installation using the full Superbuild Installation from Source
The `whole-body-controllers` project was installed along.

## Source Installation using the Dependencies from the Conda Robotology Channel
Refer to section [Binary Installation Using Conda](#binary-installation-using-conda).
