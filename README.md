# How to make it work

SIC! This tutorial only for Ubuntu 16.04, assuming that you have install Gazebo simulator.

Firtly install required packages.

```bash
$ sudo apt install libprotobuf-dev protobuf-compiler
```
After download source files with git.

```bash
$ git clone https://github.com/olzhas/StaubliKinematicModelPlugin.git
```

After all the source code was downloaded, you must enter its directory and create a **"build"** directory inside.

```bash
$ cd StaubliKinematicModelPlugin
$ mkdir build
$ cd build
```

Run CMake to create appropriate building scripts for your system.

```bash
cmake ..
```

The last step is to build the sources. Do this typing the following into your terminal.

```bash
$ make
```

To test the plugin update the plugin search directory list typing the following into your terminal.

```bash
$ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/StaubliKinematicModelPlugin/build
```
