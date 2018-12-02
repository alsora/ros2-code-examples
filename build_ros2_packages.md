# ROS2 build workspace

The workspace is the folder where you can build and install ROS2 packages.

## Creating a workspace

A ROS2 workspace, in order to be built using the native **Colcon** build system must have the following structure.


```
.
└── src
```
The first step is the creation of such workspace.

    $ mkdir -p  my_workspace/src

Place your source code inside the `src` directory.

For example, to create a symbolic link for the sources provided in this repository

    $ cd my_workspace/src
    $ ln -s ../../src/interfaces .
    $ cd ..

## Build and install your code

Build the sources

    $ colcon build

After you have built some sources, your workspace will look like this

```
.
├── build
├── install
├── log
└── src
```

 - The `build` directory will be where intermediate files are stored. For each package a subfolder will be created in which e.g. CMake is being invoked.

 - The `install` directory is where each package will be installed to. By default each package will be installed into a separate subdirectory.

 - The `log` directory contains various logging information about each colcon invocation.


To use the executables and libraries with the command line interface you need to source the install directory.

    $ source install/setup.sh

## Tips

 - If you do not want to build a specific package place an empty file named `COLCON_IGNORE` in the directory and it will not be indexed.

        $ touch src/ros2/rviz/COLCON_IGNORE

 - Colcon by default builds packages in parallel. This could cause unpredictable errors. To build sequentially add `--executor sequential` as parameter to `colcon build`.
 - Colcon by default installs each package in a separate subdirectory. To install all packages into a combined directory add `--merge-install` as parameter to `colcon build`.

 - If you want to avoid configuring and building tests in CMake packages add `--cmake-args -DBUILD_TESTING=0` as parameter to `colcon build`.


