# USV
# moos-ivp-extend

|              |                        |
|:------------ |:---------------------- |
| FILE:        | moos-ivp-extend/README |
| DATE:        | 2014/01/02             |
| DESCRIPTION: | Contains important information regarding the moos-ivp-extend repository. |


# Introduction

The moos-ivp-extend repository contains examples for extending the MOOS-IvP
Autonomy system. This includes a MOOS application and an IvP behavior.


# Directory Structure

The directory structure for the moos-ivp-extend is described below:

| Directory        | Description                                 |
|:---------------- |:------------------------------------------- |
| bin              | Directory for generated executable files    |
| build            | Directory for build object files            |
| build.sh         | Script for building moos-ivp-extend         |
| CMakeLists.txt   | CMake configuration file for the project    |
| data             | Directory for storing data                  |
| lib              | Directory for generated library files       |
| missions         | Directory for mission files                 |
| README           | Contains helpful information - (this file). |
| scripts          | Directory for script files                  |
| src              | Directory for source code                   |


# Useful Commands
```bash
#Activate CAN
'''
sudo slcand -o -c -s5 /dev/ttyACM2 slcan0
sudo ip link set slcan0 up
'''

printf '$VNRST*4D\r\n' | sudo tee /dev/ttyUSB1
```

# Build Instructions

#Before Building

Run:
```bash
   sudo apt install libboost-all-dev
   sudo apt-get install libcurl4-openssl-dev
   sudo apt-get install libjsoncpp-dev
   sudo apt install libssl-dev

   git clone https://github.com/machinezone/IXWebSocket
    
   mkdir build # make a build dir so that you can build out of tree.
   cd build
   cmake -DUSE_TLS=1 ..
   make -j
   make install
   

```

# Install the IXWebSocket
https://github.com/machinezone/IXWebSocket

## Linux and Mac Users

To build on Linux and Apple platforms, execute the build script within this
directory:

```bash
   $ ./build.sh
```

To build without using the supplied script, execute the following commands
within this directory:

```bash
   $ mkdir -p build
   $ cd build
   $ cmake ../
   $ make
   $ cd ..
```


## Windows Users

To build on Windows platform, open CMake using your favorite shortcut. Then
set the source directory to be this directory and set the build directory
to the "build" directory inside this directory.

The source directory is typically next to the question:
   "Where is the source code?"

The build directory is typically next to the question:
   "Where to build the binaries?"

Alternatively, CMake can be invoked via the command line. However, you must
specify your generator. Use "cmake --help" for a list of generators and
additional help.


# Environment variables

The moos-ivp-extend binaries files should be added to your path to allow them
to be launched from pAntler.

In order for generated IvP Behaviors to be recognized by the IvP Helm, you
should add the library directory to the "IVP_BEHAVIOR_DIRS" environment
variable.

# END of README

