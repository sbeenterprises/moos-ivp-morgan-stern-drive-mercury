# Documentation of the backend code of the SwiftShips USV
# moos-ivp-morgan-stern-drive-mercury

|              |                        |
|:------------ |:---------------------- |
| FILE:        | moos-ivp-morgan-stern-drive-mercury/README |
| DATE:        | 2026            |
| DESCRIPTION: | Contains important information regarding the moos-ivp-morgan-stern-drive-mercury repository. |


# Introduction

The moos-ivp-morgan-stern-drive-mercury repository contains the backend code used in the USV produced by SwiftShips, all based in the MOOS-IvP
Autonomy system. 

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

Inside the src folder (That is one of the most important folder of the directory) you have the following:

| Directory        | Description                                 |
|:---------------- |:------------------------------------------- |
| iCAN              | App responsible for sending CAN messages through the CAN BUS    |
| iCANREC            | App that receives data from CAN bus and decode the messages coming from the engine            |
| IDivisorNMEA   | Parsing of NMEA messages    |
| iNMEA             | Send NMEA messages to signalk when running in simulated mode                  |
| iRadarTargets              | MOOS application to receive radar target data via UDP from NMEA $RATTM (Tracked Target Message) sentences and publish them as NODE_REPORT messages to the MOOSDB.       |
| iWebSocket          | App that makes the connection with the frontend                  |
| pColavEval              | App to determine metrics for collision avoidance with USVs                   |
| pContactSpawn            | Directory for build object files            |
| pSensorFusionTracker         | App used for fusion of contacts coming from camera, radar and ais         |
| pTrafficSim             | Simulator for AIS contacts                  |
| pVisionDetector              | Detects targets through the camera and feeds the pHelmIvP       |
| planchaPID              | PID Controller of the vehicle                   |

# Useful Commands
```bash
#Activate CAN
'''
sudo slcand -o -c -s5 /dev/ttyACM2 slcan0
sudo ip link set slcan0 up
'''

# Restart satelital antenna
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



# END of README

