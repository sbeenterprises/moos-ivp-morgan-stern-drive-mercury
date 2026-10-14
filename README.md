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

All the backend is based in the MOOS-IvP structure that you can lookup in MIT documentation (https://oceanai.mit.edu/moos-ivp/pmwiki/pmwiki.php?n=Main.HomePage).

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

# Comms-Loss Return to Base

The Swiftships-UI runs ashore and reaches the boat through `iWebSocket`. If nothing at all is heard from any UI - no
command, heartbeat or ping reply - for `comms_loss_timeout` seconds, `iWebSocket` sends the boat back to its base:

1. The mission, station keep and follow are stood down and the helm takes control (`RETURN=true`).
2. `comms_return` in `alpha.bhv` takes the boat to the base, with collision avoidance (`avdcol_`) running.
3. On arrival `comms_hold` keeps station on the base (`RETURN_HOLD=true`).
4. When the link comes back the boat stays in the return or the hold. The operator's next command from the UI (a
   mission, Stop, station keep, follow, or manual rudder/thrust) ends it.

It does so whenever pHelmIvP is in DRIVE, NothingToDo included, or the boat is underway: a mission, station keep or
follow running, thrust commanded, or moving faster than `comms_loss_underway_speed` (drifting counts). With the helm
in PARK and the boat idle - the boat at the dock after power-up, as `real.moos` starts the helm parked - it stays
where it is, unless `comms_loss_always = true`. A boat in DRIVE at the dock leaves for the base too.

The base starts at `base_lat`/`base_lon` (29.72282, -91.21539) and is moved from the UI (right click on the chart >
Define base). `iWebSocket` converts it to the local grid, updates both behaviours (`RETURN_UPDATE`,
`RETURN_HOLD_UPDATE`) and saves it to `base_file`, so a restart keeps it; delete the file to go back to the default.
`AUTONOMOUS_RETURN_TO_BASE=true` from the UI runs the same return on demand. pMarineViewer shows the base as a red diamond labelled
"BASE (comms loss)" (`VIEW_MARKER`), moved whenever the base changes.

The return is a straight line to the base. Collision avoidance deals with traffic, not with the banks, so the base
has to be reachable in a straight line from where the boat operates.

| iWebSocket parameter        | Default              | Description |
|:--------------------------- |:-------------------- |:----------- |
| port                        | 8000                 | Websocket port the UI connects to |
| comms_loss_return           | true                 | Return to base on a comms loss (false: only report the loss) |
| comms_loss_timeout          | 30                   | Seconds without a frame from any UI (minimum 10) |
| comms_loss_speed            | 5                    | Return speed, in the units of the UI's speed slider (knots) |
| comms_loss_always           | false                | Also return with the helm in PARK and the boat idle |
| comms_loss_underway_speed   | 1.0                  | NAV_SPEED above which the boat counts as underway |
| base_lat / base_lon         | 29.72282 / -91.21539 | Default base |
| base_file                   | (none)               | Keeps the base set from the UI across restarts |

The appcast of `iWebSocket` (uMAC) shows the link, the last loss and what was done about it, and the base in use.

# Collision Alerts

Every contact pHelmIvP is steering around is raised in the Alert List of the Swiftships-UI, with its name, heading,
speed, range, bearing, CPA, TCPA and COLREGS situation. `iWebSocket` builds the list (`COLAV_ALERTS`) from the helm
itself: an `avdcol_` instance in `IVPHELM_BHV_ACTIVE` is one whose avoidance is part of the heading and speed the boat
is being given. A contact the avoidance is only watching - in the alert range but outside its envelope, or held off
by the AIS limit - is not an alert. The list is only believed while the helm is in DRIVE.

The figures come from each contact's last `NODE_REPORT` and the boat's `NAV_*`, with the `CPAEngine` the behaviour
uses. The COLREGS mode comes from the behaviour's `COLAV_BHV_TRACE` flag in `alpha.bhv`, which runs one helm
iteration behind, so a new alert waits for the encounter to be classified (2 s at most). An alert stays up, marked
ended, for `colav_alert_hold` seconds after the avoidance ends, so a behaviour dropping in and out of active does not
raise it again. Stand-on in extremis (Rule 17(b)) is raised as an alarm, everything else as a warning.

This works in every mission `iWebSocket` runs in. `pColavEval`, which feeds the UI's COLREGS box and test metrics,
only runs in `simulated.moos`.

| iWebSocket parameter        | Default              | Description |
|:--------------------------- |:-------------------- |:----------- |
| nav_speed_units             | knots                | Units of NAV_SPEED: `knots` (GPS, on the boat) or `mps` (uSimMarineV22). The CPA is worked out in m/s, the units of the contacts |
| colav_alert_hold            | 15                   | Seconds an alert stays up after the avoidance ends |

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
# Reset VectorNav INS/GNSS
PORT=/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_AU04Q2FP-if00-port0
sudo stty -F $PORT 115200 raw -echo
printf '$VNRST*4D\r\n' | sudo tee $PORT > /dev/null

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

