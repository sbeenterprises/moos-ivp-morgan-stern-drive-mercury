# iRadarTargets

MOOS application to receive radar target data via UDP from NMEA $RATTM (Tracked Target Message) sentences and publish them as NODE_REPORT messages to the MOOSDB.

## Overview

This application listens for NMEA $RATTM sentences on a UDP port (default: 8091) and converts radar target data into MOOS NODE_REPORT format. It uses the vessel's own position (NAV_LAT, NAV_LON) to convert range/bearing measurements into latitude/longitude coordinates.

## NMEA $RATTM Format

The application expects NMEA TTM (Tracked Target Message) sentences with the following structure:

```
$RATTM,26,5.76,133.8,T,22.87,232.7,R,5.7,2.3,N,,,T,,,M*22
```

| Field | Description |
|-------|-------------|
| RA    | Sender ID (radar) |
| TTM   | Sentence ID (Tracked Target Message) |
| 26    | Target number (00-99) |
| 5.76  | Target distance from own ship (NM) |
| 133.8 | Target bearing from own ship (degrees) |
| T     | Bearing reference (T=true, R=relative) |
| 22.87 | Target speed (knots) |
| 232.7 | Target course (degrees) |
| R     | Course reference (T=true, R=relative) |
| 5.7   | Distance at CPA (NM) |
| 2.3   | Time to CPA (minutes) |
| N     | Units (N=nautical miles and knots) |
| ,,    | Target label (blank) |
| T     | Target status (T=tracking, L=lost, Q=acquired) |
| ,,    | Reference target (blank) |
| ,,    | UTC time (blank) |
| M     | Acquisition type (M=manual, A=automatic) |
| *22   | Checksum |

## Output Format

The application publishes NODE_REPORT messages to MOOSDB with this format:

```
NODE_REPORT = "NAME=contato_26,TYPE=SHIP,TIME=1234567890.123,LAT=41.12345,LON=-70.98765,SPD=22.87,HDG=232.7,LENGTH=3.8,MODE=DRIVE,X=-70.98765,Y=41.12345"
```

## Configuration

Add the following to your .moos file:

```
ProcessConfig = iRadarTargets
{
  AppTick   = 10
  CommsTick = 10

  // UDP Configuration
  udp_port = 8091       // Port to receive NMEA $RATTM sentences
  udp_host = 127.0.0.1  // Host interface (localhost by default)
}
```

## Requirements

The application requires the following MOOS variables:
- **NAV_X**: Own vessel X position in local coordinates (meters)
- **NAV_Y**: Own vessel Y position in local coordinates (meters)

These are automatically subscribed to and used for calculating target positions.

## Testing

To test the application, you can send UDP packets with NMEA $RATTM sentences:

```bash
echo '$RATTM,26,5.76,133.8,T,22.87,232.7,T,5.7,2.3,N,,,T,,,M*22' | nc -u -w1 127.0.0.1 8091
```

## How It Works

1. **Receives $RATTM sentence** via UDP containing:
   - Target number
   - Range (nautical miles)
   - Bearing (degrees from North, true)
   - Speed (knots)
   - Course (degrees, true)

2. **Calculates target position** in local coordinates:
   - Converts range from NM to meters (1 NM = 1852 m)
   - Converts bearing to radians
   - Calculates offset: `delta_x = range * sin(bearing)`, `delta_y = range * cos(bearing)`
   - Adds offset to own ship position: `target_x = NAV_X + delta_x`, `target_y = NAV_Y + delta_y`

3. **Publishes NODE_REPORT** with target's X,Y position to MOOSDB

## Notes

- The application uses non-blocking UDP sockets
- Relative bearings and courses (R) are not fully supported yet - they are treated as true bearings/courses
- Target positions are calculated directly in local coordinates using polar to cartesian conversion
- Contacts will disappear after 35 seconds without updates (controlled by pContactMgrV20's `contact_max_age` parameter)
