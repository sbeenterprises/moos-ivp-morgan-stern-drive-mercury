# UI ↔ boat protocol (iWebSocket)

Source: `src/iWebSocket/main.cpp`, `src/iWebSocket/WebSocket.cpp`.

- Server: `ws://<boat>:8000` (`0.0.0.0`, hardcoded). No TLS, no auth, no origin check.
- Server ping every 1 s; connections idle > 60 s are dropped from the broadcast list
  (but not closed).
- All lat/lon from the UI are converted to local x/y with `LatLong2LocalUTM`
  against the mission's global `LatOrigin` / `LongOrigin` (read once at start-up).
  If the mission has no origin, route, region, return, station-keep and
  `PUT_TARGET` commands are refused with a run warning.

## Boat → UI

### Telemetry (JSON, every `Iterate` = 20 Hz on the boat, broadcast to all clients)

```json
{
  "IVPHELM_STATE": "str", "IVPHELM_ALLSTOP": "str",
  "WAYPOINT": "str", "WPT_INDEX": 0, "WPT_STAT": "str", "WPT_PCT_COMPLETE": 0, "WPT_RADII": "str",
  "VIEW_POINT": "str", "ID_ROUTE": "str", "REPEAT_MISSION": false, "BASE_COORDS": "str",
  "DESIRED_SPEED": 0, "DESIRED_RUDDER": 0, "DESIRED_THRUST": 0,
  "CAN_SEND": false, "REGION": false, "TRAILING": false, "TRAIL_UPDATES": "str",
  "SERVICE_MODE": false, "PID_GAINS": "service_mode=..,heading_kp=..,...",
  "ENGINE_ACCELERATOR_PEDAL_POSITION": 0, "ENGINE_PERCENT_LOAD": 0, "ENGINE_SPEED": 0,
  "ENGINE_PERCENT_TORQUE": 0, "ENGINE_ACTIVE_DTCS": 0, "ENGINE_COOLANT_TEMP": 0,
  "ENGINE_OIL_TEMP": 0, "ENGINE_OIL_PRESSURE": 0, "ENGINE_FUEL_RATE": 0,
  "ENGINE_INTAKE_MANIFOLD_PRESSURE": 0, "ENGINE_BATTERY_VOLTAGE": 0,
  "COLAV_TEST": false, "COLAV_TEST_ACK": false, "COLAV_METRICS": "[...json array...]",
  "TRAFFIC_SPAWNED": "str", "TRAFFIC_TARGETS": "str", "COLAV_DIST": "str",
  "CORRIDOR": false, "CORRIDOR_STATUS": "none", "XTE_NOW": 0,
  "CONTACT_INFO": { "NAME": "str", "LAT": 0, "LON": 0, "SPD": 0, "HDG": 0 }
}
```

Notes:
- Ownship position/heading are **not** in this message (the UI presumably gets them from Signal K).
- `CONTACT_INFO` holds only the **last** `NODE_REPORT` received before the tick.
- `ID_ROUTE` / `REPEAT_MISSION` echo what the UI last sent.
- `ENGINE_OIL_TEMP` is always 0 (registered as `OIL_TEMP`, handled as `ENGINE_OIL_TEMP`).
- Route corridor: `CORRIDOR` (bool, corridor armed), `CORRIDOR_STATUS`
  (`none` | `in` | `slow` | `recovering` | `stop`, from pCorridor; `none` unless a corridor
  is armed **and** a waypoint mission runs), `XTE_NOW` (metres from the centreline,
  absolute; only meaningful while the status is not `none`, reset to 0 when it goes `none`),
  `CORRIDOR_TURN` (bool, pCorridor: the boat is near a sharp corridor corner and slows for it).

### Heartbeat
Plain text (not JSON) `HEARTBEAT=<MOOSTime>` every 30 s — clients must tolerate it.
The server also sends a WebSocket ping every second; browsers answer it automatically, and each pong keeps `UI_LINK_OK` true even while the UI is idle or in the background.

The server sends a WebSocket ping every second; browsers answer it automatically and that keeps `UI_LINK_OK=true`. If nothing is heard from any client for `link_timeout` (3 s), the link is lost and, in manual control, iCAN sets rudder and throttle to 0.

Telemetry also carries iCAN state: `CAN_STATUS`, `CAN_CONTROL_OWNER`, `CAN_LOST_LINK`, `CAN_CONFLICT`, `CAN_TX_RUDDER_PCT`, `CAN_TX_THRUST_PCT`, `CAN_TX_GEAR`, `CAN_RUDDER_MISMATCH`, `CAN_PROFILE`, and on USV-722 `CAN_RUDDER_ACTUAL_PCT`, `CAN_HELM_HAS_STEERING`, `CAN_ENGINE_GEAR`, `CAN_ENGINE_THROTTLE_PCT`, `CAN_FUEL_PCT` (present once first posted).

## UI → boat

Text frames `KEY=value`, split at the first `=`. Sub-fields use `#`, point
lists use `:`, coordinates are `lat,lon`. Unknown keys are ignored.

| Key | Value | MOOS postings |
|---|---|---|
| `AUTONOMOUS_START` | `true[#SPEED[#lat,lon:...[#REPEAT=true\|false][#ID_ROUTE=id]]]` | `WAYPOINT=true`, `MOOS_MANUAL_OVERRIDE=false`, `REPEAT_MISSION`, `ID_ROUTE`, `WPT_UPDATE=speed=`, `WPT_UPDATE=polygon=x,y:...`, then `WPT_UPDATE=repeat=1000\|0` |
| `AUTONOMOUS_WAYPOINTS` | `lat,lon:lat,lon:...` | `WPT_UPDATE=xpoints=...` (live route edit) |
| `AUTONOMOUS_STOP` | `true` | `WAYPOINT=false`, `STATIONING=false`, `DESIRED_RUDDER=0`, `DESIRED_THRUST=0`, then `CORRIDOR=false` |
| `AUTONOMOUS_RETURN_TO_BASE` | `true\|false` | `RETURN` (no behavior consumes it in current alpha.bhv) |
| `RETURN_UPDATE` | `<x>=lat,lon` | `RETURN_UPDATE=points=x,y` (legacy bhv only) |
| `BASE_COORDS` | string | `BASE_COORDS` (stored + echoed) |
| `DESIRED_SPEED` | number | `WPT_UPDATE=speed=<v>` |
| `REPEAT` | integer | `WPT_UPDATE=repeat=<n>` |
| `STATION_KEEP_WAYPOINT` | `TRUE#INNER_RADIUS=..#OUTER_RADIUS=..#COORDS=lat,lon` / `FALSE` | `STATIONING`, `STATION_UPDATES` (radii, `station_pt`, `pwt`), `WPT_UPDATE=pwt=` |
| `REGION` | `true\|false[#REGION_UPDATES=core_poly=pts={lat,lon:...}]` | `REGION`, `REGION_UPDATES` |
| `REGION_UPDATES` | `...pts={lat,lon:...}...` | `REGION_UPDATES` (x,y) |
| `TRAIL_FOLLOW` | `mmsi=..#range=..#angle=..` / `off` | `WAYPOINT=false`, `STATIONING=false`, `MOOS_MANUAL_OVERRIDE=false`, `TRAIL_UPDATES=contact=AIS<mmsi>...`, `TRAILING=true` / off: `TRAILING=false`, rudder/thrust 0 |
| `CONTROLE_MANUAL` | `true\|false` | `MOOS_MANUAL_OVERRIDE` (SS UI becomes the drive owner; manual rudder/thrust latch until changed), on false also rudder/thrust 0; plus `DEPLOY=1.0` (double) from `main.cpp` |
| `AUTONOMOUS_CONTROL` | any | `MOOS_MANUAL_OVERIDE=true` (typo) |
| `DESIRED_HEADING` / `DESIRED_RUDDER` / `DESIRED_THRUST` | number | same name |
| `DESIRED_GEAR` | `1\|0\|-1` | `DESIRED_GEAR=A\|N\|R`. Used only with iCAN `manual_gear = command`: in manual control iCAN engages this gear; `DESIRED_THRUST` is then throttle only (0 = idle in gear). A gear change resets thrust to 0. Ignored in autonomy, and ignored by `real.moos`, which uses `manual_gear = thrust_sign` (one lever: `DESIRED_THRUST` -100..+100 sets gear and throttle, 0 = neutral) |
| `CAN_SEND` | `true\|false` | `CAN_SEND` (iCAN transmit enable) |
| `STOP_CAN`, `CONSTANT_HEADING` | `true\|false` | same name |
| `SETPOINT_HEADING` | number | same name |
| `SERVICE_MODE` | `true\|false` | `SERVICE_MODE` (planchaPID uses UI gains) |
| `HEADING_KP/KI/KD`, `SPEED_KP/KI/KD` | number | same name |
| `TRIM` | `1\|-1\|0` | `TRIM`: 1 = one trim-up pulse, -1 = one trim-down pulse, 0 = stop. Pulse length and bytes come from the boat profile; ignored unless the profile has the `trim` capability and `CAN_SEND=true`. State comes back as `CAN_TRIM_STATE` |
| `COLAV_TEST` | `true\|false` | `COLAV_TEST`; false also `TRAFFIC_CLEAR=all` |
| `PUT_TARGET` | `lat=..#lon=..#course=at_usv\|deg#speed=kn#type=..#length=..#beam=..` | `TRAFFIC_SPAWN=...` (pTrafficSim) |
| `CLEAR_TARGETS` | `all` / `name=AIS...` | `TRAFFIC_CLEAR` |
| `COLAV_DIST` | `pwt_outer_dist=..#pwt_inner_dist=..#completed_dist=..#max_util_cpa_dist=..#min_util_cpa_dist=..` | `CONTACT_INFO=...`, `BCM_ALERT_REQUEST=id=avdcol_,...`, `COLAV_DIST` |
| `LOOP_START` | integer >= 0 | `WPT_UPDATE=loop_start=<n>`: a repeated route restarts at point n after its last point. The UI sends it before `AUTONOMOUS_START`: points 0..n-1 are a planned approach from the boat's position (driven once), 0 = restart at the first point (stock). Non-integers are ignored with a run warning |
| `CORRIDOR` | `width=<m>#pts=<lat>,<lon>:<lat>,<lon>:...` / `off` | `CORRIDOR_UPDATE=width=<w>,pts={x1,y1:x2,y2:...}` (local x,y) then `CORRIDOR=true` / off: `CORRIDOR=false`. `width` is the full width (both sides), > 0; at least 2 points. Any malformed field, a non-number or an out-of-range lat/lon rejects the whole command (run warning, nothing posted). Needs the mission origin. The corridor acts only while `WAYPOINT=true`; it is cleared by `AUTONOMOUS_STOP` and when the route completes |
| `HEARTBEAT_PING` | any | keep-alive only |

## Testing without the UI

```bash
python3 scripts/ws_client.py        # prints telemetry for 20 s
# or e.g. with websocat:
echo 'AUTONOMOUS_START=true#3#29.72,-91.14:29.73,-91.13' | websocat ws://127.0.0.1:8000
```
