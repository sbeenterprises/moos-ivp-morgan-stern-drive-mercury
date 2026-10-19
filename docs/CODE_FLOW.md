# Code flow

End-to-end data and control paths through the stack, traced from the source.
File references are relative to the repo root. For per-app detail see
[APPS.md](APPS.md); for problems found along the way see
[KNOWN_ISSUES.md](KNOWN_ISSUES.md).

## 1. Navigation: sensors → `NAV_*`

```
VectorNav (GPRMC, GPHDT, VNYMR) ─serial─▶ Signal K ─udp-nmea-sender UDP 10111─▶ iDivisorNMEA
```

1. **iDivisorNMEA** (`src/iDivisorNMEA/DivisorNMEA.cpp`) binds UDP 10111 and,
   each `Iterate`, drains up to 200 datagrams (one NMEA sentence each).
2. Per sentence (`processSentence`, ~L316–573):
   - `$GPRMC` → lat/lon (via `CNMEAParser`) and **`NAV_SPEED` = SOG in knots**, `NAV_COG`.
   - `$GPHDT` → `NAV_HEADING`.
   - `$SDDPT` → `NAV_DEPTH`; `$AGRSA` → `ANGULO_LEME`/`NAV_YAW`; `$WIMWV` → `WIND_*`; `$VNYMR` → `GYRO_*`.
   - `!AIVDM/!AIVDO` → AIS contact table (see §3).
   - With `publish_nav=true`, `NAV_LAT/LONG/SPEED/HEADING` are re-posted on **every** datagram from cached values.
3. iDivisorNMEA does **not** publish `NAV_X/NAV_Y`. Instead:
   - **pNodeReporter** (`cross_fill_policy=global`) computes X/Y from lat/lon → `NODE_REPORT_LOCAL`.
   - **pXYpublisher** (`src/pXYpublisher/XYpublisher.cpp`) parses `NODE_REPORT_LOCAL` and posts `NAV_X`, `NAV_Y` (4 Hz).
   - Result: `NAV_X/Y` lag `NAV_LAT/LONG` by ~0.5–0.75 s.

In **simulation** (`simulated.moos`) uSimMarineV22 owns `NAV_*` (m/s), and
**iNMEA** pushes `$GPRMC/$GPHDT` to Signal K UDP 10112 so the UI map sees the
simulated boat. In **external.moos**, `tools/aissim` produces the NMEA and the
real-boat path above is used unchanged.

## 2. Command & control: UI → helm → PID → CAN

```
UI ─ws:8000 "KEY=value"─▶ iWebSocket ─▶ mode flags + WPT_UPDATE ─▶ pHelmIvP (alpha.bhv)
   ─▶ DESIRED_HEADING / DESIRED_SPEED ─▶ planchaPID ─▶ DESIRED_RUDDER / DESIRED_THRUST
   ─▶ iCAN ─J1939 slcan0─▶ steering / gear / engine ECUs
```

### 2.1 iWebSocket (`src/iWebSocket`)
- `main.cpp` starts an IXWebSocket server on `0.0.0.0:8000`; incoming frames are
  queued (mutex, max 512) and dispatched on the MOOS thread in `Iterate` via
  `m_messageHandlers` (`WebSocket.cpp` ~L715–752).
- Example — **start a route** (`AUTONOMOUS_START=true#SPEED#lat,lon:...#REPEAT=..#ID_ROUTE=..`):
  posts `WAYPOINT=true`, `MOOS_MANUAL_OVERRIDE=false`, `WPT_UPDATE=speed=..`,
  `WPT_UPDATE=polygon=x,y:...` (lat/lon → local UTM against hardcoded origin),
  then `WPT_UPDATE=repeat=..`.
- Route corridor (`CORRIDOR=width=..#pts=lat,lon:...`): posts `CORRIDOR_UPDATE`
  (local x,y) then `CORRIDOR=true`; pCorridor turns NAV_X/Y into `XTE_NOW` and
  `CORRIDOR_STATUS`, which the `corridor_*` behaviours act on (see Helm modes).
- Full key list: [UI_PROTOCOL.md](UI_PROTOCOL.md).

### 2.2 pHelmIvP + behaviors (`missions/s1_morgan_city_signalk/alpha.bhv`)
- The helm leaves PARK when `MOOS_MANUAL_OVERRIDE=false`
  (`start_in_drive=false` on the boat, `true` in sim).
- `waypt_survey` is `BHV_Waypoint_mod` from `src/lib_behaviors-test`
  (condition `WAYPOINT=true`, updates `WPT_UPDATE`). It adds `xpoints=` (replace
  route mid-transit, keep current target), live capture/slip radius, and
  `WPT_PCT_COMPLETE` / `WPT_RADII` reporting.
- Helm output: `DESIRED_HEADING` (deg), `DESIRED_SPEED` (knots on the boat).

### 2.3 planchaPID (`src/planchaPID/lanchaPID.cpp`, `Iterate` ~L542–600)
Per tick (`dt = 1/AppTick`, 20 Hz on the boat):
1. `NAV_HEADING` / `NAV_SPEED` are EMA-filtered in `OnNewMail` (α 0.15 circular / 0.1).
2. **Heading loop** → `desired_rudder`: error wrapped ±180, derivative on
   measurement, integral clamped to bounds. Outside service mode the gains come
   from a **hardcoded speed schedule** (kp 0.28→0.05, kd 0.07→0.025, ki 0);
   `.moos` `HEADING_K*` only apply in `SERVICE_MODE`.
   If `CONSTANT_HEADING=true` the setpoint is `SETPOINT_HEADING`.
3. **Rudder limit by speed**: 20 at ≤5 kt tapering linearly to 8 at ≥18 kt.
4. **Speed loop** → `desired_thrust` (`CalculateSpeed`): piecewise feed-forward
   map (kt→%: 0→0, 3→8, 5→14, 7→22, 8→27, 10→35, 12→36, 15→37, 18→41, 20→44)
   plus scheduled PI with anti-windup (I authority ±2 %).
5. Thrust capped at **50 %**.
6. **Publish gate** (L578–583): `DESIRED_THRUST`/`DESIRED_RUDDER` are posted only if
   `(WAYPOINT && OVERRIDE=="false") || (TRAILING && OVERRIDE=="false") || STATIONING || REGION`.
7. Posts `PID_GAINS` (effective gains) for the UI.

### 2.4 iCAN (`src/iCAN/CAN.cpp`)
- Subscribes `DESIRED_RUDDER`, `DESIRED_THRUST`, `CAN_SEND`.
- Transmits only while `CAN_SEND == "true"` (set by the UI).
- Every `Iterate` (and also on each `DESIRED_RUDDER` mail):
  - steering `18F30A1A` — `(int)DESIRED_RUDDER` as **% of travel**, lookup table;
  - gear `18F005D0` (ETC2) — forward / neutral / reverse from the **sign** of thrust;
  - engine speed `18000017` (TSC1) — `(int)|thrust|` % → 700…4200 rpm table.
- Frame details: [CAN_INTERFACE.md](CAN_INTERFACE.md).
- **No timeout**: last command is re-sent forever until something posts 0.

### 2.5 Stopping
- Behavior endflags in `alpha.bhv` post `WAYPOINT=false`, `DESIRED_THRUST=0`,
  `DESIRED_RUDDER=0`; `AUTONOMOUS_STOP` from the UI does the same.
- planchaPID stops publishing once its gate is false; iCAN keeps sending the last
  value (0 → neutral, 700 rpm, centred).

### 2.6 Manual control
- UI posts `DESIRED_RUDDER` / `DESIRED_THRUST` directly → iCAN.
- `CONTROLE_MANUAL` posts the misspelled `MOOS_MANUAL_OVERIDE`, which nothing
  acts on — see [KNOWN_ISSUES.md](KNOWN_ISSUES.md#critical).

## 3. Contacts & collision avoidance

```
AIS ──Signal K──UDP10111──▶ iDivisorNMEA ──NODE_REPORT NAME=AIS<mmsi> (m/s)──┐
Radar ──UDP 8091 $RATTM──▶ iRadarTargets ──NODE_REPORT contato_N (knots!)────┤
                                     └─RADAR_TARGETS─▶ pSensorFusionTracker_ref (realvision)
Camera ─RTSP─▶ pVisionDetector_ref ─VISION_REPORT─┘        └─NODE_REPORT fused_N (knots)─┤
pContactSpawn (manual poke SPAWN_CONTACT) ──NODE_REPORT────────────────────────┤
pTrafficSim (sim only) ──NODE_REPORT AIS<mmsi> + !AIVDM→Signal K────────────────┤
                                                                                ▼
pContactMgrV20 ─(alert id=avdcol_, alert/cpa range)─▶ CONTACT_INFO ─▶ pHelmIvP spawns
BHV_AvdColregsV22 "avdcol_<name>" (condition WAYPOINT=true)
   └─COLAV_BHV_* flags + IVPHELM_LIFE_EVENT─▶ pColavEval (sim only) ─▶ COLAV_METRICS ─▶ iWebSocket ─▶ UI
```

- **iDivisorNMEA AIS**: merges multipart AIVDM per MMSI, drops ownship MMSI
  (`710400014` default), converts SOG kn→m/s, posts at most one `NODE_REPORT`
  per `report_interval`, drops contacts silent > `contact_stale_secs`.
- **iRadarTargets**: `range·sin/cos(bearing)` + ownship `NAV_X/Y`; posts a
  `NODE_REPORT` per sentence and a batched `RADAR_TARGETS` at 10 Hz.
- **pSensorFusionTracker_ref**: 0.5 s buffered batches, Mahalanobis gating,
  Hungarian assignment, constant-velocity KF on position; publishes **only
  tracks seen by both radar and vision**.
- **pContactMgrV20** (core): real `alert_range=1000, cpa_range=500`,
  sim `1500/1000` with `expunge_var`.
- **pColavEval**: per-encounter predicted CPA/TCPA (CPAEngine), achieved min
  range, verdict (collision ≤20 m / near miss ≤60 m / clear), reaction time,
  heading deflection.

## 4. Feedback & telemetry

```
Engine ECU ─J1939 EEC1/EEC2/ET1/EFL-P1/LFE/IC1/VEP1/DM1─▶ iCANRec ─ENGINE_*─▶ iWebSocket ─JSON 20 Hz─▶ UI
```
Engine data is display-only; neither planchaPID nor iCAN closes a loop on it.
The speed loop closes on GPS SOG, the heading loop on `NAV_HEADING`.

## Helm modes

There is no `set MODE` hierarchy; modes are independent boolean flags
initialised `false` in `alpha.bhv`:

| Flag | Behavior (pwt) | Set by (UI key) | planchaPID gate honours override? |
|---|---|---|---|
| `WAYPOINT` | `waypt_survey` BHV_Waypoint_mod (100) **and** `avdcol_` AvdColregsV22 (250) | `AUTONOMOUS_START` / `AUTONOMOUS_STOP` | yes |
| `STATIONING` | `station-keep` BHV_StationKeep (50 → 250 via UI) | `STATION_KEEP_WAYPOINT` | **no** |
| `REGION` | `opr24` BHV_OpRegionV24 (300) | `REGION` / `REGION_UPDATES` | **no** |
| `TRAILING` | `trail_contact` BHV_Trail (100) | `TRAIL_FOLLOW` | yes |
| `CORRIDOR` (+ `WAYPOINT`) | via pCorridor's `CORRIDOR_STATUS`: `corridor_slow` (200), `corridor_recover` (200), `corridor_stop` (240) | `CORRIDOR` | yes (only with `WAYPOINT=true`) |
| `RETURN`, `DEPLOY` | *(none in the signalk alpha.bhv)* | `AUTONOMOUS_RETURN_TO_BASE`, pMarineViewer buttons | – |

Collision avoidance is active **only while `WAYPOINT=true`** (not while
station-keeping, region-holding or trailing).

## 5. Startup sequence (real boat)

1. `slcand` brings up `slcan0` (README "Activate CAN"); Signal K server running
   with the udp-nmea-sender plugin pointed at UDP 10111.
2. `cd missions/s1_morgan_city_signalk && pAntler real.moos`
   (not `launch.sh` — it looks for a non-existent `alpha.moos`).
3. Helm starts parked; UI connects to `ws://<boat>:8000`, sets `CAN_SEND=true`,
   then `AUTONOMOUS_START=...` to drive.
