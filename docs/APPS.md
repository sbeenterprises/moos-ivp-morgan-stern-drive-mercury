# App reference

One section per app in `src/` and `tools/`. "Subs" = MOOS variables
subscribed, "Pubs" = published. Line numbers are approximate pointers into
the current `main`.

| App | Built | Used by missions | Role |
|---|---|---|---|
| [iDivisorNMEA](#idivisornmea) | ✔ | real, external, realvision, alpha_fusion | NMEA/AIS in (UDP 10111) |
| [pXYpublisher](#pxypublisher) | ✔ | real, external, realvision | `NAV_X/Y` from NODE_REPORT_LOCAL |
| [planchaPID](#planchapid) | ✔ | all | heading/speed controller |
| [iCAN](#ican) | ✔ | real, realvision, alpha_fusion | J1939 commands out |
| [iCANRec](#icanrec) | ✔ | real, simulated, realvision, alpha_fusion | engine telemetry in |
| [iWebSocket](#iwebsocket) | ✔ | all | operator UI bridge |
| [iRadarTargets](#iradartargets) | ✔ | real, simulated, external, realvision | radar $RATTM in |
| [pContactSpawn](#pcontactspawn) | ✔ | real, simulated, realvision | single synthetic contact |
| [pTrafficSim](#ptrafficsim) | ✔ | simulated | simulated AIS traffic |
| [pColavEval](#pcolaveval) | ✔ | simulated | COLAV metrics |
| [pCorridor](#pcorridor) | ✔ | real, real_usv722/725, simulated, external | route corridor (XTE + zone) |
| [pSensorFusionTracker_ref](#psensorfusiontracker_ref) | ✔ | realvision | radar+vision fusion |
| [iNMEA](#inmea) | ✔ | simulated | NAV → NMEA to Signal K |
| [iSignalK](#isignalk) | ✔ | legacy only | Signal K course → WPT |
| [lib_behaviors-test](#lib_behaviors-test) | ✔ (shared libs) | alpha.bhv, alder.bhv | `BHV_Waypoint_mod`, `BHV_SimpleWaypoint` |
| [pXRelayTest](#pxrelaytest) | ✔ | none | MOOS relay benchmark |
| [tools/aissim](#toolsaissim) | ✔ | with external.moos | standalone vessel+AIS sim |
| [pVisionDetector_ref](#pvisiondetector_ref--pvisiondetector) | ✘ (needs OpenVINO) | realvision | YOLO camera detector |
| pVisionDetector | ✘ | `realvision (copy 1)` | older detector |
| pSensorFusionTracker | ✘ | none | older fusion copy |
| iSensorFusion | ✘ | none | empty template |
| iCANReceiver | ✘ | none | blocking predecessor of iCANRec |

---

## iDivisorNMEA
`src/iDivisorNMEA/` — main sensor input.
- **Config**: `udp_port` (10111), `ownship_mmsi` (710400014), `publish_nav` (true),
  `report_interval` (1.0 s), `default_length` (30 m), `contact_stale_secs` (120);
  requires global `LatOrigin/LongOrigin`.
- **Subs**: `LINHA_NMEA`, `GYRO_*`, `NAV_COG` (registered but unhandled — dead).
- **Pubs**: `NAV_LAT`, `NAV_LONG`, `NAV_SPEED` (**knots**), `NAV_HEADING`, `GPS_HEADING`,
  `NAV_COG`, `NAV_DEPTH`, `ANGULO_LEME`, `NAV_YAW`, `WIND_ANGLE`, `WIND_SPEED`,
  `GYRO_X/Y/Z`, `MSG_UDP` (raw sentence), `NODE_REPORT` (`NAME=AIS<mmsi>`, m/s, `MODE=AIS`).
- **I/O**: UDP in, non-blocking. Sentences: `$--RMC` (GP, GN or GA talker for
  position), `$--HDT` (any talker, e.g. `$GPHDT`, or `$IIHDT` from Signal K's
  signalk-to-nmea0183 plugin), `$GPGGA` (ignored),
  `$SDDPT`, `$AGRSA`, `$VNYMR`, `$WIMWV`, `!AIVDM/!AIVDO` (types 1/2/3/5/18/24 via `AisDecode`).
- Libraries: `NMEAParserLib` (VisualGPS CNMEAParser), partial `libais` (only `GetNthField`).
- Honors `USE_UTM` for contact X/Y.

## pXYpublisher
`src/pXYpublisher/` — subs `NODE_REPORT_LOCAL`, pubs `NAV_X`, `NAV_Y`.
Positional parse (expects token 2 = `X=`, 3 = `Y=`).

## planchaPID
`src/planchaPID/` (class `lanchaPID`) — low-level autopilot replacing pMarinePID.
- **Per-boat tuning comes from the boat profile** (`auto_*` keys, see
  [profiles/README.md](../profiles/README.md)): heading gains by speed (with an
  optional bounded I-term) and output limit,
  speed-dependent rudder limit, speed -> thrust feed-forward map, speed gain
  steps, speed I-term authority and the thrust cap. planchaPID loads its own
  `profile_file` line, or else the `profile_file` of the iCAN block. With a
  profile, `HEADING_*` / `RUDDER_LIMIT_*` lines in the .moos file are ignored
  (warned); without one the .moos values and the built-in defaults (the old
  hardcoded values) apply.
- **Config**: `AppTick` (sets dt), `profile_file`, `SPEED_KP/KI/KD` (service mode only),
  `SERVICE_MODE` (false); `HEADING_KP/KI/KD` (service mode only, as for speed);
  `HEADING_UPPER/LOWER_BOUND`, `RUDDER_LIMIT_MAX/MIN/SPEED_LOW/SPEED_HIGH` only
  when there is no profile. Integrals reset whenever planchaPID is not driving
  (manual control, no active mode).
  All other numeric params go through `std::stod` — a non-numeric line crashes startup.
- **Subs**: `NAV_HEADING`, `NAV_SPEED`, `DESIRED_HEADING`, `DESIRED_SPEED`, gains,
  `SERVICE_MODE`, `DEPLOY` (resets integrals), `MOOS_MANUAL_OVERRIDE`,
  `CONSTANT_HEADING`, `SETPOINT_HEADING`, `WAYPOINT`, `TRAILING`, `STATIONING`, `REGION`, `RETURN`.
- **Pubs**: `DESIRED_RUDDER`, `DESIRED_THRUST` (0 to `auto_thrust_max_pct`), `PID_GAINS`.
- Algorithm: [CODE_FLOW.md](CODE_FLOW.md) §2.3.
- `PIDParameterReader` and `*_parameters.txt` are compiled/committed but **unused**.

## iCAN
`src/iCAN/` — J1939 output on `slcan0` (hardcoded). No config params.
- **Subs**: `DESIRED_THRUST`, `DESIRED_RUDDER`, `TRIM` (disabled), `CAN_SEND`.
- **Pubs**: AppCast only (`TRIM=0` in disabled code).
- Frames: [CAN_INTERFACE.md](CAN_INTERFACE.md).

## iCANRec
`src/iCANRec/` — non-blocking SocketCAN reader.
- **Config**: `CAN_INTERFACE` (slcan0).
- **Pubs**: `ENGINE_ACCELERATOR_PEDAL_POSITION`, `ENGINE_PERCENT_LOAD`, `ENGINE_SPEED`,
  `ENGINE_PERCENT_TORQUE`, `ENGINE_ACTIVE_DTCS`, `ENGINE_COOLANT_TEMP`, `ENGINE_OIL_TEMP`,
  `ENGINE_OIL_PRESSURE`, `ENGINE_FUEL_RATE`, `ENGINE_INTAKE_MANIFOLD_PRESSURE`,
  `ENGINE_BATTERY_VOLTAGE`. Decode table in [CAN_INTERFACE.md](CAN_INTERFACE.md#feedback-icanrec).

## iWebSocket
`src/iWebSocket/` — WebSocket server `0.0.0.0:8000` (IXWebSocket), no TLS/auth.
- **Config**: `colav_pwt_outer_dist` (300), `colav_pwt_inner_dist` (150),
  `colav_completed_dist` (1500), `colav_min_util_cpa_dist` (100), `colav_max_util_cpa_dist` (300).
- Protocol (subs, pubs, JSON schema, command keys): [UI_PROTOCOL.md](UI_PROTOCOL.md).

## iRadarTargets
`src/iRadarTargets/` — radar ARPA `$RATTM` over UDP.
- **Config**: `udp_port` (8091), `udp_host` (parsed but unused; binds all interfaces).
- **Subs**: `NAV_X`, `NAV_Y` (+ `NAV_LAT/LON` for display).
- **Pubs**: `NODE_REPORT` (`NAME=contato_<n>`, SPD in **knots**, LENGTH=100),
  `RADAR_TARGETS` (`x,y,vx,vy,range,bearing,label=radar_<n>;...`, 10 Hz).
- `RadarTargets_orignal.*` are unbuilt leftovers.

## pContactSpawn
`src/pContactSpawn/` — one synthetic constant-velocity contact for testing.
- **Subs**: `NAV_X/Y/HEADING`, `SPAWN_CONTACT` (`heading=,relative_bearing=,distance=,speed=,name=`),
  `SPAWN_CLEAN`, `WPT_INDEX_TRIANGULO` (scripted scenarios with old Rio de Janeiro coords).
- **Pubs**: `NODE_REPORT` (`MODE=SPAWNED`) every tick. Runs on the real boat too.

## pTrafficSim
`src/pTrafficSim/` — AIS traffic generator (sim only; `enable_simulation` defaults **false**).
- **Config**: `enable_simulation`, `origin_lat/lon`, `aivdm_host/port` (127.0.0.1:10112),
  `mmsi_base`, `spawn_mmsi_base`, `expunge_var`, `spawn_max_range`, `max_runtime_contacts`,
  `aim_lock_range`, `random_*`, `position_interval`, `static_interval`, `report_length`,
  `contact = name=,type=,x=,y=,hdg=,spd=,...`, `loop = ...,pts=...`.
- **Subs**: `NAV_*`, `TRAFFIC_SPAWN`, `TRAFFIC_CLEAR`.
- **Pubs**: `NODE_REPORT` (`AIS<mmsi>`, `MODE=SIM`), `TRAFFIC_SPAWNED` (ack), `TRAFFIC_TARGETS`,
  `<expunge_var>`; UDP `!AIVDM` msg 1/5 via `AisEncode`.
- `hdg=at_ownship` solves an intercept course until `aim_lock_range`.

## pColavEval
`src/pColavEval/` — collision-avoidance evaluation.
- **Config**: `collision_range` (20), `near_miss_range` (60), `cpa_confirm` (15),
  `report_interval` (0.5), `max_records` (6), `test_mmsi_low/high`.
- **Subs**: `NAV_*`, `DESIRED_HEADING`, `WPT_INDEX`, `WAYPOINT`, `NODE_REPORT`, `TRAFFIC_SPAWNED`,
  `COLAV_TEST`, `COLAV_BHV_SPAWN/ACTIVE/INACTIVE/CPA/END/TRACE`, `IVPHELM_LIFE_EVENT`, `UCD_REPORT`.
- **Pubs**: `COLAV_TEST_ACK`, `COLAV_ENCOUNTER`, `ENCOUNTER_SUMMARY`, `COLAV_METRICS` (JSON array).

## pCorridor
`src/pCorridor/` — route corridor monitor. Geometry and zone logic live in
`CorridorModel.{h,cpp}` (plain C++, no MOOS; unit tests in `tests/corridor_test.cpp`).
- **Config**: `slow_frac` (0.5), `stop_factor` (2.0), `hysteresis` (2 m),
  `recover_lead` (20 m), `status_interval` (1 s), `recover_move` (1 m).
- **Subs**: `NAV_X`, `NAV_Y`, `CORRIDOR` (true/false), `CORRIDOR_UPDATE`
  (`width=<w>,pts={x1,y1:x2,y2:...}`, full width, local metres, from iWebSocket), `WAYPOINT`.
- **Active** only while `CORRIDOR=true`, `WAYPOINT=true`, the geometry is valid and NAV_X/Y are known.
- **Zones** (xte = distance to the nearest centreline segment, half = width/2):
  `in` ≤ slow_frac·half < `slow` ≤ half < `recovering` ≤ stop_factor·half < `stop`;
  `none` when inactive. Worse zones are entered at once; better zones only once
  the xte clears the boundary by `hysteresis`.
- **Pubs**: `CORRIDOR_STATUS` (on change and every `status_interval`), `XTE_NOW`
  (every iteration while active; one 0 on deactivation), `CORRIDOR_RECOVER_UPDATES =
  points=x,y` while `recovering` (centreline point `recover_lead` m past the closest
  point, clamped to the end; posted before the status). With no corridor it posts
  only `CORRIDOR_STATUS=none`.
- Acted on by `corridor_slow` / `corridor_recover` / `corridor_stop` in `alpha.bhv`
  (see [MISSIONS.md](MISSIONS.md#alphabhv-behaviors)). A boat in `stop` stays stopped
  until the operator acts (CORRIDOR=off, a wider corridor, manual control or AUTONOMOUS_STOP).

## pSensorFusionTracker_ref
`src/pSensorFusionTracker_ref/` — needs Eigen3.
- **Config used**: `association_threshold` (Mahalanobis σ, 15), `quality_threshold` (0.5),
  `radar_noise_pos` (3), `vision_noise_pos` (25). `track_timeout`, `min_hits_to_confirm`,
  `max_misses`, process-noise params are parsed but **ignored**.
- **Subs**: `NAV_X/Y/HEADING`, `RADAR_TARGETS`, `VISION_REPORT`.
- **Pubs**: `FUSED_CONTACTS`, `NODE_REPORT` (`fused_<id>`, SPD knots). Publishes only
  tracks updated by **both** radar and vision.

## pVisionDetector_ref / pVisionDetector
Not built by default. OpenCV + OpenVINO YOLO on an RTSP camera; range from
bounding-box height (pinhole), bearing from pixel offset + `NAV_HEADING`.
Pubs `VISION_REPORT` (`x,y,confidence,bearing,range,class,timestamp`).
The `_ref` version **ignores its `.moos` config** (camera URL and model path are
hardcoded in source). See `src/pVisionDetector/BUILD_INSTRUCTIONS.txt`.

## iNMEA
`src/iNMEA/` — subs `NAV_HEADING/LAT/LONG/SPEED`; sends `$GPRMC` + `$GPHDT`
to UDP 127.0.0.1:10112 (hardcoded) each tick. Assumes `NAV_SPEED` in m/s.

## iSignalK
`src/iSignalK/` — polls `http://127.0.0.1:3000/signalk/v2/api/vessels/self/navigation/course`
each tick (blocking curl). Active destination → `WPT_UPDATE=polygon=x,y`,
`DEPLOY=true`, `MOOS_MANUAL_OVERRIDE=false`, `RETURN=false`; none → `RETURN=true`.
Only run by the legacy `s1_morgan_city/alpha_simulated.moos`.

## lib_behaviors-test
`src/lib_behaviors-test/` — two shared libraries loaded by pHelmIvP via
`IVP_BEHAVIOR_DIRS` / `ivp_behavior_dir`:
- **`BHV_Waypoint_mod`** — stock BHV_Waypoint plus `xpoints=` (live route replace),
  `WPT_CAPTURE_RADIUS`/`WPT_SLIP_RADIUS` info vars, `WPT_PCT_COMPLETE`, `WPT_RADII`, `WPT_ODO`.
  Main mission behavior.
- **`BHV_SimpleWaypoint`** — MIT teaching example (alder mission only).

## pXRelayTest
`src/pXRelayTest/` — copy of MOOS pXRelay benchmark (`INCOMING_VAR`/`OUTGOING_VAR`). Not used by the boat.

## tools/aissim
`tools/aissim/` — standalone NMEA0183 vessel + AIS simulator (not a MOOS app).
- Run: `./bin/aissim tools/aissim/aissim.cfg` **after** starting `external.moos`.
- Reads `DESIRED_RUDDER/THRUST` from iWebSocket JSON (ws://127.0.0.1:8000), integrates a
  uSimMarineV22-style `HullModel`, sends `$GPRMC/$GPHDT/$AGRSA/$SDDPT` + traffic `!AIVDM`
  to Signal K UDP 10112. Unknown config keys are fatal. `ownship_mmsi` must match iDivisorNMEA.
