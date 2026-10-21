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
  steps, speed I-term authority, speed setpoint ramp rates and the thrust cap. planchaPID loads its own
  `profile_file` line, or else the `profile_file` of the iCAN block. With a
  profile, `HEADING_*` / `RUDDER_LIMIT_*` lines in the .moos file are ignored
  (warned); without one the .moos values and the built-in defaults (the old
  hardcoded values) apply.
- **Config**: `AppTick` (sets dt), `profile_file`, `SPEED_KP/KI/KD` (service mode only),
  `SERVICE_MODE` (false); `HEADING_KP/KI/KD` (service mode only, as for speed);
  `HEADING_UPPER/LOWER_BOUND`, `RUDDER_LIMIT_MAX/MIN/SPEED_LOW/SPEED_HIGH` only
  when there is no profile. Integrals reset whenever planchaPID is not driving
  (manual control, no active mode).
- **Stop = neutral**: while the helm has no decision (`IVPHELM_ALLSTOP` not `clear`,
  e.g. `NothingToDo` after a route ends with `REGION` still true) or asks for
  `DESIRED_SPEED` <= 0, thrust and rudder are 0 and the integrals reset. Before,
  the last speed (the helm often never posts 0 on `NothingToDo`) or a leftover
  I-term kept the gear in forward after the mission.
- **Turn-direction latch**: with `auto_heading_turn_latch_deg` (profile, 0 = off; 150 on
  both boats) a heading error above that angle keeps the turn going the way it started.
  Without it a full reversal (error near ±180) flipped the sign every tick and the rudder
  banged ±limit without turning (sim, 20 Oct: give-way-astern and region recovery failed).
- **Speed ramp**: with `auto_speed_ramp_up/down` > 0 the speed PID chases a setpoint
  that starts at the current `NAV_SPEED` when planchaPID starts driving and moves
  toward `DESIRED_SPEED` at those rates; 0 (the default) steps straight to it.
  All other numeric params go through `std::stod` — a non-numeric line crashes startup.
- **Subs**: `NAV_HEADING`, `NAV_SPEED`, `DESIRED_HEADING`, `DESIRED_SPEED`, `IVPHELM_ALLSTOP`, gains,
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
  `colav_completed_dist` (1500), `colav_min_util_cpa_dist` (100), `colav_max_util_cpa_dist` (300),
  `link_timeout` (3 s), `autonomy_link_timeout` (= link_timeout), `lost_link_autonomy`
  (`continue`), `region_margin` (0 m), `region_halt_lead` (0 m: breach_dist =
  margin − lead, at least 5, so the boat coasts to rest by the drawn line),
  `region_recover_speed` (unset = alpha.bhv's `recover_speed`).
- **Operating region**: the UI's polygon is the HALT line. `lib_regiongeom` checks it (3+
  corners, no crossing edges), shrinks it inward by `region_margin` (any shape; narrow
  parts vanish, a split keeps the part the boat is in or the largest) and splits the result
  into convex pieces. Posts `REGION_UPDATES = reset=true # breach_dist=<margin − halt lead>
  [# recover_speed=..]` and
  `REGION_POLY = border={x,y:...} # pts={...} ...` (BHV_OpRegionV26's `dynamic_region_var`,
  re-posted every 2 s while the region is on: the behaviour only reads it while a mission,
  follow or station-keep runs), then `REGION=true`. A refused polygon never switches the region
  on (`REGION_MSG` says why). Telemetry adds `REGION_STATE` (from `V26_STATE`),
  `REGION_CORE` (working boundary, lat,lon:...) and `REGION_MSG`.
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
  `recover_lead` (20 m), `status_interval` (1 s), `recover_move` (1 m),
  `turn_slow_dist` (60 m), `turn_min_angle` (60°), `turn_allowance` (0 m),
  `slow_speed` / `turn_speed` / `recover_speed` (unset), `stop_action` (`stop`),
  `tighten_following` (true),
  `corridor_capture_radius`/`corridor_slip_radius`/`corridor_lead` (15/30/60 m),
  `route_capture_radius`/`route_slip_radius`/`route_lead` (60/90/400 m, must match `waypt_survey`).
- **Subs**: `NAV_X`, `NAV_Y`, `CORRIDOR` (true/false), `CORRIDOR_UPDATE`
  (`width=<w>,pts={x1,y1:x2,y2:...}`, full width, local metres, from iWebSocket), `WAYPOINT`,
  `WPT_UPDATE` (only its `speed=` field: the mission speed).
- **Active** only while `CORRIDOR=true`, `WAYPOINT=true`, the geometry is valid and NAV_X/Y are known.
- **Zones** (xte = distance to the nearest centreline segment, half = width/2):
  `in` ≤ slow_frac·half < `slow` ≤ half < `recovering` ≤ stop_factor·half < `stop`;
  `none` when inactive. Worse zones are entered at once; better zones only once
  the xte clears the boundary by `hysteresis`.
- **Turn allowance**: within `turn_slow_dist` of a sharp corner (`turn_min_angle`)
  the xte is reduced by `turn_allowance` before the zones are worked out, so the
  boat's turning circle around a U-turn does not trip `slow` / `recovering`.
  0 = off. Set it from the boat's turning radius at corridor speeds.
- **Speeds**: `slow_speed`, `turn_speed`, `recover_speed` (helm speed units) are
  posted as `CORRIDOR_SLOW_UPDATES` / `CORRIDOR_TURN_UPDATES` /
  `CORRIDOR_RECOVER_UPDATES = speed=<v>`, capped at the mission speed from
  `WPT_UPDATE` (a corridor never speeds the boat up), on change only. Unset =
  the speed in `alpha.bhv` applies. They exist so each boat keeps the speed it
  needs to steer (USV-722 hardly turns below ~4 kn).
- **`stop_action`**: `stop` (default) holds the boat stopped when it is far outside;
  `recover` steers it back to the centreline instead (reported as `recovering`).
- **Pubs**: `CORRIDOR_STATUS` (on change and every `status_interval`), `XTE_NOW`
  (every iteration while active; one 0 on deactivation), `CORRIDOR_RECOVER_UPDATES =
  points=x,y` while `recovering` (centreline point `recover_lead` m past the closest
  point, clamped to the end; posted before the status), `CORRIDOR_TURN` (true within
  `turn_slow_dist` of a corridor vertex turning by `turn_min_angle` or more: a U-turn where
  an approach joins the route, the ends of a there-and-back route, a loop closing).
  `WPT_UPDATE` for `waypt_survey` on the `CORRIDOR` edges: `capture_radius=15 # slip_radius=30
  # lead=60` when a corridor goes on (the route's 60 m capture radius skips legs shorter
  than that and its 400 m lead cuts corners by more than a corridor half width), the
  `route_*` values when it goes off. With no corridor it posts only `CORRIDOR_STATUS=none`.
- Acted on by `corridor_slow` / `corridor_recover` / `corridor_turn` / `corridor_stop` in `alpha.bhv`
  (see [MISSIONS.md](MISSIONS.md#alphabhv-behaviors)). A boat in `stop` stays stopped
  until the operator acts (CORRIDOR=off, a wider corridor, manual control or AUTONOMOUS_STOP);
  with `stop_action = recover` the `stop` zone is never posted.

## pTurnLimit
`src/pTurnLimit/` — holds the boat to `turn_speed` while it turns hard. Logic in
`TurnLimitModel.{h,cpp}` (plain C++; tests in `tests/turnlimit_test.cpp`). Never steers.
- **Config**: `turn_speed` (helm units, required; unset = never limits), `heading_trigger`
  (45°), `heading_release` (15°), `corner_dist` (100 m), `corner_angle` (60°), `hold_secs` (3),
  `desired_stale_secs` (3).
- **Limited when** an autonomous mode drives (`WAYPOINT`/`TRAILING`/`STATIONING`, not
  manual) and either: within `corner_dist` of the route waypoint the boat is heading for
  (`WPT_INDEX`) where the route (`WPT_UPDATE` polygon/points/xpoints, `loop_start`,
  `repeat`) turns by `corner_angle`+ — with or without a corridor; or `DESIRED_HEADING` is
  `heading_trigger`+ off `NAV_HEADING` (region turn-away, avoidance, follow, recovery),
  released at `heading_release`. Held `hold_secs` after both clear.
- **Pubs**: `TURN_LIMIT` (condition of `turn_max_speed`, BHV_MaxSpeed pwt 300 in
  alpha.bhv), `TURN_LIMIT_REASON` (corner/heading/hold), `TURN_LIMIT_UPDATES = max_speed=..`.

## pColavScale
`src/pColavScale/` — collision-avoidance distances from closing speed, per contact.
Logic in `ColavScaleModel.{h,cpp}` (tests in `tests/colavscale_test.cpp`). Never steers.
- For each running `avdcol_` instance (`IVPHELM_LIFE_EVENT` spawn/death) with a fresh
  `NODE_REPORT`: v = the contact's speed relative to us while the range closes (0 while it
  opens); posts `CONTACT_INFO = name=<contact> # pwt_outer_dist=.. # pwt_inner_dist=..`
  (addressed to that instance only) with outer = clamp(`outer_secs`·v, `outer_min`,
  `outer_max`), inner likewise (≤ outer), on a change of `change_frac` and `change_min`.
- **Alert range**: the helm sets pContactMgrV20's alert range to the template's
  `pwt_outer_dist` (300 m) at start-up; pColavScale posts `BCM_ALERT_REQUEST` with
  `alert_range = outer_max`, every `alert_repost_secs` and whenever another app posts one.
- **Config**: `nav_speed_units` (`knots` on the boat, `mps` in the sim), `contact_speed_units`
  (`mps`), `outer_secs` (60), `inner_secs` (30), `outer_min` (150), `inner_min` (75),
  `outer_max` (1400), `inner_max` (700), `cpa_range` (1500, ≥ `completed_dist`), `change_frac`
  (0.1), `change_min` (20 m), `contact_max_age` (30 s), `alert_repost_secs` (15),
  `behavior_prefix` (`avdcol_`). The UI's `COLAV_DIST` (Calibration) values become the minimums.
- **Pubs**: `CONTACT_INFO`, `BCM_ALERT_REQUEST`, `COLAV_SCALE` (`<contact>=<outer>/<inner>,...`).

## lib_regiongeom
`src/lib_regiongeom/` — operating-region geometry for iWebSocket (plain C++, tests in
`tests/region_test.cpp`): validation, inward offset by a margin (signed-distance grid +
marching squares, made conservative so the result is at least `margin` from the drawn
line), simplification, convex decomposition (ear clipping + merging), BHV_OpRegionV26 spec.

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
  `WPT_CAPTURE_RADIUS`/`WPT_SLIP_RADIUS` info vars, `WPT_PCT_COMPLETE`, `WPT_RADII`, `WPT_ODO`,
  and `loop_start=<n>` (a repeated route restarts at point n, so an approach path at
  points 0..n-1 is driven only once; default 0 = stock).
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
