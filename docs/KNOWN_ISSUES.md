# Known issues

Found while tracing the code on `main` @ `f7af34e` (Oct 2026). Nothing here has
been fixed yet; each item cites where to look. Severity is about on-water risk.
Items marked *(unverified)* are inferred from code, not observed on the boat.

## Critical — safety / control

1. **[Fixed Oct 2026: iWebSocket posts `MOOS_MANUAL_OVERRIDE`; iCAN accepts only the owner's demands]** **Manual-override command is a no-op.** iWebSocket posts `MOOS_MANUAL_OVERIDE`
   (one R) for `CONTROLE_MANUAL` / `AUTONOMOUS_CONTROL`
   (`src/iWebSocket/WebSocket.cpp:1036,1038,1046`). The helm and planchaPID only act
   on `MOOS_MANUAL_OVERRIDE` (`src/planchaPID/lanchaPID.cpp:488`; L711 registers the
   typo but never handles it). If any mode flag is still `true`, planchaPID keeps
   publishing rudder/thrust and fights manual commands.
2. **[Fixed Oct 2026 for manual control: lost SS UI link -> rudder 0, throttle 0; `CAN_SEND=false` sends neutral first. Autonomy still has no deadman if planchaPID dies]** **No command timeout / deadman.** iCAN re-sends the last `DESIRED_RUDDER`/`DESIRED_THRUST`
   forever (`src/iCAN/CAN.cpp:99-109`); planchaPID just stops publishing when its gate
   closes. If planchaPID or the helm dies, the last throttle/steering persists.
   `CAN_SEND=false` stops transmission without first sending neutral.
3. **[Fixed Oct 2026]** **STATIONING and REGION bypass the override check** in planchaPID's publish gate
   (`lanchaPID.cpp:578-580`), and the UI's station-keep/region commands do not set
   `MOOS_MANUAL_OVERRIDE=false` — so the PID may drive actuators from stale
   `DESIRED_HEADING/SPEED` while the helm is parked *(unverified)*.
4. **Collision avoidance only while `WAYPOINT=true`** (`missions/s1_morgan_city_signalk/alpha.bhv`,
   `avdcol_` condition). Trailing, station-keeping and region modes have no COLAV.
5. **iWebSocket has no authentication** on `0.0.0.0:8000`; the remote viewer has full
   write access to MOOSDB `:9000`. Anyone on the boat LAN can command the vessel.
6. **Plaintext camera credentials committed** in `src/pVisionDetector*/` sources,
   `.moos` files and `missions/s1_morgan_city_signalk/realvision*.moos`, including a
   committed binary and `.o`. Rotate the password; scrub history if the repo is shared.

## High — wrong behavior

7. **Speed units are mixed.** Real-boat `NAV_SPEED` is knots (`DivisorNMEA.cpp:449-456`);
   radar and fusion `NODE_REPORT` SPD are knots (`RadarTargets.cpp:467`); AIS / pTrafficSim /
   pContactSpawn contacts are m/s. Radar contacts look ~1.94× faster to pContactMgrV20,
   the helm CPA and pColavEval. iNMEA assumes m/s.
8. **Stray speed-controller call** `speedController.Calculate(...)` at `lanchaPID.cpp:548`
   runs the heading-style PID on the speed controller before `CalculateSpeed`, sharing
   its integral — the speed I-term ends up nearly inert.
9. **TSC1 throttle table**: 71 % = 50 % (2408 rpm) (`CAN.cpp:499`, `:520`); 23–25 % is
   non-monotonic. Autonomous mode is capped at 50 %, manual can hit 71 %.
10. **Rudder units**: planchaPID treats `DESIRED_RUDDER` as degrees (±20), iCAN as
    percent of steering travel; values with |x|<1 truncate to centre.
11. **Return-to-base does nothing** in the signalk mission — no behavior in `alpha.bhv`
    uses `RETURN` / `RETURN_UPDATE`.
12. **`real.moos` missing sim COLAV fixes**: alert 1000/500 vs bhv `completed_dist=1500`,
    no `expunge_var` (so `can_disable` is a no-op), `contact_max_age=5`.
13. **iRadarTargets ignores** bearing reference (T/R), units and target status
    (lost targets stay live); `udp_host` unused (binds all interfaces).
14. **iCANRec suspect decodes**: torque, oil temp, battery voltage (see
    [CAN_INTERFACE.md](CAN_INTERFACE.md#feedback-icanrec)).
15. **[Partly fixed Oct 2026: NAV_* held until the first valid RMC (`require_valid_fix`); `GPS_AGE` shows GPS loss; any talker accepted]**
    **iDivisorNMEA** publishes `NAV_LAT/LONG=0` before first fix and re-publishes stale
    nav on every datagram (GPS loss invisible while AIS flows); only `GP` talker accepted;
    `$SDDPT`/`$AGRSA` parse can throw and kill the process; `$VNYMR` gyro fields off by one.
16. **iWebSocket**: `m_webSockets` / `m_lastMessageTime` mutated from ws threads without a
    lock (race / use-after-free); unguarded `std::stod` on UI input crashes the app;
    `usleep(100ms)` inside `Iterate`; origin hardcoded in 7 places; `OIL_TEMP` vs
    `ENGINE_OIL_TEMP` mismatch; `DEPLOY` posted as double.
17. **pSensorFusionTracker_ref**: tracks never deleted when inputs stop; repeated
    `RADAR_TARGETS` snapshots create duplicate tracks; several config params ignored;
    radar targets reach the helm twice (`contato_N` + `fused_N`) in realvision.
18. **pContactSpawn** runs on the real boat; `cleanContact()` leaves a ghost contact at (0,0).

## Medium — build / repo hygiene

19. `missions/s1_morgan_city_signalk/launch.sh` runs `alpha.moos`, which doesn't exist there.
20. `alpha_fusion.moos`, `test_fusion.moos` reference apps not in the repo
    (`pRadarBridge`, `pSensorFusion`); legacy mission references `iRaspberryServer`,
    `/home/d2snc/...`.
21. `ivp_behavior_dir=../../lib` (real/external/realvision) vs `IVP_BEHAVIOR_DIRS` env:
    both set → fatal "dangerously ambiguous" helm error.
22. Committed artefacts: `src/pVisionDetector/build/` (34 files incl. binary),
    `*.broken`, `debug*.log`, `scripts/__pycache__`, `realvision (copy 1).moos`,
    `*.moos++`, ~28 MB of MOOS logs, 20 MB video.
23. Dead code: `iCANReceiver`, `iSensorFusion`, `pSensorFusionTracker`,
    `RadarTargets_orignal.*`, `PIDParameterReader` + `*_parameters.txt`.
24. `README.md` app table has errors (pContactSpawn description, fusion "AIS").
25. `scripts/server_can_*.py` keep transmitting after client disconnect (missing `global`).
