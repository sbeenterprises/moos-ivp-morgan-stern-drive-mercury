# Missions & runtime configuration

All current missions live in `missions/s1_morgan_city_signalk/`. Common to all:
`ServerPort=9000`, `Community=alpha`, Morgan City origin
(`29.71970895316288, -91.14705165281887`), behaviors from `alpha.bhv`.

## How to launch

`launch.sh` in this directory runs `pAntler alpha.moos`, but there is **no
`alpha.moos` here**. Launch the file you want directly:

```bash
cd missions/s1_morgan_city_signalk
pAntler real_usv722.moos   # on USV-722 (Yamaha outboard)
pAntler real_usv725.moos   # on USV-725 (Mercury stern drive)
pAntler simulated.moos     # desktop sim (uSimMarineV22 + pTrafficSim + pColavEval)
pAntler external.moos      # boat topology fed by tools/aissim; then: ../../bin/aissim ../../tools/aissim/aissim.cfg
uMAC real.moos             # appcast monitor (any mission)
```

`clean.sh` deletes `MOOSLog_*`, `*.moos++` etc. — note that includes committed
`.tar.gz` logs and `.moos++` files in this directory.

## Mission files

| File | Purpose |
|---|---|
| `real_usv722.moos`, `real_usv725.moos` | Real boat, one file per boat: sensors via Signal K, CAN actuation, radar, UI. They differ in the boat profile (`profile_file`, also used by planchaPID), the telemetry `vessel_id`, the own-ship MMSI line, and the site: `real_usv725.moos` uses the Lumut origin (4.2245, 100.5868) and `lumut.tif`, with pMarineViewer off, as on that boat. |
| `real.moos` | The generic template the two files above were made from; its `profile_file` line must be edited per boat. Prefer the per-boat files. |
| `simulated.moos` | Desktop sim: uSimMarineV22 owns NAV_*, iNMEA feeds Signal K, pTrafficSim targets, pColavEval metrics, iMOOSGateway (TCP 9501). No pLogger. |
| `external.moos` | Real-boat topology, but sensors come from `tools/aissim` via Signal K. No iCAN/iCANRec. Best end-to-end test of the real data path without hardware. |
| `realvision.moos` | `real.moos` + `pVisionDetector_ref` + `pSensorFusionTracker_ref`. |
| `realvision (copy 1).moos` | Older duplicate (runs non-ref pVisionDetector). Stale. |
| `alpha_fusion.moos` | Uses `pRadarBridge`/`pSensorFusion`, which are **not in this repo** — cannot launch. |
| `remote_viewer.moos` / `remote_viewer_simulated.moos` | Only pMarineViewer, connecting to a remote MOOSDB (`192.168.143.16` / `192.168.6.120`). |
| `simulated.moos++`, `external.moos++` | pAntler-expanded outputs, committed. `simulated.moos++` is stale. |
| `MOOSLog_29_5_2026_*.tar.gz` | Sea-trial logs (29 May 2026) used to recalibrate the speed feed-forward map. |

## Process matrix

R = run, c = config block only, – = absent

| Process | real | simulated | external | realvision | alpha_fusion | remote_viewer |
|---|---|---|---|---|---|---|
| MOOSDB | R | R | R | R | R | – |
| pLogger | R | c | R | R | R | c |
| uSimMarineV22 | c | **R** | c | c | – | c |
| planchaPID | R (0.15/0.26) | R (0.9/0.2) | R | R | R | c |
| iDivisorNMEA | R | c | R | R | R | c |
| pXYpublisher | R | – | R | R | R | – |
| pHelmIvP | R (`start_in_drive=false`) | R (**true**) | R | R | R | c |
| pMarineViewer | R | R | R | R | – | **R** |
| uProcessWatch, pRealm | R | R | R | R | R | – |
| pNodeReporter | R (len 100) | R (70) | R (70) | R | R | c |
| iCAN | R | – | c | R | R | – |
| iCANRec | R | R | c | R | R | – |
| iNMEA | c | **R** | c | c | – | c |
| iSignalK | c | c | c | c | – | c |
| iWebSocket | R | R | R | R | R | c |
| pContactSpawn | R | R | c | R | R | – |
| pContactMgrV20 | R (1000/500) | R (1500/1000, expunge) | R (1000/500) | R | R | – |
| iRadarTargets | R | R | R | R | R | – |
| pTrafficSim | – | **R** | – | – | – | – |
| pColavEval | – | **R** | – | – | – | – |
| pCorridor | R | R | R | – | – | – |
| iMOOSGateway | – | R (tcp 9501) | – | – | – | – |
| pVisionDetector_ref / pSensorFusionTracker_ref | – | – | – | **R** | – | – |

## Notable config differences real vs sim

| Item | real.moos | simulated.moos |
|---|---|---|
| planchaPID `HEADING_KP/KD` | 0.15 / 0.26 | 0.9 / 0.2 |
| pHelmIvP `start_in_drive` | false | true |
| pHelmIvP `ivp_behavior_dir` | `../../lib` | commented (uses `IVP_BEHAVIOR_DIRS`; both together is a fatal "ambiguous" error) |
| pContactMgrV20 alert/cpa range | 1000 / 500 | 1500 / 1000 |
| pContactMgrV20 decay / max age | 5,10 / 5 s | 30,60 / 300 s |
| pContactMgrV20 `expunge_var` | none | `XYZ_EXPUNGE_TARGET` |
| iWebSocket `colav_*` | defaults | explicit |
| pLogger | on | off |

## `alpha.bhv` behaviors

| Name | Type | pwt | Condition | Updates var |
|---|---|---|---|---|
| `waypt_survey` | BHV_Waypoint_mod | 100 | `WAYPOINT=true` | `WPT_UPDATE` |
| `station-keep` | BHV_StationKeep | 50 | `STATIONING=true` | `STATION_UPDATES` |
| `avdcol_` (templated per contact) | BHV_AvdColregsV22 | 250 | `WAYPOINT=true` | `CONTACT_INFO` |
| `opr24` | BHV_OpRegionV24 | 300 | `REGION=true` | `REGION_UPDATES` |
| `trail_contact` | BHV_Trail | 100 | `TRAILING=true` | `TRAIL_UPDATES` |
| `corridor_slow` | BHV_ConstantSpeed (speed 2) | 200 | `CORRIDOR_STATUS=slow`, `WAYPOINT=true` | `CORRIDOR_SLOW_UPDATES` |
| `corridor_recover` | BHV_Waypoint (speed 2, capture 5, reports silent) | 200 | `CORRIDOR_STATUS=recovering`, `WAYPOINT=true` | `CORRIDOR_RECOVER_UPDATES` |
| `corridor_turn` | BHV_ConstantSpeed (speed 2) | 200 | `CORRIDOR_TURN=true`, `WAYPOINT=true` | `CORRIDOR_TURN_UPDATES` |
| `corridor_stop` | BHV_ConstantSpeed (speed 0) | 240 | `CORRIDOR_STATUS=stop`, `WAYPOINT=true` | – |

Initialised false: `DEPLOY`, `RETURN`, `WAYPOINT`, `STATIONING`, `REGION`, `TRAILING`, `CORRIDOR`
(and `CORRIDOR_STATUS=none`).

The `corridor_*` blocks only run when pCorridor reports a zone, i.e. a corridor
was sent from the UI and a waypoint mission is running; without one they stay
idle. Their speeds are helm units (knots on the boat, m/s in sim); the speeds in
`alpha.bhv` are fallbacks, each mission file's pCorridor block sets the per-boat
`slow_speed` / `turn_speed` / `recover_speed`, `turn_allowance` and `stop_action`
(real_usv722: 4 / 5 / 4 kn, 35 m, recover; provisional until water-tested). Slow and
recover (200) sit between `waypt_survey` (100) and `avdcol_` (250); stop (240)
is above them but still below `avdcol_`, so collision avoidance always wins. `waypt_survey`'s endflag and `AUTONOMOUS_STOP` post
`CORRIDOR=false`, so a corridor does not carry over to the next route.

## Network ports

| Port | Proto | Who | Purpose |
|---|---|---|---|
| 9000 | TCP | MOOSDB | MOOS community `alpha` (remote viewer connects here directly) |
| 8000 | WS | iWebSocket | Operator UI |
| 3000 | HTTP | Signal K | iSignalK course API; UI |
| 10112 | UDP | Signal K (in) | NMEA from iNMEA / pTrafficSim / aissim |
| 10111 | UDP | iDivisorNMEA (in) | Signal K udp-nmea-sender output |
| 8091 | UDP | iRadarTargets (in) | Radar `$RATTM` |
| 9501 | TCP | iMOOSGateway | sim only |
| `slcan0` | CAN | iCAN / iCANRec | 250 kbit/s J1939 |

There is no pShare / multi-community setup.

## Legacy & example missions

- `missions/s1_morgan_city/` — pre-Signal K mission (`alpha.moos`, `alpha_simulated.moos`);
  has the classic DEPLOY/RETURN waypoint tree. References `iRaspberryServer` (not in repo)
  and `/home/d2snc/...` behavior dir.
- `missions/alder/` — stock moos-ivp-extend tutorial (`BHV_SimpleWaypoint`).
- `missions/xrelay/` — stock pXRelay example.
- `missions/test_fusion.moos` — references non-existent `pRadarBridge`/`pSensorFusion`.
