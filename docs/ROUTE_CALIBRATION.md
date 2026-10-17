# Route-tracking calibration: phase 1 plumbing

Branch `feature/route-calibration` (from `feature/can-drive-profiles` @ `1d3052d`), with a
matching `feature/route-calibration` in Swiftships-UI (from `design/full-navy-redesign` @ `ee3e36e`).
Status: for review. Built pieces are unit tested off the vessel; nothing has run on a boat.

## 1. Purpose

`pRouteTuner` (a new MOOS app, not in this branch) will run the boat around a test route,
score each lap, and adjust heading gains and waypoint-behaviour geometry in small bounded
steps until the boat holds the drawn route. It needs three things that do not exist yet:

| # | Change | Repo | Why |
|---|---|---|---|
| 1 | GNSS quality and fix gating in `iDivisorNMEA` | MOOS | Precheck and abort need fix type, satellites, HDOP and fix age; NAV_* must not go out before a real fix |
| 2 | `CAL_AUTH` / `CAL_ABORT` frames and `CAL_*` / `GPS_*` pass-through in `iWebSocket` | MOOS | Operator authorisation, and an abort that works even if the tuner is not running |
| 3 | Route Calibration card | UI | The operator's Authorise and Abort controls, plus live state |

Everything here can be built and tested without the tuner, using `uPokeDB` to fake its
outputs (section 6).

## 2. What the 16 Oct USV-722 logs showed

Logs: `MOOSLog_16_10_2026_____09_15_46` to `..._12_46_27`, Morgan City.

1. **No GGA reaches iDivisorNMEA.** In the 12:01 run, `MSG_UDP` carries only `$GPRMC`
   (24,120), `$IIHDT` (24,105) and `!AIVDM` (3,895). The RMC sentences have empty time and
   date fields, so Signal K's `signalk-to-nmea0183` is regenerating them.
   **Action:** enable the GGA sentence in that plugin (it builds GGA from
   `navigation.gnss.*`) on both boats, and check `udp-nmea-sender` forwards it to 10111.
   Until then `GPS_AGE` works but `GPS_FIX`, `GPS_SATS` and `GPS_HDOP` are not posted.
2. **NAV_LAT/LONG = 0 at start-up** (known issue 15). The 09:15 run had zero RMC sentences
   but posted `NAV_LAT=0` 1,046 times, putting the boat about 3,300 km away in NAV_X/Y.
   Fixed here by `require_valid_fix`.
3. **The boat ran code that is not on any pushed branch.** The 12:01 run's `._moos` has
   `profile_file` in the planchaPID block and a comment that heading gains, rudder and thrust
   limits are "auto_* keys in the boat profile". No remote branch (`main`, `dev-testing`,
   `feature/can-drive-profiles`, `fix/pid-speed-integral-rudder-rounding`) contains that
   code. **Please push it**, then merge it into this branch: the tuner writes gains through
   planchaPID, so it must build on what the boat actually runs.
4. **Rudder units.** `DESIRED_RUDDER` and `CAN_RUDDER_ACTUAL_PCT` are percent of steering
   travel, not degrees. The steady +7 seen on straights is +7 % of travel. The feedback
   tracks the command within about 1 % with no visible lag, so the tuner can rely on it.
5. Talker handling (`$IIHDT`) and the JSON telemetry format are already correct on
   `feature/can-drive-profiles`; the UI branch parses that JSON. No action.

Baseline, 12:01 run (12 legs, 4 to 9 kn), scored by `tools/score_alog.py` (`python3 tools/score_alog.py RUN.alog --plot --json --out out/`):

| Metric | Measured | Target |
|---|---|---|
| Mean cross-track | 20.3 m | 2 m |
| 95th pct cross-track | 56.3 m | 5 m |
| Max cross-track | 69.6 m | 8 m |
| RMS heading error, straights | 17.0° | 3° |
| Overshoot after turns | 23.7° | 10° |
| Settling time | 28 s | 12 s |
| Rudder saturation | 8.7 % of samples | 5 % |

Main causes: `lead = 400` and `capture_radius = 60` in `alpha.bhv` (sized for a ship), the
steady rudder offset, a heading limit cycle at Kp 0.6 near 6 kn (1750 to 1950 s), and 48
hand changes to Kp during the run.

## 3. Change 1: iDivisorNMEA

Files: `DivisorNMEA.h/.cpp`, new header-only `NmeaFix.h`, test `tests/nmeafix_test.cpp`
(hooked into `tests/run_tests.sh`). No CMake change.

- Parses any `$--GGA` (quality, satellites, HDOP, altitude) and the status field of any
  `$--RMC`. Only checksum-valid sentences count.
- Holds NAV_* until the first RMC with status `A` (`require_valid_fix`, default true).
  Simulated missions are unaffected: they run with `publish_nav = false`.
- Posts quality on a timer from `Iterate()` so `GPS_AGE` keeps growing when the receiver
  goes quiet (NAV_* is re-posted on every datagram and cannot show that).
- Two new appcast lines: fix state, and the GGA summary or a hint to enable GGA.

| Variable | Type | Meaning |
|---|---|---|
| `GPS_AGE` | double | s since last valid RMC, `-1` = never |
| `GPS_HAVE_FIX` | string | `true` once any valid RMC has been seen |
| `GPS_FIX` | double | GGA quality: 0 none, 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float, 6 DR |
| `GPS_FIX_LABEL` | string | same as text |
| `GPS_SATS` | double | satellites used |
| `GPS_HDOP` | double | HDOP |
| `GPS_GGA_AGE` | double | s since last GGA |

Config (in `real.moos`): `require_valid_fix = true`, `gps_post_interval = 1`.

## 4. Change 2: iWebSocket

New UI to MOOS frames (also in [UI_PROTOCOL.md](UI_PROTOCOL.md)):

| Frame | MOOS posts |
|---|---|
| `CAL_AUTH=true#OPERATOR=<name>#SESSION=<id>` | `CAL_AUTH_BY`, `CAL_AUTH_SESSION`, `CAL_AUTH=true`, `CAL_EVENT` |
| `CAL_AUTH=false` | `CAL_AUTH=false`, `CAL_EVENT` |
| `CAL_ABORT=true[#REASON=<text>]` | `SERVICE_MODE=false`, `CAL_AUTH=false`, `CAL_ABORT=true`, `CAL_ABORT_REQUEST`, `CAL_EVENT`, then the `cal_abort_action` |

`SERVICE_MODE=false` makes planchaPID drop the tuner's gains at once, so **abort works even
if pRouteTuner has crashed or was never started**. The tuner, when running, also restores the
behaviour parameters it saved.

`cal_abort_action` (config, default `none`):
- `none`: the route continues on planchaPID's configured gains; the operator still has Stop and Manual.
- `stop`: the same postings as `AUTONOMOUS_STOP`.
- `manual`: `MOOS_MANUAL_OVERRIDE=true`, rudder and thrust 0.

Default is `none` on purpose: cutting propulsion is not always the safest response in a
channel. Please agree the default with the operators.

`CAL_*` and `GPS_*` are added to the existing `CAN_*` pass-through (`m_can_vars`), so they are
registered, keep their type, and appear in the telemetry JSON once first posted.

## 5. Change 3: UI (Swiftships-UI, `feature/route-calibration`)

New standalone `src/app/modules/calibration/` (`<calibration-card>`), a floating draggable
panel in the same dark-navy style as System Checklist, opened from a toolbar button (icon
`tune`) after Engine Status. It subscribes to `MoosIvpWebsocketService` directly, so
`processMoosJsonMessage()` is not touched.

- State with a nine-step strip, lap, speed band, cross-track, heading error, baseline and
  candidate scores, GNSS quality, last six `CAL_EVENT`s.
- "No tuner" when `CAL_STATE` is older than 5 s; "No link" when the socket is down.
- **Authorise**: enabled only when connected, three checks ticked (inside the approved
  test area; no people, traffic or obstacles nearby; E-stop and manual takeover checked)
  and an operator name entered. Shows "Authorised" only after the boat echoes `CAL_AUTH=true`.
- **Abort**: always enabled while connected, no confirmation step.
- `#` and `=` are stripped from the operator name (frame separators).

## 6. Testing

### Build and unit tests
```
./build.sh -j$(nproc)
tests/run_tests.sh            # lib_canprofile + NmeaFix, plain g++
```
In the UI: `npm run build`.

### iDivisorNMEA on the bench
1. Real GPS connected: appcast shows `GNSS fix: yes` and a `GGA:` line (or the hint, see finding 1).
2. `uXMS GPS_AGE GPS_FIX GPS_SATS GPS_HDOP NAV_LAT`: GPS_AGE under 1 s.
3. Stop the Signal K sender: GPS_AGE keeps rising.
4. Start with no GPS: `NAV_LAT` must not appear.

### iWebSocket and the card without the tuner
```
uPokeDB real.moos CAL_STATE=BASELINE CAL_LAP_ID=1 CAL_SPEED_BAND=5 CAL_XTE=21.7 CAL_HDG_ERR=2.4 CAL_SCORE_BASELINE=79.1
uPokeDB real.moos CAL_EVENT="baseline lap 1 started"
```
The card shows BASELINE, step 2, XTE 21.7 m in red, the score and the event. Stop poking for
5 s: "No tuner". Then, watching `uXMS CAL_AUTH CAL_AUTH_BY CAL_ABORT SERVICE_MODE WAYPOINT MOOS_MANUAL_OVERRIDE`:

| Action | Expected |
|---|---|
| Three checks, name, Authorise | `CAL_AUTH=true`, `CAL_AUTH_BY=<name>`; card shows "Authorised by" |
| Withdraw | `CAL_AUTH=false` |
| Poke `SERVICE_MODE=true`, then Abort | `SERVICE_MODE=false`, `CAL_ABORT=true`; `PID_GAINS` shows `service_mode=false` next tick |
| Abort with `cal_abort_action = stop` | also `WAYPOINT=false`, `STATIONING=false`, rudder and thrust 0 |

### Acceptance
- [ ] `tests/run_tests.sh` passes
- [ ] GGA visible in `MSG_UDP` on USV-722 and USV-725
- [ ] No `NAV_LAT=0` in a log started without GPS
- [ ] `GPS_AGE` rises when GPS is removed
- [ ] Card shows live `CAL_*` from `uPokeDB`
- [ ] Authorise disabled until the three checks and a name are present
- [ ] Abort reverts `SERVICE_MODE` with pRouteTuner not running
- [ ] Existing controls unaffected: route start/stop, station keep, follow, live PID, manual

## 7. MOOS contract for pRouteTuner

| Variable | Producer | Consumers | Notes |
|---|---|---|---|
| `GPS_AGE`, `GPS_FIX`, `GPS_SATS`, `GPS_HDOP` | iDivisorNMEA | pRouteTuner, iWebSocket | precheck: age < 2 s, sats >= 6, HDOP <= 2.5 |
| `CAL_AUTH`, `CAL_AUTH_BY`, `CAL_AUTH_SESSION` | iWebSocket | pRouteTuner | may not leave PRECHECK unless `CAL_AUTH=true` |
| `CAL_ABORT`, `CAL_ABORT_REQUEST` | iWebSocket | pRouteTuner | go to ABORT, restore saved bhv params |
| `SERVICE_MODE`, `HEADING_KP/KI/KD` | pRouteTuner (iWebSocket on abort) | planchaPID | existing path |
| `WPT_UPDATE` (lead, lead_damper, capture_radius) | pRouteTuner | pHelmIvP | only between laps, on straights |
| `CAL_STATE`, `CAL_LAP_ID`, `CAL_SPEED_BAND`, `CAL_XTE`, `CAL_HDG_ERR`, `CAL_RUDDER_TRIM`, `CAL_SCORE_*`, `CAL_GAINS_*`, `CAL_ABORT_REASON`, `CAL_VERDICT`, `CAL_EVENT` | pRouteTuner | iWebSocket, grafana_bridge | `CAL_STATE`: PRECHECK, BASELINE, CALIBRATION, CONTROLLED_TEST, ANALYSIS, APPLY_CANDIDATE, VALIDATION, ACCEPT_OR_ROLLBACK, COMPLETE, ABORT |

Tuner design in brief: score per lap `J = XTE_p95 + 0.5 XTE_mean + 0.3 RMS heading error +
0.2 overshoot + 0.1 settling`; one parameter at a time (lead, Kp, Kd, lead_damper), one
bounded step each way, keep only if J improves > 5 % and no guard rail trips (overshoot >
15°, rudder saturation > 5 %, rudder activity > 1.5 x baseline, oscillation), otherwise roll
back. Never changes gains mid-turn; never touches station keep, collision avoidance,
OpRegion or the manual/stop path.

## 8. Not in this branch

1. `pRouteTuner` itself.
2. `grafana_bridge`: subscribe to `CAL_*` / `GPS_*`, write to telemetry and `CAL_EVENT` to
   events, tagged by vessel.
3. Grafana Calibration tab (service view only).
4. `alpha.bhv` geometry: proposed `lead = 20`, `capture_radius = 10`, `slip_radius = 15`.
   Needs sign-off and a simulation run first; it changes every mission.

## 9. Open questions

1. Where is the per-boat `auto_*` gain code the boat ran on 16 Oct? (finding 3)
2. Can Signal K on both boats send GGA; is the GNSS source N2K or NMEA 0183?
3. `cal_abort_action` default for each boat?
4. Does anything on the helm side rely on NAV_* being re-posted with every datagram?
