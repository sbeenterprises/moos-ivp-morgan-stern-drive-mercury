# CAN / J1939 interface

Bus: SocketCAN `slcan0`, 250 kbit/s, brought up with

```bash
sudo slcand -o -c -s5 /dev/ttyACM2 slcan0
sudo ip link set slcan0 up
```

All commands use 29-bit extended IDs; the steering actuator (0x210) on both boats is 11-bit.

## Commands out (iCAN + boat profile)

Since October 2026 iCAN has no CAN IDs or payloads in its code. Each boat
has a profile in [`profiles/`](../profiles/README.md) and the mission's
iCAN block names it (`profile_file = ../../profiles/<boat>.prf`). A profile
that fails validation leaves iCAN running but sending nothing
(`CAN_STATUS=INHIBITED`).

Behaviour common to every boat (`src/lib_canprofile/DriveCore.cpp`):

- **One control owner.** `MOOS_MANUAL_OVERRIDE=true` makes SS UI the owner;
  only `DESIRED_RUDDER`/`DESIRED_THRUST` posted by `iWebSocket` are then
  accepted. Otherwise planchaPID, pHelmIvP end-flags and iWebSocket's stop
  zeros are accepted. Others are ignored and counted on the appcast. An
  owner change clears both setpoints to 0.
- **Manual settings latch.** A rudder or throttle value is held until the
  operator sends another one; there is no command timeout.
- **Lost link.** iWebSocket posts `UI_LINK_OK` (client connected and heard
  from within `link_timeout`, default 3 s; the server pings each second and
  browsers answer automatically). In manual, a lost link (or `UI_LINK_OK`
  older than `link_stale_secs`) sets rudder and throttle to the profile's
  `lost_link_*` values (0), and they stay there after the link returns until
  the operator sends a new command. With `lost_link_autonomy = stop` (iCAN
  and iWebSocket, set in the real-boat missions) the same happens while
  autonomy drives, judged by `UI_LINK_OK_AUTO` (iWebSocket
  `autonomy_link_timeout`, 5 s in the real-boat missions, vs 3 s for manual):
  iCAN sends the lost-link values for as long as the link is down, and iWebSocket ends the mission / station-keep / follow (as the
  UI's Stop does, posting `LOST_LINK_STOP=<modes>`), so the boat stays
  stopped when the link returns. Default `continue`: autonomy carries on.
- **Manual gear.** In manual control the gear comes from SS UI's gear
  buttons (`DESIRED_GEAR` = A/N/R) and `DESIRED_THRUST` is throttle only
  (sign ignored). Forward or reverse with 0 thrust = in gear at idle;
  thrust in neutral does nothing. Choosing a different gear resets thrust
  to 0. Autonomy still takes the gear from the sign of thrust. iCAN option
  `manual_gear = command` (default) or `thrust_sign` (one lever: the
  sign of `DESIRED_THRUST` sets the gear, 0 = neutral; `DESIRED_GEAR` ignored).
  `real.moos` uses `thrust_sign`.
- **Rudder vs speed.** If the profile has `rudder_limit_speed`, iCAN reads
  `NAV_SPEED` and limits the rudder (manual and autonomy) to the table's
  value for the current speed; with no `NAV_SPEED` for `speed_stale_secs`
  the smallest value applies. Posted as `CAN_RUDDER_LIMIT_PCT`.
  `rudder_rate_pct_s` limits how fast the rudder command may change.
- **Throttle ramp-up.** `thrust_rate_up_pct_s` lets the throttle actually sent
  rise by at most that % per second, for the UI lever and the autopilot alike;
  less throttle, stop, neutral and lost link are sent at once.
- **Shift interlock.** Reversing passes through neutral for
  `shift_neutral_secs`; throttle stays at 0 for `shift_settle_secs` after a
  gear engages. Both are 0 on USV-725 (placeholder, to confirm with the drive maker).
- **Guard.** iCAN listens `guard_secs` before sending and will not send an ID
  another node is sending (`CAN_CONFLICT`). On USV-722 the helm must have its
  steering switched off before the PC can steer.
- **CAN_SEND.** `false` sends one neutral frame of each message and then
  stops; `true` starts again from neutral.
- **Trim.** `TRIM=1` (up) or `TRIM=-1` (down) from SS UI starts one pulse of
  `trim_up_secs` / `trim_down_secs` (at most 5 s); `TRIM=0` stops at once.
  Frames are sent only during the pulse, then one idle frame if the profile
  defines one. Lost link, owner change and `CAN_SEND=false` stop trim. Only
  accepted from `trim_sources` (iWebSocket), only with `CAN_SEND=true`, and
  only if the profile lists the `trim` capability. State: `CAN_TRIM_STATE`.

### USV-725, Mercury stern-drive (`profiles/mercury_sterndrive.prf`)

From the 15 Oct 2026 captures (`USV-725-CAN-ID-Mapping`). The PC sends only
the four commands the boat accepted from the PC, all with `guard=true`. The
helm sends EEC2 and ETC2 itself, so its lever must be silent before the PC
drives throttle and gear.

| CAN ID | Meaning | Payload |
|---|---|---|
| `18FF11D0` | Steering command (PGN 65297) | Bytes 0–1 LE: 0 full port, 511 centre, 1012 full starboard (usv-722 coding, accepted by the 725 actuator; full-lock values to confirm). Every 130 ms. |
| `0CF003D0` | EEC2 (PGN 61443) throttle lever | Byte 1 = throttle % / 0.4 (`00`–`FA`), same ahead and astern. Every 40 ms. |
| `18F005D0` | ETC2 (PGN 61445) gear lever | `7E` forward, `7D` neutral, `7C` reverse. Forward/reverse still to be seen with the engine running. |
| `18FF08D0` | Trim (PGN 65288) | `F1` up / `F4` down every 50 ms, 1.2 s per command, one `F0` idle frame when the pulse ends. `0CFF08D0` is the helm's own switch. |

**Never sent:** TSC1 `18000017` (SA 23 owns engine speed; it follows the
lever as rpm = 700 + 35 × throttle %), the steering controller's
`18F30A1A`, and the key switch. Until October 2026 this profile sent the old
hardcoded frames (`18F30A1A` + TSC1); they were replaced to match the
mapping.

Feedback: `CAN_STEER_DEMAND_PCT` (`18F30A1A` byte 0 / 39), `CAN_HELM_GEAR`,
`CAN_HELM_THROTTLE_PCT`, `CAN_ENGINE_RPM_REQUEST` (TSC1 from SA 23),
`CAN_FUEL_PCT`. The actual rudder (0x210 frames with byte 5 = `64`) shares
its ID with command frames, and `rx_field` cannot filter on a byte yet, so
there is no rudder-mismatch alarm on this boat.

### USV-722 (`profiles/usv722_glendinning.prf`)

From the 9 Oct 2026 captures (`USV-722-CAN-ID-Mapping.xlsx`). The PC sends
the Glendinning helm's own messages:

| CAN ID | Meaning | Payload |
|---|---|---|
| `18FF11D0` | Steering, 130 ms | Bytes 0–1 LE: 0 port, 511 centre, 1012 starboard |
| `18F005D0` | Gear lever (ETC2), 130 ms | `7E` / `7D` / `7C` |
| `0CF003D0` | Throttle lever (EEC2), 70 ms | Byte 1 = % / 0.4 (`FA` = 100 %), forward and reverse |

Decoded feedback: `CAN_RUDDER_ACTUAL_PCT`, `CAN_RUDDER_ECHO_PCT`,
`CAN_STEER_MOTOR_PCT` (11-bit `0x210`), `CAN_HELM_HAS_STEERING`
(`18FF08D0` high nibble), `CAN_HELM_GEAR`, `CAN_HELM_THROTTLE_PCT`,
`CAN_ENGINE_GEAR`, `CAN_ENGINE_THROTTLE_PCT` (processor 226), `CAN_FUEL_PCT`.
Trim `18FF08D0` (`F1` up / `F4` down / `F0` idle; up 1.0 s, down 0.5 s)
is **enabled**. Open points: the helm sends this ID continuously (the guard
blocks PC trim while the helm is on), the trim pump wiring is unconfirmed, and the byte's high
nibble is the helm-in-command flag. The key switch is never sent.

## Feedback (iCANRec, `src/iCANRec/CANRec.cpp`)

Non-blocking reader; values ≠ 0xFF/0xFFFF are posted when their frame arrives.

| MOOS var | PGN | Bytes / scale | Status |
|---|---|---|---|
| `ENGINE_ACCELERATOR_PEDAL_POSITION` | EEC2 61443 | d[1] × 0.4 % | ok |
| `ENGINE_PERCENT_LOAD` | EEC2 | d[2] % | ok |
| `ENGINE_SPEED` | EEC1 61444 | d[3..4] LE × 0.125 rpm | ok |
| `ENGINE_PERCENT_TORQUE` | EEC1 | d[1] − 125 | suspect (SPN 513 is d[2]) |
| `ENGINE_ACTIVE_DTCS` | DM1 65226 | count, single-frame only, never cleared | partial |
| `ENGINE_COOLANT_TEMP` | ET1 65262 | d[0] − 40 °C | ok |
| `ENGINE_OIL_TEMP` | ET1 | d[2] − 40 | suspect (SPN 175 is d[2..3] 16-bit) |
| `ENGINE_OIL_PRESSURE` | EFL/P1 65263 | d[3] × 4 kPa | ok |
| `ENGINE_FUEL_RATE` | LFE 65266 | d[0..1] LE × 0.05 L/h | ok |
| `ENGINE_INTAKE_MANIFOLD_PRESSURE` | IC1 65270 | d[1] × 2 kPa | ok |
| `ENGINE_BATTERY_VOLTAGE` | VEP1 65271 | d[5..6] LE × 0.05 V | suspect (SPN 168 is d[4..5]) |

"Suspect" = differs from the standard SAE J1939 layout; verify against a candump
from the boat before changing.

## Bench tools (`scripts/`)

- `server_can_8.py` / `server_can_9.py` — standalone TCP:5000 → `cansend` bridge
  (legacy manual control, EEC2 throttle + bucket gear on `canb0`/`can0`). Not used
  by MOOS. **They keep transmitting after the TCP client disconnects** (missing `global`).
- `test_server_raw*.py`, `raspberry_client.py`, `loop_can.sh` — earlier bench tests.
- `can_documentation.txt` — notes from bus experiments (mostly an older jet/bucket setup).

Useful when debugging:

```bash
candump -tA slcan0                      # watch everything
candump slcan0,18F30A1A:1FFFFFFF        # steering only
uPokeDB real.moos CAN_SEND=false        # stop iCAN transmitting
```
