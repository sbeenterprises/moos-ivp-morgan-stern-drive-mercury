# CAN / J1939 interface

Bus: SocketCAN `slcan0`, 250 kbit/s, brought up with

```bash
sudo slcand -o -c -s5 /dev/ttyACM2 slcan0
sudo ip link set slcan0 up
```

All Mercury frames use 29-bit extended IDs; USV-722 steering feedback is 11-bit.

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
  the operator sends a new command.
- **Shift interlock.** Reversing passes through neutral for
  `shift_neutral_secs`; throttle stays at 0 for `shift_settle_secs` after a
  gear engages. Both are 0 on the Mercury boat (unchanged behaviour).
- **Guard.** iCAN listens `guard_secs` before sending and will not send an ID
  another node is sending (`CAN_CONFLICT`). On USV-722 the helm must have its
  steering switched off before the PC can steer.
- **CAN_SEND.** `false` sends one neutral frame of each message and then
  stops; `true` starts again from neutral.

### Mercury stern-drive (`profiles/mercury_sterndrive.prf`)

Frames are byte-for-byte those of the old hardcoded iCAN
(`tests/canprofile_test.cpp` compares 3,600+ frames).

| CAN ID | Meaning | Payload |
|---|---|---|
| `18F30A1A` | Steering (PGN 0xF30A, SA 0x1A; source of this format unconfirmed) | DLC 7. Bytes 0–1 big-endian int16 from `tables/mercury_steer.csv`, indexed by truncated `DESIRED_RUDDER` (% of travel). Bytes 5–6 `FF E5` to port, `1A FD` centre/starboard. |
| `18F005D0` | ETC2 (PGN 61445) selected gear | `7E` forward, `7D` neutral, `7C` reverse; rest `FF`. |
| `18000017` | TSC1 (PGN 0) to engine #1, SA 0x17 | `C1`, bytes 1–2 requested speed LE 0.125 rpm/bit from `tables/mercury_rpm.csv` (700–4200 rpm), `E1`, `FF…`. |

Table defects kept on purpose (fix in the CSV, not in code): 71 % equals
50 %; 23–27 % is non-monotonic. Trim (`0CFF08D0`) is not in the profile.

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
Trim and the key switch are deliberately not sent.

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
