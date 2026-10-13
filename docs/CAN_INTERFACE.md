# CAN / J1939 interface

Bus: SocketCAN `slcan0`, 250 kbit/s, brought up with

```bash
sudo slcand -o -c -s5 /dev/ttyACM2 slcan0
sudo ip link set slcan0 up
```

All frames use 29-bit extended IDs.

## Commands out (iCAN, `src/iCAN/CAN.cpp`)

Sent every `Iterate` (20 Hz on the boat) **only while `CAN_SEND == "true"`**.
The rudder frame is also sent on every `DESIRED_RUDDER` mail.

| CAN ID | Meaning | Payload |
|---|---|---|
| `18F30A1A` | Steering command (PGN 0xF30A, SA 0x1A; proprietary — Mercury, presumed) | Bytes 0–1 big-endian int16 position from lookup table indexed by `(int)DESIRED_RUDDER` clamped to ±100 (**percent of travel**). Negative = port. Centre = `0000`. Tail bytes differ for port (`FFFFFFFFE5`) vs starboard (`FFFFFF1AFD`). |
| `18F005D0` | ETC2 (PGN 61445) selected gear | `7E` forward if thrust > 0, `7D` neutral if thrust == 0, `7C` reverse if thrust < 0; rest `FF`. |
| `18000017` | TSC1 (PGN 0) to engine #1, SA 0x17 | `C1` (speed-control override), bytes 1–2 requested speed LE 0.125 rpm/bit, `E1`, `FF…`. Table indexed by `(int)|thrust|`: 0 % = 700 rpm … 100 % = 4200 rpm (≈ +35 rpm/%). |
| `0CFF08D0` | Trim (disabled in code) | `F1…` up / `F4…` down, 12× at 100 ms |

Gear is decided purely from the sign of thrust — no shift interlock or dwell.
`DESIRED_GEAR` from the UI is not consumed.

Table defects: 71 % equals 50 % (`CAN.cpp:499`, `:520`); 23–25 % is
non-monotonic. See [KNOWN_ISSUES.md](KNOWN_ISSUES.md).

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
