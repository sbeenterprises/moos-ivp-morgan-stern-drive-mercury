# Boat drive profiles

Each boat has one `.prf` file here. It holds everything about that boat's
CAN interface: IDs, byte layout, scaling, limits, shift timing, lost-link
values and the feedback to decode. `iCAN` contains no CAN IDs or payloads;
the mission file points it at a profile:

```
ProcessConfig = iCAN
{
  profile_file = ../../profiles/usv722_glendinning.prf
}
```

| File | Boat | Status |
|---|---|---|
| `mercury_sterndrive.prf` | Mercury stern-drive boat | Same frames as the old hardcoded iCAN (checked by the tests) |
| `usv722_glendinning.prf` | USV-722, Yamaha F115B, Glendinning helm | From the 9 Oct 2026 captures. Sends only while the helm is not sending the same ID |
| `tables/*.csv` | Lookup tables used by profiles | Mercury steering and RPM tables, generated from the old code |

## Adding a boat

1. Capture the bus following the USV-722 runbook (center/port/starboard,
   gear, throttle sweep). Note every ID, byte and scale.
2. Copy the closest `.prf` and change the values. Mark each one `[V]`
   (verified on the boat) or `[P]` (placeholder).
3. Run `tests/run_tests.sh`, then check the frames on a virtual bus before
   the boat: `sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up`,
   start the mission with `can_interface = vcan0` in the iCAN block and
   watch `candump vcan0`.
4. Dock test with the boat tied up, engine off, per the runbook.

If iCAN finds anything wrong in the profile it does not transmit at all.
The appcast (`uMAC`) and `CAN_STATUS=INHIBITED` show why.

## File format

One `key = value` per line. `//` starts a comment.

### Boat settings

| Key | Default | Meaning |
|---|---|---|
| `profile` | required | Name, posted as `CAN_PROFILE` |
| `can_interface` | required | `slcan0`, `can0`, `vcan0`... (the iCAN block may override) |
| `capabilities` | none | `steer`, `thrust`. Messages of a capability not listed are never sent |
| `rudder_limit_pct` | 100 | Largest rudder command sent, % of travel |
| `thrust_limit_pct` | 100 | Largest throttle command sent, % |
| `thrust_deadband_pct` | 0 | Thrust within this of 0 counts as neutral |
| `shift_neutral_secs` | 0 | Neutral is held this long before a gear engages (reversing always passes through neutral) |
| `shift_settle_secs` | 0 | Throttle stays at 0 this long after a gear engages |
| `rudder_rate_pct_s`, `thrust_rate_pct_s` | 0 (off) | Rate limits |
| `lost_link_rudder_pct`, `lost_link_thrust_pct` | 0 | Applied when the SS UI link is lost in manual control |
| `guard_secs` | 1.0 | Listen this long before sending; do not send an ID another node sent within this time |
| `rudder_feedback` | none | rx_field holding the actual rudder %, for the mismatch alarm |
| `rudder_alarm_pct`, `rudder_alarm_secs` | 0 (off), 3 | `CAN_RUDDER_MISMATCH=true` when feedback stays this far off for this long |

### Messages sent

```
tx_msg   = name=steer, cap=steer, id=18FF11D0, dlc=8, period_ms=130, fill=FF, guard=true
tx_field = msg=steer, src=rudder_pct, byte=0, bits=16, order=le, map=points, points=-100:0|0:511|100:1012
tx_const = msg=tsc1, byte=0, value=C1
```

`tx_msg`: `id` in hex (11-bit IDs need `ext=false`), `dlc` 1-8,
`period_ms` 10-1000, `fill` for unused bytes, `cap` = `steer` or `thrust`.

`tx_field` inputs (`src`):

| src | Values |
|---|---|
| `rudder_pct` | -100 (port) .. +100 (starboard) |
| `rudder_side` | `port`, `center`, `stbd` (from the truncated rudder %) |
| `thrust_pct` | -100 (astern) .. +100 (ahead) |
| `thrust_abs_pct` | 0 .. 100 |
| `gear` | `fwd`, `neutral`, `rev` (after the shift interlock) |

Mappings (`map`):

| map | Parameters | Result |
|---|---|---|
| `linear` | `scale`, `offset` | round(input × scale + offset) |
| `points` | `points=x:y\|x:y...` | straight lines between points, rounded |
| `table` | `table=tables/file.csv` | row for the input truncated to an integer (`input,raw` rows) |
| `enum` | `values=state:hex\|...` | fixed value per state |

`bits` is 8/16/24/32, `order` is `le` (low byte first, default) or `be`,
`signed=true` for two's complement. Values are clamped to the field width.

### Feedback decoded

```
rx_field = name=CAN_RUDDER_ACTUAL_PCT, id=210, ext=false, parts=B3:0-7|B6:2-3, map=points, points=0:-100|511:0|1012:100
```

`parts` lists byte and bit ranges, least significant first (`B4` is a whole
byte). `map` is `raw`, `linear`, `points` or `enum` (`values=hex:text|...`).
Each value is posted to MOOS under `name` on change and once a second.

### Validation

Rejected, with the line number: unknown keys, missing IDs, two messages on
one ID, a byte written twice, a field past the DLC, a value that does not
fit its bits, points or tables that do not cover the whole input range, an
enum missing a state, a capability with no message, a missing table file.
