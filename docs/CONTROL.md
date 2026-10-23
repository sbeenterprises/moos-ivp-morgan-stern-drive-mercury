# Roles and exclusive control

Who may command the boat, and how control passes between operators. Enforced
on the boat by iWebSocket (`src/iWebSocket`, lease model in `ControlLease.cpp`,
unit tests in `tests/controllease_test.cpp`); the SS UI shows it and disables
what a session may not do.

Related: [UI_PROTOCOL.md](UI_PROTOCOL.md) (messages), [KNOWN_ISSUES.md](KNOWN_ISSUES.md).

## Where

- C2 (the operator UI): `http://<boat>:3000/c2/` (USV-722 `192.168.22.11`, USV-725
  `192.168.143.251`). With `"landingPage": "/c2/"` in Signal K's `~/.signalk/settings.json`
  the bare `http://<boat>:3000/` opens it too.
- Roles: Signal K admin, `http://<boat>:3000/admin/`.
- Signal K on the boats runs as root (`signalk.service`, `User=root`): its config folder
  is `/root/.signalk` — that is where C2 is registered and where `landingPage` goes,
  not `/home/douglas/.signalk`.

## Switches (iWebSocket block of the mission file)

| Parameter | Default | Meaning |
|---|---|---|
| `command_auth` | `off` | `signalk`: each UI connection's user is checked with the Signal K server (`signalk_http`, host:port). Read-only and anonymous users get telemetry only. |
| `signalk_http` | `127.0.0.1:3000` | Signal K, plain HTTP, `host:port` (a `.moos` file reads `//` as a comment). |
| `exclusive_control` | `false` | `true` (needs `command_auth = signalk`): one session at a time holds control. |
| `roles_file` | none | `/var/lib/swiftships-c2/roles.json` in the real missions: written by the SwiftShips C2 plugin whenever Signal K starts it or its settings are saved (`{"configuration": {"roles": [{user, role}]}}`, readable by all). Signal K runs as root on the boats, and its own config folder (`/root/.signalk`) is not readable by the MOOS user. |
| `control_request_timeout` | 15 s | An unanswered request for control hands it over. |
| `audit_dir` | none | Folder for the daily audit file (see Audit); `/home/douglas/c2_audit` in the real missions. |
| `link_timeout` / `autonomy_link_timeout` | 3 s / 5 s | Unchanged; with exclusive control they apply to the holder only, and the control lease expires with `autonomy_link_timeout`. |

The real missions ship with `command_auth = off` and `exclusive_control = false`
(behaviour as before). Turn them on per boat once Signal K security is checked.

## Roles

Assigned in the Signal K admin page: Server → Plugin Config → SwiftShips C2 →
"Boat control roles". Signal K's own Admin / Read-Write rights do not give boat
control by themselves; account management stays in Signal K.

| Role | May |
|---|---|
| viewer | watch: map, telemetry, routes, mission status (the operator's selections are mirrored live) |
| planner | viewer + draw, edit and save draft routes / waypoints / notes in Signal K; never commands the boat |
| operator | planner + take control (one at a time) and command while holding it; ALL STOP at any time |
| supervisor | operator + force a takeover (with a logged reason) |

Users not listed: Signal K read-only → viewer, read/write → operator,
admin → supervisor. A read-only Signal K account is always a viewer.

## Control

A **session** is one browser tab (an id the UI makes per page load). Another
tab or device of the same account, or a reload, is a new session and does not
inherit control.

| State | Meaning |
|---|---|
| `held` | one operator session has control; only it may command |
| `free` | nobody has control (start-up, release, logout, MOOS link restarted) |
| `lost` | the holder was not heard for `autonomy_link_timeout`; other operators are asked to take over |

Transfers:

- **Take** (free or lost): the first operator to ask gets it; others are refused
  ("controlled by …") — simultaneous requests are decided atomically.
- **Release**: only while the boat is idle (no mission, manual control, station
  keep or follow). While it is busy, control is handed over instead.
- **Request**: the holder is asked to hand over or keep control; with no answer
  in `control_request_timeout` control passes to the requester.
- **Force**: a supervisor takes control at once; the reason is logged.
- **Logout** of the holder frees control.
- **MOOS link restart** (iWebSocket reconnects to the MOOSDB): control is
  revoked; an operator must take it again. While iWebSocket's MOOS link is down
  every command is refused and the UI shows "MOOS Disconnected".
- **iWebSocket restart**: starts with nobody in control (nothing is restored).

Every change of holder bumps a generation number. A command queued or delayed
under an older generation (sent just before a takeover) is dropped, not carried
out (`UI_AUTH_EVENT … why=stale`).

A change of control never starts, stops or changes a mission by itself.

## ALL STOP

Any operator or supervisor may press it, holding control or not. It ends a
running mission, station keep or follow (as Stop does), sets rudder and thrust
to 0, posts `ALL_STOP=true` and latches: starts (mission, return, station keep,
follow) and non-zero thrust are refused until the holder resets it. It does not
transfer control. It is a software stop over the same network as everything
else; it does not replace a hardwired or independent radio kill switch.

## What the boat does while no operator holds control

Unchanged vessel behaviour — exclusive control only decides whose link counts.
`UI_LINK_OK` / `UI_LINK_OK_AUTO` are true only while a holder is heard, so when
the holder is lost, logs out or releases:

| Situation | Boat |
|---|---|
| manual control, holder not heard > `link_timeout` (3 s) | iCAN: rudder 0, throttle 0, neutral (profile `lost_link_*`); stays at 0 until a new holder commands |
| mission / station keep / follow, holder not heard > `autonomy_link_timeout` (5 s), `lost_link_autonomy = stop` | mission ended (`LOST_LINK_STOP`), neutral; not resumed — the new holder starts it again |
| idle, nobody in control | nothing moves; planchaPID gives neutral with no helm decision |
| ALL STOP latched | as above, starts refused until reset |

A handover (request accepted or timed out, force) keeps the link up — the new
holder's traffic counts from the moment it takes over — so a running mission
continues. A viewer or planner left connected does not keep the link up.

## Audit

**Audit file** (`audit_dir`, `/home/douglas/c2_audit` on the boats): one line
per event, UTC time, a file per UTC day (`c2_audit_YYYY-MM-DD.log`), appended
as it happens and kept across mission restarts. Example:

```
2026-10-23T03:50:00Z auth login user=alice level=readwrite role=operator ip=192.168.22.5 session=k3f9a1x0
2026-10-23T03:50:01Z control take user=alice role=operator gen=1
2026-10-23T03:50:02Z control request user=frank holder=alice
2026-10-23T03:50:03Z control handover user=frank role=operator gen=2 from=alice
2026-10-23T03:50:05Z control release user=frank
2026-10-23T03:50:09Z control force user=carol role=supervisor gen=5 from=alice
2026-10-23T03:50:09Z control force reason=operator not answering
2026-10-23T03:50:11Z control all_stop user=carol
2026-10-23T03:50:12Z auth logout user=carol ip=192.168.22.5 session=q81mz2c4
2026-10-23T03:50:13Z auth rejected key=DESIRED_THRUST user=bob level=readonly why=level
2026-10-23T03:50:14Z auth disconnect user=bob role=viewer ip=192.168.22.7 session=t0b6e9d2
```

`auth`: login, logout, anonymous (connected without a login), disconnect,
rejected (with why: level, role, control, moos, stale, all_stop).
`control`: start (iWebSocket started, nobody in control), take, release,
request, deny, cancel, handover, handover-timeout, force + reason, lost (holder
not heard), logout, revoke (MOOS link restarted), refused (a control action
that was not allowed), all_stop, all_stop_reset.

The same lines are posted to MOOS as `CONTROL_EVENT` / `UI_AUTH_EVENT`, so they
are also in each mission's pLogger log next to what the boat did.

## Not covered (yet)

- MOOSDB (port 9000) and port 8000 are still reachable on the boat network:
  pMarineViewer, uPokeDB or a direct client bypass the UI. Firewall them; give
  remote viewers a one-way `pShare` copy.
- Signal K resource writes (routes, regions) are checked by Signal K's own
  permissions, not by the control lease. An active mission has its own copy of
  the route on the boat; only the holder can push live route edits.
- One boat per iWebSocket; no fleet-wide control.
