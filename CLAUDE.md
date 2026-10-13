# CLAUDE.md

MOOS-IvP backend for the SwiftShips USV (Mercury / Morgan Stern drive).
C++11, CMake, Ubuntu 22.04. Full docs in [docs/](docs/ARCHITECTURE.md).

## Build & run
- Build: `./build.sh -j$(nproc)` → `bin/`, `lib/`. Needs moos-ivp at `../moos-ivp` or `~/moos-ivp`, IXWebSocket, jsoncpp, curl, boost, Eigen3.
- Sim: `cd missions/s1_morgan_city_signalk && pAntler simulated.moos` (not `launch.sh` — it targets a missing `alpha.moos`).
- Boat: `pAntler real.moos`. Monitor: `uMAC <file>.moos`.
- Unit tests only for lib_canprofile (`tests/run_tests.sh`); otherwise verify in `simulated.moos` / `external.moos` + `tools/aissim`.

## Map
- Control chain: UI → `iWebSocket` (ws :8000) → mode flags / `WPT_UPDATE` → `pHelmIvP` (`alpha.bhv`, `BHV_Waypoint_mod` in `src/lib_behaviors-test`) → `planchaPID` → `iCAN` (J1939, `slcan0`).
- Nav: Signal K → UDP 10111 → `iDivisorNMEA` (NAV_LAT/LONG/HEADING/SPEED) → `pNodeReporter` → `pXYpublisher` (NAV_X/Y).
- Contacts: `iDivisorNMEA` (AIS), `iRadarTargets` (UDP 8091), `pTrafficSim` (sim) → `NODE_REPORT` → `pContactMgrV20` → `BHV_AvdColregsV22`; `pColavEval` scores encounters.
- Telemetry: `iCANRec` → `ENGINE_*` → `iWebSocket` JSON.
- Details: docs/CODE_FLOW.md, docs/APPS.md, docs/UI_PROTOCOL.md, docs/CAN_INTERFACE.md.

## Gotchas
- Real-boat `NAV_SPEED` is **knots**; sim is m/s. `NODE_REPORT` SPD should be m/s (radar/fusion currently send knots).
- `DESIRED_RUDDER` is ±% of steering travel in iCAN, though planchaPID calls it degrees.
- CAN IDs/payloads live in `profiles/<boat>.prf` (lib_canprofile), never in iCAN. Mercury boat: `mercury_sterndrive.prf`; USV-722: `usv722_glendinning.prf`. Run `tests/run_tests.sh` after touching either (needs only g++).
- Manual control (SS UI): only iWebSocket may set rudder/thrust; values latch; lost UI link -> 0.
- Origin (Morgan City) is hardcoded in iWebSocket/iSignalK/aissim as well as in `.moos` files. `USE_UTM=ON` must match the moos-ivp core.
- Helm modes are flat flags: `WAYPOINT`, `STATIONING`, `REGION`, `TRAILING`. planchaPID only publishes when one is true (see gate in `lanchaPID.cpp`).
- `CONTROLE_MANUAL` now posts `MOOS_MANUAL_OVERRIDE`; `AUTONOMOUS_CONTROL` still posts the old `MOOS_MANUAL_OVERIDE` typo (no consumer) — see docs/KNOWN_ISSUES.md before touching control code.
- Code comments are partly Portuguese.
- Never commit credentials, `build/`, logs. Work on `dev` or topic branches, not `main`.
