# Development guide

## Branches

| Branch | Use |
|---|---|
| `main` | What runs on the boat. Only merge tested work. |
| `dev` | Integration branch for local development. Branch features from here. |
| `fix/*`, `feat/*` | Short-lived topic branches off `dev`; PR back into `dev`. |

Other remote branches (as of Oct 2026): `fix/pid-speed-integral-rudder-rounding`
(1 commit ahead of main — touches the speed-loop issue in KNOWN_ISSUES #8),
`dev-testing`, `sensorfusion_test`, `ais_collision_avoidance`, `stationkeep_onepoint`
(all well behind main).

Typical flow:

```bash
git switch dev && git pull
git switch -c fix/manual-override-typo
# ... edit, build, test in sim ...
git push -u origin fix/manual-override-typo   # open PR into dev
```

## Prerequisites (Ubuntu 22.04 / WSL)

- MOOS-IvP built from source at `~/moos-ivp` (or `../moos-ivp`); CMake searches
  `../moos-ivp`, `../../moos-ivp`, `/home/usv/moos-ivp`, `/home/d2snc/moos-ivp`.
  The core must be built with `USE_UTM=ON` (same as this repo).
- Packages:
  ```bash
  sudo apt install build-essential cmake libboost-all-dev libcurl4-openssl-dev \
                   libjsoncpp-dev libssl-dev zlib1g-dev libeigen3-dev can-utils
  ```
- [IXWebSocket](https://github.com/machinezone/IXWebSocket) (iWebSocket, aissim):
  ```bash
  git clone https://github.com/machinezone/IXWebSocket && cd IXWebSocket
  mkdir build && cd build && cmake -DUSE_TLS=1 .. && make -j && sudo make install
  ```
- Optional: OpenCV + OpenVINO for `pVisionDetector_ref` (not in the default build).

## Build

```bash
./build.sh            # -j$(nproc) etc. are passed through to make
./build.sh -k         # keep going past a failing app
./clean.sh            # remove build/, bin/, lib/
```

Outputs: `bin/` (apps, `aissim`), `lib/` (behavior `.so`s). Add to your shell:

```bash
export PATH=$PATH:$HOME/realboat/moos-ivp-morgan-stern-drive-mercury/bin
export IVP_BEHAVIOR_DIRS=$HOME/realboat/moos-ivp-morgan-stern-drive-mercury/lib
```

If `IVP_BEHAVIOR_DIRS` is set, comment out `ivp_behavior_dir` in the mission's
pHelmIvP block (both together is a fatal helm error).

## Run locally (no boat)

**Option A — pure MOOS sim:**
```bash
cd missions/s1_morgan_city_signalk
pAntler simulated.moos
```
uSimMarineV22 replaces the boat; pTrafficSim spawns AIS targets; pColavEval
reports metrics. The UI can connect to `ws://localhost:8000`. A Signal K server
on UDP 10112 is needed only to see the boat on the UI map.

**Option B — real data path with aissim** (exercises iDivisorNMEA/Signal K):
1. Run Signal K with the udp-nmea-sender plugin → UDP 10111, NMEA input on UDP 10112.
2. `pAntler external.moos`
3. `../../bin/aissim ../../tools/aissim/aissim.cfg`

Monitor with `uMAC simulated.moos` / `uXMS`, poke with `uPokeDB`, e.g.
`uPokeDB simulated.moos CAN_SEND=true`.

## Conventions

- New MOOS apps: copy an existing AppCasting app layout (`X.cpp/.h`, `X_Info.cpp/.h`,
  `main.cpp`, `CMakeLists.txt`) and add `ADD_SUBDIRECTORY` in `src/CMakeLists.txt`.
- Read origins from `LatOrigin/LongOrigin` in the mission file, not literals.
- Honour `#ifdef USE_UTM` when converting lat/lon ↔ x/y.
- `NODE_REPORT` SPD must be m/s.
- Don't commit build output, logs or credentials (see `.gitignore`).

## Docs

[ARCHITECTURE](ARCHITECTURE.md) · [CODE_FLOW](CODE_FLOW.md) · [APPS](APPS.md) ·
[MISSIONS](MISSIONS.md) · [UI_PROTOCOL](UI_PROTOCOL.md) · [CAN_INTERFACE](CAN_INTERFACE.md) ·
[KNOWN_ISSUES](KNOWN_ISSUES.md)
