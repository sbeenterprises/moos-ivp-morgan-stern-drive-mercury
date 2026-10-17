#!/usr/bin/env python3
"""
score_alog.py - route-tracking scorer for MOOS-IvP .alog files (Swiftships USV).

Reconstructs every waypoint leg the helm actually ran (from VIEW_SEGLIST and
WPT_STAT), then scores the vessel's track against the planned line:

  cross-track error  (mean |XTE|, 95th pct, max, signed bias)
  heading error      (RMS on straights, overshoot and settling after each turn)
  waypoint arrival   (distance to the waypoint when the helm advanced)
  rudder             (activity deg/s, saturation %, cmd vs actual lag/offset)

Usage:  score_alog.py RUN.alog [--out DIR] [--rudder-limit 20] [--plot]
"""
import argparse, json, math, os, re, sys
from collections import defaultdict

# ----------------------------------------------------------------------------
# alog parsing
# ----------------------------------------------------------------------------
VARS = {"NAV_X","NAV_Y","NAV_HEADING","NAV_SPEED","NAV_COG","GPS_HEADING",
        "DESIRED_HEADING","DESIRED_SPEED","DESIRED_RUDDER","DESIRED_THRUST",
        "CAN_RUDDER_ACTUAL_PCT","CAN_RUDDER_ECHO_PCT","CAN_TX_RUDDER_PCT",
        "CAN_RUDDER_LIMIT_PCT","WAYPOINT","WPT_STAT","VIEW_SEGLIST","WPT_UPDATE",
        "PID_GAINS","MOOS_MANUAL_OVERRIDE","IVPHELM_ALLSTOP","STATIONING","TRAILING"}

def parse(path):
    series = defaultdict(list)          # var -> [(t, value)]
    with open(path, errors="replace") as f:
        for line in f:
            if not line or line[0] in "%\n": continue
            parts = line.split(None, 3)
            if len(parts) < 4: continue
            var = parts[1]
            if var not in VARS: continue
            try: t = float(parts[0])
            except ValueError: continue
            val = parts[3].rstrip("\n")
            if var in ("WPT_STAT","VIEW_SEGLIST","WPT_UPDATE","PID_GAINS",
                       "WAYPOINT","MOOS_MANUAL_OVERRIDE","IVPHELM_ALLSTOP",
                       "STATIONING","TRAILING"):
                series[var].append((t, val.strip()))
            else:
                try: series[var].append((t, float(val)))
                except ValueError: pass
    return series

def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0

def interp(ts, t):
    """ts: sorted [(t,v)] numeric; last-value hold at time t (step)."""
    lo, hi = 0, len(ts)
    while lo < hi:
        mid = (lo + hi)//2
        if ts[mid][0] <= t: lo = mid + 1
        else: hi = mid
    return ts[lo-1][1] if lo > 0 else None

# ----------------------------------------------------------------------------
# geometry
# ----------------------------------------------------------------------------
def seg_xte(px, py, ax, ay, bx, by):
    """Signed cross-track distance of P from directed line A->B.
    Positive = starboard of the line (right of direction of travel, MOOS x-east/y-north)."""
    dx, dy = bx-ax, by-ay
    L = math.hypot(dx, dy)
    if L < 1e-6: return math.hypot(px-ax, py-ay), 0.0
    # cross product (AB x AP) / |AB|; sign flipped so starboard is positive
    cross = (dx*(py-ay) - dy*(px-ax)) / L
    along = ((px-ax)*dx + (py-ay)*dy) / L   # distance along the leg from A
    return -cross, along

def parse_pts(s):
    m = re.search(r"pts=\{([^}]*)\}", s)
    if not m: return None
    pts = []
    for p in m.group(1).split(":"):
        x, y = p.split(",")
        pts.append((float(x), float(y)))
    return pts

# ----------------------------------------------------------------------------
# leg reconstruction
# ----------------------------------------------------------------------------
def reconstruct_legs(S):
    """Returns list of legs: dict(t0,t1,A,B,index,route_id)."""
    # active route: last waypt_survey seglist that is not active=false
    routes = []
    for t, v in S["VIEW_SEGLIST"]:
        if "waypt_survey" not in v: continue
        pts = parse_pts(v)
        if not pts: continue
        active = "active=false" not in v
        routes.append((t, pts, active))

    # WAYPOINT true/false windows
    wp_on = []
    cur = None
    for t, v in S["WAYPOINT"]:
        if v == "true" and cur is None: cur = t
        elif v == "false" and cur is not None:
            wp_on.append((cur, t)); cur = None
    if cur is not None:
        wp_on.append((cur, S["NAV_X"][-1][0] if S["NAV_X"] else cur))

    # WPT_STAT index over time
    idx = []
    for t, v in S["WPT_STAT"]:
        m = re.search(r"index=(\d+)", v)
        if m: idx.append((t, int(m.group(1))))

    legs = []
    for (w0, w1) in wp_on:
        if w1 - w0 < 5: continue
        # walk index changes within the window
        in_win = [(t, i) for t, i in idx if w0 <= t <= w1]
        if not in_win: continue
        # route in force at window start (latest active before/at w0, else latest)
        def route_at(t):
            best = None
            for rt, pts, act in routes:
                if rt <= t + 0.5 and act: best = (rt, pts)
            if best is None:
                for rt, pts, act in routes:
                    if rt <= t + 0.5: best = (rt, pts)
            return best
        # segment by (index, route) runs
        cur_i, t_start = in_win[0][1], w0
        cur_route = route_at(w0)
        for k in range(1, len(in_win)+1):
            if k < len(in_win) and in_win[k][1] == cur_i and route_at(in_win[k][0]) == cur_route:
                continue
            t_end = in_win[k][0] if k < len(in_win) else w1
            if cur_route and cur_i < len(cur_route[1]) and t_end - t_start > 3:
                pts = cur_route[1]
                B = pts[cur_i]
                if cur_i > 0: A = pts[cur_i-1]
                else:
                    x0 = interp(S["NAV_X"], t_start); y0 = interp(S["NAV_Y"], t_start)
                    A = (x0, y0) if x0 is not None else B
                legs.append(dict(t0=t_start, t1=t_end, A=A, B=B, index=cur_i,
                                 route_t=cur_route[0], npts=len(pts)))
            if k < len(in_win):
                cur_i, t_start = in_win[k][1], in_win[k][0]
                cur_route = route_at(in_win[k][0])
    return legs

# ----------------------------------------------------------------------------
# scoring
# ----------------------------------------------------------------------------
def pct(v, p):
    if not v: return float("nan")
    s = sorted(v); k = (len(s)-1)*p
    f = math.floor(k); c = min(f+1, len(s)-1)
    return s[f] + (s[c]-s[f])*(k-f)

def score_leg(S, leg, rudder_limit):
    t0, t1, A, B = leg["t0"], leg["t1"], leg["A"], leg["B"]
    leg_len = math.hypot(B[0]-A[0], B[1]-A[1])
    leg_brg = (math.degrees(math.atan2(B[0]-A[0], B[1]-A[1])) + 360) % 360
    xs = [(t, v) for t, v in S["NAV_X"] if t0 <= t <= t1]
    track, xte, hdg_err, spd, cog_err = [], [], [], [], []
    for t, x in xs:
        y = interp(S["NAV_Y"], t)
        if y is None: continue
        e, along = seg_xte(x, y, A[0], A[1], B[0], B[1])
        h = interp(S["NAV_HEADING"], t); dh = interp(S["DESIRED_HEADING"], t)
        v = interp(S["NAV_SPEED"], t); cog = interp(S["NAV_COG"], t)
        track.append((t, x, y, e, along))
        xte.append(e)
        if h is not None and dh is not None: hdg_err.append((t, wrap180(dh - h)))
        if v is not None: spd.append(v)
        if cog is not None: cog_err.append(wrap180(leg_brg - cog))
    if len(track) < 8: return None

    # ---- turn / settling analysis: first part of the leg after a course change
    settle_t, overshoot = float("nan"), float("nan")
    if hdg_err:
        e0 = abs(hdg_err[0][1])
        if e0 > 15:                      # there was a real course change
            # first zero crossing
            sign0 = 1 if hdg_err[0][1] > 0 else -1
            cross = next((i for i, (t, e) in enumerate(hdg_err) if e*sign0 <= 0), None)
            if cross is not None:
                over = [e*sign0 for t, e in hdg_err[cross:]]
                overshoot = -min(over) if over else 0.0
                # settled: |err| < 5 deg continuously for 5 s
                for i, (t, e) in enumerate(hdg_err):
                    if abs(e) < 5:
                        tt = t
                        ok = all(abs(ee) < 5 for t2, ee in hdg_err[i:] if t2 <= tt + 5)
                        if ok: settle_t = t - hdg_err[0][0]; break
    # "straight" portion = after settling (or after 25% of the leg if no turn)
    t_straight = hdg_err[0][0] + (settle_t if not math.isnan(settle_t) else 0)
    straight_err = [e for t, e in hdg_err if t >= t_straight]
    straight_xte = [tr[3] for tr in track if tr[0] >= t_straight]
    rms_hdg = math.sqrt(sum(e*e for e in straight_err)/len(straight_err)) if straight_err else float("nan")

    # ---- rudder
    rud = [(t, v) for t, v in S["DESIRED_RUDDER"] if t0 <= t <= t1]
    act = [(t, v) for t, v in S.get("CAN_RUDDER_ACTUAL_PCT", []) if t0 <= t <= t1]
    rud_rate, sat = float("nan"), float("nan")
    if len(rud) > 2:
        d = [abs(rud[i][1]-rud[i-1][1])/max(rud[i][0]-rud[i-1][0], 1e-3) for i in range(1, len(rud))]
        rud_rate = sum(d)/len(d)
        sat = 100.0*sum(1 for t, v in rud if abs(v) >= rudder_limit - 0.5)/len(rud)
    rud_vs_act = float("nan")
    if act and rud:
        diffs = []
        for t, a in act:
            c = interp(rud, t)
            if c is not None: diffs.append(a - c)
        if diffs: rud_vs_act = sum(diffs)/len(diffs)

    # ---- arrival: distance to B at leg end
    xe, ye = track[-1][1], track[-1][2]
    arrival = math.hypot(xe-B[0], ye-B[1])
    abs_xte = [abs(e) for e in straight_xte] or [abs(e) for e in xte]
    return dict(
        index=leg["index"], t0=round(t0,1), t1=round(t1,1), dur_s=round(t1-t0,1),
        leg_len_m=round(leg_len,1), leg_brg=round(leg_brg,1),
        speed_mean_kn=round(sum(spd)/len(spd),1) if spd else float("nan"),
        xte_mean_m=round(sum(abs_xte)/len(abs_xte),1),
        xte_p95_m=round(pct(abs_xte,0.95),1), xte_max_m=round(max(abs_xte),1),
        xte_bias_m=round(sum(straight_xte)/len(straight_xte),1) if straight_xte else float("nan"),
        hdg_rms_deg=round(rms_hdg,1),
        turn_deg=round(abs(hdg_err[0][1]),0) if hdg_err else float("nan"),
        overshoot_deg=round(overshoot,1), settle_s=round(settle_t,1),
        cog_vs_leg_deg=round(sum(cog_err)/len(cog_err),1) if cog_err else float("nan"),
        rudder_rate_dps=round(rud_rate,2), rudder_sat_pct=round(sat,1),
        rudder_act_minus_cmd=round(rud_vs_act,2),
        arrival_m=round(arrival,1), n=len(track),
        track=track, A=A, B=B)

def summarize(legs):
    good = [l for l in legs if l and l["leg_len_m"] > 30 and l["dur_s"] > 10]
    if not good: return {}
    allx = []
    for l in good: allx += [abs(t[3]) for t in l["track"]]
    def mean(k):
        v = [l[k] for l in good if not math.isnan(l[k])]
        return round(sum(v)/len(v),1) if v else float("nan")
    return dict(legs=len(good), xte_mean_m=round(sum(allx)/len(allx),1),
                xte_p95_m=round(pct(allx,0.95),1), xte_max_m=round(max(allx),1),
                hdg_rms_deg=mean("hdg_rms_deg"), overshoot_deg=mean("overshoot_deg"),
                settle_s=mean("settle_s"), arrival_m=mean("arrival_m"),
                rudder_rate_dps=mean("rudder_rate_dps"), rudder_sat_pct=mean("rudder_sat_pct"),
                speed_mean_kn=mean("speed_mean_kn"))

# ----------------------------------------------------------------------------
def plot(run, legs, S, out):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    good = [l for l in legs if l and l["leg_len_m"] > 30]
    if not good: return
    fig, ax = plt.subplots(figsize=(11, 9))
    ax.set_facecolor("#0b1624"); fig.patch.set_facecolor("#0b1624")
    for sp in ax.spines.values(): sp.set_color("#1f3147")
    ax.tick_params(colors="#8aa3bd"); ax.grid(color="#16263a")
    # full track (all NAV)
    tx = [v for t, v in S["NAV_X"]]; ty = [interp(S["NAV_Y"], t) for t, v in S["NAV_X"]]
    ax.plot(tx, ty, color="#334b66", lw=0.8, label="all track")
    for l in good:
        A, B = l["A"], l["B"]
        ax.plot([A[0], B[0]], [A[1], B[1]], "--", color="#38bdf8", lw=1.6)
        ax.plot([t[1] for t in l["track"]], [t[2] for t in l["track"]], color="#22c55e", lw=1.6)
        ax.plot(B[0], B[1], "o", mfc="#0b1624", mec="#38bdf8", ms=7)
        k = max(l["track"], key=lambda t: abs(t[3]))
        ax.annotate(f"leg {l['index']}: p95 {l['xte_p95_m']} m, max {l['xte_max_m']} m",
                    (k[1], k[2]), color="#fbbf24", fontsize=8)
    ax.plot([], [], "--", color="#38bdf8", label="planned legs")
    ax.plot([], [], color="#22c55e", label="track while WAYPOINT=true")
    ax.set_aspect("equal"); ax.legend(facecolor="#0e1c2e", labelcolor="#e6eef7", edgecolor="#1f3147")
    ax.set_title(f"{run}: planned route vs track", color="#e6eef7")
    ax.set_xlabel("x east (m)", color="#8aa3bd"); ax.set_ylabel("y north (m)", color="#8aa3bd")
    fig.tight_layout(); fig.savefig(os.path.join(out, run + "_route_vs_track.png"), dpi=130)
    plt.close(fig)

    # XTE and heading error timeline
    fig, axs = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
    fig.patch.set_facecolor("#0b1624")
    for a in axs:
        a.set_facecolor("#0b1624"); a.tick_params(colors="#8aa3bd"); a.grid(color="#16263a")
        for sp in a.spines.values(): sp.set_color("#1f3147")
    for l in good:
        axs[0].plot([t[0] for t in l["track"]], [t[3] for t in l["track"]], color="#22c55e", lw=1)
    axs[0].axhline(0, color="#1f3147"); axs[0].set_ylabel("XTE m (+stbd)", color="#8aa3bd")
    he = [(t, wrap180(interp(S["DESIRED_HEADING"], t) - h)) for t, h in S["NAV_HEADING"]
          if interp(S["DESIRED_HEADING"], t) is not None and any(l["t0"] <= t <= l["t1"] for l in good)]
    axs[1].plot([t for t, e in he], [e for t, e in he], color="#38bdf8", lw=0.8)
    axs[1].set_ylabel("heading err deg", color="#8aa3bd"); axs[1].set_ylim(-60, 60)
    rd = [(t, v) for t, v in S["DESIRED_RUDDER"] if any(l["t0"] <= t <= l["t1"] for l in good)]
    axs[2].plot([t for t, v in rd], [v for t, v in rd], color="#fbbf24", lw=0.7, label="DESIRED_RUDDER")
    ra = [(t, v) for t, v in S.get("CAN_RUDDER_ACTUAL_PCT", []) if any(l["t0"] <= t <= l["t1"] for l in good)]
    if ra: axs[2].plot([t for t, v in ra], [v for t, v in ra], color="#f87171", lw=0.7, label="CAN_RUDDER_ACTUAL_PCT")
    axs[2].set_ylabel("rudder", color="#8aa3bd"); axs[2].set_xlabel("t (s)", color="#8aa3bd")
    axs[2].legend(facecolor="#0e1c2e", labelcolor="#e6eef7", edgecolor="#1f3147", fontsize=8)
    axs[0].set_title(f"{run}: cross-track, heading error, rudder", color="#e6eef7")
    fig.tight_layout(); fig.savefig(os.path.join(out, run + "_timeline.png"), dpi=130)
    plt.close(fig)

# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("alog"); ap.add_argument("--out", default="."); ap.add_argument("--rudder-limit", type=float, default=40)
    ap.add_argument("--plot", action="store_true"); ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    S = parse(a.alog)
    run = os.path.basename(a.alog).replace(".alog", "")
    legs = reconstruct_legs(S)
    scored = [score_leg(S, l, a.rudder_limit) for l in legs]
    summ = summarize(scored)
    gains = sorted(set(v for t, v in S["PID_GAINS"]))
    os.makedirs(a.out, exist_ok=True)
    print(f"== {run}  legs={len(legs)}  duration={S['NAV_X'][-1][0] if S['NAV_X'] else 0:.0f}s")
    for g in gains[:6]: print("   gains:", g)
    hdr = ("idx  t0      dur   len_m  spd  xte_mean p95   max   bias  hdg_rms turn  over  settle arr_m rud_dps sat%  act-cmd")
    print(hdr)
    for l in scored:
        if not l or l["leg_len_m"] < 30: continue
        print(f"{l['index']:>3} {l['t0']:>7} {l['dur_s']:>5} {l['leg_len_m']:>6} {l['speed_mean_kn']:>4} "
              f"{l['xte_mean_m']:>8} {l['xte_p95_m']:>5} {l['xte_max_m']:>5} {l['xte_bias_m']:>5} "
              f"{l['hdg_rms_deg']:>7} {l['turn_deg']:>5} {l['overshoot_deg']:>5} {l['settle_s']:>6} "
              f"{l['arrival_m']:>5} {l['rudder_rate_dps']:>7} {l['rudder_sat_pct']:>5} {l['rudder_act_minus_cmd']:>7}")
    print("SUMMARY:", json.dumps(summ))
    if a.json:
        slim = [{k: v for k, v in l.items() if k != "track"} for l in scored if l]
        with open(os.path.join(a.out, run + "_score.json"), "w") as f:
            json.dump(dict(run=run, gains=gains, summary=summ, legs=slim), f, indent=1)
    if a.plot: plot(run, scored, S, a.out)

if __name__ == "__main__":
    main()
