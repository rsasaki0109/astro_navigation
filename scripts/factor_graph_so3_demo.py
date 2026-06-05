#!/usr/bin/env python3
"""Skyline Lock Phase 7 (2): lift the pose graph to SO(3) + a metric-scale state
-- an observability triptych on real lunar slopes.

Phase 7 (1) (`factor_graph_so2_demo.py`) promoted *yaw* to a graph state and
solved a nonlinear SO(2) factor graph. That assumed a planar world and a known
VO scale. A rover on real terrain pitches and rolls over crater walls, and its
visual odometry reports translation only up to an unknown metric scale. Lifting
the graph to full SO(3) attitude plus a global scale state forces three honest
questions -- one per class of degree of freedom -- and the answers are different:

  * roll / pitch  are GRAVITY-observable. An always-on accelerometer sees the
    gravity vector in the body frame, which pins tilt (2 of the 3 rotational
    DOF) at *every* pose -- even mid-blackout, with no star and no future
    anchor. So roll/pitch never really drift. The honest message: don't oversell
    "SO(3) attitude recovery" -- two of three axes were never the hard part.
    (The figure shows the counterfactual: kill the gravity factor and roll/pitch
    drift like yaw -- gravity is precisely what holds them.)

  * yaw  is the gravity-UNOBSERVABLE rotational DOF (rotation about the gravity
    axis leaves the accelerometer unchanged). Across a star-tracker blackout it
    is carried only by the VO gyro increment, which is biased, so it drifts. A
    star/Skyline fix that RESUMES after the blackout flows backward through the
    VO yaw-increment chain and corrects it -- batch smoothing, exactly as in
    Phase 7 (1). When the blackout ENDS the traverse there is no future anchor,
    so the joint solve does no better: the same cliff, now isolated to the one
    rotational DOF gravity cannot see.

  * metric scale  is observable only when absolute (Skyline) fixes BRACKET a VO
    chain -- the fixes pin the chain's true length, which recovers the scale.
    A traverse with no absolute fix at all leaves scale a pure gauge freedom:
    the whole map is similarity-ambiguous and scale is unrecoverable.

The solver is a self-contained Gauss-Newton on SO(3) x R^3 x R+ : an exp/log
retraction on each pose rotation, a single global log-scale, and a *generic
numerical Jacobian* so the factor set stays declarative (analytic SO(3)
Jacobians are verbose; this keeps the reference solver compact and obviously
correct -- the analytic/GTSAM path is the production route). Skyline fixes use
the real matcher and the real uniqueness margin -> sigma, same as Phase 5.

Conventions match skyline_lock_demo.py: world +X east, +Y north, +Z up; azimuth
0 = +Y (north), clockwise to +X, so a forward heading psi has flat-ground
direction (sin psi, cos psi, 0). Body axes are x=right, y=forward, z=up, so the
attitude error Log(R_true^T R_est) reads [pitch, roll, yaw].
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from skyline_lock_demo import build_dem  # noqa: E402
from factor_graph_fusion_demo import (  # noqa: E402
    make_truth_trajectory, true_yaw_series, SkylineLocalizer, margin_to_sigma,
)

DOWN = np.array([0.0, 0.0, -1.0])


# --------------------------------------------------------------------------- #
# SO(3) helpers                                                               #
# --------------------------------------------------------------------------- #
def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def Exp(phi):
    """so(3) -> SO(3) (Rodrigues)."""
    th = float(np.linalg.norm(phi))
    if th < 1e-9:
        return np.eye(3) + skew(phi)
    K = skew(phi / th)
    return np.eye(3) + math.sin(th) * K + (1.0 - math.cos(th)) * (K @ K)


def Log(R):
    """SO(3) -> so(3) (rotation vector)."""
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    th = math.acos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if th < 1e-9:
        return v / 2.0
    return th / (2.0 * math.sin(th)) * v


# --------------------------------------------------------------------------- #
# 3D truth on the DEM surface                                                 #
# --------------------------------------------------------------------------- #
class Surface:
    """Bilinear height + gradient-derived normal over a DEM (metres)."""

    def __init__(self, dem, px_to_m):
        self.dem = dem
        self.px = px_to_m
        self.size = dem.shape[0]

    def z(self, x, y):
        c = min(max(x / self.px, 0.0), self.size - 1.001)
        r = min(max(y / self.px, 0.0), self.size - 1.001)
        c0, r0 = int(c), int(r)
        fc, fr = c - c0, r - r0
        d = self.dem
        return float((1 - fr) * ((1 - fc) * d[r0, c0] + fc * d[r0, c0 + 1])
                     + fr * ((1 - fc) * d[r0 + 1, c0] + fc * d[r0 + 1, c0 + 1]))

    def normal(self, x, y):
        d = self.px
        zx = (self.z(x + d, y) - self.z(x - d, y)) / (2 * d)
        zy = (self.z(x, y + d) - self.z(x, y - d)) / (2 * d)
        n = np.array([-zx, -zy, 1.0])
        return n / np.linalg.norm(n)

    def body_frame(self, psi, x, y):
        """SO(3) world<-body: heading psi tilted onto the local surface."""
        fwd_flat = np.array([math.sin(psi), math.cos(psi), 0.0])
        up = self.normal(x, y)
        fwd = fwd_flat - np.dot(fwd_flat, up) * up
        fwd /= np.linalg.norm(fwd)
        right = np.cross(fwd, up)
        right /= np.linalg.norm(right)
        up = np.cross(right, fwd)
        return np.column_stack([right, fwd, up])


def make_truth_3d(surf, extent_m, n_poses, kind):
    xy = make_truth_trajectory(extent_m, n_poses, kind)
    yaw = true_yaw_series(xy)
    P = np.array([[x, y, surf.z(x, y)] for x, y in xy])
    R = np.stack([surf.body_frame(yaw[k], xy[k, 0], xy[k, 1]) for k in range(n_poses)])
    return P, R, xy, yaw


EY = np.array([0.0, 1.0, 0.0])
EZ = np.array([0.0, 0.0, 1.0])


def attitude_errs(R_est, R_truth):
    """Per-pose (tilt_deg, heading_deg) error -- a gravity-aligned split.

    tilt is the angle between the estimated and true body-up axes: exactly the
    quantity an accelerometer observes (gravity-observable, roll+pitch). heading
    is the azimuth error of the body-forward axis about the gravity axis: the
    rotation-about-vertical that gravity is blind to (the yaw DOF). This avoids
    the body-frame Log decomposition, whose components mix and blow up near 180
    deg for a tilted rover with a large heading error."""
    n = len(R_truth)
    tilt = np.zeros(n)
    head = np.zeros(n)
    for k in range(n):
        ue, ut = R_est[k] @ EZ, R_truth[k] @ EZ
        tilt[k] = math.degrees(math.acos(max(-1.0, min(1.0, float(ue @ ut)))))
        fe, ft = R_est[k] @ EY, R_truth[k] @ EY
        az_e = math.atan2(fe[0], fe[1])
        az_t = math.atan2(ft[0], ft[1])
        head[k] = abs(math.degrees(math.atan2(math.sin(az_e - az_t),
                                              math.cos(az_e - az_t))))
    return tilt, head


# --------------------------------------------------------------------------- #
# Sensors                                                                     #
# --------------------------------------------------------------------------- #
def simulate_sensors(P, R, n_poses, blackout, rng, *, scale_bias, gyro_bias_deg,
                     grav_sigma_deg, star_sigma_deg, vo_sigma_frac):
    """VO (scaled body translation + biased gyro increment), gravity (all
    poses), star attitude (healthy poses)."""
    s_true = 1.0 / (1.0 + scale_bias)
    vo_t = np.zeros((n_poses - 1, 3))
    vo_R = np.zeros((n_poses - 1, 3, 3))
    gb = np.radians(np.asarray(gyro_bias_deg, float))     # pitch, roll, yaw bias
    for k in range(n_poses - 1):
        t_body = R[k].T @ (P[k + 1] - P[k])
        vo_t[k] = (1 + scale_bias) * t_body + rng.normal(
            0.0, vo_sigma_frac * max(np.linalg.norm(t_body), 1.0), 3)
        dR_true = R[k].T @ R[k + 1]
        vo_R[k] = dR_true @ Exp(gb + rng.normal(0.0, math.radians(0.05), 3))
    star = {k: R[k] @ Exp(rng.normal(0.0, math.radians(star_sigma_deg), 3))
            for k in range(n_poses) if k not in blackout}
    grav = {k: R[k].T @ DOWN + rng.normal(0.0, math.radians(grav_sigma_deg), 3)
            for k in range(n_poses)}
    return dict(vo_t=vo_t, vo_R=vo_R, star=star, grav=grav, s_true=s_true)


def gravity_correct(R_pred, g_meas):
    """Rotate R_pred so its body-frame gravity matches g_meas, touching only
    tilt (roll/pitch) and leaving yaw about gravity unchanged."""
    a = R_pred.T @ DOWN
    a /= np.linalg.norm(a)
    b = g_meas / np.linalg.norm(g_meas)
    # want R_new^T DOWN = b, with R_new = R_pred @ Exp(w); so Exp(w)^T a = b,
    # i.e. Exp(w) maps b -> a: axis = b x a (not a x b).
    axis = np.cross(b, a)
    s = np.linalg.norm(axis)
    if s < 1e-9:
        return R_pred
    ang = math.atan2(s, float(np.dot(a, b)))
    return R_pred @ Exp(axis / s * ang)


def forward_filter(meas, P0, n_poses, fixes, *, use_grav=True):
    """Method A -- a causal forward filter (no scale estimation, no backward
    smoothing): star/gyro attitude with gravity tilt correction, VO dead
    reckoning at NOMINAL scale, absolute fixes blended in as they arrive."""
    R = np.zeros((n_poses, 3, 3))
    P = np.zeros((n_poses, 3))
    R[0] = meas["star"].get(0, np.eye(3))
    if use_grav:
        R[0] = gravity_correct(R[0], meas["grav"][0])
    P[0] = P0
    fix_by_pose = {k: (xy, sig) for (k, xy, sig) in fixes}
    for k in range(1, n_poses):
        if k in meas["star"]:
            R[k] = meas["star"][k]
        else:
            R[k] = R[k - 1] @ meas["vo_R"][k - 1]
        if use_grav:
            R[k] = gravity_correct(R[k], meas["grav"][k])
        P[k] = P[k - 1] + R[k - 1] @ (1.0 * meas["vo_t"][k - 1])
        if k in fix_by_pose:
            xy, _ = fix_by_pose[k]
            P[k, :2] = 0.5 * P[k, :2] + 0.5 * xy        # causal complementary blend
    return P, R


# --------------------------------------------------------------------------- #
# Joint SO(3) x R^3 x scale factor graph -- self-contained Gauss-Newton       #
# --------------------------------------------------------------------------- #
class So3ScaleGraph:
    """States: (p_k in R^3, R_k in SO(3)) per pose + one global scale s.
    Tangent layout: [dp_k(3), dphi_k(3)] per pose, then [ds] last."""

    def __init__(self, n, meas, prior_p, prior_R, fixes, *, sig_p, sig_rot,
                 sig_vo_p, sig_vo_rot, sig_grav, sig_star, use_grav=True,
                 lam0=1e-3):
        self.n = n
        self.m = meas
        self.prior_p = np.asarray(prior_p, float)
        self.prior_R = np.asarray(prior_R, float)
        self.fixes = fixes
        self.sig_p, self.sig_rot = sig_p, sig_rot
        self.sig_vo_p, self.sig_vo_rot = sig_vo_p, sig_vo_rot
        self.sig_grav, self.sig_star = sig_grav, sig_star
        self.use_grav = use_grav
        self.lam0 = lam0

    def residual(self, est):
        P, R, s = est["P"], est["R"], est["s"]
        m = self.m
        r = []
        r += list((P[0] - self.prior_p) / self.sig_p)
        r += list(Log(self.prior_R.T @ R[0]) / self.sig_rot)
        for k in range(self.n - 1):
            pred = P[k] + R[k] @ (s * m["vo_t"][k])
            r += list((pred - P[k + 1]) / self.sig_vo_p)
            predR = R[k] @ m["vo_R"][k]
            r += list(Log(predR.T @ R[k + 1]) / self.sig_vo_rot)
        if self.use_grav:
            for k in range(self.n):
                r += list((R[k].T @ DOWN - m["grav"][k]) / self.sig_grav)
        for k, Rs in m["star"].items():
            r += list(Log(Rs.T @ R[k]) / self.sig_star)
        for (k, xy, sig) in self.fixes:
            r += list((P[k, :2] - xy) / sig)
        return np.asarray(r)

    def retract(self, est, dz):
        P = est["P"].copy()
        R = est["R"].copy()
        for k in range(self.n):
            P[k] += dz[6 * k:6 * k + 3]
            R[k] = R[k] @ Exp(dz[6 * k + 3:6 * k + 6])
        return dict(P=P, R=R, s=est["s"] + dz[6 * self.n])

    def numjac(self, est, r0):
        nd = 6 * self.n + 1
        J = np.zeros((len(r0), nd))
        eps = 1e-6
        for j in range(nd):
            dz = np.zeros(nd)
            dz[j] = eps
            J[:, j] = (self.residual(self.retract(est, dz)) - r0) / eps
        return J

    def solve(self, est0, iters=25):
        est = dict(P=est0["P"].copy(), R=est0["R"].copy(), s=float(est0["s"]))
        lam = self.lam0
        hist = [dict(P=est["P"].copy(), R=est["R"].copy(), s=est["s"])]
        for _ in range(iters):
            r = self.residual(est)
            J = self.numjac(est, r)
            H = J.T @ J + lam * np.eye(J.shape[1])
            dz = np.linalg.solve(H, -(J.T @ r))
            est2 = self.retract(est, dz)
            if 0.5 * self.residual(est2) @ self.residual(est2) < 0.5 * r @ r:
                est = est2
                lam = max(lam * 0.5, 1e-8)
            else:
                lam *= 4.0
            hist.append(dict(P=est["P"].copy(), R=est["R"].copy(), s=est["s"]))
        return est, hist


# --------------------------------------------------------------------------- #
# Scene                                                                       #
# --------------------------------------------------------------------------- #
def run_scene(surf, dem, px_to_m, args, blackout, *, rng_seed, use_fixes=True):
    size = dem.shape[0]
    extent_m = size * px_to_m
    rng = np.random.default_rng(rng_seed)

    P, R, xy, yaw = make_truth_3d(surf, extent_m, args.n_poses, args.trajectory)
    star_sigma = args.yaw_sigma_deg
    healthy = [k for k in range(args.n_poses) if k not in blackout]

    meas = simulate_sensors(
        P, R, args.n_poses, blackout, rng, scale_bias=args.vo_scale_bias,
        gyro_bias_deg=[args.gyro_pitch_bias_deg, args.gyro_roll_bias_deg,
                       args.gyro_yaw_bias_deg],
        grav_sigma_deg=args.grav_sigma_deg, star_sigma_deg=star_sigma,
        vo_sigma_frac=args.vo_sigma_frac)

    # Skyline fixes: real matcher, real margin -> sigma; only at healthy poses
    # (Skyline needs the star yaw prior to break rotational ambiguity, Phase 1),
    # and never hard-rejected (low margin self-down-weights, Phase 5).
    fixes, fix_records = [], []
    if use_fixes:
        loc = SkylineLocalizer(
            dem, px_to_m, grid=args.grid, margin_frac=args.grid_margin_frac,
            n_az=args.n_az, n_range=args.n_range, mast_height_m=args.mast_height_m,
            yaw_sigma_deg=args.yaw_sigma_deg)
        for k in range(0, args.n_poses, args.skyline_every):
            if k in blackout:
                continue
            est_xy, margin, ncc = loc.fix(
                tuple(xy[k]), yaw[k], yaw[k], rng, args.noise_arcmin)
            sig = margin_to_sigma(
                margin, sigma_lock_m=args.sigma_lock_m, margin_ref=args.margin_ref,
                margin_floor=args.margin_floor, sigma_cap_m=args.sigma_cap_m)
            fixes.append((k, est_xy, sig))
            fix_records.append({"pose": k, "margin": round(margin, 4),
                                "sigma_m": round(sig, 1),
                                "unique": bool(margin >= args.margin_unique)})

    # --- Method A: causal forward filter (no scale, no backward smoothing) ---
    Pa, Ra = forward_filter(meas, P[0], args.n_poses, fixes, use_grav=True)

    # --- Method B: joint SO(3) x scale batch smoother ---
    est0 = dict(P=Pa.copy(), R=Ra.copy(), s=1.0)
    graph = So3ScaleGraph(
        args.n_poses, meas, P[0], meas["star"].get(0, Ra[0]), fixes,
        sig_p=args.sigma_prior_m, sig_rot=math.radians(args.sigma_prior_rot_deg),
        sig_vo_p=args.sigma_vo_m, sig_vo_rot=math.radians(args.sigma_vo_rot_deg),
        sig_grav=math.radians(args.sigma_grav_deg),
        sig_star=math.radians(args.yaw_sigma_deg), use_grav=True)
    est, hist = graph.solve(est0, iters=args.gn_iters)

    # --- counterfactual: same joint solve, gravity factor OFF ---
    graph_ng = So3ScaleGraph(
        args.n_poses, meas, P[0], meas["star"].get(0, Ra[0]), fixes,
        sig_p=args.sigma_prior_m, sig_rot=math.radians(args.sigma_prior_rot_deg),
        sig_vo_p=args.sigma_vo_m, sig_vo_rot=math.radians(args.sigma_vo_rot_deg),
        sig_grav=math.radians(args.sigma_grav_deg),
        sig_star=math.radians(args.yaw_sigma_deg), use_grav=False)
    est_ng, _ = graph_ng.solve(est0, iters=args.gn_iters)

    tilt_a, yaw_a = attitude_errs(Ra, R)
    tilt_b, yaw_b = attitude_errs(est["R"], R)
    tilt_ng, yaw_ng = attitude_errs(est_ng["R"], R)
    pe_a = np.linalg.norm(Pa - P, axis=1)
    pe_b = np.linalg.norm(est["P"] - P, axis=1)
    bo = sorted(blackout)

    def at(arr):
        return arr[bo] if bo else arr

    metrics = {
        "n_skyline_locks": len(fixes),
        "vo_scale_true": round(meas["s_true"], 4),
        "vo_scale_assumed": 1.0,
        "vo_scale_joint": round(float(est["s"]), 4),
        "rmse_a_m": round(float(np.sqrt(np.mean(pe_a ** 2))), 1),
        "rmse_b_m": round(float(np.sqrt(np.mean(pe_b ** 2))), 1),
    }
    if bo:
        metrics.update({
            "blackout_yaw_a_deg": round(float(at(yaw_a).mean()), 2),
            "blackout_yaw_b_deg": round(float(at(yaw_b).mean()), 2),
            "blackout_yaw_a_max_deg": round(float(at(yaw_a).max()), 2),
            "blackout_yaw_b_max_deg": round(float(at(yaw_b).max()), 2),
            "blackout_tilt_b_deg": round(float(at(tilt_b).mean()), 2),
            "blackout_tilt_nograv_deg": round(float(at(tilt_ng).mean()), 2),
            "blackout_pos_a_m": round(float(at(pe_a).mean()), 1),
            "blackout_pos_b_m": round(float(at(pe_b).mean()), 1),
        })

    return dict(
        blackout=bo, truth=P, truth_xy=xy, extent_m=extent_m,
        Pa=Pa, Pb=est["P"], est=est, hist=hist, graph=graph,
        yaw_a=yaw_a, yaw_b=yaw_b, tilt_b=tilt_b, tilt_ng=tilt_ng,
        pe_a=pe_a, pe_b=pe_b,
        fixes=fixes, fix_records=fix_records, metrics=metrics)


# --------------------------------------------------------------------------- #
# Args / main                                                                 #
# --------------------------------------------------------------------------- #
def build_args():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["synth", "lola"], default="synth")
    ap.add_argument("--terrain", choices=["hills", "craters", "flat"], default="craters")
    ap.add_argument("--target", default="tycho")
    ap.add_argument("--ldem-ppd", type=int, default=16, choices=[4, 16, 64])
    ap.add_argument("--half-width-deg", type=float, default=3.0)
    ap.add_argument("--cache-dir", type=Path, default=REPO_ROOT / "datasets" / "lro_cache")
    ap.add_argument("--size-px", type=int, default=384)
    ap.add_argument("--px-to-m", type=float, default=30.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--trajectory", choices=["radial", "s_curve", "diag"], default="radial")
    ap.add_argument("--n-poses", type=int, default=30)
    ap.add_argument("--skyline-every", type=int, default=4)
    ap.add_argument("--blackout-start", type=int, default=12)
    ap.add_argument("--blackout-len", type=int, default=10)
    ap.add_argument("--mast-height-m", type=float, default=2.0)
    ap.add_argument("--n-az", type=int, default=180)
    ap.add_argument("--n-range", type=int, default=140)
    ap.add_argument("--grid", type=int, default=61)
    ap.add_argument("--grid-margin-frac", type=float, default=0.12)
    ap.add_argument("--noise-arcmin", type=float, default=2.0)
    ap.add_argument("--yaw-sigma-deg", type=float, default=1.0)
    ap.add_argument("--margin-unique", type=float, default=0.05)
    # VO / IMU drift.
    ap.add_argument("--vo-scale-bias", type=float, default=0.08)
    ap.add_argument("--gyro-pitch-bias-deg", type=float, default=0.45)
    ap.add_argument("--gyro-roll-bias-deg", type=float, default=-0.40)
    ap.add_argument("--gyro-yaw-bias-deg", type=float, default=0.60)
    ap.add_argument("--vo-sigma-frac", type=float, default=0.02)
    ap.add_argument("--grav-sigma-deg", type=float, default=0.8)
    # Information model.
    ap.add_argument("--sigma-prior-m", type=float, default=20.0)
    ap.add_argument("--sigma-prior-rot-deg", type=float, default=1.0)
    ap.add_argument("--sigma-vo-m", type=float, default=30.0)
    ap.add_argument("--sigma-vo-rot-deg", type=float, default=2.0)
    ap.add_argument("--sigma-grav-deg", type=float, default=1.5)
    ap.add_argument("--sigma-lock-m", type=float, default=400.0)
    ap.add_argument("--margin-ref", type=float, default=0.15)
    ap.add_argument("--margin-floor", type=float, default=0.01)
    ap.add_argument("--sigma-cap-m", type=float, default=40000.0)
    ap.add_argument("--gn-iters", type=int, default=25)
    ap.add_argument("--output", type=Path,
                    default=REPO_ROOT / "docs" / "figures" / "skyline_lock" / "factor_graph_so3.png")
    ap.add_argument("--output-json", type=Path,
                    default=REPO_ROOT / "docs" / "figures" / "skyline_lock" / "factor_graph_so3.json")
    return ap


def main() -> int:
    args = build_args().parse_args()
    dem, px_to_m = build_dem(args)
    surf = Surface(dem, px_to_m)
    extent_m = dem.shape[0] * px_to_m
    scene = f"lola:{args.target}" if args.source == "lola" else f"synth:{args.terrain}"
    print(f"scene={scene}  extent={extent_m/1000:.1f} km  px_to_m={px_to_m:.1f}")

    bs, bl = args.blackout_start, args.blackout_len
    mid = set(range(bs, bs + bl))
    end = set(range(args.n_poses - bl, args.n_poses))
    print("solving mid-traverse blackout ...")
    mid_r = run_scene(surf, dem, px_to_m, args, mid, rng_seed=args.seed)
    print("solving end-of-traverse blackout (the yaw cliff) ...")
    end_r = run_scene(surf, dem, px_to_m, args, end, rng_seed=args.seed)
    print("solving no-fix traverse (the scale gauge freedom) ...")
    nofix_r = run_scene(surf, dem, px_to_m, args, set(), rng_seed=args.seed,
                        use_fixes=False)

    summary = {
        "scene": scene,
        "n_poses": args.n_poses,
        "vo_scale_bias": args.vo_scale_bias,
        "gyro_bias_deg": {"pitch": args.gyro_pitch_bias_deg,
                          "roll": args.gyro_roll_bias_deg,
                          "yaw": args.gyro_yaw_bias_deg},
        "mid_traverse_blackout": {"poses": mid_r["blackout"], **mid_r["metrics"]},
        "end_traverse_blackout": {"poses": end_r["blackout"], **end_r["metrics"]},
        "no_fix_scale_gauge": {
            "vo_scale_true": nofix_r["metrics"]["vo_scale_true"],
            "vo_scale_joint": nofix_r["metrics"]["vo_scale_joint"],
            "rmse_a_m": nofix_r["metrics"]["rmse_a_m"],
            "rmse_b_m": nofix_r["metrics"]["rmse_b_m"],
            "note": "no absolute fix -> scale unobservable (similarity gauge freedom)",
        },
    }
    print(json.dumps(summary, indent=2))

    _render(args, dem, scene, mid_r, end_r)

    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(summary, indent=2))
        print(f"wrote {args.output_json}")
    return 0


FILTER = "#ff8c00"
JOINT = "#1f77ff"
PITCHC = "#2ca02c"
NOGRAV = "#9467bd"


def _render(args, dem, scene, mid_r, end_r) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    extent_m = mid_r["extent_m"]
    fig, axes = plt.subplots(2, 3, figsize=(17.5, 10.2))
    nofix = mid_r  # for annotation reuse only
    fig.suptitle(
        f"Skyline Lock Phase 7(2): SO(3) attitude + metric-scale factor graph over {scene}\n"
        "one observability story per DOF class — roll/pitch held by gravity, "
        "yaw needs a future anchor, scale needs fixes that bracket the VO chain",
        fontsize=13)

    for row, (r, label) in enumerate([
            (mid_r, "mid-traverse blackout (a Skyline lock resumes after)"),
            (end_r, "end-of-traverse blackout (no fix ever returns — the cliff)")]):
        truth = r["truth"]
        bo = r["blackout"]
        m = r["metrics"]

        # col 0: top-down map
        ax = axes[row][0]
        ax.imshow(dem, origin="lower", extent=[0, extent_m, 0, extent_m], cmap="terrain")
        ax.plot(truth[:, 0], truth[:, 1], "-", color="white", lw=2.4, label="ground truth")
        ax.plot(r["Pa"][:, 0], r["Pa"][:, 1], "--", color=FILTER, lw=1.9,
                label=f"forward filter  rmse {m['rmse_a_m']:.0f} m")
        ax.plot(r["Pb"][:, 0], r["Pb"][:, 1], "-", color=JOINT, lw=1.9,
                label=f"joint SO(3)+scale  rmse {m['rmse_b_m']:.0f} m")
        ax.plot(truth[bo, 0], truth[bo, 1], "o", color="k", ms=3.0, alpha=0.5,
                label="blackout poses")
        rec = {rc["pose"]: rc for rc in r["fix_records"]}
        for (k, fxy, _) in r["fixes"]:
            c = "#19c819" if rec[k]["unique"] else "#ff8c00"
            ax.scatter(fxy[0], fxy[1], c=c, marker="P", s=55, edgecolors="k", lw=0.4, zorder=6)
        ax.scatter([], [], c="#19c819", marker="P", s=55, edgecolors="k",
                   label="skyline lock (margin-weighted)")
        ax.set_title(label, fontsize=10.5)
        ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
        ax.legend(loc="upper left", fontsize=7.2)
        ax.text(0.03, 0.03,
                f"VO scale +{args.vo_scale_bias*100:.0f}% (s=1.000 assumed)\n"
                f"joint recovers s={m['vo_scale_joint']:.3f}  (truth {m['vo_scale_true']:.3f})",
                transform=ax.transAxes, fontsize=7.6, va="bottom", ha="left",
                bbox=dict(boxstyle="round", fc="white", alpha=0.8))

        # col 1: attitude error per pose -- heading (yaw) vs tilt (roll/pitch)
        ax = axes[row][1]
        ax.axvspan(bo[0], bo[-1], color="0.85", label="blackout")
        ax.plot(r["yaw_a"], "--", color=FILTER, lw=1.7, label="heading — forward filter")
        ax.plot(r["yaw_b"], "-", color=JOINT, lw=2.1, label="heading — joint")
        ax.plot(r["tilt_b"], "-", color=PITCHC, lw=1.6, label="tilt — joint (gravity-held)")
        ax.plot(r["tilt_ng"], ":", color=NOGRAV, lw=1.6,
                label="tilt — no-gravity counterfactual")
        ax.set_title(
            f"attitude error — blackout heading {m['blackout_yaw_a_deg']:.1f}°→"
            f"{m['blackout_yaw_b_deg']:.1f}°; tilt {m['blackout_tilt_b_deg']:.2f}° "
            f"(no-grav {m['blackout_tilt_nograv_deg']:.1f}°)", fontsize=9.2)
        ax.set_xlabel("pose index"); ax.set_ylabel("|attitude error| (deg)")
        ax.legend(fontsize=7.2, loc="upper left")

        # col 2: position error per pose
        ax = axes[row][2]
        ax.axvspan(bo[0], bo[-1], color="0.85", label="blackout")
        ax.plot(r["pe_a"], "--", color=FILTER, lw=1.9, label="forward filter")
        ax.plot(r["pe_b"], "-", color=JOINT, lw=1.9, label="joint SO(3)+scale")
        ax.set_title(
            f"position error — blackout mean {m['blackout_pos_a_m']:.0f}→"
            f"{m['blackout_pos_b_m']:.0f} m", fontsize=10.5)
        ax.set_xlabel("pose index"); ax.set_ylabel("position error (m)")
        ax.legend(fontsize=8)

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=120)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    sys.exit(main())
