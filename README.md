# astro_navigation

[![ci](https://github.com/rsasaki0109/astro_navigation/actions/workflows/ci.yml/badge.svg)](https://github.com/rsasaki0109/astro_navigation/actions/workflows/ci.yml)
![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C)
![Python](https://img.shields.io/badge/Python-3.x-3776AB)
![OpenCV](https://img.shields.io/badge/OpenCV-TRN%20%7C%20VO-5C3EE8)

GNSS-denied space navigation for lunar robots: star-tracker attitude, terrain-relative position
locks, navigation health, and hazard-aware route planning.

[MP4 video](docs/figures/confidence_aware_replanning_demo.mp4)

![Confidence-aware replanning preview: a lunar rover follows a TRN-confidence-aware route, detects a new blocked hazard, replans through more localizable terrain, and tracks navigation risk](docs/figures/confidence_aware_replanning_preview.png)

The headline demo is a lunar autopilot replay: a rover gets a TRN position lock over Tycho, follows a
route biased toward stronger terrain-relative navigation confidence, detects a newly blocked
segment, replans with the C++ `hazard_route_demo`, and tracks route-level navigation risk while it
continues toward the waypoint.

See the [demo gallery](docs/demo_gallery.md) for the full visual index.

The project is intentionally space-native: **star tracker attitude**, **lost-in-space star
identification** against public catalogs, **lunar visual odometry**, and **terrain-relative
navigation on real LRO/LOLA data** — not generic Earth robotics VO with lunar branding. The
implementation is deliberately small so experiments converge quickly, and Python prototypes live
alongside the C++ apps.

## What Is Inside

| Capability | Current artifact |
| --- | --- |
| Star tracker attitude | `build/apps/star_tracker_attitude` |
| Mission navigation state | `build/apps/mission_navigation_demo`, JSON/CSV `NavState`, route risk score |
| Terrain-relative navigation | LRO WAC + LOLA Tycho fixtures, TRN summaries, confidence-aware routing |
| Horizon localization (Skyline Lock) | `scripts/skyline_lock_demo.py`, real LOLA horizons, position + heading + localizability margin; curvature-correct model in `scripts/skyline_curvature_demo.py` |
| Confidence-weighted fusion | `scripts/four_factor_fusion_demo.py` (star + VO + Skyline + TRN, complementary cliffs), `scripts/converse_cliff_demo.py` (same stack on Tycho highland vs Apollo 11 mare — the honest, terrain-shaped asymmetry), `scripts/factor_graph_fusion_demo.py` (3-factor base), `scripts/factor_graph_so2_demo.py` (nonlinear SO(2) backend, heading as a graph state across a star-tracker blackout), `scripts/factor_graph_so3_demo.py` (SO(3) attitude + metric-scale state, one observability story per DOF), margin-driven information, per-pose covariance |
| Hazard-aware routing | C++ `hazard_route_demo`, route metrics, dynamic replanning demo |
| Benchmark harness | HYG stars, NASA POLAR, replay renderers, smoke tests |

## Why Watch This Repo

- **Real mission-shaped demos:** star tracker attitude and terrain-relative navigation run together
  in a lunar descent story, with no inertial prior or temporal filter hiding the per-frame result.
- **Public-data reproducibility:** HYG stars, NASA POLAR, LRO WAC, and LOLA are the main validation
  sources; scripts record source URLs, checksums, and data-size warnings.
- **C++ deliverables, Python iteration:** core paths are moving into C++20 while Python remains the
  fast experiment harness for benchmarks, renderers, and dataset adapters.
- **Honest envelopes:** the docs keep both wins and cliffs, including false-detection star ID
  failures, WAC/LOLA altitude limits, and current TRN parallax failure modes.

## Five-Minute Demo

This synthetic star-tracker smoke test has no external dataset dependency. It generates an identified
star field, estimates camera attitude in C++, and prints the recovered quaternion.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure

python3 scripts/generate_star_tracker_case.py --output-dir outputs/quick_star_case

build/apps/star_tracker_attitude \
  --catalog outputs/quick_star_case/catalog.csv \
  --observations outputs/quick_star_case/observations.csv \
  --fx 1000 --fy 1000 --cx 512 --cy 512
```

Expected shape:

```text
success,correspondences,rms_direction_error_rad,qx,qy,qz,qw,status
1,30,...
```

The first navigation-facing CLI wraps that attitude lock in a mission state and can optionally attach
a terrain-relative position lock:

```bash
build/apps/mission_navigation_demo \
  --catalog outputs/quick_star_case/catalog.csv \
  --observations outputs/quick_star_case/observations.csv \
  --fx 1000 --fy 1000 --cx 512 --cy 512 \
  --trn-summary docs/figures/trn_lro_tycho_terminal/summary.json \
  --localizability-score 0.63 \
  --route-trn-confidence 0.38 \
  --output-json outputs/quick_star_case/nav_state.json \
  --output-csv outputs/quick_star_case/nav_state.csv
```

Expected shape:

```text
status,status_reason,attitude_lock,position_lock,correspondences,attitude_sigma_rad,position_sigma_m,localizability_score,route_trn_confidence,navigation_risk_score,trn_matches,trn_inliers,frame,x,y,z,qx,qy,qz,qw,message
DEGRADED,ROUTE_RISK_HIGH,1,1,...
```

## Featured Demos

The first-screen demos show the current navigation direction: confidence-weighted sensor fusion
with complementary modalities, horizon-based absolute localization, state estimation,
terrain-relative position lock, and hazard-aware route planning.

### Four-factor fusion — star + VO + Skyline + TRN cover each other's blind spots

The capstone: all four localization modalities in one pose graph, where the point is **not** "more
sensors → better" but that the two *absolute* fixes — Skyline (far horizon) and TRN (nadir image) —
have **complementary localizability cliffs**, so their union stays pinned where either alone fails.
A rover drives radially out of Tycho. Skyline reads the 360° horizon and aliases across the
rotationally symmetric rim (equal-radius positions look identical), while TRN matches a nadir LROC
WAC patch against the orbital map and locks on the *same* texture-rich rim that defeats the skyline.
The two absolute factors come from **different public datasets at different scales** (LOLA elevation
vs WAC imagery), each weighted by its *own* real uniqueness margin, so an aliased fix is down-weighted,
not trusted. The honest payoff over real Tycho: fused-4 RMSE **148 m** vs **1090 m** with skyline-only
fusion (**7.4×**) and **3789 m** for VO-only (**26×**) — the rover stays localized across the entire
traverse, including the symmetric exterior where horizon-only fusion drifts away with VO. (Where the
two cliffs actually fall is *terrain*-shaped, and not the symmetric story intuition suggests — see the
converse-cliff demo below.)

[MP4 video](docs/figures/skyline_lock/four_factor_fusion_demo.mp4)

![Four-factor fusion over Tycho: a rover drives radially out of the crater on the LROC WAC ortho; the fused-plus-TRN estimate hugs ground truth with a tight covariance ellipse across the whole traverse, while skyline-only fusion and VO-only drift away in the rotationally symmetric exterior. Skyline fixes (green when unique, orange when aliased) lock only near the distinctive interior and exit; TRN fixes (blue dots) lock everywhere the rim texture is distinctive. A log error-vs-pose panel shows fused-plus-TRN staying low while the others climb](docs/figures/skyline_lock/four_factor_fusion_demo.gif)

```bash
# Reuses the cached LOLA LDEM and fetches a small LROC WAC patch on first run.
python3 scripts/render_four_factor_fusion_demo.py \
  --output docs/figures/skyline_lock/four_factor_fusion_demo.gif

# Static summary figure (3 panels: scene, error vs pose, complementary margins) + JSON:
python3 scripts/four_factor_fusion_demo.py --source lola --target tycho \
  --output docs/figures/skyline_lock/four_factor_fusion.png \
  --output-json docs/figures/skyline_lock/four_factor_fusion.json
# ...or fully offline on synthetic terrain (hillshade appearance map, no download):
python3 scripts/four_factor_fusion_demo.py --source synth --terrain craters \
  --output outputs/factor_graph_fusion/synth_4f.png
```

### Converse cliff — the same stack on two real terrains, and the honest asymmetry

The four-factor demo is easy to oversell as "two sensors, two complementary cliffs." Run that *same*
stack, unchanged, over two real LRO scenes and the truth is more interesting — and **asymmetric**.
Over **Tycho** the kilometre-high rim gives the horizon relief, so Skyline locks across the
distinctive interior (**7/15** fixes unique) before aliasing on the symmetric exterior. Over the
**Apollo 11 mare** the basalt plain is flat, so the 360° horizon is nearly featureless and Skyline
aliases almost everywhere (**2/15** unique) — but the mare is *not* textureless from above: with the
real LROC WAC ortho its albedo speckle is rich enough that the nadir TRN matcher still locks **20/20**
and carries the fix (fused+TRN RMSE **159 m** vs **1778 m** skyline-only). So the honest lesson is not
"complementary cliffs" but that **Skyline is a relief-dependent cue and TRN a texture-dependent one,
and the Moon feeds them unequally per terrain.** Fusion survives both because each factor is weighted
by its own real uniqueness margin — whichever cue the terrain starves is discounted automatically,
with no terrain classifier in the loop. (This also corrects an earlier overstatement that TRN starves
on mare while the horizon pins; the real data shows the opposite direction.)

[MP4 video](docs/figures/skyline_lock/converse_cliff_demo.mp4)

![Converse cliff: two rows, Tycho highland on top and Apollo 11 mare on the bottom, each showing the LROC WAC appearance map with ground-truth, VO-only, skyline-only and fused-plus-TRN tracks, a log error-vs-pose panel, and a uniqueness-margin panel. Over Tycho the skyline margin spikes across the distinctive interior then collapses; over the mare it never lifts off the floor, while the TRN margin sits high on both — so fused-plus-TRN hugs truth in both rows while skyline-only peels away with VO on the mare from the start](docs/figures/skyline_lock/converse_cliff_demo.gif)

```bash
# Two-scene static figure (Tycho vs Apollo 11 mare) + JSON; reuses the cached
# LOLA LDEM and fetches the small Apollo 11 WAC patch on first run.
python3 scripts/converse_cliff_demo.py \
  --output docs/figures/skyline_lock/converse_cliff.png \
  --output-json docs/figures/skyline_lock/converse_cliff.json

# Side-by-side animation (GIF + MP4):
python3 scripts/render_converse_cliff_demo.py --mp4 \
  --output docs/figures/skyline_lock/converse_cliff_demo.gif
```

### Factor-graph fusion — star + VO + Skyline, each trusted only as far as it earns

The three-factor predecessor of the demo above: the project's localization modalities fused into one estimator that lets each carry
the fix only as far as its own confidence justifies. A rover drives out of Tycho's distinctive
interior into self-similar terrain, fusing a star-tracker attitude factor, a visual-odometry
between-factor (locally good, globally drifting), and a Skyline position factor whose information is
the *real* uniqueness margin from the demo below. Inside the crater the horizon fix is tight and the
fused track hugs ground truth with a small covariance ellipse; out in the rotationally symmetric
exterior the margin collapses, so the skyline fixes scatter to aliased positions and the graph
**down-weights them automatically**, coasting on VO with a visibly growing ellipse instead of
snapping to a wrong lock. The honest payoff: fused error stays bounded where the terrain is
distinctive (~4× lower RMSE than VO-only over Tycho, 958 m vs 3.8 km) and degrades *gracefully*,
not catastrophically, where it is not. Confidence is terrain-driven and reproducible from public
LOLA data, never a hand-tuned schedule.

[MP4 video](docs/figures/skyline_lock/factor_graph_fusion_demo.mp4)

![Factor-graph fusion over Tycho: a rover drives radially out of the crater; the fused estimate tracks ground truth with a tight covariance ellipse over the distinctive interior where skyline fixes are unique (green), and coasts on visual odometry with a growing ellipse over the self-similar exterior where the fixes alias (orange) and are down-weighted; an error-vs-pose panel shows fused error staying bounded while VO-only drifts away](docs/figures/skyline_lock/factor_graph_fusion_demo.gif)

```bash
# Reuses the cached LOLA LDEM from the Skyline Lock demo (~33 MB on first run).
python3 scripts/render_factor_graph_fusion_demo.py \
  --output docs/figures/skyline_lock/factor_graph_fusion_demo.gif

# Static summary figure + JSON metrics (synthetic, no download):
python3 scripts/factor_graph_fusion_demo.py --source synth --terrain craters \
  --trajectory radial --output outputs/factor_graph_fusion/synth.png
# ...or the headline run on real terrain:
python3 scripts/factor_graph_fusion_demo.py --source lola --target tycho \
  --trajectory radial --output outputs/factor_graph_fusion/tycho.png
```

### Heading as a graph state — a nonlinear SO(2) factor graph, and the star-tracker blackout

The factor-graph fusion above is *linear*: it fixes each pose's attitude to the star tracker and
rotates VO into the world with it. That is exactly right — while the star tracker is healthy. This
demo asks the honest follow-up: what happens across a star-tracker **blackout** (sun glare, limited
sky, a dropped frame)? With attitude a fixed input, the estimator can only dead-reckon heading
*forward* from the last good fix, and the VO heading bias makes that error ramp — several degrees off
by the far side, every VO step then rotated into the wrong world direction. Promoting yaw to a graph
state turns the estimator into a batch smoother over `(x, y, θ)`: VO *yaw-increment* factors chain
the gap, so the fixes that resume **after** the blackout flow backward through that chain and correct
the heading *during* it. The solver is a self-contained Gauss-Newton with an SO(2) retraction (angles
add, then wrap) and analytic Jacobians — ~120 lines, no GTSAM/Ceres dependency, fully reproducible.

The honest envelope, two scenes side by side:

- **Mid-traverse blackout** (a lock resumes on the far side): heading recovers from **5.4° → 0.3°**
  mean inside the gap (peak ~10° → 0.6°) and the dead-reckoned arc snaps back onto truth (RMSE
  148 m → 113 m).
- **End-of-traverse blackout** (no fix ever comes back): there is nothing to smooth backward from, so
  the joint solve does **no better** than fixed-yaw (5.6° → 5.6°). Batch smoothing needs a future
  anchor — the cliff. Same mechanism, opposite consequence, as in the curvature and four-factor demos.

[MP4 video](docs/figures/skyline_lock/factor_graph_so2_demo.mp4)

![SO(2) factor-graph demo: a mid-traverse star-tracker blackout is held fixed while the Gauss-Newton iterations replay. The optimizer starts from the Phase-5 fixed-yaw estimate whose trajectory bulges away from ground truth across the blackout because the dead-reckoned heading drifted ~10 degrees; iteration by iteration the resumed star and skyline fixes on the far side flow backward through the VO yaw-increment chain, the per-pose heading error collapses inside the shaded blackout band, and the estimated arc snaps onto the white ground-truth path](docs/figures/skyline_lock/factor_graph_so2_demo.gif)

```bash
# Watch the Gauss-Newton iterations straighten the blackout arc (synthetic, no download):
python3 scripts/render_factor_graph_so2_demo.py --mp4 \
  --output docs/figures/skyline_lock/factor_graph_so2_demo.gif

# Static figure (mid vs end blackout) + JSON metrics:
python3 scripts/factor_graph_so2_demo.py            # synth craters, rover-scale
python3 scripts/factor_graph_so2_demo.py --source lola --target tycho   # real LOLA
```

### SO(3) attitude + metric scale — one observability story per degree of freedom

The SO(2) backend above assumes a planar world and a known visual-odometry scale. A rover on real
crater slopes pitches and rolls, and its VO reports translation only *up to an unknown metric scale*.
Lifting the graph to full **SO(3)** attitude plus a **global scale state** forces three honest
questions — one per class of degree of freedom — and the answers are deliberately different:

- **Roll & pitch are gravity-observable.** An always-on accelerometer sees the gravity vector in the
  body frame, which pins tilt (2 of the 3 rotational DOF) at *every* pose — even mid-blackout, with no
  star and no future anchor. So tilt never really drifts: in an end-of-traverse blackout it stays
  **0.48°** with gravity versus **2.86°** with the gravity factor removed. The honest message is *don't
  oversell "SO(3) attitude recovery"* — two of three axes were never the hard part, and the figure
  shows the counterfactual that proves gravity is what holds them.
- **Yaw is the gravity-unobservable DOF** (rotation about the gravity axis leaves the accelerometer
  unchanged). It is the same batch-smoothing story as the SO(2) demo, now correctly isolated: across a
  mid-traverse blackout a resuming fix flows backward and recovers heading **2.2° → 1.3°** (peak
  4.8° → 1.4°); at end-of-traverse there is no future anchor and the joint solve ties the forward
  filter (**3.7° → 3.8°**) — the cliff.
- **Metric scale is observable only when absolute fixes bracket a VO chain.** The Skyline locks pin the
  chain's true length, recovering the **+8%** VO scale error (estimated **0.914**, truth **0.926**) and
  pulling position RMSE from **158 m → 40 m**. Run the same traverse with *no* absolute fix and scale
  is a pure gauge freedom: the estimate stays at **1.000**, the whole map similarity-ambiguous.

The solver is a self-contained Gauss-Newton on **SO(3) × ℝ³ × ℝ₊**: an exp/log retraction on each pose
rotation, one global log-scale, and a generic *numerical* Jacobian so the factor set stays declarative
(analytic SO(3) Jacobians are verbose; this keeps the reference solver compact and obviously correct —
the analytic/GTSAM path is the production route). Skyline fixes use the real matcher and real
uniqueness-margin → information, same honesty bar as the fusion demos.

[MP4 video](docs/figures/skyline_lock/factor_graph_so3_demo.mp4)

![SO(3)+scale factor-graph demo: a mid-traverse star-tracker blackout is held fixed while the Gauss-Newton iterations replay across three panels. Left, the estimated trajectory snaps onto the white ground track as the solve converges; middle, the per-pose attitude error shows heading (yaw) collapsing inside the shaded blackout band while tilt (roll+pitch) stays flat near zero throughout, held by gravity; right, the global metric-scale state slides from the assumed 1.000 down toward the true 0.926 as the absolute Skyline fixes bracket the VO chain](docs/figures/skyline_lock/factor_graph_so3_demo.gif)

```bash
# Watch the GN iterations: heading collapses, tilt stays flat, scale converges (synthetic, no download):
python3 scripts/render_factor_graph_so3_demo.py --mp4 \
  --output docs/figures/skyline_lock/factor_graph_so3_demo.gif

# Static figure (mid blackout, end-of-traverse cliff) + JSON metrics incl. the no-fix gauge freedom:
python3 scripts/factor_graph_so3_demo.py
```

### Skyline Lock — lost on the Moon from a single horizon

A GNSS-denied rover with a star-tracker attitude looks at the black-sky / terrain boundary. The
elevation-vs-azimuth horizon profile is a fingerprint of *where you are standing*: it is matched
against horizons predicted from real LOLA terrain across a candidate-position grid (the star tracker
supplies a yaw prior), recovering both position and heading. This replay traverses Tycho. Over the
distinctive interior the match is a single sharp peak — a unique lock (uniqueness margin ~0.43). As
the rover crosses the rim into self-similar terrain, the circular rim's rotational symmetry aliases
the position and the estimate slides along the equal-rim-distance arc (ambiguous). Position stays
sub-cell and heading recovers to ~1° throughout; what changes is *localizability* — and the demo
keeps both the lock and its cliffs on screen. (This replay uses the flat-plane horizon model; the
curvature-correct model and *when it matters* are the demo below.)

[MP4 video](docs/figures/skyline_lock/skyline_lock_demo.mp4)

![Skyline Lock demo: a rover traverses Tycho while its observed horizon is matched against LOLA-predicted horizons; the score surface is a single sharp peak (LOCKED) over the distinctive interior and broadens into an aliased arc (AMBIGUOUS) as the rover leaves it](docs/figures/skyline_lock/skyline_lock_demo.gif)

```bash
# Downloads the LOLA LDEM_16 global model (~33 MB) on first run; cached after.
python3 scripts/render_skyline_lock_demo.py \
  --output docs/figures/skyline_lock/skyline_lock_demo.gif

# Single-shot localization on one scene (synthetic, no download):
python3 scripts/skyline_lock_demo.py --source synth --terrain hills \
  --yaw-prior-deg 37 --output outputs/skyline_lock/synth_hills.png
# ...or on real terrain, locking from Tycho's centre:
python3 scripts/skyline_lock_demo.py --source lola --target tycho \
  --yaw-prior-deg 37 --output outputs/skyline_lock/lola_tycho.png
```

### Lunar curvature — the horizon model, and where it costs you 16 km

A technical deepening of the matcher above. On the airless Moon the horizon is a *spherical*-datum
cue, not a flat one: the surface drops below the observer's tangent plane by ≈ r²/2R
(R = 1 737.4 km), so from a 2 m mast the bare horizon sits just **2.6 km** away and a distant feature
is visible only if its height clears r²/2R (~0.5 km at 40 km, ~7.4 km at 160 km). The rover observes
that true curved horizon; the honest question is whether localizing it with the old flat-plane model
(Phases 0–6) versus the curvature-correct model actually changes the fix. The answer is the
project's stance in miniature: **it depends on what your skyline leans on.** Over Tycho a near,
kilometre-high rim dominates, its flat-model bias is nearly uniform across candidates and cancels in
the normalized match, so both models lock the centre — curvature is essentially free. Over **mare**,
the lock leans on faint distant relief; the flat model counts terrain that is physically *below* the
lunar horizon — phantom cues — and snaps the fix **16.6 km** onto a false mode, while the
curvature-correct model removes the phantoms and recovers the right cell (the margin stays small —
mare is genuinely aliased — so the picture gets *more* honest, not rosier). Same correction, opposite
consequence. The fix is one term — `curvature_radius_m` in `render_horizon` — and refraction-free,
unlike a terrestrial viewshed.

[MP4 video](docs/figures/skyline_lock/skyline_curvature_demo.mp4)

![Lunar curvature demo: the model curvature is dialled from a flat plane to the true Moon while the rover observes the real curved horizon. Over Tycho the position fix holds on truth the whole sweep (a near, tall rim dominates the skyline); over Apollo 11 mare the flat model places the fix ~60 km away on a phantom mode built from terrain below the lunar horizon, and as curvature is dialled in those phantom cues sink away and the fix snaps home to the truth cell](docs/figures/skyline_lock/skyline_curvature_demo.gif)

```bash
# Animation: sweep the model curvature flat → Moon over Tycho and a mare scene.
python3 scripts/render_skyline_curvature_demo.py --mp4 \
  --output docs/figures/skyline_lock/skyline_curvature_demo.gif

# Static comparison figure (horizon profiles, score surfaces, horizon geometry) + JSON:
python3 scripts/skyline_curvature_demo.py \
  --output docs/figures/skyline_lock/skyline_curvature.png \
  --output-json docs/figures/skyline_lock/skyline_curvature.json
```

### Skyline localizability routing — don't get lost

The same horizon match yields a *map*: at every position, how unambiguously
could a rover pin itself from the horizon (heading known from the star tracker)?
This turns into a routing cost. A baseline A* takes the shortest path; a
localizability-aware A* adds cost for aliased terrain, so it detours onto the
distinctive crater rim where a horizon fix holds. Over Tycho the aware route is
~2× more localizable on average and spends 22% (vs 69%) of its length in aliased
terrain, for only ~6% extra distance — the navigation-health sibling of the
existing TRN-confidence routing demo, driven purely by terrain shape.

![Skyline localizability routing over Tycho: a localizability map (bright on the distinctive rim, dark over self-similar terrain) with a shortest route crossing aliased terrain and a localizability-aware route detouring onto the rim](docs/figures/skyline_lock/skyline_localizability_route.png)

```bash
# Reuses the cached LOLA LDEM from the Skyline Lock demo.
python3 scripts/skyline_localizability_map.py \
  --output docs/figures/skyline_lock/skyline_localizability_route.png
```

### Lost Robot Challenge — one star frame + one lunar frame

A lunar robot wakes up with no GNSS. It gets one synthetic star-camera frame
and one nadir lunar camera frame, then recovers attitude and position through
the C++ navigation state demo plus the Tycho terminal TRN fixture. The result
is a single mission-control card: star-camera lock, lunar camera view,
LRO/LOLA map lock, and final navigation telemetry.

![Lost Robot Challenge mission card: one star frame and one lunar frame localize a GNSS-denied lunar robot over Tycho](docs/figures/lost_robot_challenge.png)

Reproduce without external downloads:

```bash
cmake --build build --parallel
python3 scripts/lost_robot_challenge.py \
  --output docs/figures/lost_robot_challenge.png
```

### Navigation replay — LOST → DEGRADED → OK

The navigation replay shows the state machine becoming useful: the robot starts with no locks,
gets a star-camera attitude lock, then reaches full navigation lock once TRN provides position. The
Tycho map overlays the estimated position and the conservative TRN sigma circle; the replay also
marks the dominant uncertainty source as map resolution.

![Navigation replay demo: LOST to DEGRADED to OK with star-camera lock, TRN map lock, sigma circle, and map-resolution-limited uncertainty](docs/figures/navigation_replay_demo.gif)

```bash
python3 scripts/render_navigation_replay_demo.py \
  --output docs/figures/navigation_replay_demo.gif
```

### Hazard-aware lunar navigation

This guidance demo turns the Tycho terminal TRN fixture into a local cost map:
dark/shadowed terrain and sharp image gradients become hazard cost, A* plans a
route around the high-cost regions, and the rover follows the route while the
navigation status moves through lost, attitude-only, TRN-locked, relocalizing,
and arrived phases. The overlay keeps the mission-facing uncertainty visible
with the same conservative TRN sigma used by `mission_navigation_demo`. The
route planner itself is available as a reusable C++ API in
`astro_navigation/navigation/hazard_guidance.hpp`; the renderer just builds an
image-derived cost grid for the demo and can call the C++ `hazard_route_demo`
CLI for the actual route plan. The CLI reports route length, straight-line
length, detour ratio, mean/max route cost, and minimum clearance from blocked
hazard cells.

[MP4 video](docs/figures/hazard_aware_navigation_demo.mp4)

![Hazard-aware lunar navigation demo fallback: red hazard regions, blue planned route, green rover progress, waypoint, relocalizing phase, and navigation telemetry over the Tycho terminal TRN map](docs/figures/hazard_aware_navigation_demo.gif)

```bash
cmake --build build --parallel
python3 scripts/render_hazard_aware_navigation_demo.py \
  --planner-app build/apps/hazard_route_demo \
  --output docs/figures/hazard_aware_navigation_demo.gif
ffmpeg -y -i docs/figures/hazard_aware_navigation_demo.gif \
  -movflags +faststart -pix_fmt yuv420p \
  -vf "fps=12,scale=trunc(iw/2)*2:trunc(ih/2)*2" \
  docs/figures/hazard_aware_navigation_demo.mp4
```

### Dynamic hazard replanning

This autopilot replay starts from the same Tycho hazard map, then injects a new blocked hazard on the
active route. The rover marks the route invalid, replans from its current TRN position with
`hazard_route_demo`, and locks a new path around the obstacle. The side panel tracks the replan count,
old/new detour ratio, and new-route clearance.

[MP4 video](docs/figures/dynamic_hazard_replanning_demo.mp4)

![Dynamic hazard replanning demo fallback: a lunar rover invalidates an old route, replans around a new blocked hazard, and resumes toward the waypoint](docs/figures/dynamic_hazard_replanning_demo.gif)

```bash
cmake --build build --parallel
python3 scripts/render_dynamic_hazard_replanning_demo.py \
  --planner-app build/apps/hazard_route_demo \
  --output docs/figures/dynamic_hazard_replanning_demo.gif
ffmpeg -y -i docs/figures/dynamic_hazard_replanning_demo.gif \
  -movflags +faststart -pix_fmt yuv420p \
  -vf "fps=12,scale=trunc(iw/2)*2:trunc(ih/2)*2" \
  docs/figures/dynamic_hazard_replanning_demo.mp4
```

### Confidence-aware replanning

This replay uses the same dynamic hazard event, but the planner receives a fused
cost map: blocked hazards remain hard constraints, while low TRN confidence
adds route cost. The side panel exposes the mission-facing risk fields now
available in `NavState`: localizability score, route TRN confidence, and the
derived navigation risk score.

[MP4 video](docs/figures/confidence_aware_replanning_demo.mp4)

[GIF animation](docs/figures/confidence_aware_replanning_demo.gif)

![Confidence-aware replanning preview: heatmap confidence, blocked hazard, replanned localizable route, and route-level navigation risk over Tycho](docs/figures/confidence_aware_replanning_preview.png)

```bash
cmake --build build --parallel
python3 scripts/render_confidence_aware_replanning_demo.py \
  --planner-app build/apps/hazard_route_demo \
  --output docs/figures/confidence_aware_replanning_demo.gif
ffmpeg -y -i docs/figures/confidence_aware_replanning_demo.gif \
  -movflags +faststart -pix_fmt yuv420p \
  -vf "fps=12,scale=trunc(iw/2)*2:trunc(ih/2)*2" \
  docs/figures/confidence_aware_replanning_demo.mp4
```

### TRN confidence heatmap

The hazard map asks where the rover should avoid driving. The TRN confidence
heatmap asks a different navigation question: where is the terrain visually
localizable enough for terrain-relative navigation to lock position? The
renderer scores the Tycho ortho fixture from gradient energy, local texture
richness, feature density, and illumination balance, then writes both a PNG
overview and a JSON summary for downstream planning experiments.

![TRN confidence heatmap over Tycho: blue regions have weak texture or poor lighting, while yellow and red regions have stronger terrain-relative navigation lock potential](docs/figures/trn_confidence_heatmap.png)

```bash
python3 scripts/render_trn_confidence_heatmap.py \
  --output docs/figures/trn_confidence_heatmap.png
```

### Localizability-aware routing

The confidence map can also shape route planning. This demo compares a
hazard-only route against a route that keeps the same blocked terrain but adds
a cost penalty for visually weak TRN regions. The result is a slightly longer
route with a higher average TRN confidence and fewer low-confidence segments.

![Localizability-aware route over Tycho: gray shows the hazard-only route, green shows the route biased toward stronger terrain-relative navigation confidence, and red marks blocked hazard regions](docs/figures/localizability_aware_route.png)

```bash
cmake --build build --parallel
python3 scripts/render_localizability_aware_route.py \
  --planner-app build/apps/hazard_route_demo \
  --output docs/figures/localizability_aware_route.png
```

<details>
<summary>More demos and benchmark visuals</summary>

### TRN trajectory — frame-by-frame position recovery

A 9-frame descent trajectory over Tycho central peak (38 → 30 km altitude,
3 km lateral motion) showing TRN locking position from a single nadir image
each frame. The recovered estimates accumulate on the top-down ortho map as
green dots; the truth path is the yellow line; the red bar visualises the
current frame's position error.

![TRN trajectory demo: per-frame position recovery on a descent over Tycho central peak. LEFT = top-down ortho map with truth path (yellow) and recovered positions (green). RIGHT TOP = nadir rover view at the current altitude. RIGHT BOTTOM = telemetry HUD with position truth, recovered, current error, and running mean over successful PnP frames](docs/figures/trn_trajectory_demo.gif)

No inertial prior, no temporal filter — every frame solves PnP from scratch
on the rover image vs the LRO ortho. **9/9 frames produce a position
estimate**, 8/9 within 100 m, current pipeline mean ~150 m on this trajectory.

```bash
python3 scripts/render_trn_trajectory_gif.py \
  --output docs/figures/trn_trajectory_demo.gif
```

### Lunar landing mission — star tracker + TRN, end-to-end

The two localisation modules running together as a single mission story —
six descent moments from orbital insertion (400 km) down to touchdown burn
(30 km), with the **star tracker** confirming attitude against a different
recognisable constellation each frame and the **terrain-relative navigation**
recovering position from real LRO/LOLA imagery as the camera samples finer
WAC ortho + LOLA DEM tiles on the way down.

![Mission demo: star tracker (constellation IDs left) + TRN (lunar nadir camera right) + telemetry HUD across six descent moments from 400 km orbital insertion to 30 km touchdown burn](docs/figures/mission_demo.gif)

Both modules run independently per frame (no inertial prior, no temporal
filtering — every frame solves attitude from a single star image and
position from a single nadir image). Per-frame attitude is rendered into
the star image, identified through `apps/lost_in_space_pair_id`, and the
recovered ids drive the cyan constellation lines + gold star labels. TRN
uses `scripts/lro_trn_demo.py` end-to-end: WAC tile fetch → LOLA crop →
forward ray-march → SIFT + AP3P PnP. The telemetry HUD shows the truth-
vs-recovered position with the absolute error in metres; the altitude bar
falls from 100 % at orbit to ~7 % at terminal.

```bash
python3 scripts/render_mission_demo_gif.py \
  --index-bin <path-to>/hyg_pair_index_full.bin \
  --output docs/figures/mission_demo.gif
```

### Lost-in-space star identification

A satellite that just powered on doesn't know where it's looking. *Lost-in-space* star
identification recovers attitude from a single star tracker image with **no prior** — match
detected centroids against a public catalog by their pairwise angles, then solve Wahba/Kabsch
for the camera-inertial rotation.

![Lost-in-space identification across six famous-constellation attitudes — Orion, Big Dipper, Cygnus+Lyra, Cassiopeia, Leo, Scorpius — with constellation lines and bright-star labels drawn from the recovered identification](docs/figures/lost_in_space_demo.gif)

Six attitudes whose boresights land on recognisable asterisms are run through the full
pipeline — synthetic exposure → centroid detection → pair-angle index lookup → Wahba
rotation — producing **759 / 768 correct, 7 wrong, 2 unassigned** at 128 centroids per
frame against an 8 920-star HYG mag≤6.5 index. The constellation stick-figures (cyan)
and bright-star labels (gold) are drawn purely from the catalog ids the C++ identifier
emits — they only appear once the matcher recovers attitude. Green rings = correct,
red = wrong, blue = unassigned.

Reproduce the GIF (uses the C++ identifier and a `.bin` index emitted by
`scripts/build_star_pair_index.py --write-bin` or `apps/build_star_pair_index`):

```bash
python3 scripts/render_constellation_demo_gif.py \
  --catalog datasets/star_catalogs/hyg-v42/converted/hyg_v42_bright_mag6p5_unit.csv \
  --index-bin <path-to>/hyg_pair_index_full.bin \
  --output docs/figures/lost_in_space_demo.gif
```

The unannotated random-attitude variant (no constellation overlays, no asterism
preselection) is still available via `scripts/render_lost_in_space_gif.py` for bench
runs.

<details>
<summary>Run a single attitude through the underlying three-step pipeline</summary>

```bash
python3 scripts/render_star_image.py \
  --catalog datasets/star_catalogs/hyg-v42/converted/hyg_v42_mag8p0_unit.csv \
  --output-image outputs/exposure.png \
  --output-truth outputs/truth.csv \
  --yaw-deg 30 --pitch-deg 20 --roll-deg 10

python3 scripts/centroid_stars_from_image.py \
  --input-image outputs/exposure.png \
  --output-observations outputs/observations_unlabeled.csv

python3 scripts/identify_stars_with_pair_index.py \
  --observations outputs/observations_unlabeled.csv \
  --index <path-to>/hyg_pair_index_16000.npz \
  --output outputs/assignments.csv \
  --fx 1000 --fy 1000 --cx 512 --cy 512 \
  --pyramid-size 6 --neighbor-bins 1 --tolerance-arcsec 120 \
  --pyramid-restarts 3 --confidence-fraction 0.5
```

</details>

### Terrain-relative navigation on real LRO + LOLA data

A virtual descent camera at orbital altitude looks down on the lunar surface;
the matcher recovers its position from a single frame against a public LRO
mosaic + LOLA elevation model — no inertial prior, no rover trajectory.

![Rendered nadir-pointing rover view at 400 km altitude over Tycho — synthesised from a real LRO WAC mosaic + LOLA elevation by per-pixel ray-marching, then matched back against the same mosaic to recover position](docs/figures/trn_lro_tycho/rover.png)

The pipeline fetches LROC WAC tiles via NASA Trek WMTS (~600 KB per scene at
zoom 5) and a LOLA `LDEM_<ppd>.img` from PDS Geosciences (~2 MB at 4 ppd),
forward-renders the rover view by ray-marching every pixel through the real
heightmap, and recovers the camera pose with `cv2.solvePnPRansac(SOLVEPNP_AP3P)`
on (3D world, 2D rover) correspondences.

6-target sweep at 400 km altitude / WAC z=5 (~660 m/px ortho, ~500 km mosaic):

| Target | Matches | Inliers | Position error |
| --- | ---: | ---: | ---: |
| Apollo 11 (Mare Tranquillitatis) | 79 | 24 | 1383 m |
| Apollo 12 (Oceanus Procellarum) | 37 | 16 | 300 m |
| Apollo 15 (Hadley Rille) | 107 | 20 | 574 m |
| Apollo 17 (Taurus-Littrow) | 89 | 16 | 622 m |
| **Tycho (bright ejecta)** | **113** | **24** | **179 m** |
| Copernicus (ray crater) | 58 | 17 | 391 m |

All six recover position with no false positives. Mare targets have ~10x worse
error than crater rim targets because mare SIFT features are dim and self-similar.

Reproduce (downloads ~600 KB ortho + 2 MB DEM on first run, then ~5 s per scene):

```bash
python3 scripts/lro_trn_demo.py --target tycho \
  --output-dir docs/figures/trn_lro_tycho
```

**Terminal descent (30-100 km altitude, finer LRO data):**

![Real LRO WAC mosaic of the Tycho central peak — rendered nadir-pointing rover view at 30 km altitude using WAC z=8 (~82 m/px ortho) + LOLA LDEM_64 (~470 m/px DEM)](docs/figures/trn_lro_tycho_terminal/rover.png)

Stepping the ortho up to WAC z=8 (~82 m/px, 25 tiles ≈ 1 MB at `--tile-radius 2`)
and the heightmap up to `LDEM_64` (~470 m/px, ~530 MB one-time download) brings
the rover camera within terminal-descent range. Best per-target altitude on the
6-target sweep:

| Target | Altitude | Matches | Inliers | Position error |
| --- | ---: | ---: | ---: | ---: |
| Copernicus (ray crater) | 50 km | 87 | 13 | **30 m** |
| **Tycho (central peak)** | 30 km | 82 | 11 | **32 m** |
| Apollo 17 (Taurus-Littrow) | 30 km | 68 | 6 | 43 m |
| Apollo 12 (Procellarum) | 100 km | 14 | 8 | 93 m |
| Apollo 15 (Hadley Rille) | 50 km | 105 | 20 | 131 m |
| Apollo 11 (Tranquillitatis) | 100 km | 35 | 18 | 172 m |

Median ~80 m on a ~92 km × 92 km mosaic — about an order of magnitude tighter
than the orbital cycle 3 numbers. Below ~30 km altitude, parallax distortion
from the real heightmap (Tycho rim at +1.8 km vs camera at 30 km altitude →
~6% image-position shift) starts breaking SIFT scale-space matching; that
cliff is the next-cycle target (ASIFT or render-time orthorectification).

```bash
python3 scripts/lro_trn_demo.py --target tycho \
  --zoom 8 --tile-radius 2 --ldem-ppd 64 \
  --rover-altitude-m 30000 \
  --output-dir docs/figures/trn_lro_tycho_terminal
```

### Lunar visual odometry on NASA POLAR Traverses 1-6

NASA POLAR Traverse 1 (lunar-analogue testbed), left camera 50 ms exposure, 11 frames. Animated:
SIFT keypoints per frame on the left, the SIFT-monocular VO trajectory accumulating on the right
(Sim(3) aligned to ground truth, ATE RMSE 0.019 m). The same SIFT + rectified-stereo PnP path
with `--ratio-test 0.85` (looser-than-textbook for dim-light traverses) extends to **65/66
frames OK across Traverses 1-6**, mean ATE 0.118 m — see headline table for the per-traverse
breakdown.

![POLAR Traverse 1 SIFT features + VO trajectory animation](docs/figures/polar_traverse1_vo_demo.gif)

Static comparison plot — SIFT monocular and rectified-stereo PnP overlaid on ground truth:

![POLAR Traverse 1 VO trajectories vs ground truth](docs/figures/polar_traverse1_vo_demo.png)

Reproduce locally:

```bash
build/apps/lunar_visual_odometry \
  --images outputs/polar_view1_traverse1_left_50ms/images.txt \
  --fx 1452.71 --fy 1452.88 --cx 999.53 --cy 1035.4 \
  --feature sift \
  --trajectory outputs/trajectory_sift.tum

python3 scripts/plot_trajectory_comparison.py \
  --ground-truth outputs/polar_view1_traverse1_left_50ms/refined_poses.tsv \
  --trajectory "SIFT monocular (Sim(3))" outputs/trajectory_sift.tum sim3 \
  --output outputs/trajectory_sift_demo.png
```

</details>

## Headline Results

Numbers below are the current best on the corresponding benchmark. Full per-iteration history is in
[`docs/experiments.md`](docs/experiments.md).

| Module | Benchmark | Result |
| --- | --- | --- |
| Star tracker attitude | 30 stars synthetic, 0.1 px noise | mean attitude error **0.00459 deg** |
| Lost-in-space, idealized (HYG mag≤8, 40k indexed stars — mag≤8 catalog density ceiling) | 32 true + up to 12 false detections, 0.1 px noise, `--pyramid-size 6 --neighbor-bins 1 --tolerance-arcsec 120 --skip-pkl` | **64/64 correct, 0 wrong**, query 61-94 s, build 277 s, .npz 1016 MB, 332 M pairs |
| Lost-in-space, deeper-catalog scout (HYG mag≤9, 60k indexed stars, false=0 smoke) | 1 trial, ps=6, default tight params | **32/32 correct, 0 wrong**, query 294 s (~5 min), build 654 s, .npz 2196 MB, 748 M pairs. Correctness extends past mag≤8; sky-cell partitioning is the prerequisite for routine operation at this density |
| Lost-in-space, high-false-rate idealized (HYG mag≤8, 16k indexed stars) | 32 true + 16/24/32 false detections (33-50% false rate), ps=6 | **64/64 correct, 0 wrong** at every level, query ~6 s |
| Lost-in-space, realistic camera effects + pyramid restart (HYG mag≤8, 16k, ps=6, trials=24, restarts=3) | mag-weighted detection (limiting 7.0) + 50% near-real-star false detections, full sweep false 0/4/8/12 | **768/768 correct, 0 wrong** across 96 trials. Residual catastrophic-failure rate **`<3.1%` at 95% CI** (Rule of three; vs `<17%` no-restart baseline). 86/96 trials succeeded on attempt 0 |
| Lost-in-space, + magnitude-dependent centroid noise (HYG mag≤8, 16k, ps=6, trials=6, restarts=3) | All three realism axes stacked: mag-weighted detection, near-real false, σ_centroid = noise_px·10^(0.4·(mag−6)) | **192/192 correct, 0 wrong**. cand_gen 1.7× of constant-noise baseline at false=12 (faint-star noise widens effective tolerance) |
| Lost-in-space, + 500-year stale catalog (HYG mag≤8, 16k, ps=6, trials=6, restarts=3) | All four realism axes plus `--apply-proper-motion-years 500` drifting RA/Dec by 500·pmra/pmdec mas before projection (matcher still uses J2000 index) | **766/768 correct (99.7%), 0 wrong**. Graceful degradation: high-pm stars (Groombridge 1830 at 7 arcsec/yr → 1400 arcsec drift) drop out of verification, but the recovered attitude is correct in every trial |
| Lost-in-space, **5-axis** realism stack (HYG mag≤8, 16k, ps=6, trials=6, restarts=3) | Above 4 axes (with pm=200) plus `--hot-pixel-fraction 0.5` placing 50% of false detections at fixed sensor hot-pixel positions | **767/768 correct (99.87%), 0 wrong**. 2/24 trials hit the 4-attempt restart-budget ceiling but still recovered. Five realism axes stacked still preserve correctness via restart |
| Lunar VO (POLAR Traverse1, L 50 ms, monocular SIFT) | 11 frames, Sim(3) alignment | ATE RMSE **0.0186 m**, 11/11 frames OK |
| Lunar VO (POLAR Traverse1, L 50 ms, rectified stereo + PnP) | 11 frames, SE(3) | ATE RMSE **0.0650 m**, path 10.18 m vs 9.98 m GT |
| Lunar VO (POLAR Traverse**1-6**, L 50 ms, rectified stereo + PnP, **SIFT** + CLAHE + `--ratio-test 0.85`) | 66 frames total | **65/66 frames OK** (ORB+default-ratio baseline was 15/33 on T4-T6). T1 11/11 ATE 0.028 m, T2 11/11 ATE 0.037 m, T3 11/11 ATE 0.043 m, T4 11/11 ATE 0.069 m, T5 11/11 ATE 0.080 m, T6 10/11 ATE 0.413 m |
| TRN orbital (real LRO WAC z=5 + LOLA LDEM_4) | 6 targets at 400 km nadir, ~500 km mosaic | All 6 recover position, 0 false positives. Tycho **179 m**, Copernicus 391 m, Apollo 12 300 m, Apollo 15 574 m, Apollo 17 622 m, Apollo 11 1383 m |
| TRN terminal (real LRO WAC z=8 + LOLA LDEM_64) | 6 targets, best per-target altitude 30-100 km, ~92 km mosaic | All 6 recover position, 0 false positives. Copernicus **30 m**, Tycho **32 m**, Apollo 17 43 m, Apollo 12 93 m, Apollo 15 131 m, Apollo 11 172 m. Median ~80 m, ~10x tighter than orbital |

`--pyramid-size 6 --neighbor-bins 1 --tolerance-arcsec 120` is the operational default for honest-density
HYG mag≤8 lost-in-space work.

## Build

Dependencies: CMake 3.20+, C++20 compiler, OpenCV 4 (`features2d`, `calib3d`, `imgcodecs`, `imgproc`),
Eigen3.

```bash
cd astro_navigation
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

## Quick Start

Run lunar visual odometry on a POLAR Traverse left-camera 50 ms sequence:

```bash
python3 scripts/download_dataset.py --dataset polar-traverse-view1 --output datasets --confirm-large
python3 scripts/prepare_polar_traverse.py \
  --root datasets/polar-traverse-view1/extracted --camera L --exposure-ms 50 \
  --output outputs/polar_view1_left_50ms

build/apps/lunar_visual_odometry \
  --images outputs/polar_view1_left_50ms/images.txt \
  --fx 1452.71 --fy 1452.88 --cx 999.53 --cy 1035.4 \
  --feature sift --clahe \
  --trajectory outputs/trajectory_sift.tum
```

Run lost-in-space star identification against a public HYG catalog subset:

```bash
python3 scripts/download_star_catalog.py --catalog hyg-v42 --output datasets/star_catalogs
python3 scripts/convert_star_catalog.py \
  --input datasets/star_catalogs/hyg-v42/raw/hyg_v42.csv.gz \
  --output datasets/star_catalogs/hyg-v42/converted/hyg_v42_mag8p0_unit.csv \
  --format hyg --max-magnitude 8.0

python3 scripts/build_star_pair_index.py \
  --catalog datasets/star_catalogs/hyg-v42/converted/hyg_v42_mag8p0_unit.csv \
  --output outputs/hyg_pair_index_40000.pkl --limit 40000 --skip-pkl

python3 scripts/identify_stars_with_pair_index.py \
  --index outputs/hyg_pair_index_40000.npz \
  --observations <observations_unlabeled.csv> \
  --output <assignments.csv> \
  --pyramid-size 6 --neighbor-bins 1 --tolerance-arcsec 120 \
  --fx 1000 --fy 1000 --cx 512 --cy 512
```

More detailed example commands (synthetic generators, ambiguity / robustness benchmarks, multi-traverse
suites) are in [`docs/space_localization.md`](docs/space_localization.md) and the benchmark scripts
under `benchmarks/`.

## Public Datasets

```bash
python3 scripts/download_dataset.py --list
```

- **NASA POLAR Traverse**: stereo traverses with poses and calibration —
  https://ti.arc.nasa.gov/dataset/PolarTrav/
- **NASA POLAR Stereo**: HDR stereo terrain with LiDAR ground truth —
  https://ti.arc.nasa.gov/dataset/IRG_PolarDB/
- **LunarLoc**: simulator traverses + `.lac` playback —
  https://github.com/mit-acl/lunarloc-data
- **Synthetic Lunar Terrain**: multimodal RGB/event/laser terrain —
  https://zenodo.org/records/13218780
- **Apollo Surface Panoramas** — https://catalog.data.gov/dataset/apollo-surface-panoramas
- **HYG Database v4.2** star catalog — https://codeberg.org/astronexus/hyg

Dataset licenses stay with the upstream providers; `manifest.json` records source URL, citation, and
checksum.

## Documentation

- [`docs/space_localization.md`](docs/space_localization.md) — primary modes, star tracker / TRN
  interfaces, near-term priorities.
- [`docs/experiments.md`](docs/experiments.md) — full experiment log: ORB vs SIFT, essential vs PnP,
  CLAHE, every HYG pair-index density iteration.
- [`docs/decisions.md`](docs/decisions.md) — design decisions and rationale.
- [`docs/interfaces.md`](docs/interfaces.md) — CSV, JSON, and binary interface contracts.
- [`PLAN.md`](PLAN.md) — current and upcoming work.

## Contributing

The most useful contributions are reproducible experiments, small C++ ports of proven Python paths,
dataset adapters, and focused benchmark fixes. See [`CONTRIBUTING.md`](CONTRIBUTING.md) for the
development loop and good first contribution areas.

## Roadmap

Navigation state health; star tracker catalog adapters; a star + VO + Skyline + TRN pose-graph fusion
landed (`scripts/four_factor_fusion_demo.py`), with Skyline and TRN as absolute factors whose
localizability cliffs are *terrain*-shaped rather than tidily symmetric — a two-scene converse-cliff
study (`scripts/converse_cliff_demo.py`) shows the horizon leading on Tycho's relief and the ground
texture carrying the flat Apollo 11 mare — a self-contained nonlinear factor-graph backend now carries heading as an SO(2) graph
state (`scripts/factor_graph_so2_demo.py`, Gauss-Newton on the circle), recovering attitude across a
star-tracker blackout where the linear positions-only solver could only dead-reckon it; that solver now
extends to full **SO(3) attitude plus a metric-scale state** (`scripts/factor_graph_so3_demo.py`,
Gauss-Newton on SO(3) × ℝ³ × ℝ₊), exposing one observability story per DOF class — gravity-held
roll/pitch, anchor-dependent yaw, and a VO scale that is recoverable only when absolute fixes bracket
the chain; next, stereo VO with PnP for the scale prior; crater descriptor matching against orbital
maps; visual-inertial fusion; LiDAR scan matching; the curvature-correct horizon model landed
(`scripts/skyline_curvature_demo.py`, `render_horizon(..., curvature_radius_m=...)`) — next, fold it
into the fusion matcher by default; orbital navigation with star tracker fusion; ROS 2 integration;
repeatable simulation benchmarks.

## References

- Hansen, M., Wong, U., and Fong, T. POLAR Traverse Dataset. NASA Ames Research Center, 2023.
- Wong, U., Nefian, A., Edwards, L., Buoyssounouse, X., Furlong, P. M., Deans, M., and Fong, T.
  POLAR Stereo Dataset. NASA Ames Research Center, 2017.
- LunarLoc: Segment-Based Global Localization on the Moon. https://arxiv.org/abs/2506.16940
- Synthetic Lunar Terrain: A Multimodal Open Dataset. https://arxiv.org/abs/2408.16971
