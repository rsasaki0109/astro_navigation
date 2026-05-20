# Demo Gallery

This page collects the visual demos in one place. README keeps the main story short; this gallery is
the quick index for videos, GIFs, still previews, and JSON sidecars.

## Mission Navigation

### Confidence-aware replanning

Route planning uses both traversability and TRN localizability. The rover detects a dynamic hazard,
replans through terrain with stronger TRN confidence, and reports route-level navigation risk.

- [MP4 video](figures/confidence_aware_replanning_demo.mp4)
- [GIF animation](figures/confidence_aware_replanning_demo.gif)
- [JSON summary](figures/confidence_aware_replanning_demo.json)

![Confidence-aware replanning preview](figures/confidence_aware_replanning_preview.png)

### Dynamic hazard replanning

The rover follows a hazard-aware route, detects a newly blocked segment, marks the route invalid, and
replans around the obstacle with the C++ `hazard_route_demo` planner.

- [MP4 video](figures/dynamic_hazard_replanning_demo.mp4)
- [GIF animation](figures/dynamic_hazard_replanning_demo.gif)
- [JSON summary](figures/dynamic_hazard_replanning_demo.json)

![Dynamic hazard replanning](figures/dynamic_hazard_replanning_demo.gif)

### Hazard-aware lunar navigation

The Tycho terminal TRN fixture is converted into an image-derived hazard cost map. The C++ planner
routes around high-cost terrain while the navigation state moves through lock, relocalization, and
arrival phases.

- [MP4 video](figures/hazard_aware_navigation_demo.mp4)
- [GIF animation](figures/hazard_aware_navigation_demo.gif)
- [JSON summary](figures/hazard_aware_navigation_demo.json)

![Hazard-aware lunar navigation](figures/hazard_aware_navigation_demo.gif)

### Lost Robot Challenge

A GNSS-denied lunar robot receives one synthetic star-camera frame and one lunar nadir frame, then
recovers attitude and position into a single mission-control card.

- [PNG card](figures/lost_robot_challenge.png)
- [JSON summary](figures/lost_robot_challenge.json)

![Lost Robot Challenge](figures/lost_robot_challenge.png)

### Navigation replay

The navigation state machine starts lost, gains star-camera attitude, then reaches a full TRN position
lock over Tycho with a conservative sigma circle.

- [GIF animation](figures/navigation_replay_demo.gif)
- [JSON summary](figures/navigation_replay_demo.json)

![Navigation replay](figures/navigation_replay_demo.gif)

## TRN And Localizability

### TRN confidence heatmap

A localizability map over the Tycho ortho fixture. It scores where terrain-relative navigation should
have stronger lock potential from gradient energy, local texture, feature density, and illumination.

- [PNG heatmap](figures/trn_confidence_heatmap.png)
- [JSON summary](figures/trn_confidence_heatmap.json)

![TRN confidence heatmap](figures/trn_confidence_heatmap.png)

### Localizability-aware routing

Compares a hazard-only route with a route that keeps the same blocked terrain while adding cost for
weak TRN localizability.

- [PNG comparison](figures/localizability_aware_route.png)
- [JSON summary](figures/localizability_aware_route.json)

![Localizability-aware route](figures/localizability_aware_route.png)

### TRN trajectory

Frame-by-frame position recovery on a Tycho descent trajectory. Each frame solves PnP from a nadir
image against the LRO ortho without an inertial prior or temporal filter.

- [GIF animation](figures/trn_trajectory_demo.gif)

![TRN trajectory](figures/trn_trajectory_demo.gif)

### Lunar landing mission

Star tracker attitude and TRN position recovery shown together across six descent moments from
orbital insertion to touchdown burn.

- [GIF animation](figures/mission_demo.gif)

![Lunar landing mission](figures/mission_demo.gif)

## Star Tracker And VO

### Lost-in-space identification

Synthetic attitudes across recognizable constellations, with recovered star IDs, constellation lines,
and labels.

- [GIF animation](figures/lost_in_space_demo.gif)
- [PNG still](figures/lost_in_space_demo.png)

![Lost-in-space identification](figures/lost_in_space_demo.gif)

### POLAR visual odometry

NASA POLAR traverse imagery rendered with SIFT features and a visual-odometry trajectory overlay.

- [GIF animation](figures/polar_traverse1_vo_demo.gif)
- [PNG still](figures/polar_traverse1_vo_demo.png)
- [Feature snapshot](figures/polar_traverse1_features.png)
- [Sample frame](figures/polar_traverse1_sample.png)

![POLAR visual odometry](figures/polar_traverse1_vo_demo.gif)
