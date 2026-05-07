# GitHub About / Repo Metadata

Drafts for the GitHub repository **About** sidebar (description + topics + website). Pick whichever
description fits the framing; the topic list is the recommended set in either case.

## Description (≤ 350 chars)

### Short (recommended)

```
Space-native localization OSS in C++20: star tracker attitude, lost-in-space star identification at
HYG mag≤8 (40k indexed stars, 64/64 correct), lunar visual odometry on NASA POLAR, and terrain-relative
navigation. Python prototypes alongside the C++ apps.
```

### Concise alternative

```
C++20 OSS for GNSS-denied space localization: star trackers, lost-in-space ID, lunar VO, TRN.
```

## Website

If a project page is added later, point it at the docs landing or experiments log:

- `https://github.com/<owner>/astro_localization/blob/main/docs/space_localization.md`
- or the experiments log: `.../docs/experiments.md`

## Topics

Recommended GitHub topics (max 20; GitHub normalizes to lowercase, hyphenated):

- `space-robotics`
- `localization`
- `visual-odometry`
- `slam`
- `star-tracker`
- `lost-in-space`
- `attitude-estimation`
- `wahba-problem`
- `hyg-catalog`
- `lunar`
- `planetary-exploration`
- `terrain-relative-navigation`
- `crater-detection`
- `polar-dataset`
- `cpp20`
- `opencv`
- `eigen`
- `python`
- `gnss-denied`

## Setting these via gh CLI

```bash
gh repo edit <owner>/astro_localization \
  --description "Space-native localization OSS in C++20: star tracker attitude, lost-in-space star identification at HYG mag≤8 (40k indexed stars, 64/64 correct), lunar visual odometry on NASA POLAR, and terrain-relative navigation. Python prototypes alongside the C++ apps." \
  --add-topic space-robotics \
  --add-topic localization \
  --add-topic visual-odometry \
  --add-topic slam \
  --add-topic star-tracker \
  --add-topic lost-in-space \
  --add-topic attitude-estimation \
  --add-topic wahba-problem \
  --add-topic hyg-catalog \
  --add-topic lunar \
  --add-topic planetary-exploration \
  --add-topic terrain-relative-navigation \
  --add-topic crater-detection \
  --add-topic polar-dataset \
  --add-topic cpp20 \
  --add-topic opencv \
  --add-topic eigen \
  --add-topic python \
  --add-topic gnss-denied
```
