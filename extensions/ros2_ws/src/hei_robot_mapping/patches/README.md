# RTAB-Map numerical compatibility patch

Base RTAB-Map commit: `25d2135b266761cfd2c0c1e772775a470802e4b0`.

`rtabmap-g2o-information-symmetry.patch` repairs only negligible asymmetry
(relative Frobenius tolerance 1e-10) of finite 6D graph-edge information matrices
before passing them to g2o. Full FAST-LIO covariance inversion can produce this
round-off error even when its input is symmetric and positive definite.
Diagonal values and covariance weights are retained. Significant asymmetry,
non-finite values and non-positive-definite matrices remain subject to g2o's
original validation. No localization rejection threshold is disabled.

The patch is already applied to the AGX RTAB-Map checkout. For a fresh checkout
at the base commit, apply from its root, then rebuild rtabmap:

```bash
git apply ../hei_robot_mapping/patches/rtabmap-g2o-information-symmetry.patch
```

Rebuilding requires restarting RTAB-Map for the new library to take effect.
