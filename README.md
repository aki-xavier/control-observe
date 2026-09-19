# control-observe — the estimator layer of the control stack

A project of its own: `../z1-arm` and `../g1-biped` (its consumers) depend on it as
sibling path dependencies, so no estimator lives in either tree and this crate can
be built, tested and released alone. MIT-licensed (see `LICENSE`).

Five modules, no plant and no engine:

```text
state_belief       what an estimator is allowed to know: q, its 2n x 2n
                   covariance P, and the gains it runs on
obs_vel_filter     a constant-velocity Kalman filter over an obstacle's
                   [px py pz vx vy vz]
on_manifold_ekf    the EKF over x = [q; qd] whose state update lives on the
                   configuration manifold
linear_joint_obs   the joint-encoder observation
pose_motor_obs     the end-effector pose observation, reported as a PGA motor
                   read through control-model's PgaFk
```

It depends on the crates below it — [`control-math`](../control-math) for the
arithmetic and [`control-model`](../control-model) for the `PgaFk` motion model
the pose observation reads — plus [`pga`](../pga). It has no `build.rs` and
needs no engine: a caller hands it measurements and it returns an estimate.

## Provenance

Extracted from `simu`'s `src/` at commit `872c8fd`. It left on the dependency
graph's own evidence:

- the cluster is **closed under itself** — of the five modules only
  `state_belief` is shared, and only inside (`on_manifold_ekf`,
  `linear_joint_obs` and `pose_motor_obs` all read it);
- its external edges are only the two crates below it, plus `pga`;
- and it is deliberately OUT of the legged loop: a walk simulation has no
  measurement noise, so it cannot judge an estimator — and the biped's
  `tests/layering.rs` asserts that no legged-path file
  names one of them. The layer's consumer is the arm's observation bench,
  `../z1-arm/src/bench/bench_avoid.rs`.

## Tests

`tests/observer.rs` came with the code: the obstacle filter's constant-velocity
track, the on-manifold EKF update, the joint and pose-motor observations against
their analytic predictions, and the deterministic noise stream. It is
engine-free, and it resolves the Z1 model through
`control_model::urdf::urdf_path()` (control-model owns `models/`).
