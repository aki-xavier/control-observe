// control-observe — the estimator layer of the control stack, as a project of its own.
//
// Five modules and nothing else: `state_belief` (what an estimator is allowed to know — q, its
// covariance, and the gains it runs on) and the four readers and filters built over it:
// `obs_vel_filter` (a constant-velocity Kalman filter over an obstacle's position),
// `on_manifold_ekf` (the EKF update whose state lives on the configuration manifold),
// `linear_joint_obs` (a joint-space linear observation) and `pose_motor_obs` (the pose observation
// that reads a `PgaFk` motor). It carries no plant and no engine: a caller hands it measurements
// and it returns an estimate.
//
// It was extracted from the simu crate's `src/` once the dependency graph made the order obvious:
// the cluster is closed under itself (only `state_belief` is shared, and only inside), and its
// external edges are the two crates below it — `control-math` for the arithmetic, `control-model`
// for the `PgaFk` motion model the pose observation reads — plus `pga`.
//
// GA_PID_AUDIT.md #14 keeps this layer OUT of the legged loop: an estimator is judged by an
// observation bench, never by the walk (`simu`'s tests/layering.rs asserts it, and its
// bench/bench_avoid.rs is the layer's consumer). simu consumes this as a sibling path dependency
// (`{ path = "../control-observe" }`).

pub mod linear_joint_obs;
pub mod obs_vel_filter;
pub mod on_manifold_ekf;
pub mod pose_motor_obs;
pub mod state_belief;
