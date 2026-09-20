// control-observe — the estimator layer of the control stack, as a project of its own.
//
// It was extracted from the simu crate's `src/` once the dependency graph made the order obvious:
// the cluster is closed under itself (only `state_belief` is shared, and only inside), and its
// external edges are the two crates below it — `control-math` for the arithmetic, `control-model`
// for the `PgaFk` motion model the pose observation reads — plus `pga`.
//
// It stays OUT of the legged loop: a walk simulation has no measurement noise, so it cannot judge
// an estimator. A caller hands the layer measurements and it returns an estimate, which is why it
// carries no plant and no engine.

pub mod linear_joint_obs;
pub mod obs_vel_filter;
pub mod on_manifold_ekf;
pub mod pose_motor_obs;
pub mod state_belief;
