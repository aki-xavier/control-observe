// state_belief.rs — StateBelief, the joint-state belief (q, v) with the 2n x 2n covariance P;
// estimator layer for the GA observation benchmarks (bench_avoid_est), pure math, no engine dependency.

use control_math::mat::Mat;

#[derive(Clone, Debug)]
pub struct StateBelief {
    pub n: usize,
    pub q: Vec<f64>,
    pub v: Vec<f64>,
    pub p: Mat,
}

impl StateBelief {
    pub fn new(n: usize, q0: &[f64], p0: &Mat) -> StateBelief {
        let mut q = vec![0.0; n];
        if !q0.is_empty() {
            q[..n].copy_from_slice(&q0[..n]);
        }
        let mut p = Mat::eye(2 * n);
        if p0.rows > 0 {
            p = p0.clone();
        }
        StateBelief {
            n,
            q,
            v: vec![0.0; n],
            p,
        }
    }

    /// x returns the full joint state [q; v].
    pub fn x(&self) -> Vec<f64> {
        let mut x = vec![0.0; 2 * self.n];
        for i in 0..self.n {
            x[i] = self.q[i];
            x[self.n + i] = self.v[i];
        }
        x
    }
}
