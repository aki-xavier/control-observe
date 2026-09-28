use crate::state_belief::StateBelief;
use control_math::mat::Mat;

#[derive(Clone, Debug)]
pub struct LinearJointObs {
    pub n: usize,
    pub h: Mat,
    pub r_mat: Mat,
    pub z: Vec<f64>,
}

impl LinearJointObs {
    pub fn new(z: &[f64], n: usize, sigma: f64) -> LinearJointObs {
        let mut h = Mat::zeros(n, 2 * n);
        h.set_block(&Mat::eye(n), 0, 0);
        LinearJointObs {
            n,
            h,
            r_mat: Mat::eye_scaled(sigma * sigma, n),
            z: z.to_vec(),
        }
    }

    pub fn residual(&self, b: &StateBelief) -> Vec<f64> {
        let bx = b.x();
        let mut r = vec![0.0; self.n];
        for i in 0..self.n {
            r[i] = self.z[i] - bx[i];
        }
        r
    }

    pub fn jacobian(&self) -> Mat {
        self.h.clone()
    }

    pub fn r(&self) -> Mat {
        self.r_mat.clone()
    }
}
