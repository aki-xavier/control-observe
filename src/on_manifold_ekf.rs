use crate::state_belief::StateBelief;
use control_math::mat::Mat;

#[derive(Clone, Debug)]
pub struct OnManifoldEKF {
    pub belief: StateBelief,
    pub sigma_a: f64,
}

impl OnManifoldEKF {
    pub fn new(belief: StateBelief, sigma_a: f64) -> OnManifoldEKF {
        OnManifoldEKF { belief, sigma_a }
    }

    pub fn predict(&mut self, dt: f64) {
        let n = self.belief.n;
        let inx = Mat::eye(n);
        let f = Mat::from_blocks(&inx, &Mat::eye_scaled(dt, n), &Mat::zeros(n, n), &inx);
        let xp = f.mul_vec(&self.belief.x());
        self.belief.q[..n].copy_from_slice(&xp[..n]);
        self.belief.v[..n].copy_from_slice(&xp[n..2 * n]);
        let dt2 = dt * dt;
        let dt3 = dt2 * dt;
        let dt4 = dt2 * dt2;
        let sig2 = self.sigma_a * self.sigma_a;
        let qm = Mat::from_blocks(
            &Mat::eye_scaled(sig2 * dt4 / 4.0, n),
            &Mat::eye_scaled(sig2 * dt3 / 2.0, n),
            &Mat::eye_scaled(sig2 * dt3 / 2.0, n),
            &Mat::eye_scaled(sig2 * dt2, n),
        );
        self.belief.p = f.mul(&self.belief.p).mul(&f.transposed()).add(&qm);
    }

    pub fn update(&mut self, h: &Mat, r_vec: &[f64], r_mat: &Mat) {
        let n = self.belief.n;
        let m = r_vec.len();
        let hp = h.mul(&self.belief.p);
        let s_tmp = hp.mul(&h.transposed());
        let mut s = Mat::zeros(m, m);
        for i in 0..m {
            for j in 0..m {
                s.set(i, j, s_tmp.at(i, j) + r_mat.at(i, j));
            }
        }
        let s_inv = s.inv();
        let k = hp.transposed().mul(&s_inv);
        let dx = k.mul_vec(r_vec);
        for i in 0..n {
            self.belief.q[i] += dx[i];
            self.belief.v[i] += dx[n + i];
        }
        let kh = k.mul(h);
        let mut i2n = Mat::eye(2 * n);
        for i in 0..2 * n {
            for j in 0..2 * n {
                i2n.set(i, j, i2n.at(i, j) - kh.at(i, j));
            }
        }
        self.belief.p = i2n.mul(&self.belief.p).symmetrized();
    }
}
