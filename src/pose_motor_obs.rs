use crate::state_belief::StateBelief;
use control_math::mat::Mat;
use control_model::pga_fk::PgaFk;
use control_model::pga_layer::pga_biv_to_axial;
use pga::Multivector;

#[derive(Clone, Debug)]
pub struct PoseMotorObs {
    pub fk: PgaFk,
    pub n: usize,
    pub z: Multivector,
    pub r_mat: Mat,
    pub eps: f64,
}

fn log_components(b: Multivector) -> [f64; 6] {
    let (w, v) = pga_biv_to_axial(b);
    [w[0], w[1], w[2], v[0], v[1], v[2]]
}

impl PoseMotorObs {
    pub fn new(fk: PgaFk, n: usize, sigma: f64, eps_fd: f64, z: Multivector) -> PoseMotorObs {
        PoseMotorObs {
            fk,
            n,
            z,
            r_mat: Mat::eye_scaled(sigma * sigma, 6),
            eps: eps_fd,
        }
    }

    fn residual_at(&self, q: &[f64]) -> [f64; 6] {
        let m_hat = self.fk.motor(q);
        let rel = m_hat.reverse().gp(self.z);
        let lg = rel.log();
        let lgc = log_components(lg);
        let r: [f64; 6] = std::array::from_fn(|i| -lgc[i]);
        r
    }

    pub fn residual(&self, b: &StateBelief) -> Vec<f64> {
        self.residual_at(&b.q).to_vec()
    }

    pub fn jacobian(&self, b: &StateBelief) -> Mat {
        let mut jq = Mat::zeros(6, self.n);
        for i in 0..self.n {
            let mut qp = b.q.clone();
            let mut qm = b.q.clone();
            qp[i] += self.eps;
            qm[i] -= self.eps;
            let rp = self.residual_at(&qp);
            let rm = self.residual_at(&qm);
            for k in 0..6 {
                jq.set(k, i, -(rp[k] - rm[k]) / (2.0 * self.eps));
            }
        }
        let mut j = Mat::zeros(6, 2 * self.n);
        j.set_block(&jq, 0, 0);
        j
    }

    pub fn r(&self) -> Mat {
        self.r_mat.clone()
    }
}
