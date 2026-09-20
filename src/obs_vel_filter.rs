use control_math::mat::Mat;

#[derive(Clone, Debug)]
pub struct ObsVelFilter {
    pub dt: f64,
    pub q: f64,
    pub r: f64,
    pub x: Vec<f64>,
    pub p: Mat,
    pub init: bool,
}

impl ObsVelFilter {
    pub fn new(dt: f64, q: f64, r: f64) -> ObsVelFilter {
        ObsVelFilter {
            dt,
            q,
            r,
            x: vec![0.0; 6],
            p: Mat::eye(6),
            init: false,
        }
    }

    pub fn step(&mut self, z: &[f64]) -> (Vec<f64>, Vec<f64>) {
        if !self.init {
            self.x[..3].copy_from_slice(&z[..3]);
            self.init = true;
            return (self.x[..3].to_vec(), self.x[3..].to_vec());
        }
        let dt = self.dt;
        let mut a = Mat::eye(6);
        for i in 0..3 {
            a.set(i, 3 + i, dt);
        }
        let h = Mat::from_blocks(
            &Mat::eye(3),
            &Mat::zeros(3, 3),
            &Mat::zeros(3, 3),
            &Mat::zeros(3, 3),
        );
        let dt2 = dt * dt;
        let dt3 = dt2 * dt;
        let q11 = Mat::eye_scaled(self.q * dt3 / 3.0, 3);
        let q12 = Mat::eye_scaled(self.q * dt2 / 2.0, 3);
        let q22 = Mat::eye_scaled(self.q * dt, 3);
        let qm = Mat::from_blocks(&q11, &q12, &q12, &q22);
        let xp = a.mul_vec(&self.x);
        let pp = a.mul(&self.p).mul(&a.transposed()).add(&qm);
        let hppt = pp.mul(&h.transposed());
        let s_tmp = h.mul(&pp).mul(&h.transposed());
        let mut s = Mat::zeros(3, 3);
        for i in 0..3 {
            for j in 0..3 {
                s.set(i, j, s_tmp.at(i, j));
            }
            s.set(i, i, s.at(i, i) + self.r);
        }
        let s_inv = s.inv();
        let k = hppt.mul(&s_inv);
        let mut innov = vec![0.0; 3];
        for i in 0..3 {
            innov[i] = z[i] - xp[i];
        }
        let dx = k.mul_vec(&innov);
        for i in 0..6 {
            self.x[i] = xp[i] + dx[i];
        }
        let kh = k.mul(&h);
        let mut i6 = Mat::eye(6);
        for i in 0..6 {
            for j in 0..6 {
                i6.set(i, j, i6.at(i, j) - kh.at(i, j));
            }
        }
        self.p = i6.mul(&pp);
        (self.x[..3].to_vec(), self.x[3..].to_vec())
    }
}
