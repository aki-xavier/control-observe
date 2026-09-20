use control_math::mat::Mat;
use control_math::rng::Mt19937;
use control_model::pga_fk::PgaFk;
use control_model::urdf::home_q;
use control_observe::linear_joint_obs::LinearJointObs;
use control_observe::obs_vel_filter::ObsVelFilter;
use control_observe::on_manifold_ekf::OnManifoldEKF;
use control_observe::pose_motor_obs::PoseMotorObs;
use control_observe::state_belief::StateBelief;

#[test]
fn obs_vel_filter_tracks_constant_velocity() {
    let mut f = ObsVelFilter::new(1e-3, 1e-4, 4e-6);
    for k in 0..300 {
        let t = 1e-3 * k as f64;
        let z = [0.1 + 0.2 * t, 0.0, 0.0];
        f.step(&z);
    }
    let (pf, vf) = f.step(&[0.1 + 0.2 * 0.301, 0.0, 0.0]);
    assert!((pf[0] - (0.1 + 0.2 * 0.301)).abs() < 1e-3);
    assert!((vf[0] - 0.2).abs() < 0.02);
}

#[test]
fn ekf_linear_update() {
    let b = StateBelief::new(2, &[0.0, 0.0], &Mat::eye(4));
    let mut e = OnManifoldEKF::new(b, 0.01);
    e.predict(0.01);
    let obs = LinearJointObs::new(&[1.0, -0.5], 2, 0.1);
    e.update(&obs.jacobian(), &obs.residual(&e.belief), &obs.r());
    assert!((e.belief.q[0] - 1.0).abs() < 0.2);
    assert!(e.belief.p.at(0, 0) < 1.0);
}

#[test]
fn ekf_pose_obs_consistent() {
    let urdf = control_model::urdf::urdf_path();
    let fk = PgaFk::new(&urdf, "link00", "link06").expect("z1 pga fk");
    let home = home_q();
    let z = fk.motor(&home);
    let b = StateBelief::new(6, &home, &Mat::eye(12));
    let obs = PoseMotorObs::new(fk, 6, 1e-3, 1e-6, z);
    let r = obs.residual(&b);
    for i in 0..6 {
        assert!(
            r[i].abs() < 1e-6,
            "the residual at the belief's own pose is {} at component {i}",
            r[i]
        );
    }
    let j = obs.jacobian(&b);
    for i in 0..6 {
        for jj in 0..12 {
            assert!(!j.at(i, jj).is_nan());
        }
    }
}

#[test]
fn mt19937_deterministic() {
    let mut r1 = Mt19937::new(0);
    let mut r2 = Mt19937::new(0);
    for _ in 0..100 {
        assert_eq!(r1.next_f64(), r2.next_f64());
    }
    let mut sum = 0.0;
    let mut sum2 = 0.0;
    let mut r = Mt19937::new(0);
    for _ in 0..20000 {
        let x = r.randn();
        sum += x;
        sum2 += x * x;
    }
    let mean = sum / 20000.0;
    let std = (sum2 / 20000.0 - mean * mean).sqrt();
    assert!(mean.abs() < 0.05, "the generator's mean is {mean}");
    assert!((std - 1.0).abs() < 0.05, "the generator's std is {std}");
}

/// The first call is a SEED, not an update (position := observation, velocity := 0); nothing else here touches a cold filter.
#[test]
fn the_first_observation_seeds_the_filter() {
    let mut f = ObsVelFilter::new(1e-3, 1e-4, 4e-6);
    assert!(!f.init);
    let (p, v) = f.step(&[0.7, -0.2, 0.05]);
    assert!(f.init);
    assert_eq!(p, vec![0.7, -0.2, 0.05]);
    assert_eq!(v, vec![0.0, 0.0, 0.0]);
    assert_eq!(f.x.len(), 6);
    assert_eq!(f.x[0], 0.7);
}
