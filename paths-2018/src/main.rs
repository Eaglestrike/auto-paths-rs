extern crate coord_frames;
use coord_frames::*;
#[macro_use]
extern crate dimensioned as dim;
use self::dim::si;

#[macro_use]
mod frames;
use self::frames::PathFrame;

const_meter!(BASELINE_TO_SWITCH_NEAR, 3.556);
const_meter!(BASELINE_TO_SWITCH_FAR, 4.9784);
const_meter!(BASELINE_TO_SCALE_PLATE_EDGE, 7.61111);
const_meter!(BASELINE_TO_PLATFORM, 6.641338);

const_meter!(MIDLINE_TO_EXCHANGE_EDGE, 0.3048);

const_meter!(WALL_TO_SCALE_PLATE_EDGE, 1.82245);
const_meter!(WALL_TO_SWITCH_EDGE, 2.16535);

const_meter!(SCALE_PLATE_WIDTH, 0.9144);
const_meter!(SCALE_PLATE_LENGTH, 1.2192);

const_meter!(FIELD_WIDTH, 8.2296);

// TODO update this
const_meter!(ROBOT_LENGTH, 0.9525);
const_meter!(ROBOT_WIDTH, 0.8636);

use std::f64::consts::PI;

const PI2: f64 = PI / 2.0;

fn main() {
    // create all things on the right, mirror for the left
    let center_start = TfPoint::new(
        PathFrame::Field,
        ROBOT_WIDTH / 2. - MIDLINE_TO_EXCHANGE_EDGE,
        ROBOT_LENGTH / 2.,
        PI2,
    );
    let side_start = TfPoint::new(
        PathFrame::Field,
        FIELD_WIDTH / 2. - WALL_TO_SWITCH_EDGE + ROBOT_WIDTH / 2.,
        ROBOT_LENGTH / 2.,
        PI2,
    );

    export_pose(center_start.raw_data(), "centerStart");
    export_pose(side_start.raw_data(), "rightSideStart");
    export_pose(side_start.raw_data().mirror(Axis::Y), "leftSideStart");

    let near_switch = TfPoint::new(
        PathFrame::Field,
        FIELD_WIDTH / 2. - WALL_TO_SWITCH_EDGE - SCALE_PLATE_WIDTH / 2.,
        BASELINE_TO_SWITCH_NEAR - ROBOT_LENGTH / 2.,
        PI2,
    );
    let far_switch = near_switch.mirror(Axis::Y);
    // faces from the outside looking in
    let near_scale = TfPoint::new(
        PathFrame::Field,
        FIELD_WIDTH / 2. - WALL_TO_SCALE_PLATE_EDGE + ROBOT_LENGTH / 2. + 0.254 * si::M,
        BASELINE_TO_SCALE_PLATE_EDGE + SCALE_PLATE_LENGTH / 2.,
        PI,
    );
    // comes in facing forward
    let far_scale = TfPoint::new(
        PathFrame::Field,
        -(FIELD_WIDTH / 2. - WALL_TO_SCALE_PLATE_EDGE + 0.1 * si::M),
        BASELINE_TO_SCALE_PLATE_EDGE - ROBOT_LENGTH / 2.,
        3. * PI / 10.,
    );

    let gen_center_switch = || {
        export_path(
            vec![
                zero_kappa(center_start.raw_data()),
                zero_kappa(far_switch.raw_data()),
            ],
            vec![basic_param(10.0)],
            "centerToLeftSwitch",
            300,
        );
        export_path(
            vec![
                zero_kappa(center_start.raw_data()),
                zero_kappa(near_switch.raw_data()),
            ],
            vec![basic_param(10.0)],
            "centerToRightSwitch",
            300,
        );
    };
    gen_center_switch();

    let gen_side_scale = || {
        use self::Axis::Y;
        // near scales
        export_path(
            vec![
                zero_kappa(side_start.raw_data()),
                with_kappa(near_scale.raw_data(), 2., 0.),
            ],
            vec![EtaParam::new(5., 10.0, 0., 20., 0., 0.)],
            "rightToRightScale",
            400,
        );
        export_path(
            vec![
                zero_kappa(side_start.raw_data().mirror(Y)),
                zero_kappa(near_scale.raw_data().mirror(Y)),
            ],
            vec![EtaParam::new(5., 10.0, 0., 20., 0., 0.)],
            "leftToLeftScale",
            400,
        );
        // far scales
        export_path(
            vec![
                zero_kappa(side_start.raw_data()),
                zero_kappa(
                    TfPoint::new(
                        PathFrame::Field,
                        FIELD_WIDTH / 2. - WALL_TO_SWITCH_EDGE / 2.,
                        BASELINE_TO_SWITCH_FAR,
                        PI2,
                    )
                    .raw_data(),
                ),
                with_kappa(far_scale.raw_data(), 0., 0.),
            ],
            vec![basic_param(5.0), EtaParam::new(15.0, 20.0, 0., 0., 0., 0.)],
            "rightToLeftScale",
            400,
        );
        export_path(
            vec![
                zero_kappa(side_start.raw_data().mirror(Y)),
                zero_kappa(
                    TfPoint::new(
                        PathFrame::Field,
                        FIELD_WIDTH / 2. - WALL_TO_SWITCH_EDGE / 2.,
                        BASELINE_TO_SWITCH_FAR,
                        PI2,
                    )
                    .raw_data()
                    .mirror(Y),
                ),
                with_kappa(far_scale.raw_data().mirror(Y), 0., 0.),
            ],
            vec![basic_param(5.0), EtaParam::new(15.0, 20.0, 0., 0., 0., 0.)],
            "leftToRightScale",
            400,
        );
    };
    gen_side_scale();

    // near switch
    let gen_near_switch = || {
        use self::Axis::Y;
        export_path(
            vec![
                zero_kappa(side_start.raw_data()),
                zero_kappa(near_switch.raw_data()),
            ],
            vec![basic_param(10.0)],
            "rightToRightSwitch",
            300,
        );
        export_path(
            vec![
                zero_kappa(side_start.raw_data().mirror(Y)),
                zero_kappa(near_switch.raw_data().mirror(Y)),
            ],
            vec![basic_param(10.0)],
            "leftToLeftSwitch",
            300,
        );
    };
    gen_near_switch();
}

fn with_kappa(point: PointData, k: f64, dk: f64) -> MotionState<f64> {
    MotionState {
        // convert m to feet for the robot
        x: *(point.x() / si::M) * 3.28084,
        y: *(point.y() / si::M) * 3.28084,
        t: point.rot(),
        k,
        dk,
    }
}

use std::fs;
fn export_pose(point: PointData, name: &str) {
    fs::write(
        format!("out/{}.java", name),
        format!(
            "public static Pose {} = new Pose({}, {}, {}, 0.0);",
            name,
            *(point.x() / si::M) * 3.28084,
            *(point.y() / si::M) * 3.28084,
            point.rot()
        ),
    )
    .unwrap();
}

fn zero_kappa(point: PointData) -> MotionState<f64> {
    with_kappa(point, 0., 0.)
}

fn basic_param(a: f64) -> EtaParam<f64> {
    assert!(a > 0.0);
    EtaParam::new(a, a, 0., 0., 0., 0.)
}

extern crate eta3_spline;
use eta3_spline::*;
extern crate csv;
fn export_path(
    points: Vec<MotionState<f64>>,
    params: Vec<EtaParam<f64>>,
    name: &str,
    num_pts: usize,
) {
    let path = EtaCurve::new(points.as_slice(), params.as_slice()).unwrap();
    let mut wtr = csv::WriterBuilder::new()
        .has_headers(false)
        .from_path(format!("out/{}.114path", name))
        .unwrap();
    wtr.write_record(&["x", "y", "distanceSoFar", "isEndPointInterpolation"])
        .unwrap();
    let mut t = 0.0;
    let dt = 1.0 / num_pts as f64;
    let mut dist = 0.0;

    let mut last_point = path.eval(0.0);
    while t < 1.0 {
        let point = path.eval(t);

        dist += f64::sqrt((point.0 - last_point.0).powi(2) + (point.1 - last_point.1).powi(2));
        last_point = point;

        wtr.serialize((point.0, point.1, dist, "False")).unwrap();
        t += dt;
    }
    // end point interpolation
    let last_state = points.get(points.len() - 1).unwrap();
    let dx = last_state.t.cos();
    let dy = last_state.t.sin();
    // add approximately 6 feet of endpoint interpolation
    const INTERP_DIST: f64 = 6.0;
    let mut p = 0.0;
    let dp = 0.1;

    while p < INTERP_DIST {
        wtr.serialize((
            last_state.x + dx * p,
            last_state.y + dy * p,
            dist + p,
            "True",
        ))
        .unwrap();
        p += dp;
    }

    match wtr.serialize((dx, dy)) {
        Err(e) => match e.into_kind() {
            csv::ErrorKind::UnequalLengths { .. } => (),
            _ => panic!(),
        },
        _ => (),
    };
}
