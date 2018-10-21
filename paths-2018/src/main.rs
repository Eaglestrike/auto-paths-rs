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

const_meter!(WALL_TO_SCALE_PLATE_EDGE, 1.82245);
const_meter!(WALL_TO_SWITCH_EDGE, 2.16535);

const_meter!(SCALE_PLATE_WIDTH, 0.9144);
const_meter!(SCALE_PLATE_LENGTH, 1.2192);
const_meter!(SWITCH_WIDTH, 1.2192);

const_meter!(FIELD_WIDTH, 8.213852);

// TODO update this
const_meter!(ROBOT_LENGTH, 1.2192);
const_meter!(WIDTH, 0.9144);

use std::f64::consts::PI;

const PI2: f64 = PI / 2.0;

fn main() {
    // create all things on the right, mirror for the left
    let center_start = TfPoint::new(PathFrame::Field, 0. * si::M, ROBOT_LENGTH / 2., PI2);
    let side_start = TfPoint::new(PathFrame::Field, SWITCH_WIDTH / 2., ROBOT_LENGTH / 2.0, PI2);

    let close_switch = TfPoint::new(
        PathFrame::Field,
        SWITCH_WIDTH / 2. - SCALE_PLATE_WIDTH / 2.,
        BASELINE_TO_SWITCH_NEAR - ROBOT_LENGTH / 2.,
        PI2,
    );
    let far_switch = close_switch.mirror(Axis::Y);
    // faces from the outside looking in
    let near_scale = TfPoint::new(
        PathFrame::Field,
        FIELD_WIDTH / 2. - WALL_TO_SCALE_PLATE_EDGE + ROBOT_LENGTH / 2. + 0.4 * si::M,
        BASELINE_TO_SCALE_PLATE_EDGE + SCALE_PLATE_LENGTH / 2.,
        PI,
    );
    // comes in facing forward
    let far_scale = TfPoint::new(
        PathFrame::Field,
        -(FIELD_WIDTH / 2. - WALL_TO_SCALE_PLATE_EDGE - SCALE_PLATE_WIDTH / 2.),
        BASELINE_TO_SCALE_PLATE_EDGE - ROBOT_LENGTH / 2. - 0.4 * si::M,
        PI2,
    );
}
