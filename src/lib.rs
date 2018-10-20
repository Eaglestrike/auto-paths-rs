#![feature(self_struct_ctor)]
extern crate dimensioned as dim;

#[cfg(test)]
#[macro_use]
extern crate assert_approx_eq;
use dim::si;
use std::marker::PhantomData;

type Meter = dim::si::Meter<f64>;
type Radians = f64;

// /// Represents a Rigid transformation in two dimensions, a rotation
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct PointData {
    pos: (Meter, Meter),
    rot: Radians,
}

impl PointData {
    pub(crate) fn inverse_relative_to(&self, other: Self) -> Self {
        // what do I need to transform other to to get where I am
        let s = self.pos;
        let t = other.rot;
        Self {
            pos: (
                s.0 * t.cos() - s.1 * t.sin() + other.pos.0,
                s.0 * t.sin() + s.1 * t.cos() + other.pos.1,
            ),
            rot: self.rot + other.rot,
        }
    }

    pub(crate) fn invert_parent_child_relation(&self) -> Self {
        let s = self.pos;
        let t = -self.rot;
        Self {
            pos: (
                -s.0 * t.cos() + s.1 * t.sin(),
                -s.0 * t.sin() - s.1 * t.cos(),
            ),
            rot: t,
        }
    }
}

impl Default for PointData {
    fn default() -> Self {
        Self {
            pos: (0. * si::M, 0. * si::M),
            rot: 0.,
        }
    }
}

#[cfg(test)]
mod point_data_test {
    use super::*;
    extern crate rand;
    use self::rand::{distributions::Uniform, Rng, SeedableRng, XorShiftRng};

    #[test]
    fn frame_relative_tos() {
        let p = PointData {
            pos: (1.7 * si::M, -1.5 * si::M),
            rot: 0.73,
        };
        let frame = PointData {
            pos: (-1.6 * si::M, 0.33 * si::M),
            rot: 0.27,
        };

        let prime = p.inverse_relative_to(frame);
        assert_approx_eq!(prime.pos.0 / si::M, 0.4385, 1e-4);
        assert_approx_eq!(prime.pos.1 / si::M, -0.6622, 1e-4);
    }

    #[test]
    fn invert_parent_child_relation() {
        fn test_point(f: PointData) {
            let fprime = f
                .invert_parent_child_relation()
                .invert_parent_child_relation();
            assert_approx_eq!(fprime.pos.0 / si::M, f.pos.0 / si::M, 1e-4);
            assert_approx_eq!(fprime.pos.1 / si::M, f.pos.1 / si::M, 1e-4);
            assert_approx_eq!(fprime.rot, f.rot, 1e-4);
            println!("Double inversion passed for {:?}", f);
        }

        let mut rng = XorShiftRng::from_seed([
            123, 243, 121, 35, 31, 76, 87, 123, 243, 121, 35, 205, 76, 87, 9, 14,
        ]);
        let dist = Uniform::new(-100.0, 100.0);
        let mut s = || rng.sample(dist);
        for _ in 0..50000 {
            test_point(PointData {
                pos: (s() * si::M, s() * si::M),
                rot: s(),
            });
        }
    }

    #[test]
    fn inverting_frame_relative_to() {
        fn test_point(p: PointData, frame: PointData) {
            let prime = p
                .inverse_relative_to(frame)
                .inverse_relative_to(frame.invert_parent_child_relation());
            assert_approx_eq!(prime.pos.0 / si::M, p.pos.0 / si::M, 1e-4);
            assert_approx_eq!(prime.pos.1 / si::M, p.pos.1 / si::M, 1e-4);
            assert_approx_eq!(prime.rot, p.rot, 1e-4);
            println!("Inverse relative to reversal passed for {:?}", p);
        }

        fn test_point_2(p: PointData, frame: PointData, frame2: PointData) {
            let prime = p
                .inverse_relative_to(frame)
                .inverse_relative_to(frame2)
                .inverse_relative_to(frame2.invert_parent_child_relation())
                .inverse_relative_to(frame.invert_parent_child_relation());
            assert_approx_eq!(prime.pos.0 / si::M, p.pos.0 / si::M, 1e-4);
            assert_approx_eq!(prime.pos.1 / si::M, p.pos.1 / si::M, 1e-4);
            assert_approx_eq!(prime.rot, p.rot, 1e-4);
            println!("Inverse relative to reversal passed for {:?}", p);
        }

        let mut rng = XorShiftRng::from_seed([
            123, 243, 121, 123, 31, 76, 87, 123, 243, 68, 35, 205, 76, 87, 9, 14,
        ]);
        let dist = Uniform::new(-100.0, 100.0);
        let mut s = || rng.sample(dist);
        for _ in 0..50000 {
            test_point(
                PointData {
                    pos: (s() * si::M, s() * si::M),
                    rot: s(),
                },
                PointData {
                    pos: (s() * si::M, s() * si::M),
                    rot: s(),
                },
            );
            test_point_2(
                PointData {
                    pos: (s() * si::M, s() * si::M),
                    rot: s(),
                },
                PointData {
                    pos: (s() * si::M, s() * si::M),
                    rot: s(),
                },
                PointData {
                    pos: (s() * si::M, s() * si::M),
                    rot: s(),
                },
            );
        }
    }
}

pub enum ParentFrame<S: PointHeirarchy + Sized> {
    Root,
    Parent(S),
}

pub trait PointHeirarchy: Sized + Copy + Into<usize> + Eq {
    fn parent(&self) -> ParentFrame<Self>;

    /// find a path between two nodes in a transformation heirarchy
    /// In the return tuple `(a, b)` it is guaranteed that `a.iter().last().parent() == b[1].parent()`
    // TODO(Lytigas) optimize to not go all the way to root each time
    fn path_to(&self, other: Self) -> (Vec<Self>, Vec<Self>) {
        let mut other_up = Vec::new();
        let mut current = other;
        loop {
            other_up.push(current);
            match current.parent() {
                ParentFrame::Root => {
                    break;
                }
                ParentFrame::Parent(x) => {
                    current = x;
                }
            };
        }
        let mut self_up = Vec::new();
        current = *self;
        loop {
            self_up.push(current);
            match current.parent() {
                ParentFrame::Root => {
                    break;
                }
                ParentFrame::Parent(x) => {
                    current = x;
                }
            };
        }
        other_up.reverse();
        (self_up, other_up)
    }

    fn order() -> usize;
    // fn registry() -> MutexGuard<'static, Vec<PointData>>;
}

pub struct FrameRegistry<S: PointHeirarchy>(Vec<PointData>, PhantomData<S>);

impl<S: PointHeirarchy> FrameRegistry<S> {
    pub fn new() -> Self {
        Self(vec![PointData::default(); S::order()], PhantomData)
    }

    pub fn set_relative_to_parent_raw(&mut self, frame: S, location: PointData) {
        *self.0.get_mut(frame.into()).unwrap() = location;
    }

    pub fn get_raw_tf(&self, frame: S) -> PointData {
        *self.0.get(frame.into()).unwrap()
    }
}

pub struct TfPoint<S: PointHeirarchy>(S, PointData);

impl<S: PointHeirarchy> TfPoint<S> {
    pub fn new(frame: S, x: Meter, y: Meter, rot: Radians) -> Self {
        Self(frame, PointData { pos: (x, y), rot })
    }

    pub fn in_frame(&self, register: FrameRegistry<S>, frame: S) -> Self {
        let (up, down) = self.0.path_to(frame);
        let mut result = self.1;
        up.iter()
            .for_each(|x| result = result.inverse_relative_to(register.get_raw_tf(*x)));
        down.iter().for_each(|x| {
            result =
                result.inverse_relative_to(register.get_raw_tf(*x).invert_parent_child_relation())
        });
        Self(frame, result)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    #[repr(usize)]
    pub enum PathFrames {
        Robot,
        Field,
        Camera,
        Switch,
        Scale,
        ScaleEst,
        CubeDepo,
    }

    impl From<PathFrames> for usize {
        fn from(p: PathFrames) -> usize {
            p as usize
        }
    }

    // static PATH_FRAMES_REGISTRY: Mutex<Vec<PointData>> = Mutex::new(Vec::new());

    // TODO enumify this and the child method
    impl PointHeirarchy for PathFrames {
        fn parent(&self) -> ParentFrame<Self> {
            use self::PathFrames::*;
            match *self {
                Robot => ParentFrame::Parent(Field),
                Switch => ParentFrame::Parent(Field),
                Scale => ParentFrame::Parent(Field),
                CubeDepo => ParentFrame::Parent(Switch),
                Camera => ParentFrame::Parent(Robot),
                Field => ParentFrame::Root,
                ScaleEst => ParentFrame::Parent(Camera),
            }
        }

        fn order() -> usize {
            7
        }

        // fn registry() -> MutexGuard<'static, Vec<PointData>> {
        //     PATH_FRAMES_REGISTRY.lock().unwrap()
        // }
    }

    #[test]
    fn test() {
        println!("{:?}", PathFrames::ScaleEst.path_to(PathFrames::CubeDepo));
        use self::PathFrames::*;
        assert_eq!(
            PathFrames::ScaleEst.path_to(PathFrames::CubeDepo),
            (
                vec![ScaleEst, Camera, Robot, Field],
                vec![Field, Switch, CubeDepo]
            )
        );

        assert_eq!(
            PathFrames::ScaleEst.path_to(PathFrames::Field),
            (vec![ScaleEst, Camera, Robot, Field], vec![Field])
        );
    }
}

// // ideal interface:
// /*
// let camera = Robot::add(x, y, orientation)
// Field::AddTransform(from=Robot, to=Field, tf=RigidTransform)
// camera.getAs::<Field>()
// */
// use std::rc::*;

// pub trait SysMarker where Self: Sized {
//     fn rootFrame() -> &'static CoordinateFrame<Self>;
// }

// pub struct CoordinateFrame<S: SysMarker> {
//     tag: PhantomData<S>,
//     parent: Origin<S>,
// }

// enum Origin<S: SysMarker> {
//     Root,
//     Origin(TfPoint<S>),
// }

// pub struct TfPoint<S: SysMarker> {
//     parent: Rc<CoordinateFrame<S>>,
//     data: PointData,
// }

// impl<S: SysMarker> TfPoint<S> {
//     fn inParentFrame(&self, frame: Rc<CoordinateFrame<S>>) -> Option<TfPoint<S>> {

//     }
// }

// struct FieldSystem;
// impl SysMarker for FieldSystem {}

// fn test() {
//     let field = Rc::new(CoordinateFrame {
//         tag: PhantomData,
//         parent: Origin::Root,
//     });
//     let point = TfPoint {
//         parent: field.clone(),
//         data: PointData {
//             pos: (0. * si::M, 0. * si::M),
//             rot: 0.,
//         },
//     };
//     let RobotFrame
// }
