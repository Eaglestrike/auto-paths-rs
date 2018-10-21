extern crate coord_frames;

fn main() {
    println!("Hello, world!");
}

mod frames {
    use coord_frames::*;
    // TODO macroify this and the impl trait
    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    #[repr(usize)]
    pub enum PathFrame {
        Field,
    }

    impl From<PathFrame> for usize {
        fn from(p: PathFrame) -> usize {
            p as usize
        }
    }

    impl PointHeirarchy for PathFrame {
        fn parent(&self) -> ParentFrame<Self> {
            match *self {
                PathFrame::Field => ParentFrame::Root,
            }
        }

        fn order() -> usize {
            1
        }
    }
}
