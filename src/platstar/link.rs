use crate::platstar::Platform;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Endpoint {
    From,
    To,
}

impl Endpoint {
    pub const fn opposite(&self) -> Self {
        match self {
            Endpoint::From => Endpoint::To,
            Endpoint::To => Endpoint::From,
        }
    }

    pub const fn to_index(&self) -> usize {
        match self {
            Endpoint::From => 0,
            Endpoint::To => 1,
        }
    }
}

pub trait Link {
    type P: Platform<Point = Self::Point>;
    type Point;

    fn endpoints(&self, p1: &Self::P, p2: &Self::P) -> (Self::Point, Self::Point);

    fn endpoint(&self, p: &Self::P, end: Endpoint) -> Self::Point;
}