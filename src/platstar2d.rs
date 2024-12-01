//! Some types that provide a basic implementation of PlatStar for a 2D platformer

use crate::platstar::*;
use petgraph::graph::DiGraph;
use glam::{Vec2, Mat2};


/// A basic, platform that occupies a line segment in 2D space
#[derive(Clone, Debug, PartialEq)]
pub struct Platform2D {
    pub start: Vec2,
    pub end: Vec2,
    pub weight: f32,
}

impl Platform for Platform2D {
    type Point = Vec2;

    fn closest_point(&self, p: &Vec2) -> Vec2 {
        if self.start == self.end {
            return self.start;
        }

        let dir = self.end - self.start;
        let v = *p - self.start;
        
        let mut t = v.dot(dir) / dir.length_squared();
        t = t.clamp(0.0, 1.0);

        self.start + dir * t
    }

    fn closest_below(&self, down: &Vec2, p: &Vec2) -> Vec2 {
        // find the poisition where the two lines intersect
        let dir = self.end - self.start;
        let m = Mat2::from_cols(dir, *down);
        if m.determinant().abs() < 0.0001 {
            return self.closest_point(p);
        }

        let inv = m.inverse();
        let v = *p - self.start;

        let mut t = inv * v;

        t.x = t.x.clamp(0.0, 1.0);
        return self.start + dir * t.x;
        
    }
}

impl Platform2D {
    pub fn new(start: Vec2, end: Vec2) -> Self {
        Self { start, end, weight: 1.0 }
    }

    pub fn new_weighted(start: Vec2, end: Vec2, weight: f32) -> Self {
        Self { start, end, weight }
    }
}

#[derive(Clone, Debug, PartialEq)]
/// A basic link between two Platform2Ds
pub struct Link2D {
    /// The type of link this is
    pub link_type: Link2DType,

    pub t_from: f32,
    pub t_to: f32,

    pub weight: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
/// The type of link between 2 platforms. Affects whether, an how, an agent my cross the link
pub enum Link2DType {
    Walk,
    Jump,
    Fall,
}

impl Link for Link2D {
    type P = Platform2D;
    type Point = Vec2;

    fn endpoints(&self, p1: &Self::P, p2: &Self::P) -> (Vec2, Vec2) {
        (
            p1.start.lerp(p1.end, self.t_from),
            p2.start.lerp(p2.end, self.t_to),
        )
    }

    fn endpoint(&self, p: &Self::P, end: Endpoint) -> Vec2 {
        match end {
            Endpoint::From => p.start.lerp(p.end, self.t_from),
            Endpoint::To => p.start.lerp(p.end, self.t_to),
        }
    }
}

impl Link2D {
    pub fn new_weighted(link_type: Link2DType, t_from: f32, t_to: f32, weight: f32) -> Self {
        Self { link_type, t_from, t_to, weight }
    }

    pub fn new(link_type: Link2DType, t_from: f32, t_to: f32) -> Self {
        Self::new_weighted(link_type, t_from, t_to, 1.0)
    }
}

impl Point for Vec2 {
    fn distance_squared(&self, other: &Self) -> f32 {
        (self - other).length_squared()
    }

    fn distance(&self, other: &Self) -> f32 {
        (self - other).length()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Agent2D {
    pub jump_height: f32,
}

impl Agent for Agent2D {
    type Point = Vec2;
    type P = Platform2D;
    type L = Link2D;

    fn can_cross(&self, platgraph: &DiGraph<Self::P, Self::L>, step: &Step<Self::Point>) -> bool {
        self.cost_profile(platgraph, step, None).is_some()
    }

    /// Whether or not the agent can perform the step, and it's acciociated costs, all in one fuction!
    fn cost_profile(&self, platgraph: &DiGraph<Self::P, Self::L>, step: &Step<Self::Point>, dest: Option<&Self::Point>) -> Option<CostProfile> {
        let (from_pos, to_pos) = match step {
            Step::ToLink(plat, pos, link) => {
                let plat = &platgraph[*plat];
                let link = &platgraph[*link];
                let pos2 = link.endpoint(plat, Endpoint::From);
                (*pos, pos2)
            },
            Step::FromLink(plat, pos, link) => {
                let plat = &platgraph[*plat];
                let link = &platgraph[*link];
                let pos2 = link.endpoint(plat, Endpoint::To);
                (pos2, *pos)
            },
            Step::BetweenLinks(link1, link2) => {
                let plat = platgraph.edge_endpoints(*link1).unwrap().1;
                let plat = &platgraph[plat];
                let link1 = &platgraph[*link1];
                let link2 = &platgraph[*link2];
                let pos1 = link1.endpoint(plat, Endpoint::To);
                let pos2 = link2.endpoint(plat, Endpoint::From);
                (pos1, pos2)
            },
            Step::AccrossLink(link) => {
                let (plat1, plat2) = platgraph.edge_endpoints(*link).unwrap();
                let (plat1, plat2) = (&platgraph[plat1], &platgraph[plat2]);
                let link = &platgraph[*link];
                let pos1 = link.endpoint(plat1, Endpoint::To);
                let pos2 = link.endpoint(plat2, Endpoint::From);
                if self.jump_height < (pos2.y - pos1.y) {
                    return None;
                }
                (pos1, pos2)
            },
            Step::AccrossPlatform(_plat, pos1, pos2) => (*pos1, *pos2)
        };

        let cost = from_pos.distance_squared(to_pos);
        let heuristic = dest.map(|dest| to_pos.distance_squared(*dest)).unwrap_or(0.0);
        Some(CostProfile { cost, heuristic })
    }

    /// The direction the agent falls. Opposite of the direction the agent jumps
    fn down(&self) -> Self::Point {
        Vec2::new(0.0, -1.0)
    }
}

pub type PlatStar2D = PlatStar<Platform2D, Link2D, Vec2>;