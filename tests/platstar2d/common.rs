use platstar::platstar2d::*;
use platstar::Step;
use glam::Vec2;

use core::convert::AsRef;

#[test]
fn test_add() {
    assert_eq!(3 + 2, 5);
}

/// A PlatStar graph with pre calculated shortest routes used to test the correctness of the pathfinding algorithm
pub trait PlatStarTest {
    fn graph() -> PlatStar2D;
    fn paths() -> Vec<PlatStarTestPath>;
}

pub struct PlatStarTestPath {
    // The agent that traversed the path
    pub agent: Agent2D,
    // something to identify the path
    pub name: String,
    pub start: Vec2,
    pub end: Vec2,
    pub steps: Vec<Step<Vec2>>,
}

impl PlatStarTestPath {
    pub fn new<S: AsRef<str> + ?Sized>(name: &S, agent: Agent2D, start: Vec2, end: Vec2, steps: Vec<Step<Vec2>>) -> Self {
        Self {
            name: name.as_ref().to_string(),
            agent,
            start,
            end,
            steps,
        }
    }
}

/// A PlatStar 2D graph with a small number of platforms ansic edge types
pub struct BasicTest;
impl PlatStarTest for BasicTest {
    fn graph() -> PlatStar2D {
        let mut platstar = PlatStar2D::new();

        let ps = [
            // bottom plat
            platstar.add_platform(Platform2D::new(Vec2::new(-25.0, -40.0), Vec2::new(25.0, -40.0))),
            platstar.add_platform(Platform2D::new(Vec2::new(-50.0, -20.0), Vec2::new(-25.0, -40.0))),
            platstar.add_platform(Platform2D::new(Vec2::new(25.0, -40.0), Vec2::new(50.0, -20.0))),
            // accessible by jumping
            platstar.add_platform(Platform2D::new(Vec2::new(-25.0, -10.0), Vec2::new(25.0, -10.0))),
            platstar.add_platform(Platform2D::new(Vec2::new(-30.0, 30.0), Vec2::new(30.0, 30.0))),
        ];

        let _ls = [
            platstar.add_link(ps[0], ps[1], Link2D::new(Link2DType::Walk, 0.0, 1.0)),
            platstar.add_link(ps[1], ps[0], Link2D::new(Link2DType::Walk, 1.0, 0.0)),
            platstar.add_link(ps[0], ps[2], Link2D::new(Link2DType::Walk, 1.0, 0.0)),
            platstar.add_link(ps[2], ps[0], Link2D::new(Link2DType::Walk, 0.0, 1.0)),

            platstar.add_link(ps[0], ps[3], Link2D::new(Link2DType::Jump, 0.5, 0.5)),
            platstar.add_link(ps[3], ps[0], Link2D::new(Link2DType::Fall, 0.5, 0.5)),
            platstar.add_link(ps[0], ps[3], Link2D::new(Link2DType::Jump, 0.0, 0.0)),
            platstar.add_link(ps[3], ps[0], Link2D::new(Link2DType::Fall, 0.0, 0.0)),
            platstar.add_link(ps[0], ps[3], Link2D::new(Link2DType::Jump, 1.0, 1.0)),
            platstar.add_link(ps[3], ps[0], Link2D::new(Link2DType::Fall, 1.0, 1.0)),

            platstar.add_link(ps[1], ps[3], Link2D::new(Link2DType::Jump, 0.5, 0.0)),
            platstar.add_link(ps[3], ps[1], Link2D::new(Link2DType::Fall, 0.0, 0.75)),
            platstar.add_link(ps[2], ps[3], Link2D::new(Link2DType::Jump, 0.5, 1.0)),
            platstar.add_link(ps[3], ps[2], Link2D::new(Link2DType::Fall, 1.0, 0.25)),

            platstar.add_link(ps[3], ps[4], Link2D::new(Link2DType::Jump, 0.5, 0.5)),
            platstar.add_link(ps[4], ps[3], Link2D::new(Link2DType::Fall, 0.5, 0.5)),
            platstar.add_link(ps[3], ps[4], Link2D::new(Link2DType::Jump, 0.0, 0.0)),
            platstar.add_link(ps[4], ps[3], Link2D::new(Link2DType::Fall, 0.0, 0.0)),
            platstar.add_link(ps[3], ps[4], Link2D::new(Link2DType::Jump, 1.0, 1.0)),
            platstar.add_link(ps[4], ps[3], Link2D::new(Link2DType::Fall, 1.0, 1.0)),

        ];

        platstar
    }

    fn paths() -> Vec<PlatStarTestPath> {
        let low_jumper = Agent2D { jump_height: 30.0 };
        let high_jumper = Agent2D { jump_height: 100.0 };
        
        vec![
            PlatStarTestPath::new(
                "Basic Jump", low_jumper.clone(), Vec2::new(0.0, -30.0), Vec2::new(10.0, -10.0),
                vec![
                    Step::ToLink(0.into(), Vec2::new(0.0, -40.0), 4.into()),
                    Step::AccrossLink(4.into()),
                    Step::FromLink(3.into(), Vec2::new(10.0, -10.0), 4.into()),
                ],
            )
        ]
    }
}