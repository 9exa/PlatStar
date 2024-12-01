use crate::platstar2d::common::*;
use glam::Vec2;
use platstar::strategy::AStar;
use platstar::platstar2d::*;

#[test]
fn basic_test() {
    let platstar = BasicTest::graph();
    let astar = AStar;
    let test_paths = BasicTest::paths();

    for (i, test_path) in test_paths.into_iter().enumerate() {
        let test_name = format!("BasicTest path {}", i);
        let (start_pos, end_pos, agent) = (test_path.start, test_path.end, test_path.agent);
        
        let path = platstar.find_path(&astar, &agent, &start_pos, &end_pos);
        assert_eq!(path, test_path.steps, "Test failed: {}", test_name);
    }

}