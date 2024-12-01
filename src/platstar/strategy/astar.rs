use crate::platstar::strategy::Strategy;
use crate::platstar::{*, self};
use platstar::agent::CostProfile;

use petgraph::prelude::*;

use std::cmp::Ordering;
use std::collections::BinaryHeap;
use bitvec::vec::BitVec;

const INVALID_INDEX: usize = usize::MAX;

pub struct AStar;

struct AStarStep {
    step: usize,
    // The total cost of the step and all preceeding steps
    total_cost: f32,
    // The estimated cost to the end
    est_cost: f32,
}

impl AStarStep {
    pub fn new(step: usize, total_cost: f32, est_cost: f32) -> Self {
        Self {
            step,
            total_cost,
            est_cost,
        }
    }
}

impl PartialEq for AStarStep {
    fn eq(&self, other: &Self) -> bool {
        self.est_cost == other.est_cost
    }
}

impl Eq for AStarStep {}

impl PartialOrd for AStarStep {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.est_cost.partial_cmp(&other.est_cost).map(|o| o.reverse())
    }
}

impl Ord for AStarStep {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap()
    }
}

impl<P, L, Point> Strategy<P, L, Point> for AStar
where
    P: Platform<Point = Point>,
    L: Link<P = P, Point = Point>,
    Point: platstar::Point
{

    fn update<A>(&mut self, _platstar: &PlatStar<P, L, Point>, _agent: &A)
    where
        A: Agent<P = P, L = L, Point = Point> {}

    fn find_path<A>(
        &self, 
        platstar: &PlatStar<P, L, Point>,
        agent: &A, 
        start_pos: &Point, 
        end_pos: &Point
    ) -> Vec<Step<Point>>
    where
        A: Agent<P = P, L = L, Point = Point> 
    {
        if platstar.num_platforms() == 0 {
            return vec![];
        }
        
        // Find start and end points
        // it's okay unwrap here because we know there is at least one platform
        let (start, start_plat, _) = platstar.graph()
            .raw_nodes()
            .iter()
            .enumerate()
            .map(|(i, p)| (PlatIndex::new(i), &p.weight, p.weight.closest_point(start_pos).distance_squared(start_pos)))
            .reduce(|acc, next| if acc.2 > next.2 { next } else { acc })
            .unwrap();

        let (end, end_plat, _) = platstar.graph()
            .raw_nodes()
            .iter()
            .enumerate()
            .map(|(i, p)| (PlatIndex::new(i), &p.weight, p.weight.closest_point(end_pos).distance_squared(end_pos)))
            .reduce(|acc, next| if acc.2 > next.2 { next } else { acc })
            .unwrap();

        let start_pos = start_plat.closest_below(&agent.down(), start_pos);
        let end_pos = end_plat.closest_below(&agent.down(), end_pos);
        
        if start == end {
            return vec![
                Step::AccrossPlatform(start, start_pos.clone(), end_pos.clone())
            ];
        }

        // A* search
        let mut frontier: BinaryHeap<AStarStep> = BinaryHeap::new();
        let mut visited: BitVec<u32> = BitVec::new();
        visited.resize(platstar.num_links(), false);
        let mut steps: Vec<(Step<Point>, usize, usize)> = Vec::with_capacity(20);

        // Start by adding the endpoints of the starting platform
        // self.astar_add_steps(&mut frontier, &mut visited, &mut steps, agent, start, start_pos, end_pos, end, 0.0, INVALID_INDEX);
        {
            let prev_step_id = INVALID_INDEX;
            let platgraph = platstar.graph();
            for e in platgraph.edges_directed(start, Direction::Outgoing) {
                visited.set(e.id().index(), true);

                // Movement on the platform
                let plat_step = Step::ToLink(start, start_pos.clone(), e.id());
                let Some(CostProfile { cost: plat_cost, ..} ) = agent.cost_profile(platgraph, &plat_step, None) else {
                    continue;
                };

                // Movement across the link
                let link_step = Step::AccrossLink(e.id());
                let Some(CostProfile { cost: link_cost, heuristic }) = agent.cost_profile(platgraph, &link_step, Some(&end_pos)) else {
                    continue;
                };

                let platstep_id = steps.len();
                steps.push((plat_step, prev_step_id, 1));
                steps.push((link_step, platstep_id, 2));

                let cost = plat_cost + link_cost;
                let est_cost = cost + heuristic;

                frontier.push(AStarStep::new(steps.len() - 1, cost, est_cost));

            }
        }
        
        while let Some(AStarStep { step: step_id, total_cost: cost, .. }) = frontier.pop() {
            if visited[step_id] {
                continue;
            }

            let (step, _, n_steps) = &steps[step_id];
            
            if let Step::FromLink(..) = step{
                // Found the end
                let mut path = Vec::with_capacity(*n_steps);
                let mut prev_step_id = step_id;
                while prev_step_id != INVALID_INDEX {
                    let (step, next_id, _) = &steps[prev_step_id];
                    path.push(step.clone());
                    
                    prev_step_id = *next_id;
                }
                path.reverse();
                return path;
            }

            let from_link = match step {
                Step::AccrossLink(link) => *link,
                _ => unreachable!("The main step of the astar should only contain FromLink and BetweenLink Steps"),
            };

            // let from = step.to;
            // let pos = step.pos_to.clone();
            // let cost = step.cost;

            self.astar_add_steps(
                platstar.graph(),
                &mut frontier, 
                &mut visited, 
                &mut steps, 
                agent, 
                from_link,
                &end_pos, 
                end, 
                cost, 
                step_id
            )
        }

        // No path found
        vec![]
    }
}

impl AStar {
    fn astar_add_steps<P, L, Point, A>(
        &self, 
        platgraph: &DiGraph<P, L>,
        frontier: &mut BinaryHeap<AStarStep>, 
        visited: &mut BitVec<u32>, 
        steps: &mut Vec<(Step<Point>, usize, usize)>,
        agent: &A, 
        from_link: LinkIndex,
        end_pos: &Point,
        end: PlatIndex,
        prev_cost: f32,
        prev_step_id: usize,
    ) 
    where
        P: Platform<Point = Point>,
        L: Link<P = P, Point = Point>,
        Point: platstar::Point,
        A: Agent<P = P, L = L, Point = Point> 
    {
        let from = platgraph.edge_endpoints(from_link).unwrap().1;
        let (_, _, n_steps) = steps[prev_step_id];

        // Movement on final platform
        if from == end {
            let step = Step::FromLink(end, end_pos.clone(), from_link);
            if let Some(CostProfile { cost, .. }) = agent.cost_profile(platgraph, &step, Some(end_pos)) {
                let step_id = steps.len();
                steps.push((step, prev_step_id, n_steps + 1));
                frontier.push(AStarStep::new(step_id, cost + prev_cost, 0.0));
            }
            
            return;
        }

        visited.set(from.index(), true);
        for e in platgraph.edges_directed(from, Outgoing) {
            if visited[e.id().index()] {
                continue;
            }

            let to_link = e.id();

            // Movement on the platform
            let plat_step = Step::BetweenLinks(from_link, to_link);
            let Some(CostProfile { cost: plat_cost, .. }) = agent.cost_profile(platgraph, &plat_step, Some(end_pos)) else {
                continue;
            };
            
            // Movement across the link
            let step = Step::AccrossLink(to_link);
            let Some(CostProfile { cost: link_cost, heuristic}) = agent.cost_profile(platgraph, &step, Some(end_pos)) else {
                continue;
            };

            let plat_step_id = steps.len();
            steps.push((plat_step, prev_step_id, n_steps + 1));
            steps.push((step, plat_step_id, n_steps + 2));

            let cost = prev_cost + plat_cost + link_cost;
            let est_cost = cost + heuristic;

            frontier.push(AStarStep::new(steps.len(), cost, est_cost));
        }
    }
}
