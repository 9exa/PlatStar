use crate::platstar::Step;
use petgraph::graph::DiGraph;


#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct AgentId(u32);
impl AgentId {
    pub fn from_raw(id: u32) -> Self {
        Self(id)
    }

    pub fn raw(&self) -> u32 {
        self.0
    }
}

pub struct CostProfile {
    /// Actual cost of moving
    pub cost: f32,
    /// Estimated remaining cost to destination
    pub heuristic: f32,
}


/// An Agent that can transverse a Platgraph of platforms (nodes) with weight P 
/// and links (edges) with weights L
pub trait Agent {
    type P;
    type L;
    type Point;

    /// Whether the agent can cross from one platform to another across the given edge
    fn can_cross(&self, platgraph: &DiGraph<Self::P, Self::L>, step: &Step<Self::Point>) -> bool {
        self.cost_profile(platgraph, step, None).is_some()
    }

    /// Estimated (heuristic) cost of moving from the given platform to the given destination
    ///
    /// Plat: The platform the agent would be on at the end of the step
    /// pos: The position the agent would be at on the platform
    /// dest: The destination the agent is trying to reach
    // fn heuristic_cost(&self, step: &Step<Self::Point>) -> f32;

    /// Actual cost of moving from the given platform to the given destination
    /// If your cost reuses a lot of the same calculations as the heuristic cost, 
    /// should use #[inline] to make sure the compiler inlines the function
    /// 
    /// 
    /// Plat: The platform the agent would be on at the end of the step
    /// pos: The position the agent would be at on the platform
    /// dest: The destination the agent is trying to reach
    // fn calc_cost(&self, plat: &Self::P, pos: &Self::Point, dest: &Self::Point) -> f32;

    /// Whether or not the agent can perform the step, and it's acciociated costs, all in one fuction!
    fn cost_profile(&self, platgraph: &DiGraph<Self::P, Self::L>, step: &Step<Self::Point>, dest: Option<&Self::Point>) -> Option<CostProfile>;

    /// The direction the agent falls. Opposite of the direction the agent jumps
    fn down(&self) -> Self::Point; // Todo: Should probably be a Vector

}