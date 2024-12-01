use crate::platstar::{*, self};
/// A type that represents a strategy for finding the shortest path an agent can take in the plat graph
pub trait Strategy<P, L, Point> 
where 
    P: Platform<Point = Point>,
    L: Link<P = P, Point = Point>,
    Point: platstar::Point
{
    

    /// Find the shortest path an agent can take between two points
    fn find_path<A>(
        &self, 
        platstar: &PlatStar<P, L, Point>, 
        agent: &A, 
        start_pos: &Point, 
        end_pos: &Point
    ) -> Vec<Step<Point>>
    where
        A: Agent<P = P, L = L, Point = Point>;
    
    /// Update the internal state of the strategy to work for the current state of the platgraph
    fn update<A>(&mut self, platstar: &PlatStar<P, L, Point>, agent: &A)
    where
        A: Agent<P = P, L = L, Point = Point>;
}