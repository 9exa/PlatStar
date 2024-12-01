use crate::platstar::{self, Link, Platform, Agent};
use crate::platstar::strategy::Strategy;

use petgraph::graph::{DiGraph, EdgeIndex, NodeIndex};

pub (crate) type PlatIndex = NodeIndex<u32>;
pub (crate) type LinkIndex = EdgeIndex<u32>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Step<Point> {
    /// Starting from a point on a platform to the 'from' of a link
    ToLink(PlatIndex, Point, LinkIndex),
    /// Starting from the 'to' of a link to a point on a platform
    FromLink(PlatIndex, Point, LinkIndex),
    /// Move from the 'to' of a link to the 'from' of another link
    BetweenLinks(LinkIndex, LinkIndex),
    /// Moving along a link
    AccrossLink(LinkIndex),
    /// Moving accross 2 points on a platform
    AccrossPlatform(PlatIndex, Point, Point),
}



/// A Graph of Platforms and Links that can be traversed by an Agent
/// The underling graph is ALWAYS Directed
/// 
/// P: Data held by each platform
/// L: Data held by each link
/// Point: The type of a single point in the space being represented by the PlatStar
pub struct PlatStar<P, L, Point>
where
    P: Platform<Point = Point>,
    L: Link<P = P, Point = Point>,
    Point: platstar::Point,
{
    // Contains a description of the platforms and links in the graph
    pub (crate) platgraph: DiGraph<P, L>,
}

impl<P, L, Point> PlatStar<P, L, Point>
where
    P: Platform<Point = Point>,
    L: Link<P = P, Point = Point>,
    Point: platstar::Point,
{
    /// Create a new PlatStar
    pub fn new() -> Self {
        Self {
            platgraph: DiGraph::new(),
        }
    }

    pub fn add_platform(&mut self, platform: P) -> PlatIndex {
        self.platgraph.add_node(platform)
    }

    pub fn remove_platform(&mut self, platform: PlatIndex) -> Option<P> {
        self.platgraph.remove_node(platform)
    }

    pub fn num_platforms(&self) -> usize {
        self.platgraph.raw_nodes().len()
    }

    /// Add a link between two platforms
    /// 
    /// If you want to just replace the link between two platforms, use `update_link`
    pub fn add_link(&mut self, from: PlatIndex, to: PlatIndex, link: L) -> LinkIndex {
        self.platgraph.add_edge(from, to, link)
    }

    pub fn remove_link(&mut self, link: LinkIndex) -> Option<L> {
        self.platgraph.remove_edge(link)
    }

    pub fn num_links(&self) -> usize {
        self.platgraph.raw_edges().len()
    }

    /// Get the first link that goes from one platform to another.
    pub fn find_link(&self, from: PlatIndex, to: PlatIndex) -> Option<LinkIndex> {
        self.platgraph.find_edge(from, to)
    }

    pub fn update_link(&mut self, from: PlatIndex, to: PlatIndex, new_link: L) -> LinkIndex {
        self.platgraph.update_edge(from, to, new_link)
    }

    pub fn graph(&self) -> &DiGraph<P, L> {
        &self.platgraph
    }

    /// Find a path an agent can take between two points using the given strategy.
    /// 
    /// Just a wrapper for `Strategy::find_path`
    pub fn find_path<S, A>(&self, strat: &S, agent: &A, start_pos: &Point, end_pos: &Point) -> Vec<Step<Point>> 
    where 
        S: Strategy<P, L, Point>,
        A: Agent<P=P, L=L, Point=Point>,
    {
        strat.find_path(self, agent, start_pos, end_pos)
    }

}

