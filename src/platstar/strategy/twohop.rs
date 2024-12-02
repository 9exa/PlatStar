use crate::platstar::strategy::Strategy;
use crate::platstar::{*, self};
use bitvec::bitbox;
use platstar::agent::CostProfile;

use petgraph::prelude::*;

use std::cmp::{Ordering, Reverse};
use std::ops::Range; 
use std::collections::{BinaryHeap, HashMap};
use bitvec::prelude::*;


type TwoHopNodeIdx = NodeIndex<u32>;
type TwoHopEdgeIdx = EdgeIndex<u32>;

const INVALID_INDEX: usize = usize::MAX;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct TwoHopEndpoint {
    link: LinkIndex,
    endpoint: Endpoint,
}

impl TwoHopEndpoint {
    fn new(link: LinkIndex, endpoint: Endpoint) -> Self {
        Self {
            link,
            endpoint,
        }
    }

    #[inline]
    fn index(&self) -> usize {
        self.link.index() * 2 + self.endpoint as usize
    }

    #[inline]
    fn from_index(index: usize) -> Self {
        Self {
            link: LinkIndex::new(index / 2),
            endpoint: match index % 2 {
                0 => Endpoint::From,
                1 => Endpoint::To,
                _ => unreachable!(),
            }
        }
    }

    fn invalid() -> Self {
        Self {
            link: LinkIndex::new(INVALID_INDEX),
            endpoint: Endpoint::From,
        }
    }

    fn is_invalid(&self) -> bool {
        self.link.index() == INVALID_INDEX
    }
}

#[derive(Debug, Clone, Copy)]
struct Hop {
    total_cost: f32,

    // Index to the second last endpoint in the hop
    prev: TwoHopEndpoint,
    // index to the last step of the hop
    step_id: usize
}

#[derive(Default)]
pub struct TwoHop {
    steps: Vec<TwoHopStep>,
    lins: Vec<(TwoHopEndpoint,Hop)>,
    louts: Vec<(TwoHopEndpoint,Hop)>,
    lin_ranges: Vec<Range<usize>>,
    lout_ranges: Vec<Range<usize>>,
}


struct GraphHop {
    // The index containing the last step of the path
    step_id: usize,
    total_cost: f32,
    // The hop excluding the last step of the path
    prev: Option<TwoHopEdgeIdx>,
}

struct TwoHopCandidate {
    step_id: EdgeIndex,
    total_cost: f32,
    prev: Option<TwoHopEdgeIdx>,
}

#[derive(Debug, PartialEq, Eq)]
enum TwoHopStep {
    AccrossLink(LinkIndex),
    BetweenLinks(LinkIndex, LinkIndex),
    NoMove,
}

impl PartialEq for TwoHopCandidate {
    fn eq(&self, other: &Self) -> bool {
        self.total_cost == other.total_cost
    }
}

impl Eq for TwoHopCandidate {}

impl PartialOrd for TwoHopCandidate {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.total_cost.partial_cmp(&other.total_cost).map(|o| o.reverse())
    }
}

impl Ord for TwoHopCandidate {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap()
    }
}

impl<P, L, Point> Strategy<P, L, Point> for TwoHop
where
    P: Platform<Point = Point>,
    L: Link<P = P, Point = Point>,
    Point: platstar::Point
{
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
        let platgraph = platstar.graph();
        
        if platstar.num_platforms() == 0 {
            return vec![];
        }
        
        // Find start and end points
        // it's okay unwrap here because we know there is at least one platform
        let Some((start, start_plat, _)) = platgraph
            .raw_nodes()
            .iter()
            .enumerate()
            .map(|(i, p)| (PlatIndex::new(i), &p.weight, p.weight.closest_point(start_pos).distance_squared(start_pos)))
            .reduce(|acc, next| if acc.2 > next.2 { next } else { acc }) 
        else {
            return vec![];
        };

        let Some((end, end_plat, _)) = platgraph
            .raw_nodes()
            .iter()
            .enumerate()
            .map(|(i, p)| (PlatIndex::new(i), &p.weight, p.weight.closest_point(end_pos).distance_squared(end_pos)))
            .reduce(|acc, next| if acc.2 > next.2 { next } else { acc })
        else {
            return vec![];
        };

        let start_pos = start_plat.closest_below(&agent.down(), &start_pos);
        let end_pos = end_plat.closest_below(&agent.down(), &end_pos);
        
        if start == end {
            return vec![
                Step::AccrossPlatform(start, start_pos.clone(), end_pos.clone())
            ];
        }

        // The main crux of this algorithm is to treat the start and end points as new 'nodes' in the graph,
        // to find the shortest path between them, as you would a regular two-hop cover
        let mut lins: HashMap<TwoHopEndpoint, Hop> = HashMap::new();
        let mut louts: HashMap<TwoHopEndpoint, Hop> = HashMap::new();


        Self::calc_intermediate(
            platgraph, 
            &self.louts, 
            &self.lout_ranges, 
            agent, 
            start, 
            &start_pos,
            &mut louts, 
            Direction::Outgoing,
        );
        Self::calc_intermediate(
            platgraph, 
            &self.lins, 
            &self.lin_ranges, 
            agent, 
            end, 
            &end_pos,
            &mut lins, 
            Direction::Incoming,
        );

        // Find that intersection
        let Some(intermediate) = louts.iter()
            .filter_map(|(link, hop)| {
                lins.get(link)
                    .map(|other_hop| {
                        // eliminate double counting of the last step
                        (link, hop.total_cost + other_hop.total_cost)
                    })
            })
            .min_by(|(_, cost1), (_, cost2)| cost1.partial_cmp(cost2).unwrap())
            .map(|(link, _)| *link)
        else {
            return vec![];
        };

        
        // Reconstruct the path
        let mut path = Vec::new();
        // First half
        Self::connect_to_intermediate(
            intermediate, 
            &self.steps, 
            &louts, 
            &mut path,
        );
        path.reverse();

        // Second half
        Self::connect_to_intermediate(
            intermediate, 
            &self.steps, 
            &lins, 
            &mut path,
        );
        
        path

    }

    fn update<A>(&mut self, platstar: &PlatStar<P, L, Point>, agent: &A)
    where
        A: Agent<P = P, L = L, Point = Point> 
    {
        self.steps.clear();
        self.lins.clear();
        self.louts.clear();
        // self.lin_ranges.clear(); // resized and rewritten later
        // self.lout_ranges.clear();

        let platgraph = platstar.graph();
        
        // Until optimisation, store the lins and louts as a DiGraph
        // let mut linouts: DiGraph<(), GraphHop> = DiGraph::new();
        let mut linouts: DiGraph<(), GraphHop> = DiGraph::with_capacity(platgraph.edge_count() * 2, platgraph.edge_count());
        platgraph.edge_indices().for_each(|_| { 
            linouts.add_node(()); 
            linouts.add_node(()); 
        });


        // Precalculate all the costs of the steps
        let mut cost_graph: DiGraph<(), (TwoHopStep, f32)> = DiGraph::with_capacity(platgraph.edge_count() * 2, platgraph.edge_count());
        platgraph.edge_indices().for_each(|_| { 
            cost_graph.add_node(()); 
            cost_graph.add_node(()); 
        });
        // between links
        for p in platgraph.node_indices() {
            for e_in in platgraph.edges_directed(p, Direction::Incoming) {
                for e_out in platgraph.edges_directed(p, Direction::Outgoing) {
                    let step = Step::BetweenLinks(e_in.id(), e_out.id());
                    if let Some(cost) = agent
                        .cost_profile(platgraph, &step, None)
                        .map(|c| c.cost)
                    {
                        cost_graph.add_edge(
                            NodeIndex::new(TwoHopEndpoint::new(e_in.id(), Endpoint::To).index()),
                            NodeIndex::new(TwoHopEndpoint::new(e_out.id(), Endpoint::From).index()),
                            (TwoHopStep::BetweenLinks(e_in.id(), e_out.id()), cost),
                        );
                    }
                }
            }
        }
        // across links
        // each endpoint to itself
        for e in platgraph.edge_references() {
            let link_id = e.id();
            
            let step = Step::AccrossLink(link_id);
            if let Some(cost) = agent
                .cost_profile(platgraph, &step, None)
                .map(|c| c.cost)
            {
                cost_graph.add_edge(
                    NodeIndex::new(TwoHopEndpoint::new(link_id, Endpoint::From).index()),
                    NodeIndex::new(TwoHopEndpoint::new(link_id, Endpoint::To).index()),
                    (TwoHopStep::AccrossLink(e.id()), cost),
                );
            }

            // TODO: Probably a little redundant to store the NoMove operations. Can probably get away with
            // a special NO_MOVE index
            cost_graph.add_edge(
                NodeIndex::new(TwoHopEndpoint::new(link_id, Endpoint::From).index()),
                NodeIndex::new(TwoHopEndpoint::new(link_id, Endpoint::From).index()),
                (TwoHopStep::NoMove, 0.0),
            );
            cost_graph.add_edge(
                NodeIndex::new(TwoHopEndpoint::new(link_id, Endpoint::To).index()),
                NodeIndex::new(TwoHopEndpoint::new(link_id, Endpoint::To).index()),
                (TwoHopStep::NoMove, 0.0),
            );
        }

        // Determin startpoint search order
        let mut degrees: Vec<(PlatIndex, usize)> = platgraph.node_indices()
            .filter_map(|i| {
                let degrees = platgraph.neighbors_directed(i, Direction::Outgoing).count()
                    + platgraph.neighbors_directed(i, Direction::Incoming).count();
                if degrees == 0 {
                    None
                } else {
                    Some((i, degrees))
                }
            })
            .collect();
        degrees.sort_by_key(|(_, d)| Reverse(*d));


        // Perform searches for the cover
        let mut visited = bitbox![0; cost_graph.node_count()];
        let mut processed = bitbox![0; platgraph.edge_count()];
        let mut linouts = DiGraph::with_capacity(cost_graph.node_count(), cost_graph.edge_count());
        for _ in 0..cost_graph.node_count() {
            linouts.add_node(());
        }

        for (p, _) in degrees {
            for e in platgraph
                .edges_directed(p, Direction::Outgoing)
                .chain(platgraph.edges_directed(p, Direction::Incoming)) 
            {
                if processed[e.id().index()] {
                    continue;
                }

                TwoHop::djikstra(
                    &cost_graph,
                    &mut BinaryHeap::new(),
                    &processed,
                    &mut visited,
                    &mut linouts,
                    e.id(),
                    Direction::Outgoing,
                );

                TwoHop::djikstra(
                    &cost_graph,
                    &mut BinaryHeap::new(),
                    &processed,
                    &mut visited,
                    &mut linouts,
                    e.id(),
                    Direction::Incoming,
                );

                processed.set(e.id().index(), true);
            }
        }
        
        

        // TODO; Optimize lins and louts to be more cache friendly
        // Collect the lins, louts and steps from the linouts
        // Note that the way that Steps are stored in the graph results in double counting cost from the last hop in the step
        self.lins.reserve(linouts.edge_count());
        self.louts.reserve(linouts.edge_count());
        self.lin_ranges.resize(linouts.node_count(), 0..0);
        self.lout_ranges.resize(linouts.node_count(), 0..0);

        for i in 0..linouts.node_count() {
            // Since only outgoing edges are represented in louts, and incoming in lins, we can just iterate over all edges
            // We can just use the LinkIndex as the index for the lins and louts
            TwoHop::collect_hops(
                &linouts,
                &mut self.lins,
                &mut self.lin_ranges[i],
                LinkIndex::new(i / 2),
                Direction::Incoming,
            );
            TwoHop::collect_hops(
                &linouts,
                &mut self.louts,
                &mut self.lout_ranges[i],
                LinkIndex::new(i / 2),
                Direction::Outgoing,
            );
        }

        // Collect steps
        let (_, step_edges) = cost_graph.into_nodes_edges();
        self.steps.reserve_exact(step_edges.len());
        self.steps.extend(step_edges.into_iter()
            .map(|e| e.weight.0)
        );


    }

    
}

impl TwoHop
{
    fn djikstra(
        steps: &DiGraph<(), (TwoHopStep, f32)>,
        frontier: &mut BinaryHeap<TwoHopCandidate>,
        processed: &BitSlice<usize>,
        visited: &mut BitSlice<usize>,
        linouts: &mut DiGraph<(), GraphHop>,
        start: LinkIndex,
        direction: Direction,
        
    ) {
        frontier.clear();
        visited.fill(false);

        // Initialize the frontier
        let start_endpoint = match direction {
            Direction::Incoming => TwoHopEndpoint::new(start, Endpoint::To),
            Direction::Outgoing => TwoHopEndpoint::new(start, Endpoint::From),
        };
        let start_endpoint_id = NodeIndex::new(start_endpoint.index());

        let step_id = steps.find_edge(start_endpoint_id, start_endpoint_id).unwrap();
        let prev_id = linouts.add_edge(
            start_endpoint_id, 
            start_endpoint_id, 
            GraphHop { step_id: step_id.index(), total_cost: 0.0, prev: None });
        frontier.push(TwoHopCandidate {
            step_id,
            total_cost: 0.0,
            prev: Some(prev_id),
        });

        visited.set(start_endpoint_id.index(), true);

        // Search through the frontier
        while let Some(TwoHopCandidate { step_id, total_cost, prev }) = frontier.pop() {
            let (step, _) = steps.edge_weight(step_id).unwrap();
            let next_endpoint = match step {
                TwoHopStep::AccrossLink(link) => {
                    match direction {
                        Direction::Incoming => TwoHopEndpoint::new(*link, Endpoint::From),
                        Direction::Outgoing => TwoHopEndpoint::new(*link, Endpoint::To),
                    }
                },
                TwoHopStep::BetweenLinks(from, to) => {
                    match direction {
                        Direction::Incoming => TwoHopEndpoint::new(*from, Endpoint::To),
                        Direction::Outgoing => TwoHopEndpoint::new(*to, Endpoint::From),
                    }
                },
                _ => unreachable!(),
            };

            if visited[next_endpoint.index()] {
                continue;
            }

            let next_endpoint_id = NodeIndex::new(next_endpoint.index());

            // Add to the graph
            let prev_hop_id = match direction {
                Direction::Incoming => linouts.add_edge(
                    next_endpoint_id, 
                    start_endpoint_id, 
                    GraphHop { step_id: step_id.index(), total_cost, prev }
                ),
                Direction::Outgoing => linouts.add_edge(
                    start_endpoint_id,
                    next_endpoint_id,
                    GraphHop { step_id: step_id.index(), total_cost, prev }
                ),
            };

            if processed[next_endpoint.link.index()] {
                // Don't propagate an intermediate
                continue;
            }

            visited.set(next_endpoint.index(), true);

            // Add the next links to the frontier
            steps.edges_directed(next_endpoint_id, direction)
                .filter(|e| {
                    let other = match direction {
                        Direction::Incoming => e.source(),
                        Direction::Outgoing => e.target(),
                    };
                    !visited[other.index()]
                })
                .for_each(|e| {
                    let cost = e.weight().1;
                    frontier.push(TwoHopCandidate {
                        step_id: e.id(),
                        total_cost: total_cost + cost,
                        prev: Some(prev_hop_id),
                    });
                });
        }
    }

    #[inline]
    fn collect_hops(
        linouts: &DiGraph<(), GraphHop>,
        out: &mut Vec<(TwoHopEndpoint, Hop)>,
        range_out: &mut Range<usize>,
        start: LinkIndex,
        direction: Direction,
    ) {
        let start_endpoint = match direction {
            Direction::Incoming => TwoHopEndpoint::new(start, Endpoint::To),
            Direction::Outgoing => TwoHopEndpoint::new(start, Endpoint::From),
        };
        let range_start = out.len();
        let mut range_end = range_start;

        for e in linouts.edges_directed(TwoHopNodeIdx::new(start_endpoint.index()), direction) {
            let hop = e.weight();
            let other_end_id = match direction {
                Direction::Outgoing => LinkIndex::new(e.target().index()),
                Direction::Incoming => LinkIndex::new(e.source().index()),
            }.index();
            let other_end = TwoHopEndpoint::from_index(other_end_id);
            let prev_other_end = hop.prev.map(|pe| {
                TwoHopEndpoint::from_index(match direction {
                    Direction::Outgoing => linouts.edge_endpoints(pe).unwrap().1,
                    Direction::Incoming => linouts.edge_endpoints(pe).unwrap().0,
                }.index())
            }).unwrap_or(TwoHopEndpoint::invalid());
            
            let hop = Hop {
                total_cost: hop.total_cost,
                prev: prev_other_end,
                step_id: hop.step_id,
            };
            out.push((other_end, hop));
            range_end += 1;
        }

        *range_out = range_start..range_end;
    }

    #[inline]
    fn calc_intermediate<P, L, A, Point>(
        platgraph: &DiGraph<P, L>,
        links: &[(TwoHopEndpoint, Hop)],
        ranges: &[Range<usize>],
        agent: &A,
        start: PlatIndex,
        start_pos: &Point,
        links_out: &mut HashMap<TwoHopEndpoint, Hop>,
        direction: Direction,
    )
    where
        P: Platform<Point = Point>,
        L: Link<P = P, Point = Point>,
        A: Agent<P = P, L = L, Point = Point>,
        Point: platstar::Point
    {
        for e in platgraph.edges_directed(start, direction) {
            let start_link = e.id();
            let Some(CostProfile { cost: cost_to_link, ..}) = agent.cost_profile(
                platgraph, 
                &Step::ToLink(start, start_pos.clone(), start_link), 
                None
            ) else {
                continue;
            };

            let range = ranges[start_link.index()].clone();
            for (dest_endpoint, hop) in &links[range] {
                links_out.entry(*dest_endpoint)
                    .and_modify(|old_hop| {
                        let total_cost = cost_to_link + hop.total_cost;
                        if total_cost < old_hop.total_cost {
                            *old_hop = Hop {
                                total_cost,
                                prev: hop.prev,
                                step_id: hop.step_id,
                            };
                        }
                    })
                    .or_insert_with(|| {
                        Hop {
                            total_cost: cost_to_link + hop.total_cost,
                            prev: hop.prev,
                            step_id: hop.step_id,
                        }
                    });
            }
        }
    }

    fn connect_to_intermediate<Point>(
        mut intermediate: TwoHopEndpoint,
        steps: &[TwoHopStep],
        links: &HashMap<TwoHopEndpoint, Hop>,
        path: &mut Vec<Step<Point>>,
    )
    where
        Point: platstar::Point
    {
        while let Some(hop) = links.get(&intermediate) {
            let step = match &steps[hop.step_id] {
                TwoHopStep::AccrossLink(link) => Step::AccrossLink(*link),
                TwoHopStep::BetweenLinks(from, to) => Step::BetweenLinks(*from, *to),
                TwoHopStep::NoMove => continue,
            };
            path.push(step);
            intermediate = hop.prev;
        }
    }

}