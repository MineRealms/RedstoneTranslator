use std::collections::HashSet;

use eyre::ContextCompat;
use serde::{Deserialize, Serialize};

use crate::transform::place_and_route::estimate::{bounding_box, BoundingBox};
use crate::world::position::Position;
use crate::world::World3D;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PhysicalPortDirection {
    Input,
    Output,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PortConnection {
    Direct,
    InputDiode,
    OutputDiode,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PhysicalPort {
    pub name: String,
    pub direction: PhysicalPortDirection,
    pub position: Position,
    pub route_position: Option<Position>,
    pub access_points: Vec<Position>,
    pub connection: PortConnection,
}

impl PhysicalPort {
    pub fn routing_access_positions(&self) -> Vec<Position> {
        if self.access_points.is_empty() {
            return vec![self.route_position.unwrap_or(self.position)];
        }
        self.access_points.clone()
    }

    pub fn primary_route_position(&self) -> Position {
        self.routing_access_positions()
            .into_iter()
            .next()
            .unwrap_or(self.position)
    }

    pub fn requires_input_diode(&self) -> bool {
        self.connection == PortConnection::InputDiode
    }

    pub fn requires_output_diode(&self) -> bool {
        self.connection == PortConnection::OutputDiode
    }
}

/// Physical metrics of one local candidate. The first two fields are the
/// historical scalar cost; the rest feed the Pareto frontier so global P&R can
/// choose a slightly larger candidate with better routing access instead of
/// only the smallest one.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct LayoutCandidateCost {
    pub block_count: usize,
    pub bbox_volume: usize,
    pub bbox_footprint: usize,
    pub height: usize,
    pub port_count: usize,
    pub access_point_count: usize,
    pub blocked_cell_count: usize,
}

impl LayoutCandidateCost {
    /// Objectives where smaller is better. Port and access-point counts are
    /// not dominance objectives because more routing access is an advantage.
    pub fn minimization_metrics(&self) -> [usize; 4] {
        [
            self.block_count,
            self.bbox_volume,
            self.bbox_footprint,
            self.height,
        ]
    }
}

/// True when `left` is no worse than `right` in every minimization objective
/// and strictly better in at least one.
pub fn dominates(left: &LayoutCandidateCost, right: &LayoutCandidateCost) -> bool {
    let left = left.minimization_metrics();
    let right = right.minimization_metrics();
    let mut strictly_better = false;
    for (a, b) in left.iter().zip(right.iter()) {
        if a > b {
            return false;
        }
        if a < b {
            strictly_better = true;
        }
    }
    strictly_better
}

/// Retains a bounded Pareto frontier. No returned candidate is dominated by
/// another, and candidates with identical minimization metrics collapse. If
/// the frontier exceeds `limit`, the most compact candidates are kept.
pub fn pareto_frontier(candidates: Vec<LayoutCandidate>, limit: usize) -> Vec<LayoutCandidate> {
    let mut frontier = Vec::<LayoutCandidate>::new();
    for candidate in candidates {
        let metrics = candidate.cost.minimization_metrics();
        if frontier.iter().any(|existing| {
            dominates(&existing.cost, &candidate.cost)
                || existing.cost.minimization_metrics() == metrics
        }) {
            continue;
        }
        frontier.retain(|existing| !dominates(&candidate.cost, &existing.cost));
        frontier.push(candidate);
    }
    if frontier.len() > limit {
        frontier.sort_by_key(|candidate| {
            (
                candidate.cost.bbox_volume,
                candidate.cost.block_count,
                candidate.cost.height,
            )
        });
        frontier.truncate(limit);
    }
    frontier
}

#[derive(Clone, Debug)]
pub struct LayoutCandidate {
    pub module_name: String,
    pub world: World3D,
    pub bbox: BoundingBox,
    pub ports: Vec<PhysicalPort>,
    pub occupied_cells: HashSet<Position>,
    pub blocked_cells: HashSet<Position>,
    pub cost: LayoutCandidateCost,
}

impl LayoutCandidate {
    pub fn from_world(
        module_name: String,
        world: World3D,
        ports: Vec<PhysicalPort>,
    ) -> eyre::Result<Self> {
        let bbox = bounding_box(&world).context("layout candidate world has no blocks")?;
        let occupied_cells = world
            .iter_block()
            .into_iter()
            .map(|(position, _)| position)
            .collect::<HashSet<_>>();
        let cost = LayoutCandidateCost {
            block_count: occupied_cells.len(),
            bbox_volume: bbox.volume(),
            bbox_footprint: bbox.width() * bbox.depth(),
            height: bbox.height(),
            port_count: ports.len(),
            access_point_count: ports.iter().map(|port| port.access_points.len()).sum(),
            blocked_cell_count: 0,
        };

        Ok(Self {
            module_name,
            world,
            bbox,
            ports,
            occupied_cells,
            blocked_cells: HashSet::new(),
            cost,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_port(connection: PortConnection, access_points: Vec<Position>) -> PhysicalPort {
        PhysicalPort {
            name: "p".to_owned(),
            direction: PhysicalPortDirection::Input,
            position: Position(1, 2, 3),
            route_position: Some(Position(4, 5, 6)),
            access_points,
            connection,
        }
    }

    #[test]
    fn physical_port_prefers_explicit_access_points() {
        let port = test_port(PortConnection::Direct, vec![Position(7, 8, 9)]);

        assert_eq!(port.routing_access_positions(), vec![Position(7, 8, 9)]);
    }

    #[test]
    fn physical_port_falls_back_to_route_position_as_access_point() {
        let port = test_port(PortConnection::Direct, Vec::new());

        assert_eq!(port.routing_access_positions(), vec![Position(4, 5, 6)]);
    }

    #[test]
    fn physical_port_connection_describes_diode_requirement() {
        let input = test_port(PortConnection::InputDiode, Vec::new());
        let output = test_port(PortConnection::OutputDiode, Vec::new());

        assert!(input.requires_input_diode());
        assert!(!input.requires_output_diode());
        assert!(!output.requires_input_diode());
        assert!(output.requires_output_diode());
    }

    fn candidate(name: &str, metrics: (usize, usize, usize, usize)) -> LayoutCandidate {
        let (block_count, bbox_volume, bbox_footprint, height) = metrics;
        LayoutCandidate {
            module_name: name.to_owned(),
            world: World3D::new(crate::world::position::DimSize(1, 1, 1)),
            bbox: BoundingBox {
                min: Position(0, 0, 0),
                max: Position(0, 0, 0),
            },
            ports: Vec::new(),
            occupied_cells: HashSet::new(),
            blocked_cells: HashSet::new(),
            cost: LayoutCandidateCost {
                block_count,
                bbox_volume,
                bbox_footprint,
                height,
                ..Default::default()
            },
        }
    }

    #[test]
    fn dominance_requires_all_objectives_to_be_no_worse() {
        let small = LayoutCandidateCost {
            block_count: 10,
            bbox_volume: 20,
            bbox_footprint: 8,
            height: 2,
            ..Default::default()
        };
        let larger = LayoutCandidateCost {
            block_count: 12,
            bbox_volume: 24,
            bbox_footprint: 8,
            height: 2,
            ..Default::default()
        };
        let tradeoff = LayoutCandidateCost {
            block_count: 9,
            bbox_volume: 30,
            bbox_footprint: 10,
            height: 3,
            ..Default::default()
        };

        assert!(dominates(&small, &larger));
        assert!(!dominates(&larger, &small));
        assert!(!dominates(&small, &tradeoff));
        assert!(!dominates(&tradeoff, &small));
    }

    #[test]
    fn pareto_frontier_drops_dominated_and_keeps_tradeoffs() {
        let candidates = vec![
            candidate("small", (10, 20, 8, 2)),
            candidate("bigger", (12, 24, 8, 2)),
            candidate("wide", (9, 30, 10, 3)),
            candidate("duplicate", (10, 20, 8, 2)),
        ];
        let frontier = pareto_frontier(candidates, 8);
        let names = frontier
            .iter()
            .map(|candidate| candidate.module_name.as_str())
            .collect::<Vec<_>>();

        assert!(names.contains(&"small"));
        assert!(names.contains(&"wide"));
        assert!(!names.contains(&"bigger"));
        assert!(!names.contains(&"duplicate"));
        assert_eq!(frontier.len(), 2);
    }

    #[test]
    fn pareto_frontier_respects_the_limit_by_compactness() {
        let candidates = (0..5)
            .map(|index| candidate(&format!("c{index}"), (10 - index, 20 + index, 8, 2)))
            .collect::<Vec<_>>();

        let frontier = pareto_frontier(candidates, 2);

        assert_eq!(frontier.len(), 2);
        assert_eq!(frontier[0].module_name, "c0");
        assert_eq!(frontier[1].module_name, "c1");
    }
}
