//! Negotiated-congestion resources for the routing engine (M4).
//!
//! PathFinder-style routing needs two cost layers per cell: the present
//! overuse (how many routes want this cell right now) and the history of
//! persistent overuse across iterations. The map is deterministic: every
//! iteration walks cells in sorted order.

use std::collections::HashMap;

use crate::world::position::Position;

/// Cost weights for negotiated congestion. Both default to zero so the
/// routing engine stays behavior-equivalent until a negotiated flow enables
/// them explicitly.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CongestionConfig {
    pub present_penalty: usize,
    pub history_penalty: usize,
}

/// Per-cell usage and accumulated overuse.
#[derive(Clone, Debug, Default)]
pub struct CongestionMap {
    usage: HashMap<Position, usize>,
    history: HashMap<Position, usize>,
}

impl CongestionMap {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn is_empty(&self) -> bool {
        self.usage.is_empty() && self.history.is_empty()
    }

    pub fn usage(&self, position: Position) -> usize {
        self.usage.get(&position).copied().unwrap_or(0)
    }

    pub fn history(&self, position: Position) -> usize {
        self.history.get(&position).copied().unwrap_or(0)
    }

    pub fn present_overuse(&self, position: Position) -> usize {
        self.usage(position).saturating_sub(1)
    }

    pub fn add_route(&mut self, path: &[Position]) {
        for &position in path {
            *self.usage.entry(position).or_default() += 1;
        }
    }

    pub fn rip_route(&mut self, path: &[Position]) {
        for &position in path {
            let Some(entry) = self.usage.get_mut(&position) else {
                continue;
            };
            *entry = entry.saturating_sub(1);
            if *entry == 0 {
                self.usage.remove(&position);
            }
        }
    }

    /// Extra cost of occupying `position` under the current negotiated state.
    pub fn cell_cost(&self, position: Position, config: &CongestionConfig) -> usize {
        self.present_overuse(position) * config.present_penalty
            + self.history(position) * config.history_penalty
    }

    /// Folds the current present overuse into history for the next iteration.
    pub fn commit_iteration(&mut self) {
        for (&position, &usage) in &self.usage {
            let overuse = usage.saturating_sub(1);
            if overuse > 0 {
                *self.history.entry(position).or_default() += overuse;
            }
        }
    }

    /// Penalizes every cell of a path in history. Used when a routed candidate
    /// fails simulation, so the next pass steers away from those cells.
    pub fn add_history(&mut self, path: &[Position]) {
        for &position in path {
            *self.history.entry(position).or_default() += 1;
        }
    }

    /// Overused cells in deterministic order, for rip-up decisions.
    pub fn overused_cells(&self) -> Vec<Position> {
        let mut cells = self
            .usage
            .iter()
            .filter(|(_, usage)| **usage > 1)
            .map(|(position, _)| *position)
            .collect::<Vec<_>>();
        cells.sort();
        cells
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config(present_penalty: usize, history_penalty: usize) -> CongestionConfig {
        CongestionConfig {
            present_penalty,
            history_penalty,
        }
    }

    #[test]
    fn shared_cells_report_present_overuse() {
        let mut map = CongestionMap::new();
        let shared = Position(1, 0, 0);
        map.add_route(&[Position(0, 0, 0), shared]);
        map.add_route(&[shared, Position(2, 0, 0)]);

        assert_eq!(map.usage(shared), 2);
        assert_eq!(map.present_overuse(shared), 1);
        assert_eq!(map.overused_cells(), vec![shared]);
        assert_eq!(map.cell_cost(shared, &config(10, 0)), 10);
        assert_eq!(map.cell_cost(Position(0, 0, 0), &config(10, 0)), 0);
    }

    #[test]
    fn ripping_a_route_releases_its_usage() {
        let mut map = CongestionMap::new();
        let path = [Position(0, 0, 0), Position(1, 0, 0)];
        map.add_route(&path);
        assert_eq!(map.usage(Position(1, 0, 0)), 1);

        map.rip_route(&path);
        assert!(map.is_empty());
        assert_eq!(map.usage(Position(1, 0, 0)), 0);
    }

    #[test]
    fn commit_accumulates_history_that_outlives_rip_up() {
        let mut map = CongestionMap::new();
        let shared = Position(1, 0, 0);
        map.add_route(&[Position(0, 0, 0), shared]);
        map.add_route(&[shared, Position(2, 0, 0)]);
        map.commit_iteration();
        assert_eq!(map.history(shared), 1);

        map.rip_route(&[Position(0, 0, 0), shared]);
        map.rip_route(&[shared, Position(2, 0, 0)]);
        assert_eq!(map.present_overuse(shared), 0);
        assert_eq!(map.cell_cost(shared, &config(10, 5)), 5);
    }

    #[test]
    fn default_config_makes_congestion_free() {
        let mut map = CongestionMap::new();
        let position = Position(1, 0, 0);
        map.add_route(&[position, position]);

        assert_eq!(map.cell_cost(position, &CongestionConfig::default()), 0);
    }

    #[test]
    fn explicit_history_penalty_outlives_rip_up() {
        let mut map = CongestionMap::new();
        let path = [Position(0, 0, 0), Position(1, 0, 0)];
        map.add_history(&path);

        assert_eq!(map.usage(Position(1, 0, 0)), 0);
        assert_eq!(map.history(Position(1, 0, 0)), 1);
        assert_eq!(map.cell_cost(Position(1, 0, 0), &config(10, 5)), 5);
    }
}
