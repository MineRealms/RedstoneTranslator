//! Route goals for the point-to-point routing engine.

use std::collections::HashSet;

use crate::transform::place_and_route::detailed_router;
use crate::world::position::Position;
use crate::world::World3D;

#[derive(Clone, Copy)]
pub(crate) enum RouteGoal {
    ConnectPosition { target: Position },
    PowerCobble { cobble: Position },
    PlaceRedstone { target: Position },
}

impl RouteGoal {
    pub(crate) fn for_sink(world: &World3D, sink: Position) -> Self {
        if world[sink].kind.is_cobble() {
            return Self::PowerCobble { cobble: sink };
        }
        if world[sink].kind.is_air() {
            return Self::PlaceRedstone { target: sink };
        }
        Self::ConnectPosition { target: sink }
    }

    pub(crate) fn placement_target(self) -> Position {
        match self {
            Self::ConnectPosition { target } => target,
            Self::PowerCobble { cobble } => cobble,
            Self::PlaceRedstone { target } => target,
        }
    }

    pub(crate) fn rejects_bound_position(self, position: Position) -> bool {
        match self {
            Self::PlaceRedstone { .. } => false,
            _ => position == self.placement_target(),
        }
    }

    pub(crate) fn accepts(self, world: &World3D, redstone: Position) -> bool {
        match self {
            Self::ConnectPosition { target } => {
                detailed_router::target_powers_position(world, target, redstone)
            }
            Self::PowerCobble { cobble } => {
                detailed_router::redstone_powers_cobble(world, redstone, cobble)
            }
            Self::PlaceRedstone { target } => {
                redstone == target && world[redstone].kind.is_redstone()
            }
        }
    }

    pub(crate) fn allowed_short_positions(self) -> Option<HashSet<Position>> {
        match self {
            Self::PowerCobble { cobble } => Some(
                cobble
                    .cardinal()
                    .into_iter()
                    .chain([cobble, cobble.up()])
                    .collect(),
            ),
            Self::ConnectPosition { .. } | Self::PlaceRedstone { .. } => None,
        }
    }
}
