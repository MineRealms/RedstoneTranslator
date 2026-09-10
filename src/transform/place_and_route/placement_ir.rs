//! Coordinate-bearing physical IR for the CAD placement flow (M1).
//!
//! The circuit IR (`LogicalDesign`/`RoutableDesign`) is coordinate-free and
//! must stay that way. This module owns the physical objects the new flow
//! manipulates: verified macro templates, macro instances, pins, and nets.
//! It is deliberately Minecraft-aware only at the block level (`Block`,
//! `World3D`) and does not depend on candidate generation or the local placer.
//!
//! M1 scope: the model, conversion from an existing verified `LayoutCandidate`,
//! instantiation back into a `World3D`, and deterministic net metrics. M3.0
//! adds pin facings and first-class placement legality (overlap, bounds) and
//! bounding-box cost queries. The placement and routing engines land later.

use std::collections::{BTreeMap, BTreeSet};

use eyre::{bail, ContextCompat};

use crate::transform::place_and_route::global_pnr::ir::{
    LayoutCandidate, PhysicalPortDirection, PortConnection,
};
use crate::world::block::{Block, Direction};
use crate::world::position::{DimSize, Position};
use crate::world::World3D;

/// Conservative rotation set. Redstone is not rotation-invariant (torch
/// attachment, repeater facing, gravity), so only transforms proven by a
/// target-specific verification may be added later. M1 supports identity only.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum MacroRotation {
    None,
    Yaw90,
    Yaw180,
    Yaw270,
}

impl MacroRotation {
    pub fn is_identity(self) -> bool {
        matches!(self, MacroRotation::None)
    }
}

/// One public pin of a macro template, in macro-local coordinates.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MacroPin {
    pub name: String,
    pub position: Position,
    pub direction: PhysicalPortDirection,
    pub connection: PortConnection,
    /// Spatial facing: the direction the signal leaves or enters the macro,
    /// derived from the nearest escape cell and the macro boundary.
    pub facing: Direction,
    /// Precomputed local escape cells the router may start from.
    pub escape: Vec<Position>,
}

/// A verified, reusable macro: a normalized block layout plus its interface.
#[derive(Clone, Debug, PartialEq)]
pub struct MacroTemplate {
    /// Macro definition name, e.g. `not.compact`.
    pub name: String,
    /// Implementation variant this template implements, e.g. `not`.
    pub variant: String,
    pub size: DimSize,
    /// Blocks in macro-local coordinates (origin at `(0, 0, 0)`).
    pub blocks: Vec<(Position, Block)>,
    /// Cells that routing must not occupy.
    pub forbidden_routing_cells: Vec<Position>,
    pub pins: Vec<MacroPin>,
    /// Cells that must stay free around the macro.
    pub halo: usize,
    pub allowed_rotations: Vec<MacroRotation>,
    /// True when the template came from a simulator-validated candidate.
    pub verified: bool,
}

impl MacroTemplate {
    /// Normalizes a verified candidate into a reusable macro template.
    ///
    /// Blocks, pins, access points, and blocked cells are shifted so the
    /// candidate bounding box minimum becomes the local origin.
    pub fn from_candidate(candidate: &LayoutCandidate, variant: &str) -> eyre::Result<Self> {
        let bbox = candidate.bbox;
        let local = |position: Position| {
            Position(
                position.0 - bbox.min.0,
                position.1 - bbox.min.1,
                position.2 - bbox.min.2,
            )
        };
        let size = DimSize(bbox.width(), bbox.depth(), bbox.height());

        let blocks = candidate
            .world
            .iter_block()
            .into_iter()
            .map(|(position, block)| (local(position), block))
            .collect::<Vec<_>>();

        let mut pins = Vec::with_capacity(candidate.ports.len());
        for port in &candidate.ports {
            let mut escape = port
                .access_points
                .iter()
                .copied()
                .map(local)
                .collect::<Vec<_>>();
            escape.sort();
            escape.dedup();
            let position = local(port.position);
            let facing = derive_facing(position, size, &escape);
            pins.push(MacroPin {
                name: port.name.clone(),
                position,
                direction: port.direction.clone(),
                connection: port.connection,
                facing,
                escape,
            });
        }
        pins.sort_by(|left, right| left.name.cmp(&right.name));

        let mut forbidden_routing_cells = candidate
            .blocked_cells
            .iter()
            .copied()
            .map(local)
            .collect::<Vec<_>>();
        forbidden_routing_cells.sort();
        forbidden_routing_cells.dedup();

        Ok(Self {
            name: candidate.module_name.clone(),
            variant: variant.to_owned(),
            size,
            blocks,
            forbidden_routing_cells,
            pins,
            halo: candidate.halo,
            allowed_rotations: vec![MacroRotation::None],
            verified: true,
        })
    }

    pub fn pin(&self, name: &str) -> Option<&MacroPin> {
        self.pins.iter().find(|pin| pin.name == name)
    }

    /// Absolute block positions of one instance of this macro.
    pub fn instantiate(
        &self,
        origin: Position,
        rotation: MacroRotation,
    ) -> eyre::Result<Vec<(Position, Block)>> {
        if !rotation.is_identity() {
            bail!(
                "macro `{}` does not support rotation {:?} yet",
                self.name,
                rotation
            );
        }
        let mut placed = Vec::with_capacity(self.blocks.len());
        for (local, block) in &self.blocks {
            let position = Position(
                origin
                    .0
                    .checked_add(local.0)
                    .context("macro block x overflow")?,
                origin
                    .1
                    .checked_add(local.1)
                    .context("macro block y overflow")?,
                origin
                    .2
                    .checked_add(local.2)
                    .context("macro block z overflow")?,
            );
            placed.push((position, *block));
        }
        Ok(placed)
    }

    /// Absolute pin position of one instance.
    pub fn pin_position(&self, origin: Position, pin: &str) -> eyre::Result<Position> {
        let pin = self
            .pin(pin)
            .with_context(|| format!("macro `{}` has no pin `{pin}`", self.name))?;
        Ok(Position(
            origin.0 + pin.position.0,
            origin.1 + pin.position.1,
            origin.2 + pin.position.2,
        ))
    }
}

/// One placed macro instance.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MacroInstance {
    pub id: usize,
    pub macro_name: String,
    pub position: Position,
    pub rotation: MacroRotation,
}

/// A reference to one pin of one instance.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PinRef {
    pub instance: usize,
    pub pin: String,
}

/// Placement legality report. Only hard conflicts are listed; spacing and halo
/// rules belong to the placement engine.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PlacementLegality {
    /// Pairs of instance ids whose block cells intersect.
    pub overlaps: Vec<(usize, usize)>,
    /// Instance id and the first offending block cell outside the world.
    pub out_of_bounds: Vec<(usize, Position)>,
}

impl PlacementLegality {
    pub fn is_legal(&self) -> bool {
        self.overlaps.is_empty() && self.out_of_bounds.is_empty()
    }
}

/// A physical net: one driver and any number of sinks.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PhysicalNet {
    pub name: String,
    pub source: PinRef,
    pub sinks: Vec<PinRef>,
    /// Detailed route, filled by the detailed router.
    pub route: Option<Vec<Position>>,
    /// Coarse region sequence, filled by global routing.
    pub region_sequence: Option<Vec<usize>>,
    /// Congestion accumulated while routing this net.
    pub congestion: usize,
}

/// A complete placement problem: macro library, instances, and nets.
#[derive(Clone, Debug)]
pub struct PlacementProblem {
    pub macros: BTreeMap<String, MacroTemplate>,
    pub instances: Vec<MacroInstance>,
    pub nets: Vec<PhysicalNet>,
}

impl PlacementProblem {
    pub fn new() -> Self {
        Self {
            macros: BTreeMap::new(),
            instances: Vec::new(),
            nets: Vec::new(),
        }
    }

    pub fn add_macro(&mut self, template: MacroTemplate) {
        self.macros.insert(template.name.clone(), template);
    }

    pub fn add_instance(
        &mut self,
        macro_name: &str,
        position: Position,
        rotation: MacroRotation,
    ) -> eyre::Result<usize> {
        if !self.macros.contains_key(macro_name) {
            bail!("unknown macro `{macro_name}`");
        }
        let id = self.instances.len();
        self.instances.push(MacroInstance {
            id,
            macro_name: macro_name.to_owned(),
            position,
            rotation,
        });
        Ok(id)
    }

    pub fn template_for(&self, instance: usize) -> eyre::Result<&MacroTemplate> {
        let instance = self
            .instances
            .get(instance)
            .with_context(|| format!("unknown macro instance {instance}"))?;
        self.macros
            .get(&instance.macro_name)
            .with_context(|| format!("missing macro `{}`", instance.macro_name))
    }

    /// Absolute position of a pin reference.
    pub fn pin_position(&self, reference: &PinRef) -> eyre::Result<Position> {
        let instance = self
            .instances
            .get(reference.instance)
            .with_context(|| format!("unknown macro instance {}", reference.instance))?;
        let template = self.template_for(reference.instance)?;
        template.pin_position(instance.position, &reference.pin)
    }

    /// Spatial facing of a pin reference.
    pub fn pin_facing(&self, reference: &PinRef) -> eyre::Result<Direction> {
        let template = self.template_for(reference.instance)?;
        let pin = template
            .pin(&reference.pin)
            .with_context(|| format!("macro `{}` has no pin `{}`", template.name, reference.pin))?;
        Ok(pin.facing)
    }

    /// Absolute block cells of one instance.
    pub fn instance_blocks(&self, instance: usize) -> eyre::Result<Vec<(Position, Block)>> {
        let instance = self
            .instances
            .get(instance)
            .with_context(|| format!("unknown macro instance {instance}"))?;
        let template = self.template_for(instance.id)?;
        template.instantiate(instance.position, instance.rotation)
    }

    /// Pairs of instances whose macro footprints intersect, in deterministic
    /// order. The whole template bounding box is reserved, not only its blocks.
    pub fn overlaps(&self) -> Vec<(usize, usize)> {
        let mut bounds = Vec::with_capacity(self.instances.len());
        for instance in &self.instances {
            let Ok(template) = self.template_for(instance.id) else {
                continue;
            };
            let size = template.size;
            bounds.push((
                instance.id,
                instance.position,
                Position(
                    instance.position.0 + size.0.saturating_sub(1),
                    instance.position.1 + size.1.saturating_sub(1),
                    instance.position.2 + size.2.saturating_sub(1),
                ),
            ));
        }

        let mut pairs = BTreeSet::new();
        for first in 0..bounds.len() {
            for second in (first + 1)..bounds.len() {
                let (first_id, first_min, first_max) = bounds[first];
                let (second_id, second_min, second_max) = bounds[second];
                let intersects = first_min.0 <= second_max.0
                    && second_min.0 <= first_max.0
                    && first_min.1 <= second_max.1
                    && second_min.1 <= first_max.1
                    && first_min.2 <= second_max.2
                    && second_min.2 <= first_max.2;
                if intersects {
                    pairs.insert((first_id.min(second_id), first_id.max(second_id)));
                }
            }
        }
        pairs.into_iter().collect()
    }

    /// Block cells that fall outside a world of the given size.
    pub fn out_of_bounds(&self, size: DimSize) -> Vec<(usize, Position)> {
        let mut result = Vec::new();
        for instance in &self.instances {
            let Ok(blocks) = self.instance_blocks(instance.id) else {
                continue;
            };
            for (position, _) in blocks {
                if !size.bound_on(position) {
                    result.push((instance.id, position));
                }
            }
        }
        result.sort();
        result.dedup();
        result
    }

    pub fn legality(&self, size: DimSize) -> PlacementLegality {
        PlacementLegality {
            overlaps: self.overlaps(),
            out_of_bounds: self.out_of_bounds(size),
        }
    }

    /// Inclusive bounding box over all instance footprints.
    pub fn bounding_box(&self) -> Option<(Position, Position)> {
        let mut min = Position(usize::MAX, usize::MAX, usize::MAX);
        let mut max = Position(0, 0, 0);
        let mut any = false;
        for instance in &self.instances {
            let Ok(template) = self.template_for(instance.id) else {
                continue;
            };
            any = true;
            min = Position(
                min.0.min(instance.position.0),
                min.1.min(instance.position.1),
                min.2.min(instance.position.2),
            );
            max = Position(
                max.0
                    .max(instance.position.0 + template.size.0.saturating_sub(1)),
                max.1
                    .max(instance.position.1 + template.size.1.saturating_sub(1)),
                max.2
                    .max(instance.position.2 + template.size.2.saturating_sub(1)),
            );
        }
        any.then_some((min, max))
    }

    /// Half-perimeter wire length of one net, in blocks.
    pub fn net_half_perimeter(&self, net: &PhysicalNet) -> eyre::Result<usize> {
        let mut positions = vec![self.pin_position(&net.source)?];
        for sink in &net.sinks {
            positions.push(self.pin_position(sink)?);
        }
        Ok(half_perimeter(&positions))
    }

    /// Instantiates every macro into a world. Overlaps and out-of-bounds
    /// blocks are rejected.
    pub fn to_world(&self, size: DimSize) -> eyre::Result<World3D> {
        let mut world = World3D::new(size);
        for instance in &self.instances {
            let template = self.template_for(instance.id)?;
            for (position, block) in template.instantiate(instance.position, instance.rotation)? {
                if !world.size.bound_on(position) {
                    bail!(
                        "macro instance {} block {:?} is out of bounds",
                        instance.id,
                        position
                    );
                }
                if !world[position].kind.is_air() {
                    bail!(
                        "macro instance {} block {:?} overlaps an existing block",
                        instance.id,
                        position
                    );
                }
                world[position] = block;
            }
        }
        Ok(world)
    }

    /// Total half-perimeter wire length over all nets, in blocks.
    pub fn estimate_wire_length(&self) -> eyre::Result<usize> {
        let mut total = 0;
        for net in &self.nets {
            total += self.net_half_perimeter(net)?;
        }
        Ok(total)
    }
}

impl Default for PlacementProblem {
    fn default() -> Self {
        Self::new()
    }
}

fn half_perimeter(positions: &[Position]) -> usize {
    let (mut min_x, mut max_x) = (usize::MAX, 0);
    let (mut min_y, mut max_y) = (usize::MAX, 0);
    let (mut min_z, mut max_z) = (usize::MAX, 0);
    for position in positions {
        min_x = min_x.min(position.0);
        max_x = max_x.max(position.0);
        min_y = min_y.min(position.1);
        max_y = max_y.max(position.1);
        min_z = min_z.min(position.2);
        max_z = max_z.max(position.2);
    }
    (max_x - min_x) + (max_y - min_y) + (max_z - min_z)
}

fn derive_facing(position: Position, size: DimSize, escape: &[Position]) -> Direction {
    let mut candidates = escape
        .iter()
        .copied()
        .filter(|cell| *cell != position)
        .collect::<Vec<_>>();
    candidates.sort_by_key(|cell| (cell.manhattan_distance(&position), cell.0, cell.1, cell.2));
    if let Some(cell) = candidates.first() {
        return position.diff(*cell);
    }
    boundary_facing(position, size)
}

fn boundary_facing(position: Position, size: DimSize) -> Direction {
    let faces = [
        (position.0, Direction::West),
        (
            size.0.saturating_sub(1).saturating_sub(position.0),
            Direction::East,
        ),
        (position.1, Direction::South),
        (
            size.1.saturating_sub(1).saturating_sub(position.1),
            Direction::North,
        ),
        (position.2, Direction::Bottom),
        (
            size.2.saturating_sub(1).saturating_sub(position.2),
            Direction::Top,
        ),
    ];
    let mut best = (usize::MAX, Direction::None);
    for (distance, direction) in faces {
        if distance < best.0 {
            best = (distance, direction);
        }
    }
    if best.0 == 0 {
        best.1
    } else {
        Direction::None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transform::place_and_route::global_pnr::ir::PhysicalPort;
    use crate::world::block::{BlockKind, Direction};

    fn cobble() -> Block {
        Block {
            kind: BlockKind::Cobble {
                on_count: 0,
                on_base_count: 0,
            },
            direction: Direction::None,
        }
    }

    fn torch() -> Block {
        Block {
            kind: BlockKind::Torch { is_on: false },
            direction: Direction::Bottom,
        }
    }

    fn not_candidate() -> LayoutCandidate {
        // A minimal NOT cell: support cobble with a torch on top, one block in
        // from the bounding box minimum so normalization is exercised.
        let mut world = World3D::new(DimSize(4, 4, 4));
        world[Position(1, 1, 1)] = cobble();
        world[Position(1, 1, 2)] = torch();
        let ports = vec![
            PhysicalPort {
                name: "a".to_owned(),
                direction: PhysicalPortDirection::Input,
                position: Position(1, 1, 1),
                route_position: Some(Position(1, 1, 1)),
                access_points: vec![Position(2, 1, 1)],
                connection: PortConnection::Direct,
            },
            PhysicalPort {
                name: "y".to_owned(),
                direction: PhysicalPortDirection::Output,
                position: Position(1, 1, 2),
                route_position: Some(Position(1, 1, 2)),
                access_points: vec![Position(1, 2, 2)],
                connection: PortConnection::Direct,
            },
        ];
        LayoutCandidate::from_world("not".to_owned(), world, ports).expect("candidate")
    }

    #[test]
    fn macro_template_round_trips_through_a_candidate() -> eyre::Result<()> {
        let candidate = not_candidate();
        let template = MacroTemplate::from_candidate(&candidate, "not")?;

        assert_eq!(template.name, "not");
        assert_eq!(template.size, DimSize(1, 1, 2));
        assert_eq!(template.blocks.len(), 2);
        assert!(template.verified);

        // Normalization moves the bbox minimum to the origin.
        let cobble_position = template
            .blocks
            .iter()
            .find(|(_, block)| block.kind.is_cobble())
            .map(|(position, _)| *position)
            .expect("cobble block");
        assert_eq!(cobble_position, Position(0, 0, 0));

        // Pins keep their interface and gain local escape cells and facings.
        let input = template.pin("a").expect("input pin");
        assert_eq!(input.position, Position(0, 0, 0));
        assert_eq!(input.escape, vec![Position(1, 0, 0)]);
        assert_eq!(input.facing, Direction::East);
        let output = template.pin("y").expect("output pin");
        assert_eq!(output.position, Position(0, 0, 1));
        assert_eq!(output.facing, Direction::North);

        // Instantiation restores absolute positions.
        let placed = template.instantiate(Position(10, 10, 0), MacroRotation::None)?;
        assert!(placed
            .iter()
            .any(|(position, block)| *position == Position(10, 10, 0) && block.kind.is_cobble()));
        assert!(placed
            .iter()
            .any(|(position, block)| *position == Position(10, 10, 1) && block.kind.is_torch()));
        assert_eq!(
            template.pin_position(Position(10, 10, 0), "y")?,
            Position(10, 10, 1)
        );
        Ok(())
    }

    #[test]
    fn placement_problem_composes_macros_and_measures_wire_length() -> eyre::Result<()> {
        let template = MacroTemplate::from_candidate(&not_candidate(), "not")?;
        let mut problem = PlacementProblem::new();
        problem.add_macro(template);

        let first = problem.add_instance("not", Position(0, 0, 0), MacroRotation::None)?;
        let second = problem.add_instance("not", Position(5, 0, 0), MacroRotation::None)?;
        problem.nets.push(PhysicalNet {
            name: "n0".to_owned(),
            source: PinRef {
                instance: first,
                pin: "y".to_owned(),
            },
            sinks: vec![PinRef {
                instance: second,
                pin: "a".to_owned(),
            }],
            route: None,
            region_sequence: None,
            congestion: 0,
        });

        assert_eq!(
            problem.pin_position(&problem.nets[0].source)?,
            Position(0, 0, 1)
        );
        assert_eq!(
            problem.pin_position(&problem.nets[0].sinks[0])?,
            Position(5, 0, 0)
        );
        assert_eq!(problem.estimate_wire_length()?, 6);

        let world = problem.to_world(DimSize(8, 4, 4))?;
        assert_eq!(world.iter_block().len(), 4);
        Ok(())
    }

    #[test]
    fn placement_problem_rejects_overlap_and_bad_references() -> eyre::Result<()> {
        let template = MacroTemplate::from_candidate(&not_candidate(), "not")?;
        let mut problem = PlacementProblem::new();
        problem.add_macro(template);
        problem.add_instance("not", Position(0, 0, 0), MacroRotation::None)?;
        problem.add_instance("not", Position(0, 0, 0), MacroRotation::None)?;

        let error = problem.to_world(DimSize(4, 4, 4)).unwrap_err().to_string();
        assert!(error.contains("overlaps"), "{error}");

        let unknown = problem.pin_position(&PinRef {
            instance: 9,
            pin: "y".to_owned(),
        });
        assert!(unknown.is_err());
        Ok(())
    }

    #[test]
    fn facing_prefers_escape_cells_then_boundary() {
        assert_eq!(
            derive_facing(Position(0, 0, 0), DimSize(2, 1, 1), &[Position(0, 1, 0)]),
            Direction::North
        );
        assert_eq!(
            derive_facing(Position(1, 0, 0), DimSize(2, 1, 1), &[]),
            Direction::East
        );
        assert_eq!(
            derive_facing(Position(0, 0, 0), DimSize(2, 2, 2), &[]),
            Direction::West
        );
        assert_eq!(
            derive_facing(Position(1, 1, 1), DimSize(3, 3, 3), &[]),
            Direction::None
        );
    }

    #[test]
    fn placement_legality_reports_overlaps_and_bounds() -> eyre::Result<()> {
        let template = MacroTemplate::from_candidate(&not_candidate(), "not")?;
        let mut problem = PlacementProblem::new();
        problem.add_macro(template);

        let first = problem.add_instance("not", Position(0, 0, 0), MacroRotation::None)?;
        let second = problem.add_instance("not", Position(0, 0, 0), MacroRotation::None)?;
        assert_eq!(problem.overlaps(), vec![(first, second)]);

        let legality = problem.legality(DimSize(8, 4, 4));
        assert!(!legality.is_legal());
        assert!(legality.out_of_bounds.is_empty());

        let third = problem.add_instance("not", Position(6, 0, 0), MacroRotation::None)?;
        assert_eq!(
            problem.out_of_bounds(DimSize(4, 4, 4)),
            vec![(third, Position(6, 0, 0)), (third, Position(6, 0, 1))]
        );
        Ok(())
    }

    #[test]
    fn placement_bounding_box_covers_all_instances() -> eyre::Result<()> {
        let template = MacroTemplate::from_candidate(&not_candidate(), "not")?;
        let mut problem = PlacementProblem::new();
        problem.add_macro(template);

        assert_eq!(problem.bounding_box(), None);

        problem.add_instance("not", Position(0, 0, 0), MacroRotation::None)?;
        problem.add_instance("not", Position(5, 2, 0), MacroRotation::None)?;
        assert_eq!(
            problem.bounding_box(),
            Some((Position(0, 0, 0), Position(5, 2, 1)))
        );
        Ok(())
    }

    #[test]
    fn pin_facing_and_net_metrics_are_available_per_net() -> eyre::Result<()> {
        let template = MacroTemplate::from_candidate(&not_candidate(), "not")?;
        let mut problem = PlacementProblem::new();
        problem.add_macro(template);

        let first = problem.add_instance("not", Position(0, 0, 0), MacroRotation::None)?;
        let second = problem.add_instance("not", Position(5, 0, 0), MacroRotation::None)?;
        problem.nets.push(PhysicalNet {
            name: "n0".to_owned(),
            source: PinRef {
                instance: first,
                pin: "y".to_owned(),
            },
            sinks: vec![PinRef {
                instance: second,
                pin: "a".to_owned(),
            }],
            route: None,
            region_sequence: None,
            congestion: 0,
        });

        assert_eq!(
            problem.pin_facing(&PinRef {
                instance: first,
                pin: "y".to_owned()
            })?,
            Direction::North
        );
        assert_eq!(
            problem.pin_facing(&PinRef {
                instance: second,
                pin: "a".to_owned()
            })?,
            Direction::East
        );
        assert_eq!(problem.net_half_perimeter(&problem.nets[0])?, 6);
        assert_eq!(problem.estimate_wire_length()?, 6);
        Ok(())
    }
}
