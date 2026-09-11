//! Deterministic initial placement for the CAD flow (M3.1).
//!
//! The netlists are small and their topology matters, so annealing must not
//! start from random positions. This module produces the starting point:
//! a connectivity-ordered shelf seed, a barycenter relaxation pass, and an
//! overlap repair that guarantees a legal result or reports the instances that
//! could not be separated within the world bounds.

use std::cmp::Reverse;

use eyre::{bail, ContextCompat};

use crate::transform::place_and_route::placement_ir::{
    MacroInstance, PlacementLegality, PlacementProblem,
};
use crate::world::position::{DimSize, Position};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InitialPlacementConfig {
    pub world: DimSize,
    pub spacing: usize,
    pub barycenter_iterations: usize,
    pub max_step: usize,
    pub repair_iterations: usize,
}

impl Default for InitialPlacementConfig {
    fn default() -> Self {
        Self {
            world: DimSize(64, 64, 8),
            spacing: 2,
            barycenter_iterations: 32,
            max_step: 4,
            repair_iterations: 64,
        }
    }
}

#[derive(Clone, Debug)]
pub struct PlacementSolution {
    pub instances: Vec<MacroInstance>,
    pub wire_length: usize,
    pub legality: PlacementLegality,
}

impl PlacementSolution {
    pub fn is_legal(&self) -> bool {
        self.legality.is_legal()
    }
}

pub fn place_initial(
    problem: &PlacementProblem,
    config: &InitialPlacementConfig,
) -> eyre::Result<PlacementSolution> {
    let mut instances = seed_placement(problem, config)?;

    for _ in 0..config.barycenter_iterations {
        if !barycenter_pass(problem, &mut instances, config)? {
            break;
        }
    }

    repair_overlaps(problem, &mut instances, config)?;

    let mut placed = problem.clone();
    placed.instances = instances.clone();
    let wire_length = placed.estimate_wire_length()?;
    let legality = placed.legality(config.world);

    Ok(PlacementSolution {
        instances,
        wire_length,
        legality,
    })
}

pub(crate) fn seed_placement(
    problem: &PlacementProblem,
    config: &InitialPlacementConfig,
) -> eyre::Result<Vec<MacroInstance>> {
    let mut order = problem
        .instances
        .iter()
        .map(|instance| (instance_degree(problem, instance.id), instance.id))
        .collect::<Vec<_>>();
    order.sort_by_key(|(degree, id)| (Reverse(*degree), *id));

    let mut instances = problem.instances.clone();
    let mut cursor = Position(0, 0, 0);
    let mut row_depth = 0usize;
    let mut layer_height = 0usize;

    for (_, id) in order {
        let size = problem.template_for(id)?.size;
        if size.0 == 0 || size.1 == 0 || size.2 == 0 {
            bail!("macro instance {id} has an empty footprint");
        }
        if size.0 > config.world.0 || size.1 > config.world.1 || size.2 > config.world.2 {
            bail!(
                "macro instance {id} ({:?}) does not fit in world {:?}",
                size,
                config.world
            );
        }

        if cursor.0 + size.0 > config.world.0 {
            cursor.0 = 0;
            cursor.1 += row_depth + config.spacing;
            row_depth = 0;
        }
        if cursor.1 + size.1 > config.world.1 {
            cursor.1 = 0;
            cursor.2 += layer_height + config.spacing;
            layer_height = 0;
        }
        if cursor.2 + size.2 > config.world.2 {
            bail!(
                "initial placement of {} instance(s) does not fit in world {:?}",
                problem.instances.len(),
                config.world
            );
        }

        instances[id].position = cursor;
        cursor.0 += size.0 + config.spacing;
        row_depth = row_depth.max(size.1);
        layer_height = layer_height.max(size.2);
    }

    Ok(instances)
}

fn barycenter_pass(
    problem: &PlacementProblem,
    instances: &mut [MacroInstance],
    config: &InitialPlacementConfig,
) -> eyre::Result<bool> {
    let mut moved = false;
    for index in 0..instances.len() {
        let Some(step) = barycenter_step(problem, instances, index, config)? else {
            continue;
        };
        let size = problem.template_for(index)?.size;
        let origin = instances[index].position;
        let next = clamp_origin(
            Position(
                (origin.0 as i64 + step.0).max(0) as usize,
                (origin.1 as i64 + step.1).max(0) as usize,
                (origin.2 as i64 + step.2).max(0) as usize,
            ),
            size,
            config.world,
        );
        if next != origin {
            instances[index].position = next;
            moved = true;
        }
    }
    Ok(moved)
}

fn barycenter_step(
    problem: &PlacementProblem,
    instances: &[MacroInstance],
    index: usize,
    config: &InitialPlacementConfig,
) -> eyre::Result<Option<(i64, i64, i64)>> {
    let mut own_sum = (0usize, 0usize, 0usize);
    let mut own_count = 0usize;
    let mut other_sum = (0usize, 0usize, 0usize);
    let mut other_count = 0usize;

    for net in &problem.nets {
        let mut endpoints = vec![&net.source];
        endpoints.extend(net.sinks.iter());
        if !endpoints.iter().any(|endpoint| endpoint.instance == index) {
            continue;
        }
        for endpoint in endpoints {
            let position = pin_position_with(problem, instances, endpoint)?;
            if endpoint.instance == index {
                own_sum.0 += position.0;
                own_sum.1 += position.1;
                own_sum.2 += position.2;
                own_count += 1;
            } else {
                other_sum.0 += position.0;
                other_sum.1 += position.1;
                other_sum.2 += position.2;
                other_count += 1;
            }
        }
    }

    if own_count == 0 || other_count == 0 {
        return Ok(None);
    }

    let own = (
        (own_sum.0 / own_count) as i64,
        (own_sum.1 / own_count) as i64,
        (own_sum.2 / own_count) as i64,
    );
    let other = (
        (other_sum.0 / other_count) as i64,
        (other_sum.1 / other_count) as i64,
        (other_sum.2 / other_count) as i64,
    );
    let step = (
        clamp_delta(other.0 - own.0, config.max_step),
        clamp_delta(other.1 - own.1, config.max_step),
        clamp_delta(other.2 - own.2, config.max_step),
    );

    if step == (0, 0, 0) {
        Ok(None)
    } else {
        Ok(Some(step))
    }
}

pub(crate) fn repair_overlaps(
    problem: &PlacementProblem,
    instances: &mut [MacroInstance],
    config: &InitialPlacementConfig,
) -> eyre::Result<()> {
    for _ in 0..config.repair_iterations {
        let pairs = overlapping_pairs(problem, instances)?;
        if pairs.is_empty() {
            return Ok(());
        }

        for (low, high, overlaps) in pairs {
            if separate_pair(problem, instances, low, high, overlaps, config)? {
                continue;
            }
        }
    }

    let remaining = overlapping_pairs(problem, instances)?;
    if !remaining.is_empty() {
        bail!(
            "could not separate {} macro pair(s) within world {:?}",
            remaining.len(),
            config.world
        );
    }
    Ok(())
}

fn separate_pair(
    problem: &PlacementProblem,
    instances: &mut [MacroInstance],
    low: usize,
    high: usize,
    overlaps: (usize, usize, usize),
    config: &InitialPlacementConfig,
) -> eyre::Result<bool> {
    let (low_min, low_max) = instance_bounds(problem, instances, low)?;
    let (high_min, high_max) = instance_bounds(problem, instances, high)?;
    let low_center = (
        (low_min.0 + low_max.0) / 2,
        (low_min.1 + low_max.1) / 2,
        (low_min.2 + low_max.2) / 2,
    );
    let high_center = (
        (high_min.0 + high_max.0) / 2,
        (high_min.1 + high_max.1) / 2,
        (high_min.2 + high_max.2) / 2,
    );

    let mut axes = [
        (overlaps.0, 0usize),
        (overlaps.1, 1usize),
        (overlaps.2, 2usize),
    ];
    axes.sort_by_key(|(overlap, axis)| (*overlap, *axis));

    let size = problem.template_for(high)?.size;
    for (overlap, axis) in axes {
        if overlap == 0 {
            continue;
        }
        let mut next = instances[high].position;
        let forward = match axis {
            0 => high_center.0 >= low_center.0,
            1 => high_center.1 >= low_center.1,
            _ => high_center.2 >= low_center.2,
        };
        match (axis, forward) {
            (0, true) => next.0 += overlap,
            (0, false) => next.0 = next.0.saturating_sub(overlap),
            (1, true) => next.1 += overlap,
            (1, false) => next.1 = next.1.saturating_sub(overlap),
            (2, true) => next.2 += overlap,
            (2, false) => next.2 = next.2.saturating_sub(overlap),
            _ => unreachable!(),
        }
        next = clamp_origin(next, size, config.world);
        if next != instances[high].position {
            instances[high].position = next;
            return Ok(true);
        }
    }

    Ok(false)
}

fn overlapping_pairs(
    problem: &PlacementProblem,
    instances: &[MacroInstance],
) -> eyre::Result<Vec<(usize, usize, (usize, usize, usize))>> {
    let mut result = Vec::new();
    for low in 0..instances.len() {
        let (low_min, low_max) = instance_bounds(problem, instances, low)?;
        for high in (low + 1)..instances.len() {
            let (high_min, high_max) = instance_bounds(problem, instances, high)?;
            let overlap = (
                axis_overlap(low_min.0, low_max.0, high_min.0, high_max.0),
                axis_overlap(low_min.1, low_max.1, high_min.1, high_max.1),
                axis_overlap(low_min.2, low_max.2, high_min.2, high_max.2),
            );
            if overlap.0 > 0 && overlap.1 > 0 && overlap.2 > 0 {
                result.push((low, high, overlap));
            }
        }
    }
    Ok(result)
}

fn instance_bounds(
    problem: &PlacementProblem,
    instances: &[MacroInstance],
    index: usize,
) -> eyre::Result<(Position, Position)> {
    let size = problem.template_for(index)?.size;
    let origin = instances[index].position;
    Ok((
        origin,
        Position(
            origin.0 + size.0 - 1,
            origin.1 + size.1 - 1,
            origin.2 + size.2 - 1,
        ),
    ))
}

fn axis_overlap(min_a: usize, max_a: usize, min_b: usize, max_b: usize) -> usize {
    let low = min_a.max(min_b);
    let high = max_a.min(max_b);
    if high < low {
        0
    } else {
        high - low + 1
    }
}

fn pin_position_with(
    problem: &PlacementProblem,
    instances: &[MacroInstance],
    reference: &crate::transform::place_and_route::placement_ir::PinRef,
) -> eyre::Result<Position> {
    let instance = instances
        .get(reference.instance)
        .with_context(|| format!("unknown macro instance {}", reference.instance))?;
    let template = problem.template_for(reference.instance)?;
    template.pin_position(instance.position, &reference.pin)
}

fn clamp_origin(origin: Position, size: DimSize, world: DimSize) -> Position {
    Position(
        origin.0.min(world.0.saturating_sub(size.0)),
        origin.1.min(world.1.saturating_sub(size.1)),
        origin.2.min(world.2.saturating_sub(size.2)),
    )
}

fn clamp_delta(delta: i64, limit: usize) -> i64 {
    let limit = limit as i64;
    delta.clamp(-limit, limit)
}

fn instance_degree(problem: &PlacementProblem, instance: usize) -> usize {
    problem
        .nets
        .iter()
        .map(|net| {
            usize::from(net.source.instance == instance)
                + net
                    .sinks
                    .iter()
                    .filter(|sink| sink.instance == instance)
                    .count()
        })
        .sum()
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlacementCostModel {
    pub wire_length_weight: f64,
    pub bounding_box_weight: f64,
    pub blocked_pin_weight: f64,
    pub overlap_weight: f64,
    pub spacing_weight: f64,
    pub pin_access_weight: f64,
    /// Minimum free gap between macro footprints, in blocks.
    pub spacing: usize,
}

impl Default for PlacementCostModel {
    fn default() -> Self {
        Self {
            wire_length_weight: 1.0,
            bounding_box_weight: 0.1,
            blocked_pin_weight: 50.0,
            overlap_weight: 100.0,
            spacing_weight: 20.0,
            pin_access_weight: 30.0,
            spacing: 2,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlacementCost {
    pub wire_length: usize,
    pub bounding_box_volume: usize,
    pub blocked_pins: usize,
    pub overlapping_pairs: usize,
    pub spacing_violations: usize,
    pub pin_access_violations: usize,
    pub total: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AnnealingConfig {
    pub initial: InitialPlacementConfig,
    pub cost: PlacementCostModel,
    pub seed: u64,
    pub iterations: usize,
    pub moves_per_temperature: usize,
    pub initial_temperature: f64,
    pub min_temperature: f64,
    pub cooling_rate: f64,
    pub max_translate_step: usize,
    pub restarts: usize,
}

impl Default for AnnealingConfig {
    fn default() -> Self {
        Self {
            initial: InitialPlacementConfig::default(),
            cost: PlacementCostModel::default(),
            seed: 1,
            iterations: 2_000,
            moves_per_temperature: 64,
            initial_temperature: 4.0,
            min_temperature: 0.01,
            cooling_rate: 0.95,
            max_translate_step: 2,
            restarts: 1,
        }
    }
}

pub fn place_annealed(
    problem: &PlacementProblem,
    config: &AnnealingConfig,
) -> eyre::Result<PlacementSolution> {
    let initial = place_initial(problem, &config.initial)?;
    let world = config.initial.world;

    let mut best_legal: Option<(Vec<MacroInstance>, PlacementCost)> = None;
    let mut best_overall: Option<(Vec<MacroInstance>, PlacementCost)> = None;

    let mut current = initial.instances.clone();
    let model = PlacementCostModel {
        spacing: config.initial.spacing,
        ..config.cost
    };
    let mut current_cost = placement_cost(problem, &current, &model, world)?;
    record_best(&current, current_cost, &mut best_legal, &mut best_overall);

    for restart in 0..config.restarts.max(1) {
        let restart_seed = config
            .seed
            .wrapping_add((restart as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15));
        let mut rng = DeterministicRng::new(restart_seed);
        let mut temperature = config.initial_temperature;

        for _ in 0..config.iterations {
            if temperature <= config.min_temperature {
                break;
            }
            for _ in 0..config.moves_per_temperature {
                let candidate = propose_move(problem, &current, &mut rng, config)?;
                let candidate_cost = placement_cost(problem, &candidate, &model, world)?;
                let delta = candidate_cost.total - current_cost.total;
                if delta <= 0.0 || rng.next_f64() < (-delta / temperature).exp() {
                    current = candidate;
                    current_cost = candidate_cost;
                }
                record_best(&current, current_cost, &mut best_legal, &mut best_overall);
            }
            temperature *= config.cooling_rate;
        }
    }

    let mut instances = best_legal
        .map(|(instances, _)| instances)
        .or_else(|| best_overall.map(|(instances, _)| instances))
        .unwrap_or_else(|| initial.instances.clone());

    let mut placed = problem.clone();
    placed.instances = instances.clone();
    if !placed.legality(world).is_legal()
        && repair_overlaps(problem, &mut instances, &config.initial).is_err()
    {
        instances = initial.instances.clone();
    }

    placed.instances = instances.clone();
    let wire_length = placed.estimate_wire_length()?;
    let legality = placed.legality(world);
    Ok(PlacementSolution {
        instances,
        wire_length,
        legality,
    })
}

fn record_best(
    instances: &[MacroInstance],
    cost: PlacementCost,
    best_legal: &mut Option<(Vec<MacroInstance>, PlacementCost)>,
    best_overall: &mut Option<(Vec<MacroInstance>, PlacementCost)>,
) {
    if cost.overlapping_pairs == 0
        && best_legal
            .as_ref()
            .is_none_or(|(_, best)| cost.total < best.total)
    {
        *best_legal = Some((instances.to_vec(), cost));
    }
    if best_overall
        .as_ref()
        .is_none_or(|(_, best)| cost.total < best.total)
    {
        *best_overall = Some((instances.to_vec(), cost));
    }
}

pub fn placement_cost(
    problem: &PlacementProblem,
    instances: &[MacroInstance],
    model: &PlacementCostModel,
    world: DimSize,
) -> eyre::Result<PlacementCost> {
    let mut placed = problem.clone();
    placed.instances = instances.to_vec();
    let wire_length = placed.estimate_wire_length()?;
    let bounding_box_volume = placed
        .bounding_box()
        .map(|(min, max)| (max.0 - min.0 + 1) * (max.1 - min.1 + 1) * (max.2 - min.2 + 1))
        .unwrap_or(0);
    let blocked_pins = blocked_pin_count(problem, instances, world)?;
    let overlapping_pairs = placed.overlaps().len();
    let spacing_violations = spacing_violation_count(problem, instances, model.spacing)?;
    let pin_access_violations = pin_access_violation_count(problem, instances, world)?;
    let total = model.wire_length_weight * wire_length as f64
        + model.bounding_box_weight * bounding_box_volume as f64
        + model.blocked_pin_weight * blocked_pins as f64
        + model.overlap_weight * overlapping_pairs as f64
        + model.spacing_weight * spacing_violations as f64
        + model.pin_access_weight * pin_access_violations as f64;
    Ok(PlacementCost {
        wire_length,
        bounding_box_volume,
        blocked_pins,
        overlapping_pairs,
        spacing_violations,
        pin_access_violations,
        total,
    })
}

fn instance_aabb(
    problem: &PlacementProblem,
    instance: &MacroInstance,
) -> eyre::Result<(Position, Position)> {
    let size = problem.template_for(instance.id)?.size;
    Ok((
        instance.position,
        Position(
            instance.position.0 + size.0.saturating_sub(1),
            instance.position.1 + size.1.saturating_sub(1),
            instance.position.2 + size.2.saturating_sub(1),
        ),
    ))
}

fn axis_gap(first: (Position, Position), second: (Position, Position)) -> usize {
    let gap = |min_a: usize, max_a: usize, min_b: usize, max_b: usize| {
        if max_a < min_b {
            min_b - max_a - 1
        } else if max_b < min_a {
            min_a - max_b - 1
        } else {
            0
        }
    };
    gap(first.0 .0, first.1 .0, second.0 .0, second.1 .0)
        + gap(first.0 .1, first.1 .1, second.0 .1, second.1 .1)
        + gap(first.0 .2, first.1 .2, second.0 .2, second.1 .2)
}

fn spacing_violation_count(
    problem: &PlacementProblem,
    instances: &[MacroInstance],
    required: usize,
) -> eyre::Result<usize> {
    if required == 0 {
        return Ok(0);
    }
    let bounds = instances
        .iter()
        .map(|instance| instance_aabb(problem, instance))
        .collect::<eyre::Result<Vec<_>>>()?;
    let mut violations = 0usize;
    for first in 0..bounds.len() {
        for second in (first + 1)..bounds.len() {
            let first_halo = problem.template_for(instances[first].id)?.halo;
            let second_halo = problem.template_for(instances[second].id)?.halo;
            let required = required.max(first_halo).max(second_halo);
            if axis_gap(bounds[first], bounds[second]) < required {
                violations += 1;
            }
        }
    }
    Ok(violations)
}

fn pin_access_violation_count(
    problem: &PlacementProblem,
    instances: &[MacroInstance],
    world: DimSize,
) -> eyre::Result<usize> {
    let bounds = instances
        .iter()
        .map(|instance| instance_aabb(problem, instance))
        .collect::<eyre::Result<Vec<_>>>()?;
    let mut violations = 0usize;
    for (index, instance) in instances.iter().enumerate() {
        let template = problem.template_for(instance.id)?;
        for pin in &template.pins {
            if pin.escape.is_empty() {
                continue;
            }
            let any_free = pin.escape.iter().any(|local| {
                let position = Position(
                    instance.position.0 + local.0,
                    instance.position.1 + local.1,
                    instance.position.2 + local.2,
                );
                if !world.bound_on(position) {
                    return false;
                }
                bounds.iter().enumerate().all(|(other, (min, max))| {
                    other == index
                        || position.0 < min.0
                        || position.0 > max.0
                        || position.1 < min.1
                        || position.1 > max.1
                        || position.2 < min.2
                        || position.2 > max.2
                })
            });
            if !any_free {
                violations += 1;
            }
        }
    }
    Ok(violations)
}

fn blocked_pin_count(
    problem: &PlacementProblem,
    instances: &[MacroInstance],
    world: DimSize,
) -> eyre::Result<usize> {
    let mut blocked = 0usize;
    for instance in instances {
        let template = problem.template_for(instance.id)?;
        for pin in &template.pins {
            if pin.escape.is_empty() {
                blocked += 1;
                continue;
            }
            let reachable = pin.escape.iter().any(|local| {
                world.bound_on(Position(
                    instance.position.0 + local.0,
                    instance.position.1 + local.1,
                    instance.position.2 + local.2,
                ))
            });
            if !reachable {
                blocked += 1;
            }
        }
    }
    Ok(blocked)
}

fn propose_move(
    problem: &PlacementProblem,
    instances: &[MacroInstance],
    rng: &mut DeterministicRng,
    config: &AnnealingConfig,
) -> eyre::Result<Vec<MacroInstance>> {
    let mut candidate = instances.to_vec();
    if candidate.is_empty() {
        return Ok(candidate);
    }
    let count = candidate.len();
    let choice = rng.below(100);

    if choice < 60 || count < 2 {
        let index = rng.below(count);
        let size = problem.template_for(index)?.size;
        let delta = (
            random_delta(rng, config.max_translate_step),
            random_delta(rng, config.max_translate_step),
            random_delta(rng, config.max_translate_step),
        );
        candidate[index].position = clamp_origin(
            apply_delta(instances[index].position, delta),
            size,
            config.initial.world,
        );
        return Ok(candidate);
    }

    if choice < 80 {
        let first = rng.below(count);
        let second = {
            let mut index = rng.below(count - 1);
            if index >= first {
                index += 1;
            }
            index
        };
        let position = candidate[first].position;
        candidate[first].position = candidate[second].position;
        candidate[second].position = position;
        for index in [first, second] {
            let size = problem.template_for(index)?.size;
            candidate[index].position =
                clamp_origin(candidate[index].position, size, config.initial.world);
        }
        return Ok(candidate);
    }

    spread_move(problem, instances, &mut candidate, rng, config)?;
    Ok(candidate)
}

fn spread_move(
    problem: &PlacementProblem,
    instances: &[MacroInstance],
    candidate: &mut [MacroInstance],
    rng: &mut DeterministicRng,
    config: &AnnealingConfig,
) -> eyre::Result<()> {
    let count = candidate.len();
    let index = rng.below(count);
    let (min, max) = instance_bounds(problem, instances, index)?;
    let spacing = config.initial.spacing;
    let expanded = (
        Position(
            min.0.saturating_sub(spacing),
            min.1.saturating_sub(spacing),
            min.2.saturating_sub(spacing),
        ),
        Position(
            max.0.saturating_add(spacing),
            max.1.saturating_add(spacing),
            max.2.saturating_add(spacing),
        ),
    );

    let mut sum = (0usize, 0usize, 0usize);
    let mut neighbors = 0usize;
    for other in 0..count {
        if other == index {
            continue;
        }
        let (other_min, other_max) = instance_bounds(problem, instances, other)?;
        if aabb_intersects(expanded, (other_min, other_max)) {
            sum.0 += (other_min.0 + other_max.0) / 2;
            sum.1 += (other_min.1 + other_max.1) / 2;
            sum.2 += (other_min.2 + other_max.2) / 2;
            neighbors += 1;
        }
    }

    let size = problem.template_for(index)?.size;
    let delta = if neighbors == 0 {
        (
            random_delta(rng, config.max_translate_step),
            random_delta(rng, config.max_translate_step),
            random_delta(rng, config.max_translate_step),
        )
    } else {
        let neighbor_center = (sum.0 / neighbors, sum.1 / neighbors, sum.2 / neighbors);
        let own_center = (
            (min.0 + max.0) / 2,
            (min.1 + max.1) / 2,
            (min.2 + max.2) / 2,
        );
        let delta = (
            sign(own_center.0 as i64 - neighbor_center.0 as i64),
            sign(own_center.1 as i64 - neighbor_center.1 as i64),
            sign(own_center.2 as i64 - neighbor_center.2 as i64),
        );
        if delta == (0, 0, 0) {
            (1, 0, 0)
        } else {
            delta
        }
    };

    candidate[index].position = clamp_origin(
        apply_delta(instances[index].position, delta),
        size,
        config.initial.world,
    );
    Ok(())
}

fn aabb_intersects(a: (Position, Position), b: (Position, Position)) -> bool {
    a.0 .0 <= b.1 .0
        && b.0 .0 <= a.1 .0
        && a.0 .1 <= b.1 .1
        && b.0 .1 <= a.1 .1
        && a.0 .2 <= b.1 .2
        && b.0 .2 <= a.1 .2
}

fn sign(value: i64) -> i64 {
    if value > 0 {
        1
    } else if value < 0 {
        -1
    } else {
        0
    }
}

fn apply_delta(origin: Position, delta: (i64, i64, i64)) -> Position {
    Position(
        (origin.0 as i64 + delta.0).max(0) as usize,
        (origin.1 as i64 + delta.1).max(0) as usize,
        (origin.2 as i64 + delta.2).max(0) as usize,
    )
}

fn random_delta(rng: &mut DeterministicRng, step: usize) -> i64 {
    let step = step.max(1);
    rng.below(2 * step + 1) as i64 - step as i64
}

struct DeterministicRng {
    state: u64,
}

impl DeterministicRng {
    fn new(seed: u64) -> Self {
        Self { state: seed | 1 }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    fn below(&mut self, bound: usize) -> usize {
        if bound == 0 {
            0
        } else {
            (self.next_u64() % bound as u64) as usize
        }
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transform::place_and_route::global_pnr::ir::{
        PhysicalPortDirection, PortConnection,
    };
    use crate::transform::place_and_route::placement_ir::{
        MacroPin, MacroRotation, MacroTemplate, PhysicalNet, PinRef,
    };
    use crate::world::block::{Block, BlockKind, Direction};

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

    fn not_template(name: &str) -> MacroTemplate {
        MacroTemplate {
            name: name.to_owned(),
            variant: "not".to_owned(),
            size: DimSize(1, 1, 2),
            blocks: vec![(Position(0, 0, 0), cobble()), (Position(0, 0, 1), torch())],
            forbidden_routing_cells: Vec::new(),
            pins: vec![
                MacroPin {
                    name: "a".to_owned(),
                    position: Position(0, 0, 0),
                    direction: PhysicalPortDirection::Input,
                    connection: PortConnection::Direct,
                    facing: Direction::East,
                    escape: vec![Position(1, 0, 0)],
                },
                MacroPin {
                    name: "y".to_owned(),
                    position: Position(0, 0, 1),
                    direction: PhysicalPortDirection::Output,
                    connection: PortConnection::Direct,
                    facing: Direction::North,
                    escape: vec![Position(0, 1, 1)],
                },
            ],
            halo: 0,
            allowed_rotations: vec![MacroRotation::None],
            verified: true,
        }
    }

    fn chain_problem(count: usize) -> PlacementProblem {
        let mut problem = PlacementProblem::new();
        problem.add_macro(not_template("not"));
        for _ in 0..count {
            problem
                .add_instance("not", Position(0, 0, 0), MacroRotation::None)
                .expect("instance");
        }
        for index in 0..count.saturating_sub(1) {
            problem.nets.push(PhysicalNet {
                name: format!("n{index}"),
                source: PinRef {
                    instance: index,
                    pin: "y".to_owned(),
                },
                sinks: vec![PinRef {
                    instance: index + 1,
                    pin: "a".to_owned(),
                }],
                route: None,
                region_sequence: None,
                congestion: 0,
            });
        }
        problem
    }

    #[test]
    fn initial_placement_is_deterministic_and_legal() -> eyre::Result<()> {
        let problem = chain_problem(4);
        let config = InitialPlacementConfig {
            world: DimSize(32, 32, 8),
            ..Default::default()
        };

        let first = place_initial(&problem, &config)?;
        let second = place_initial(&problem, &config)?;

        assert_eq!(first.instances, second.instances);
        assert!(first.is_legal(), "{:?}", first.legality);
        assert!(first.wire_length > 0);

        let mut placed = problem.clone();
        placed.instances = first.instances.clone();
        let world = placed.to_world(config.world)?;
        assert_eq!(world.iter_block().len(), 8);
        Ok(())
    }

    #[test]
    fn barycenter_reduces_wire_length_from_seed() -> eyre::Result<()> {
        let mut problem = PlacementProblem::new();
        problem.add_macro(not_template("not"));
        problem
            .add_instance("not", Position(0, 0, 0), MacroRotation::None)
            .expect("source");
        problem
            .add_instance("not", Position(0, 0, 0), MacroRotation::None)
            .expect("unused");
        problem
            .add_instance("not", Position(0, 0, 0), MacroRotation::None)
            .expect("sink");
        problem.nets.push(PhysicalNet {
            name: "n0".to_owned(),
            source: PinRef {
                instance: 0,
                pin: "y".to_owned(),
            },
            sinks: vec![PinRef {
                instance: 2,
                pin: "a".to_owned(),
            }],
            route: None,
            region_sequence: None,
            congestion: 0,
        });
        let config = InitialPlacementConfig {
            world: DimSize(32, 32, 8),
            ..Default::default()
        };

        let seeded = seed_placement(&problem, &config)?;
        let mut seed_problem = problem.clone();
        seed_problem.instances = seeded;
        let seed_wire_length = seed_problem.estimate_wire_length()?;

        let solution = place_initial(&problem, &config)?;

        assert!(solution.is_legal(), "{:?}", solution.legality);
        assert!(
            solution.wire_length < seed_wire_length,
            "seed={seed_wire_length} final={}",
            solution.wire_length
        );
        Ok(())
    }

    #[test]
    fn repair_separates_overlapping_instances() -> eyre::Result<()> {
        let mut problem = PlacementProblem::new();
        problem.add_macro(not_template("not"));
        problem
            .add_instance("not", Position(0, 0, 0), MacroRotation::None)
            .expect("first");
        problem
            .add_instance("not", Position(0, 0, 0), MacroRotation::None)
            .expect("second");
        let mut instances = problem.instances.clone();
        let config = InitialPlacementConfig {
            world: DimSize(16, 16, 8),
            ..Default::default()
        };

        assert!(!problem.legality(config.world).is_legal());
        repair_overlaps(&problem, &mut instances, &config)?;

        let mut placed = problem.clone();
        placed.instances = instances;
        let legality = placed.legality(config.world);
        assert!(legality.is_legal(), "{legality:?}");
        assert_eq!(placed.overlaps(), Vec::new());
        Ok(())
    }

    #[test]
    fn initial_placement_reports_a_world_that_is_too_small() {
        let problem = chain_problem(3);
        let config = InitialPlacementConfig {
            world: DimSize(1, 1, 1),
            spacing: 0,
            ..Default::default()
        };

        let error = place_initial(&problem, &config).unwrap_err().to_string();
        assert!(error.contains("does not fit"), "{error}");
    }

    #[test]
    fn annealing_keeps_or_improves_the_initial_cost() -> eyre::Result<()> {
        let problem = chain_problem(5);
        let config = AnnealingConfig {
            initial: InitialPlacementConfig {
                world: DimSize(32, 32, 8),
                ..Default::default()
            },
            iterations: 200,
            moves_per_temperature: 16,
            restarts: 2,
            ..Default::default()
        };

        let initial = place_initial(&problem, &config.initial)?;
        let initial_cost = placement_cost(
            &problem,
            &initial.instances,
            &config.cost,
            config.initial.world,
        )?;
        let annealed = place_annealed(&problem, &config)?;
        let annealed_cost = placement_cost(
            &problem,
            &annealed.instances,
            &config.cost,
            config.initial.world,
        )?;

        assert!(annealed.is_legal(), "{:?}", annealed.legality);
        assert!(annealed_cost.total <= initial_cost.total + 1e-9);
        Ok(())
    }

    #[test]
    fn annealing_is_deterministic() -> eyre::Result<()> {
        let problem = chain_problem(4);
        let config = AnnealingConfig {
            iterations: 100,
            moves_per_temperature: 8,
            ..Default::default()
        };

        let first = place_annealed(&problem, &config)?;
        let second = place_annealed(&problem, &config)?;

        assert_eq!(first.instances, second.instances);
        Ok(())
    }

    #[test]
    fn annealing_respects_world_bounds_and_materializes() -> eyre::Result<()> {
        let problem = chain_problem(4);
        let config = AnnealingConfig {
            initial: InitialPlacementConfig {
                world: DimSize(16, 16, 6),
                spacing: 1,
                ..Default::default()
            },
            iterations: 100,
            moves_per_temperature: 8,
            ..Default::default()
        };

        let solution = place_annealed(&problem, &config)?;
        let mut placed = problem.clone();
        placed.instances = solution.instances.clone();
        let world = placed.to_world(config.initial.world)?;

        assert_eq!(world.iter_block().len(), 8);
        Ok(())
    }

    #[test]
    fn placement_cost_counts_overlaps_and_blocked_pins() -> eyre::Result<()> {
        let problem = chain_problem(2);
        let config = AnnealingConfig::default();
        let mut instances = seed_placement(&problem, &config.initial)?;

        let legal = placement_cost(&problem, &instances, &config.cost, config.initial.world)?;
        assert_eq!(legal.overlapping_pairs, 0);
        assert_eq!(legal.blocked_pins, 0);
        assert!(legal.wire_length > 0);

        instances[1].position = instances[0].position;
        let overlapped = placement_cost(&problem, &instances, &config.cost, config.initial.world)?;
        assert!(overlapped.overlapping_pairs > 0);
        assert!(overlapped.total > legal.total);
        Ok(())
    }
}
