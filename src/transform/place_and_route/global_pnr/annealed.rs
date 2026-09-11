//! Annealed placement adapter for the global PnR flow (M3.4).
//!
//! Converts one selected `LayoutCandidate` per child instance into a
//! `PlacementProblem`, runs the CAD initial placement plus simulated
//! annealing, and returns the result as `PlacedModule`s. The legacy placer
//! stays the default; `GlobalPlacementConfig::engine` selects this path.

use std::collections::BTreeMap;

use crate::transform::place_and_route::global_pnr::ir::LayoutCandidate;
use crate::transform::place_and_route::global_pnr::placer::{GlobalPlacementConfig, PlacedModule};
use crate::transform::place_and_route::global_pnr::topology::{
    ResolvedEndpoint, ResolvedPnrTopology,
};
use crate::transform::place_and_route::placement_ir::{
    MacroRotation, MacroTemplate, PhysicalNet, PinRef, PlacementProblem,
};
use crate::transform::place_and_route::sa_placer::{
    place_annealed, AnnealingConfig, InitialPlacementConfig,
};
use crate::world::position::{DimSize, Position};

pub fn placement_candidates_annealed(
    topology: &ResolvedPnrTopology,
    selected: &[(String, LayoutCandidate)],
    config: &GlobalPlacementConfig,
) -> eyre::Result<Vec<Vec<PlacedModule>>> {
    let mut problem = PlacementProblem::new();
    let mut instance_index = BTreeMap::<String, usize>::new();
    for (instance_name, candidate) in selected {
        let mut template = MacroTemplate::from_candidate(candidate, &candidate.module_name)?;
        template.name = instance_name.clone();
        template.variant = candidate.module_name.clone();
        problem.add_macro(template);
        let index = problem.add_instance(instance_name, Position(0, 0, 0), MacroRotation::None)?;
        instance_index.insert(instance_name.clone(), index);
    }

    for net in &topology.nets {
        let Some(source) = instance_endpoint(topology, &net.driver, &instance_index) else {
            continue;
        };
        let sinks = net
            .sinks
            .iter()
            .filter_map(|sink| instance_endpoint(topology, sink, &instance_index))
            .collect::<Vec<_>>();
        if sinks.is_empty() {
            continue;
        }
        problem.nets.push(PhysicalNet {
            name: net.display_name.clone(),
            source,
            sinks,
            route: None,
            region_sequence: None,
            congestion: 0,
        });
    }

    let total_volume = selected
        .iter()
        .map(|(_, candidate)| {
            candidate.bbox.width() * candidate.bbox.depth() * candidate.bbox.height()
        })
        .sum::<usize>();
    let max_footprint = selected
        .iter()
        .map(|(_, candidate)| candidate.bbox.width().max(candidate.bbox.depth()))
        .max()
        .unwrap_or(1);
    let side = ((total_volume as f64).cbrt().ceil() as usize * 2)
        .max(max_footprint)
        .max(4);
    let max_height = selected
        .iter()
        .map(|(_, candidate)| candidate.bbox.height())
        .max()
        .unwrap_or(1);
    let mut solutions = Vec::new();
    let mut last_error = None;
    for seed_offset in 0..ANNEALED_SEEDS {
        for attempt in 0..3 {
            let scale = 1usize << attempt;
            let annealing = AnnealingConfig {
                seed: 7u64.wrapping_add(seed_offset.wrapping_mul(0x9E37_79B9_7F4A_7C15)),
                initial: InitialPlacementConfig {
                    world: DimSize(
                        side.saturating_mul(scale),
                        side.saturating_mul(scale),
                        (max_height.max(4) + 4).saturating_mul(scale),
                    ),
                    spacing: config.spacing.max(ANNEALED_MIN_SPACING),
                    ..Default::default()
                },
                ..Default::default()
            };
            match place_annealed(&problem, &annealing) {
                Ok(found) if found.is_legal() => {
                    solutions.push(found);
                    break;
                }
                Ok(found) => {
                    last_error = Some(eyre::eyre!(
                        "annealed placement is not legal: {:?}",
                        found.legality
                    ));
                }
                Err(error) => last_error = Some(error),
            }
        }
    }
    if solutions.is_empty() {
        return Err(
            last_error.unwrap_or_else(|| eyre::eyre!("annealed placement produced no solution"))
        );
    }

    if std::env::var_os("MCHDL_DEBUG_ANNEALED").is_some() {
        for (attempt, solution) in solutions.iter().enumerate() {
            eprintln!("[annealed] attempt {attempt}:");
            for (index, (name, candidate)) in selected.iter().enumerate() {
                let origin = solution.instances[index].position;
                eprintln!(
                    "[annealed] {name} origin={origin:?} size={:?} bbox={:?}",
                    candidate.bbox.width(),
                    candidate.bbox
                );
                let template = problem.template_for(index)?;
                for pin in &template.pins {
                    let position = Position(
                        origin.0 + pin.position.0,
                        origin.1 + pin.position.1,
                        origin.2 + pin.position.2,
                    );
                    let escapes = pin
                        .escape
                        .iter()
                        .map(|escape| {
                            Position(
                                origin.0 + escape.0,
                                origin.1 + escape.1,
                                origin.2 + escape.2,
                            )
                        })
                        .collect::<Vec<_>>();
                    eprintln!(
                        "[annealed]   pin `{}` pos={position:?} facing={:?} escapes={escapes:?}",
                        pin.name, pin.facing
                    );
                }
            }
        }
    }

    let mut seen = std::collections::BTreeSet::new();
    let mut attempts = Vec::new();
    for solution in &solutions {
        let origins = solution
            .instances
            .iter()
            .map(|instance| instance.position)
            .collect::<Vec<_>>();
        if !seen.insert(origins) {
            continue;
        }
        let placed = selected
            .iter()
            .enumerate()
            .map(|(index, (_, candidate))| PlacedModule {
                module_name: candidate.module_name.clone(),
                candidate_index: index,
                origin: Position(
                    solution.instances[index].position.0 + PLACEMENT_MARGIN,
                    solution.instances[index].position.1 + PLACEMENT_MARGIN,
                    solution.instances[index].position.2 + PLACEMENT_MARGIN,
                ),
                bbox: candidate.bbox,
            })
            .collect::<Vec<_>>();
        attempts.push(placed);
    }

    Ok(attempts)
}

/// Number of deterministic annealing seeds tried per layout combination. The
/// router tries every resulting placement attempt, mirroring the legacy
/// engine's multiple placement attempts.
const ANNEALED_SEEDS: u64 = 4;

/// Minimum channel width between annealed macros so the detailed router can
/// always escape a pin. The legacy shelf placer leaves comparable channels.
const ANNEALED_MIN_SPACING: usize = 6;

/// Free cells kept between the placement box origin and the macros so the
/// router can place external input switches and escape routing on every side.
const PLACEMENT_MARGIN: usize = 4;

fn instance_endpoint(
    topology: &ResolvedPnrTopology,
    endpoint: &ResolvedEndpoint,
    instance_index: &BTreeMap<String, usize>,
) -> Option<PinRef> {
    let ResolvedEndpoint::InstancePort { instance, port } = endpoint else {
        return None;
    };
    let resolved = topology.instances.get(instance.0)?;
    let problem_index = *instance_index.get(&resolved.display_name)?;
    let port_name = topology.port(*port)?.name.clone();
    Some(PinRef {
        instance: problem_index,
        pin: port_name,
    })
}

#[cfg(test)]
mod tests {
    use eyre::{bail, ContextCompat};

    use super::*;
    use crate::ir::{LogicalDesign, RoutableModuleBody};
    use crate::transform::place_and_route::global_pnr::candidate::{
        generate_routable_module_candidates_with_progress_label, UnitCandidateConfig,
    };

    fn bounds(module: &PlacedModule) -> (Position, Position) {
        (
            module.origin,
            Position(
                module.origin.0 + module.bbox.width() - 1,
                module.origin.1 + module.bbox.depth() - 1,
                module.origin.2 + module.bbox.height() - 1,
            ),
        )
    }

    #[test]
    fn annealed_placement_adapter_places_instances_legally() -> eyre::Result<()> {
        let source = r#"
            module inv(a, y);
              input a;
              output y;
              assign y = ~a;
            endmodule

            module top(a, y);
              input a;
              output y;
              wire n;
              inv i0(.a(a), .y(n));
              inv i1(.a(n), .y(y));
            endmodule
        "#;
        let logical = LogicalDesign::from_verilog_source_named(source, "annealed-test.v")?;
        let routable = logical.lower_to_routable()?;
        let topology = ResolvedPnrTopology::from_routable(&routable)?;
        let top = routable.module(&routable.top).context("top module")?;
        let RoutableModuleBody::Composite { instances, .. } = &top.body else {
            bail!("test top module is not composite");
        };
        let child_name = instances
            .first()
            .context("top has no instances")?
            .module
            .clone();
        let inv = routable.module(&child_name).context("child module")?;
        let candidates = generate_routable_module_candidates_with_progress_label(
            inv,
            &UnitCandidateConfig {
                max_candidates: 1,
                ..Default::default()
            },
            None,
            None,
        )?;
        let candidate = candidates
            .into_iter()
            .next()
            .context("inv produced no candidate")?;
        let selected = vec![
            ("i0".to_owned(), candidate.clone()),
            ("i1".to_owned(), candidate),
        ];
        let config = GlobalPlacementConfig {
            spacing: 2,
            ..Default::default()
        };

        let attempts = placement_candidates_annealed(&topology, &selected, &config)?;

        assert!(!attempts.is_empty());
        for placed in &attempts {
            assert_eq!(placed.len(), 2);
            assert_eq!(placed[0].module_name, child_name);
            assert_eq!(placed[0].candidate_index, 0);
            assert_eq!(placed[1].candidate_index, 1);

            let (first_min, first_max) = bounds(&placed[0]);
            let (second_min, second_max) = bounds(&placed[1]);
            let overlaps = first_min.0 <= second_max.0
                && second_min.0 <= first_max.0
                && first_min.1 <= second_max.1
                && second_min.1 <= first_max.1
                && first_min.2 <= second_max.2
                && second_min.2 <= first_max.2;
            assert!(!overlaps, "{placed:?}");
        }
        Ok(())
    }
}
