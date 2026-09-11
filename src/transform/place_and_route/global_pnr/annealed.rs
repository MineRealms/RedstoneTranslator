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
    let mut solution = None;
    let mut last_error = None;
    for attempt in 0..3 {
        let scale = 1usize << attempt;
        let annealing = AnnealingConfig {
            initial: InitialPlacementConfig {
                world: DimSize(
                    side.saturating_mul(scale),
                    side.saturating_mul(scale),
                    (max_height.max(4) + 4).saturating_mul(scale),
                ),
                spacing: config.spacing,
                ..Default::default()
            },
            ..Default::default()
        };
        match place_annealed(&problem, &annealing) {
            Ok(found) => {
                solution = Some(found);
                break;
            }
            Err(error) => last_error = Some(error),
        }
    }
    let solution = solution.ok_or_else(|| {
        last_error.unwrap_or_else(|| eyre::eyre!("annealed placement produced no solution"))
    })?;
    if !solution.is_legal() {
        eyre::bail!("annealed placement is not legal: {:?}", solution.legality);
    }

    let placed = selected
        .iter()
        .enumerate()
        .map(|(index, (_, candidate))| PlacedModule {
            module_name: candidate.module_name.clone(),
            candidate_index: index,
            origin: solution.instances[index].position,
            bbox: candidate.bbox,
        })
        .collect::<Vec<_>>();

    Ok(vec![placed])
}

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

        assert_eq!(attempts.len(), 1);
        assert_eq!(attempts[0].len(), 2);
        assert_eq!(attempts[0][0].module_name, child_name);
        assert_eq!(attempts[0][0].candidate_index, 0);
        assert_eq!(attempts[0][1].candidate_index, 1);

        let (first_min, first_max) = bounds(&attempts[0][0]);
        let (second_min, second_max) = bounds(&attempts[0][1]);
        let overlaps = first_min.0 <= second_max.0
            && second_min.0 <= first_max.0
            && first_min.1 <= second_max.1
            && second_min.1 <= first_max.1
            && first_min.2 <= second_max.2
            && second_min.2 <= first_max.2;
        assert!(!overlaps, "{:?}", attempts[0]);
        Ok(())
    }
}
