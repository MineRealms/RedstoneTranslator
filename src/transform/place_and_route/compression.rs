//! Compression ladder for the CAD flow (M5).
//!
//! Generate a valid circuit first, then shrink: try descending box sizes and
//! keep the first (smallest) valid result. The ladder is a loop around the
//! flow, not a property of the placer, so this module owns the iteration and
//! acceptance bookkeeping while callers provide the per-box attempt.

use std::collections::BTreeMap;

use crate::ir::{RoutableDesign, RoutableInstance, RoutableModuleBody};
use crate::transform::place_and_route::global_pnr::physical_intent::{
    IntentRegion, PhysicalConstraint, PhysicalIntent, PHYSICAL_INTENT_FORMAT,
};
use crate::transform::place_and_route::global_pnr::topology::ResolvedPnrTopology;
use crate::transform::place_and_route::global_pnr::{
    place_and_route_routable_design_with_visualization, GlobalPnrConfig, GlobalPnrResult,
};
use crate::world::position::DimSize;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CompressionAttempt {
    pub box_size: DimSize,
    pub accepted: bool,
    pub error: Option<String>,
}

#[derive(Clone, Debug)]
pub struct CompressionResult<T> {
    pub value: T,
    pub box_size: DimSize,
    pub attempts: Vec<CompressionAttempt>,
}

pub fn default_ladder() -> Vec<DimSize> {
    vec![
        DimSize(64, 64, 16),
        DimSize(56, 56, 14),
        DimSize(48, 48, 12),
        DimSize(40, 40, 10),
        DimSize(32, 32, 8),
        DimSize(24, 24, 6),
    ]
}

pub fn compress<T>(
    ladder: &[DimSize],
    mut attempt: impl FnMut(DimSize) -> eyre::Result<T>,
) -> eyre::Result<CompressionResult<T>> {
    if ladder.is_empty() {
        eyre::bail!("compression ladder is empty");
    }
    for window in ladder.windows(2) {
        let previous = volume(window[0]);
        let next = volume(window[1]);
        if next >= previous {
            eyre::bail!(
                "compression ladder must shrink monotonically: {:?} -> {:?}",
                window[0],
                window[1]
            );
        }
    }

    let mut attempts = Vec::new();
    let mut last_error = None;
    for &box_size in ladder {
        match attempt(box_size) {
            Ok(value) => {
                attempts.push(CompressionAttempt {
                    box_size,
                    accepted: true,
                    error: None,
                });
                return Ok(CompressionResult {
                    value,
                    box_size,
                    attempts,
                });
            }
            Err(error) => {
                attempts.push(CompressionAttempt {
                    box_size,
                    accepted: false,
                    error: Some(error.to_string()),
                });
                last_error = Some(error);
            }
        }
    }

    Err(last_error.unwrap_or_else(|| eyre::eyre!("compression ladder produced no attempt")))
}

fn volume(size: DimSize) -> usize {
    size.0 * size.1 * size.2
}

fn compression_intent(
    instances: &[RoutableInstance],
    box_size: DimSize,
    top_name: &str,
) -> PhysicalIntent {
    let region = IntentRegion {
        min: [0, 0, 0],
        max: [
            box_size.0.saturating_sub(1),
            box_size.1.saturating_sub(1),
            box_size.2.saturating_sub(1),
        ],
    };
    let mut regions = BTreeMap::new();
    regions.insert("compression-box".to_owned(), region);
    let constraints = instances
        .iter()
        .enumerate()
        .map(|(index, instance)| PhysicalConstraint::Inside {
            id: format!("compression-{index}"),
            instance: instance.name.clone(),
            region: "compression-box".to_owned(),
        })
        .collect();

    PhysicalIntent {
        format: PHYSICAL_INTENT_FORMAT.to_owned(),
        design: top_name.to_owned(),
        regions,
        constraints,
    }
}

/// Runs the full PnR flow once per box in the ladder, constraining every
/// top-level instance inside the box, and returns the first (smallest) valid
/// result.
pub fn place_and_route_with_compression(
    design: &RoutableDesign,
    ladder: &[DimSize],
    config: &GlobalPnrConfig,
) -> eyre::Result<CompressionResult<GlobalPnrResult>> {
    let topology = ResolvedPnrTopology::from_routable(design)?;
    let top = design
        .module(&design.top)
        .ok_or_else(|| eyre::eyre!("missing top Routable module `{}`", design.top))?;
    let RoutableModuleBody::Composite { instances, .. } = &top.body else {
        eyre::bail!("compression requires a composite top module");
    };
    let top_name = topology
        .definition(topology.top)
        .map(|definition| definition.display_name.clone())
        .unwrap_or_else(|| design.top.clone());

    compress(ladder, |box_size| {
        let intent = compression_intent(instances, box_size, &top_name);
        let resolved = intent.bind(&topology)?;
        let mut attempt_config = config.clone();
        attempt_config.physical_intent = Some(resolved);
        place_and_route_routable_design_with_visualization(design, &attempt_config)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_ladder_shrinks_monotonically() {
        let ladder = default_ladder();
        for window in ladder.windows(2) {
            assert!(volume(window[1]) < volume(window[0]), "{window:?}");
        }
    }

    #[test]
    fn first_successful_box_wins() -> eyre::Result<()> {
        let ladder = [DimSize(64, 64, 16), DimSize(48, 48, 12), DimSize(32, 32, 8)];
        let mut tried = Vec::new();

        let result = compress(&ladder, |box_size| {
            tried.push(box_size);
            if box_size == DimSize(48, 48, 12) {
                Ok("placed")
            } else {
                eyre::bail!("does not fit")
            }
        })?;

        assert_eq!(result.value, "placed");
        assert_eq!(result.box_size, DimSize(48, 48, 12));
        assert_eq!(tried, vec![ladder[0], ladder[1]]);
        assert_eq!(result.attempts.len(), 2);
        assert!(!result.attempts[0].accepted);
        assert!(result.attempts[1].accepted);
        Ok(())
    }

    #[test]
    fn all_boxes_failing_returns_the_last_error() {
        let ladder = [DimSize(64, 64, 16), DimSize(32, 32, 8)];
        let error = compress(&ladder, |box_size| {
            Err::<(), _>(eyre::eyre!("no fit at {:?}", box_size))
        })
        .unwrap_err()
        .to_string();

        assert!(error.contains("32"), "{error}");
    }

    #[test]
    fn non_monotonic_ladder_is_rejected() {
        let ladder = [DimSize(32, 32, 8), DimSize(64, 64, 16)];
        let error = compress(&ladder, |_| Ok(())).unwrap_err().to_string();
        assert!(error.contains("shrink monotonically"), "{error}");

        assert!(compress::<()>(&[], |_| Ok(())).is_err());
    }

    #[test]
    fn compression_intent_covers_the_box_and_every_instance() {
        let instances = vec![
            RoutableInstance {
                name: "a".to_owned(),
                module: "inv".to_owned(),
                origin: None,
            },
            RoutableInstance {
                name: "b".to_owned(),
                module: "inv".to_owned(),
                origin: None,
            },
        ];

        let intent = compression_intent(&instances, DimSize(8, 4, 2), "top");

        assert_eq!(intent.design, "top");
        let region = intent.regions.get("compression-box").expect("region");
        assert_eq!(region.min, [0, 0, 0]);
        assert_eq!(region.max, [7, 3, 1]);
        assert_eq!(intent.constraints.len(), 2);
        assert!(matches!(
            &intent.constraints[0],
            PhysicalConstraint::Inside { instance, .. } if instance == "a"
        ));
        assert!(matches!(
            &intent.constraints[1],
            PhysicalConstraint::Inside { instance, .. } if instance == "b"
        ));
    }
}
