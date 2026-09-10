//! Reusable cell implementations and physical contracts.
//!
//! A cell implementation is a named physical variant of one or more stable
//! Routable definitions. It owns the candidate search policy that generates its
//! layout and a physical contract describing what global P&R may rely on.
//! Per-design floorplan intent stays in `PhysicalIntent`; this library only
//! describes reusable, verified building blocks.
//!
//! Implementations are resolved by Routable definition name through
//! `CandidatePolicySet::effective_for_definition`, so a library entry changes
//! candidate generation and therefore the preparation fingerprint.

use eyre::WrapErr;
use serde::{Deserialize, Serialize};

use super::candidate::UnitCandidateConfig;
use super::rcir::{candidate_policy_from_spec, candidate_spec_from_policy};
use crate::ir::CandidateSpec;

pub const CELL_LIBRARY_FORMAT: &str = "redstone-compiler.cell-library.v1";

/// Conservative transforms a candidate may be reused under. Redstone is not
/// invariant under reflection or vertical rotation, so only translation and
/// (when a recipe proves block states equivalent) yaw are allowed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CellTransform {
    Translation,
    Yaw,
}

/// What a generated candidate publishes to global P&R. All fields have
/// conservative defaults so an implementation can be described incrementally.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CellPhysicalContract {
    /// Empty cells that must stay free around the candidate.
    pub halo: usize,
    pub requires_input_isolation: bool,
    pub requires_output_isolation: bool,
    pub allowed_transforms: Vec<CellTransform>,
    pub max_delay: Option<usize>,
}

impl Default for CellPhysicalContract {
    fn default() -> Self {
        Self {
            halo: 0,
            requires_input_isolation: false,
            requires_output_isolation: false,
            allowed_transforms: vec![CellTransform::Translation],
            max_delay: None,
        }
    }
}

/// A named physical implementation of one or more Routable definitions.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CellImplementation {
    pub name: String,
    /// Routable definition names this implementation applies to.
    pub definitions: Vec<String>,
    pub candidate: UnitCandidateConfig,
    pub contract: CellPhysicalContract,
    /// Higher priority wins when several implementations match a definition.
    pub priority: i32,
}

/// A reusable, target-scoped collection of cell implementations.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CellLibrary {
    pub format: String,
    pub name: String,
    pub target: String,
    pub implementations: Vec<CellImplementation>,
}

impl Default for CellLibrary {
    fn default() -> Self {
        Self::redstone_v1()
    }
}

impl CellLibrary {
    /// The built-in library. It starts empty because the current flow derives
    /// candidate policies from the design and compiler defaults; designs and
    /// targets can add named implementations on top.
    pub fn redstone_v1() -> Self {
        Self {
            format: CELL_LIBRARY_FORMAT.to_owned(),
            name: "redstone-v1-default".to_owned(),
            target: crate::ir::ROUTABLE_IR_TARGET.to_owned(),
            implementations: Vec::new(),
        }
    }

    /// Best matching implementation for a Routable definition name. Higher
    /// priority wins; ties break on the lexicographically smallest name so
    /// resolution is deterministic.
    pub fn implementation_for_definition(&self, definition: &str) -> Option<&CellImplementation> {
        self.implementations
            .iter()
            .filter(|implementation| {
                implementation
                    .definitions
                    .iter()
                    .any(|name| name == definition)
            })
            .max_by(|left, right| {
                left.priority
                    .cmp(&right.priority)
                    .then_with(|| right.name.cmp(&left.name))
            })
    }

    pub fn from_json(source: &str) -> eyre::Result<Self> {
        let document: CellLibraryDto =
            serde_json::from_str(source).context("parse cell library JSON")?;
        if document.format != CELL_LIBRARY_FORMAT {
            eyre::bail!(
                "unsupported cell library format `{}`, expected `{CELL_LIBRARY_FORMAT}`",
                document.format
            );
        }
        let implementations = document
            .implementations
            .into_iter()
            .map(|implementation| CellImplementation {
                name: implementation.name,
                definitions: implementation.definitions,
                candidate: candidate_policy_from_spec(&implementation.candidate),
                contract: implementation.contract,
                priority: implementation.priority,
            })
            .collect();
        Ok(Self {
            format: document.format,
            name: document.name,
            target: document.target,
            implementations,
        })
    }

    pub fn to_json(&self) -> eyre::Result<String> {
        let document = CellLibraryDto {
            format: self.format.clone(),
            name: self.name.clone(),
            target: self.target.clone(),
            implementations: self
                .implementations
                .iter()
                .map(|implementation| CellImplementationDto {
                    name: implementation.name.clone(),
                    definitions: implementation.definitions.clone(),
                    candidate: candidate_spec_from_policy(&implementation.candidate),
                    contract: implementation.contract.clone(),
                    priority: implementation.priority,
                })
                .collect(),
        };
        Ok(serde_json::to_string_pretty(&document)?)
    }
}

#[derive(Serialize, Deserialize)]
struct CellLibraryDto {
    format: String,
    name: String,
    target: String,
    implementations: Vec<CellImplementationDto>,
}

#[derive(Serialize, Deserialize)]
struct CellImplementationDto {
    name: String,
    definitions: Vec<String>,
    candidate: CandidateSpec,
    contract: CellPhysicalContract,
    priority: i32,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn implementation(name: &str, priority: i32) -> CellImplementation {
        CellImplementation {
            name: name.to_owned(),
            definitions: vec!["d_latch".to_owned()],
            candidate: UnitCandidateConfig {
                max_candidates: 2,
                ..Default::default()
            },
            contract: CellPhysicalContract::default(),
            priority,
        }
    }

    #[test]
    fn builtin_library_is_empty_and_target_scoped() {
        let library = CellLibrary::redstone_v1();

        assert_eq!(library.format, CELL_LIBRARY_FORMAT);
        assert_eq!(library.target, crate::ir::ROUTABLE_IR_TARGET);
        assert!(library.implementations.is_empty());
        assert!(library.implementation_for_definition("d_latch").is_none());
    }

    #[test]
    fn selection_prefers_priority_then_smallest_name() {
        let library = CellLibrary {
            implementations: vec![
                implementation("low", 0),
                implementation("beta", 5),
                implementation("alpha", 5),
            ],
            ..CellLibrary::redstone_v1()
        };

        assert_eq!(
            library
                .implementation_for_definition("d_latch")
                .expect("matching implementation")
                .name,
            "alpha"
        );
        assert!(library.implementation_for_definition("other").is_none());
    }

    #[test]
    fn library_round_trips_through_json() {
        let library = CellLibrary {
            implementations: vec![CellImplementation {
                name: "d_latch.compact".to_owned(),
                definitions: vec!["d_latch".to_owned(), "slave".to_owned()],
                candidate: UnitCandidateConfig {
                    max_candidates: 3,
                    combinational_sampling_limit: Some(4),
                    ..Default::default()
                },
                contract: CellPhysicalContract {
                    halo: 1,
                    requires_output_isolation: true,
                    allowed_transforms: vec![CellTransform::Translation, CellTransform::Yaw],
                    ..Default::default()
                },
                priority: 10,
            }],
            ..CellLibrary::redstone_v1()
        };

        let json = library.to_json().expect("serialize library");
        let restored = CellLibrary::from_json(&json).expect("deserialize library");

        assert_eq!(restored, library);
    }

    #[test]
    fn unsupported_format_is_rejected() {
        let error = CellLibrary::from_json(
            r#"{"format":"other","name":"x","target":"t","implementations":[]}"#,
        )
        .unwrap_err()
        .to_string();

        assert!(error.contains("unsupported cell library format"), "{error}");
    }
}
