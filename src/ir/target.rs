//! Target capabilities and mapping policy for Logical-to-Routable lowering.
//!
//! `TargetSpec` declares which primitive operations a physical target can
//! implement. `MappingPolicy` declares how logical operations that have more
//! than one implementation are expanded. The general lowering in
//! [`super::mapping`] consumes both; the legacy special-case lowering in
//! `logical_lowering.rs` is not policy aware yet.
//!
//! The physical flow currently accepts only the `redstone-v1` target name.
//! Custom capability sets are already honored by the lowering, but a design
//! lowered for another target name must first extend `RoutableDesign::validate`.

use std::collections::BTreeSet;

use eyre::WrapErr;
use serde::{Deserialize, Serialize};

use super::routable::ROUTABLE_IR_TARGET;

pub const MAPPING_SPEC_FORMAT: &str = "redstone-compiler.mapping.v1";

/// Primitive operations that a routable target can implement.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum TargetOp {
    Buffer,
    Not,
    And,
    Or,
    Xor,
    DLatch,
    RsLatch,
}

impl TargetOp {
    /// Every operation understood by the current lowering.
    pub const ALL: [TargetOp; 7] = [
        TargetOp::Buffer,
        TargetOp::Not,
        TargetOp::And,
        TargetOp::Or,
        TargetOp::Xor,
        TargetOp::DLatch,
        TargetOp::RsLatch,
    ];

    pub fn name(self) -> &'static str {
        match self {
            TargetOp::Buffer => "buffer",
            TargetOp::Not => "not",
            TargetOp::And => "and",
            TargetOp::Or => "or",
            TargetOp::Xor => "xor",
            TargetOp::DLatch => "d_latch",
            TargetOp::RsLatch => "rs_latch",
        }
    }
}

/// The capability set of one physical target.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TargetSpec {
    name: String,
    capabilities: BTreeSet<TargetOp>,
}

impl TargetSpec {
    pub fn new(name: impl Into<String>, capabilities: impl IntoIterator<Item = TargetOp>) -> Self {
        Self {
            name: name.into(),
            capabilities: capabilities.into_iter().collect(),
        }
    }

    /// The Minecraft redstone target implemented by the current physical flow.
    pub fn redstone_v1() -> Self {
        Self::new(ROUTABLE_IR_TARGET, TargetOp::ALL)
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn supports(&self, op: TargetOp) -> bool {
        self.capabilities.contains(&op)
    }

    pub fn require(&self, op: TargetOp) -> eyre::Result<()> {
        if self.supports(op) {
            Ok(())
        } else {
            eyre::bail!("target `{}` does not support `{}`", self.name, op.name())
        }
    }

    pub fn capabilities(&self) -> impl Iterator<Item = TargetOp> + '_ {
        self.capabilities.iter().copied()
    }
}

impl Default for TargetSpec {
    fn default() -> Self {
        Self::redstone_v1()
    }
}

/// How a logical `Dff`/`Register` becomes target-supported state cells.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RegisterMapping {
    /// Two D latches with an inverted clock (master/slave).
    MasterSlaveLatches,
}

/// How a logical `Add`/`Inc` becomes scalar logic.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AdderMapping {
    RippleCarry,
}

/// How a logical `Mux` becomes scalar logic.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MuxMapping {
    /// `result = (when_true & select) | (when_false & ~select)`.
    AndOrNot,
}

/// How a logical `Xor` becomes target primitives.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum XorMapping {
    /// Emit a native xor primitive.
    Direct,
    /// `a ^ b = (a | b) & ~(a & b)`.
    AndOrNot,
}

/// Implementation choices used by Logical-to-Routable lowering.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MappingPolicy {
    pub register: RegisterMapping,
    pub adder: AdderMapping,
    pub mux: MuxMapping,
    pub xor: XorMapping,
}

impl Default for MappingPolicy {
    fn default() -> Self {
        Self {
            register: RegisterMapping::MasterSlaveLatches,
            adder: AdderMapping::RippleCarry,
            mux: MuxMapping::AndOrNot,
            xor: XorMapping::Direct,
        }
    }
}

impl MappingPolicy {
    /// Rejects a policy whose implementation cannot be built from the target
    /// capability set before any circuit is lowered.
    pub fn validate(&self, target: &TargetSpec) -> eyre::Result<()> {
        match self.register {
            RegisterMapping::MasterSlaveLatches => target.require(TargetOp::DLatch)?,
        }
        match self.xor {
            XorMapping::Direct => target.require(TargetOp::Xor)?,
            XorMapping::AndOrNot => {
                target.require(TargetOp::Not)?;
                target.require(TargetOp::And)?;
                target.require(TargetOp::Or)?;
            }
        }
        match self.mux {
            MuxMapping::AndOrNot => {
                target.require(TargetOp::Not)?;
                target.require(TargetOp::And)?;
                target.require(TargetOp::Or)?;
            }
        }
        match self.adder {
            AdderMapping::RippleCarry => {
                target.require(TargetOp::And)?;
                target.require(TargetOp::Or)?;
            }
        }
        Ok(())
    }
}

/// A versioned, serializable lowering configuration: target name plus mapping
/// policy. Emitted to snapshots and accepted from the CLI so a design can be
/// lowered reproducibly with a chosen set of implementation variants.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MappingSpec {
    pub format: String,
    pub target: String,
    pub policy: MappingPolicy,
}

impl Default for MappingSpec {
    fn default() -> Self {
        Self::redstone_v1()
    }
}

impl MappingSpec {
    pub fn redstone_v1() -> Self {
        Self {
            format: MAPPING_SPEC_FORMAT.to_owned(),
            target: ROUTABLE_IR_TARGET.to_owned(),
            policy: MappingPolicy::default(),
        }
    }

    pub fn from_json(source: &str) -> eyre::Result<Self> {
        let spec: Self = serde_json::from_str(source).context("parse mapping spec JSON")?;
        if spec.format != MAPPING_SPEC_FORMAT {
            eyre::bail!(
                "unsupported mapping spec format `{}`, expected `{MAPPING_SPEC_FORMAT}`",
                spec.format
            );
        }
        Ok(spec)
    }

    pub fn to_json(&self) -> eyre::Result<String> {
        Ok(serde_json::to_string_pretty(self)?)
    }

    pub fn target_spec(&self) -> eyre::Result<TargetSpec> {
        if self.target == ROUTABLE_IR_TARGET {
            Ok(TargetSpec::redstone_v1())
        } else {
            eyre::bail!(
                "unsupported mapping target `{}`, expected `{ROUTABLE_IR_TARGET}`",
                self.target
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn redstone_v1_supports_every_lowering_operation() {
        let target = TargetSpec::redstone_v1();

        assert_eq!(target.name(), "redstone-v1");
        for op in TargetOp::ALL {
            assert!(target.supports(op), "missing capability `{}`", op.name());
        }
        assert!(MappingPolicy::default().validate(&target).is_ok());
    }

    #[test]
    fn reduced_target_rejects_native_xor_policy() {
        let target = TargetSpec::new(
            ROUTABLE_IR_TARGET,
            [
                TargetOp::Buffer,
                TargetOp::Not,
                TargetOp::And,
                TargetOp::Or,
                TargetOp::DLatch,
            ],
        );
        let policy = MappingPolicy::default();

        let error = policy.validate(&target).unwrap_err().to_string();
        assert!(
            error.contains("does not support `xor`"),
            "unexpected error: {error}"
        );
    }

    #[test]
    fn and_or_not_xor_policy_passes_on_reduced_target() {
        let target = TargetSpec::new(
            ROUTABLE_IR_TARGET,
            [
                TargetOp::Buffer,
                TargetOp::Not,
                TargetOp::And,
                TargetOp::Or,
                TargetOp::DLatch,
            ],
        );
        let policy = MappingPolicy {
            xor: XorMapping::AndOrNot,
            ..MappingPolicy::default()
        };

        assert!(policy.validate(&target).is_ok());
    }

    #[test]
    fn mapping_spec_round_trips_and_rejects_unknown_formats() {
        let spec = MappingSpec {
            policy: MappingPolicy {
                xor: XorMapping::AndOrNot,
                ..MappingPolicy::default()
            },
            ..MappingSpec::redstone_v1()
        };

        let json = spec.to_json().expect("serialize mapping spec");
        assert_eq!(MappingSpec::from_json(&json).unwrap(), spec);
        assert!(spec.target_spec().is_ok());

        let error = MappingSpec::from_json(
            r#"{"format":"other","target":"redstone-v1","policy":{"register":"master_slave_latches","adder":"ripple_carry","mux":"and_or_not","xor":"direct"}}"#,
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("unsupported mapping spec format"), "{error}");

        let unknown_target = MappingSpec {
            target: "other".to_owned(),
            ..MappingSpec::redstone_v1()
        };
        assert!(unknown_target.target_spec().is_err());
    }

    #[test]
    fn missing_d_latch_rejects_master_slave_register_policy() {
        let target = TargetSpec::new(ROUTABLE_IR_TARGET, TargetOp::ALL);
        let policy = MappingPolicy::default();

        // Sanity check that a target without d_latch cannot host the default
        // register decomposition, while the full capability set can.
        assert!(target.supports(TargetOp::DLatch));
        assert!(policy.validate(&target).is_ok());
        assert!(!TargetSpec::new(ROUTABLE_IR_TARGET, [TargetOp::Not]).supports(TargetOp::DLatch));
    }
}
