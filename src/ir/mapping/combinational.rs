//! Scalar expansion of combinational logical cells.
//!
//! Every function here emits target primitives into a [`LeafBuilder`]. The
//! caller guarantees that all input bits are already produced; `emit_cell`
//! returns `Ok(false)` when an input is not ready yet so dependency-ordered
//! drivers can retry.

use eyre::ContextCompat;

use super::scalar::{LeafBuilder, ScalarNets};
use crate::ir::target::{MappingPolicy, TargetOp, TargetSpec, XorMapping};
use crate::ir::{LogicalCell, LogicalCellKind, LogicalValue};
use crate::logic::LogicType;

/// Emits one combinational cell into `builder`.
///
/// Returns `Ok(false)` when one of the input bits has no producer yet. Returns
/// `Ok(true)` once the cell output bits are registered in the builder.
pub(super) fn emit_cell(
    cell: &LogicalCell,
    nets: &ScalarNets,
    builder: &mut LeafBuilder,
    target: &TargetSpec,
    policy: &MappingPolicy,
) -> eyre::Result<bool> {
    let output = cell
        .outputs
        .first()
        .with_context(|| format!("logical cell `{}` has no output", cell.name))?;
    let out_bits = nets.bits(&output.net)?;

    match &cell.kind {
        LogicalCellKind::Buffer => {
            let Some(values) = input_nodes(cell, "value", out_bits.len(), nets, builder)? else {
                return Ok(false);
            };
            for (bit, node) in values.into_iter().enumerate() {
                builder.set_producer(&out_bits[bit], node);
            }
        }
        LogicalCellKind::Not => {
            target.require(TargetOp::Not)?;
            let Some(values) = input_nodes(cell, "value", out_bits.len(), nets, builder)? else {
                return Ok(false);
            };
            for (bit, value) in values.into_iter().enumerate() {
                let node = builder.add_logic(LogicType::Not, vec![value], &cell.name);
                builder.set_producer(&out_bits[bit], node);
            }
        }
        LogicalCellKind::And | LogicalCellKind::Or => {
            let (logic_type, op) = if matches!(cell.kind, LogicalCellKind::And) {
                (LogicType::And, TargetOp::And)
            } else {
                (LogicType::Or, TargetOp::Or)
            };
            target.require(op)?;
            let Some(lhs) = input_nodes(cell, "lhs", out_bits.len(), nets, builder)? else {
                return Ok(false);
            };
            let Some(rhs) = input_nodes(cell, "rhs", out_bits.len(), nets, builder)? else {
                return Ok(false);
            };
            for bit in 0..out_bits.len() {
                let node = builder.add_logic(logic_type, vec![lhs[bit], rhs[bit]], &cell.name);
                builder.set_producer(&out_bits[bit], node);
            }
        }
        LogicalCellKind::Xor => {
            let Some(lhs) = input_nodes(cell, "lhs", out_bits.len(), nets, builder)? else {
                return Ok(false);
            };
            let Some(rhs) = input_nodes(cell, "rhs", out_bits.len(), nets, builder)? else {
                return Ok(false);
            };
            for bit in 0..out_bits.len() {
                let node = emit_xor(builder, lhs[bit], rhs[bit], &cell.name, target, policy)?;
                builder.set_producer(&out_bits[bit], node);
            }
        }
        LogicalCellKind::Add => {
            let Some(lhs) = input_nodes(cell, "lhs", out_bits.len(), nets, builder)? else {
                return Ok(false);
            };
            let Some(rhs) = input_nodes(cell, "rhs", out_bits.len(), nets, builder)? else {
                return Ok(false);
            };
            let mut carry: Option<usize> = None;
            for bit in 0..out_bits.len() {
                let (a, b) = (lhs[bit], rhs[bit]);
                let xor = emit_xor(builder, a, b, &cell.name, target, policy)?;
                let sum = match carry {
                    Some(carry) => emit_xor(builder, xor, carry, &cell.name, target, policy)?,
                    None => xor,
                };
                target.require(TargetOp::And)?;
                target.require(TargetOp::Or)?;
                let next_carry = match carry {
                    Some(carry) => {
                        let product = builder.add_logic(LogicType::And, vec![a, b], &cell.name);
                        let propagate =
                            builder.add_logic(LogicType::And, vec![xor, carry], &cell.name);
                        builder.add_logic(LogicType::Or, vec![product, propagate], &cell.name)
                    }
                    None => builder.add_logic(LogicType::And, vec![a, b], &cell.name),
                };
                carry = Some(next_carry);
                builder.set_producer(&out_bits[bit], sum);
            }
        }
        LogicalCellKind::Inc => {
            let Some(values) = input_nodes(cell, "value", out_bits.len(), nets, builder)? else {
                return Ok(false);
            };
            let mut carry: Option<usize> = None;
            for bit in 0..out_bits.len() {
                let value = values[bit];
                if bit == 0 {
                    target.require(TargetOp::Not)?;
                    let node = builder.add_logic(LogicType::Not, vec![value], &cell.name);
                    builder.set_producer(&out_bits[bit], node);
                    carry = Some(value);
                } else {
                    let carry_in = carry.context("increment carry chain lost its seed")?;
                    let sum = emit_xor(builder, value, carry_in, &cell.name, target, policy)?;
                    target.require(TargetOp::And)?;
                    let next_carry =
                        builder.add_logic(LogicType::And, vec![value, carry_in], &cell.name);
                    builder.set_producer(&out_bits[bit], sum);
                    carry = Some(next_carry);
                }
            }
        }
        LogicalCellKind::Mux => {
            target.require(TargetOp::Not)?;
            target.require(TargetOp::And)?;
            target.require(TargetOp::Or)?;
            let Some(select) = input_nodes(cell, "select", 1, nets, builder)? else {
                return Ok(false);
            };
            if select.len() != 1 {
                eyre::bail!("mux `{}` select must be a single bit", cell.name);
            }
            let Some(when_true) = input_nodes(cell, "when_true", out_bits.len(), nets, builder)?
            else {
                return Ok(false);
            };
            let Some(when_false) = input_nodes(cell, "when_false", out_bits.len(), nets, builder)?
            else {
                return Ok(false);
            };
            let select = select[0];
            let not_select = builder.add_logic(LogicType::Not, vec![select], &cell.name);
            for bit in 0..out_bits.len() {
                let chosen =
                    builder.add_logic(LogicType::And, vec![when_true[bit], select], &cell.name);
                let rejected = builder.add_logic(
                    LogicType::And,
                    vec![when_false[bit], not_select],
                    &cell.name,
                );
                let node = builder.add_logic(LogicType::Or, vec![chosen, rejected], &cell.name);
                builder.set_producer(&out_bits[bit], node);
            }
        }
        LogicalCellKind::Eq { width } => {
            target.require(TargetOp::Not)?;
            target.require(TargetOp::And)?;
            let Some(lhs) = input_nodes(cell, "lhs", *width, nets, builder)? else {
                return Ok(false);
            };
            let Some(rhs) = input_nodes(cell, "rhs", *width, nets, builder)? else {
                return Ok(false);
            };
            if out_bits.len() != 1 {
                eyre::bail!("eq cell `{}` result must be one bit", cell.name);
            }
            let mut result: Option<usize> = None;
            for bit in 0..*width {
                let xor = emit_xor(builder, lhs[bit], rhs[bit], &cell.name, target, policy)?;
                let xnor = builder.add_logic(LogicType::Not, vec![xor], &cell.name);
                result = Some(match result {
                    None => xnor,
                    Some(previous) => {
                        builder.add_logic(LogicType::And, vec![previous, xnor], &cell.name)
                    }
                });
            }
            let result =
                result.with_context(|| format!("eq cell `{}` has zero width", cell.name))?;
            builder.set_producer(&out_bits[0], result);
        }
        LogicalCellKind::DLatch { .. }
        | LogicalCellKind::Dff { .. }
        | LogicalCellKind::Register { .. } => eyre::bail!(
            "sequential cell `{}` must be lowered by the state mapper",
            cell.name
        ),
    }

    Ok(true)
}

/// Resolves one input pin to scalar producer nodes.
///
/// Returns `Ok(None)` when a net bit has no producer yet, so the dependency
/// driver can retry. Constants are materialized directly and zero-extended to
/// `expected_width`, matching the logical width rules for constant operands.
fn input_nodes(
    cell: &LogicalCell,
    pin: &str,
    expected_width: usize,
    nets: &ScalarNets,
    builder: &mut LeafBuilder,
) -> eyre::Result<Option<Vec<usize>>> {
    match cell.input_value(pin)? {
        LogicalValue::Net { net } => {
            let mut nodes = Vec::new();
            for scalar in nets.bits(net)? {
                match builder.producer(&scalar) {
                    Some(node) => nodes.push(node),
                    None => return Ok(None),
                }
            }
            Ok(Some(nodes))
        }
        LogicalValue::Slice { net, bit } => {
            let scalar = nets.scalar(net, *bit)?;
            Ok(builder.producer(&scalar).map(|node| vec![node]))
        }
        LogicalValue::Constant { value, width } => {
            let mut nodes = Vec::new();
            for bit in 0..expected_width {
                let bit_value = bit < *width && (*value >> bit) & 1 == 1;
                nodes.push(builder.constant(bit_value));
            }
            Ok(Some(nodes))
        }
    }
}

/// Emits an xor of two scalar nodes according to the mapping policy.
pub(super) fn emit_xor(
    builder: &mut LeafBuilder,
    lhs: usize,
    rhs: usize,
    tag: &str,
    target: &TargetSpec,
    policy: &MappingPolicy,
) -> eyre::Result<usize> {
    match policy.xor {
        XorMapping::Direct => {
            target.require(TargetOp::Xor)?;
            Ok(builder.add_logic(LogicType::Xor, vec![lhs, rhs], tag))
        }
        XorMapping::AndOrNot => {
            target.require(TargetOp::Not)?;
            target.require(TargetOp::And)?;
            target.require(TargetOp::Or)?;
            let product = builder.add_logic(LogicType::And, vec![lhs, rhs], tag);
            let either = builder.add_logic(LogicType::Or, vec![lhs, rhs], tag);
            let not_product = builder.add_logic(LogicType::Not, vec![product], tag);
            Ok(builder.add_logic(LogicType::And, vec![either, not_product], tag))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ir::{LogicalDesign, LogicalInput, LogicalOutput};

    fn design(source: &str) -> LogicalDesign {
        LogicalDesign::from_verilog_source(source).expect("test source must parse")
    }

    #[test]
    fn add_emits_ripple_carry_without_extra_output_bits() {
        let logical = design(
            r#"
            module adder(a, b, y);
              input [1:0] a, b;
              output [1:0] y;
              assign y = a + b;
            endmodule
            "#,
        );
        let module = logical.module("adder").unwrap();
        let nets = ScalarNets::from_module(module);
        let mut builder = LeafBuilder::new();
        for port in module.ports.iter().filter(|port| port.name != "y") {
            for bit in nets.bits(&port.net).unwrap() {
                builder.add_input(&bit);
            }
        }
        let add = module
            .cells
            .iter()
            .find(|cell| matches!(cell.kind, LogicalCellKind::Add))
            .unwrap();

        assert!(emit_cell(
            add,
            &nets,
            &mut builder,
            &TargetSpec::redstone_v1(),
            &MappingPolicy::default()
        )
        .unwrap());

        for bit in nets.bits("y").unwrap() {
            let producer = builder
                .producer(&bit)
                .expect("adder output must be produced");
            builder.add_output(&bit, producer);
        }
        let leaf = builder.finish("adder").unwrap();
        let crate::ir::RoutableModuleBody::Leaf { nodes } = leaf.body else {
            panic!("adder must lower to a leaf");
        };
        // 4 inputs + 7 logic nodes (2-bit ripple carry) + 2 outputs.
        assert_eq!(nodes.len(), 13);
    }

    #[test]
    fn constants_are_materialized_instead_of_rejected() {
        let cell = LogicalCell {
            name: "add".to_owned(),
            kind: LogicalCellKind::Add,
            inputs: vec![
                LogicalInput {
                    pin: "lhs".to_owned(),
                    value: LogicalValue::Net {
                        net: "a".to_owned(),
                    },
                },
                LogicalInput {
                    pin: "rhs".to_owned(),
                    value: LogicalValue::Constant { value: 1, width: 1 },
                },
            ],
            outputs: vec![LogicalOutput {
                pin: "result".to_owned(),
                net: "y".to_owned(),
            }],
            origin: None,
        };
        let module = crate::ir::LogicalModule {
            name: "m".to_owned(),
            nets: vec![
                crate::ir::LogicalNet {
                    name: "a".to_owned(),
                    width: 1,
                    origin: None,
                },
                crate::ir::LogicalNet {
                    name: "y".to_owned(),
                    width: 1,
                    origin: None,
                },
            ],
            ports: vec![],
            cells: vec![],
            instances: vec![],
        };
        let nets = ScalarNets::from_module(&module);
        let mut builder = LeafBuilder::new();
        builder.add_input("a");

        assert!(emit_cell(
            &cell,
            &nets,
            &mut builder,
            &TargetSpec::redstone_v1(),
            &MappingPolicy::default(),
        )
        .unwrap());

        let producer = builder
            .producer("y")
            .expect("adder output must be produced");
        builder.add_output("y", producer);
        let leaf = builder.finish("m").unwrap();
        let crate::ir::RoutableModuleBody::Leaf { nodes } = leaf.body else {
            panic!("constant adder must lower to a leaf");
        };
        assert!(nodes.iter().any(|node| matches!(
            node.kind,
            crate::ir::RoutableNodeKind::Constant { value: true }
        )));
    }
}
