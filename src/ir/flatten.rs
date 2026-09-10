//! Deterministic hierarchy flattening for Logical IR.
//!
//! The PnR flow accepts only one level of scalar leaf children. Designs with
//! nested hierarchy, mixed cells and instances, or vector ports are flattened
//! into a single instance-free module before lowering. Names are prefixed with
//! the instance path so the result is deterministic and collision free.

use std::collections::HashMap;

use eyre::ContextCompat;

use super::{LogicalCell, LogicalDesign, LogicalModule, LogicalNet, LogicalValue};

impl LogicalDesign {
    /// Returns a copy of this design with every module instantiation inlined
    /// into the top module. The top module keeps its name, ports, and cells;
    /// child cells are copied with an `{instance_path}__{name}` prefix and
    /// child port nets are bound to the parent nets.
    pub fn flatten_hierarchy(&self) -> eyre::Result<Self> {
        self.validate()?;
        let top = self
            .module(&self.top)
            .with_context(|| format!("unknown logical top module `{}`", self.top))?;

        let mut flat = LogicalModule {
            name: top.name.clone(),
            nets: top.nets.clone(),
            ports: top.ports.clone(),
            cells: top.cells.clone(),
            instances: Vec::new(),
        };
        for instance in &top.instances {
            let definition = self
                .module(&instance.module)
                .with_context(|| format!("unknown logical module `{}`", instance.module))?;
            let bindings = instance
                .bindings
                .iter()
                .map(|binding| (binding.port.clone(), binding.net.clone()))
                .collect::<HashMap<_, _>>();
            inline_module(
                self,
                definition,
                &mut flat,
                &instance.name,
                &instance.name,
                &bindings,
            )?;
        }

        let flattened = Self {
            version: self.version,
            top: self.top.clone(),
            modules: vec![flat],
            debug: Default::default(),
        };
        flattened.validate()?;
        Ok(flattened)
    }
}

/// Inlines one module definition into `target`.
///
/// `prefix` names internal nets and `instance_path` names copied cells;
/// `bindings` maps this module's local net names (including its ports) to the
/// already-renamed target nets.
fn inline_module(
    design: &LogicalDesign,
    module: &LogicalModule,
    target: &mut LogicalModule,
    prefix: &str,
    instance_path: &str,
    bindings: &HashMap<String, String>,
) -> eyre::Result<()> {
    for cell in &module.cells {
        let mut inputs = Vec::with_capacity(cell.inputs.len());
        for input in &cell.inputs {
            inputs.push(super::LogicalInput {
                pin: input.pin.clone(),
                value: rewrite_value(&input.value, module, target, prefix, bindings)?,
            });
        }
        let mut outputs = Vec::with_capacity(cell.outputs.len());
        for output in &cell.outputs {
            outputs.push(super::LogicalOutput {
                pin: output.pin.clone(),
                net: resolve_net(&output.net, module, target, prefix, bindings)?,
            });
        }
        target.cells.push(LogicalCell {
            name: format!("{instance_path}__{}", cell.name),
            kind: cell.kind.clone(),
            inputs,
            outputs,
            origin: cell.origin.clone(),
        });
    }

    for instance in &module.instances {
        let definition = design
            .module(&instance.module)
            .with_context(|| format!("unknown logical module `{}`", instance.module))?;
        let mut child_bindings = HashMap::new();
        for binding in &instance.bindings {
            let resolved = resolve_net(&binding.net, module, target, prefix, bindings)?;
            child_bindings.insert(binding.port.clone(), resolved);
        }
        inline_module(
            design,
            definition,
            target,
            &format!("{prefix}__{}", instance.name),
            &format!("{instance_path}__{}", instance.name),
            &child_bindings,
        )?;
    }

    Ok(())
}

/// Resolves a module-local net to its target name, declaring the renamed
/// internal net on first use.
fn resolve_net(
    local: &str,
    module: &LogicalModule,
    target: &mut LogicalModule,
    prefix: &str,
    bindings: &HashMap<String, String>,
) -> eyre::Result<String> {
    if let Some(bound) = bindings.get(local) {
        return Ok(bound.clone());
    }
    let name = format!("{prefix}__{local}");
    if !target.nets.iter().any(|net| net.name == name) {
        let width = module
            .nets
            .iter()
            .find(|net| net.name == local)
            .with_context(|| format!("unknown logical net `{local}` in module `{}`", module.name))?
            .width;
        target.nets.push(LogicalNet {
            name: name.clone(),
            width,
            origin: Some(format!("flattened.{prefix}.{local}")),
        });
    }
    Ok(name)
}

fn rewrite_value(
    value: &LogicalValue,
    module: &LogicalModule,
    target: &mut LogicalModule,
    prefix: &str,
    bindings: &HashMap<String, String>,
) -> eyre::Result<LogicalValue> {
    Ok(match value {
        LogicalValue::Net { net } => LogicalValue::Net {
            net: resolve_net(net, module, target, prefix, bindings)?,
        },
        LogicalValue::Slice { net, bit } => LogicalValue::Slice {
            net: resolve_net(net, module, target, prefix, bindings)?,
            bit: *bit,
        },
        LogicalValue::Constant { value, width } => LogicalValue::Constant {
            value: *value,
            width: *width,
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nested_hierarchy_flattens_deterministically() -> eyre::Result<()> {
        let design = LogicalDesign::from_verilog_source(
            r#"
            module inv(a, y);
              input a;
              output y;
              assign y = ~a;
            endmodule

            module pair(a, y);
              input a;
              output y;
              wire mid;
              inv u0(.a(a), .y(mid));
              inv u1(.a(mid), .y(y));
            endmodule

            module top(a, y);
              input a;
              output y;
              pair p(.a(a), .y(y));
            endmodule
            "#,
        )?;

        let flattened = design.flatten_hierarchy()?;
        let top = flattened.module("top").expect("flat top");
        assert!(top.instances.is_empty());
        assert!(top.cells.len() >= 2, "child cells must be inlined");
        assert!(top
            .cells
            .iter()
            .any(|cell| cell.name.starts_with("p__u0__")));
        assert!(top.nets.iter().any(|net| net.name == "p__mid"));

        assert_eq!(
            flattened.to_string(),
            design.flatten_hierarchy()?.to_string()
        );
        Ok(())
    }

    #[test]
    fn two_instances_of_the_same_child_get_distinct_nets() -> eyre::Result<()> {
        let design = LogicalDesign::from_verilog_source(
            r#"
            module inv(a, y);
              input a;
              output y;
              assign y = ~a;
            endmodule

            module top(a, b, y, unused);
              input a, b;
              output y, unused;
              inv u0(.a(a), .y(y));
              inv u1(.a(b), .y(unused));
            endmodule
            "#,
        )?;

        let flattened = design.flatten_hierarchy()?;
        let top = flattened.module("top").expect("flat top");
        let outputs = top
            .cells
            .iter()
            .filter(|cell| matches!(cell.kind, crate::ir::LogicalCellKind::Not))
            .map(|cell| cell.output("result").unwrap().clone())
            .collect::<Vec<_>>();

        assert_eq!(outputs.len(), 2);
        assert_ne!(outputs[0], outputs[1]);
        assert!(outputs.iter().any(|net| net == "y"));
        assert!(outputs.iter().any(|net| net == "unused"));
        Ok(())
    }
}
