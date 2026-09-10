use std::collections::{BTreeMap, BTreeSet};

use crate::verilog::rtl::{
    RtlAssignKind, RtlExpr, RtlModule, RtlProcess, RtlSensitivity, RtlSignalRef, RtlStmt,
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SynthNetlist {
    pub cells: Vec<SynthCell>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SynthEdge {
    Posedge,
    Negedge,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SynthCell {
    DLatch {
        output: RtlSignalRef,
        data: RtlSignalRef,
        enable: RtlSignalRef,
    },
    Dff {
        output: RtlSignalRef,
        data: RtlExpr,
        clock: RtlSignalRef,
        edge: SynthEdge,
    },
    Register {
        output: RtlSignalRef,
        data: RtlExpr,
        clock: RtlSignalRef,
        edge: SynthEdge,
    },
    /// A fully assigned combinational process signal: the signal is driven by
    /// logic instead of a state element.
    Combinational { output: RtlSignalRef, data: RtlExpr },
}

pub fn synthesize_module(module: &RtlModule) -> eyre::Result<SynthNetlist> {
    let mut cells = Vec::new();
    for process in &module.processes {
        cells.extend(synthesize_process(module, process)?);
    }
    Ok(SynthNetlist { cells })
}

fn synthesize_process(module: &RtlModule, process: &RtlProcess) -> eyre::Result<Vec<SynthCell>> {
    match process.sensitivity {
        RtlSensitivity::Combinational => synthesize_combinational_process(process),
        RtlSensitivity::Posedge(clock) => {
            synthesize_clocked_process(module, process, clock, SynthEdge::Posedge)
        }
        RtlSensitivity::Negedge(clock) => {
            synthesize_clocked_process(module, process, clock, SynthEdge::Negedge)
        }
    }
}

fn synthesize_combinational_process(process: &RtlProcess) -> eyre::Result<Vec<SynthCell>> {
    let next = process_next_values(&process.statements)?;
    if next.is_empty() {
        return Ok(Vec::new());
    }
    if next
        .iter()
        .all(|(signal, _)| fully_assigned(&process.statements, *signal))
    {
        return Ok(next
            .into_iter()
            .map(|(output, data)| SynthCell::Combinational { output, data })
            .collect());
    }

    synthesize_latch_process(process).map(|cell| vec![cell])
}

fn synthesize_latch_process(process: &RtlProcess) -> eyre::Result<SynthCell> {
    let [stmt] = process.statements.as_slice() else {
        eyre::bail!(
            "latch inference currently supports a single conditional assignment; \
             fully assign the signal with an else branch or a default case arm"
        );
    };
    let RtlStmt::If {
        condition,
        then_branch,
        else_branch,
    } = stmt
    else {
        eyre::bail!("unsupported combinational process shape");
    };
    if !else_branch.is_empty() {
        eyre::bail!("if/else procedural lowering is not implemented yet");
    }
    let [then_stmt] = then_branch.as_slice() else {
        eyre::bail!("expected a single if-then statement");
    };
    let RtlStmt::Assign { kind, lhs, rhs } = then_stmt else {
        eyre::bail!("expected assignment inside if branch");
    };
    if *kind != RtlAssignKind::NonBlocking {
        eyre::bail!("only nonblocking latch assignments are supported");
    }

    let RtlExpr::Signal(enable) = condition else {
        eyre::bail!("latch enable must be a signal");
    };
    let RtlExpr::Signal(data) = rhs else {
        eyre::bail!("latch data must be a signal");
    };

    Ok(SynthCell::DLatch {
        output: *lhs,
        data: *data,
        enable: *enable,
    })
}

fn synthesize_clocked_process(
    module: &RtlModule,
    process: &RtlProcess,
    clock: RtlSignalRef,
    edge: SynthEdge,
) -> eyre::Result<Vec<SynthCell>> {
    let next = process_next_values(&process.statements)?;
    let mut cells = Vec::with_capacity(next.len());
    for (output, data) in next {
        let width = module.signal_width(output)?;
        cells.push(if width == 1 {
            SynthCell::Dff {
                output,
                data,
                clock,
                edge,
            }
        } else {
            SynthCell::Register {
                output,
                data,
                clock,
                edge,
            }
        });
    }
    Ok(cells)
}

/// Converts a procedural statement list into one next-value expression per
/// assigned signal. Later assignments take priority, and branches merge through
/// `Mux`; a path that does not assign a signal falls back to the signal itself
/// (hold), which is exactly the enable/hold behavior of the old special case.
fn process_next_values(statements: &[RtlStmt]) -> eyre::Result<Vec<(RtlSignalRef, RtlExpr)>> {
    let mut current = BTreeMap::<RtlSignalRef, RtlExpr>::new();
    apply_statements(statements, &mut current)?;
    Ok(current.into_iter().collect())
}

fn apply_statements(
    statements: &[RtlStmt],
    current: &mut BTreeMap<RtlSignalRef, RtlExpr>,
) -> eyre::Result<()> {
    for statement in statements {
        match statement {
            RtlStmt::Assign { kind, lhs, rhs } => {
                ensure_nonblocking(*kind)?;
                current.insert(*lhs, rhs.clone());
            }
            RtlStmt::If {
                condition,
                then_branch,
                else_branch,
            } => {
                let mut then_values = current.clone();
                apply_statements(then_branch, &mut then_values)?;
                let mut else_values = current.clone();
                apply_statements(else_branch, &mut else_values)?;

                let mut changed = BTreeSet::new();
                for signal in then_values.keys().chain(else_values.keys()) {
                    let before = current.get(signal);
                    if then_values.get(signal) != before || else_values.get(signal) != before {
                        changed.insert(*signal);
                    }
                }
                for signal in changed {
                    let then_value = then_values
                        .get(&signal)
                        .cloned()
                        .unwrap_or(RtlExpr::Signal(signal));
                    let else_value = else_values
                        .get(&signal)
                        .cloned()
                        .unwrap_or(RtlExpr::Signal(signal));
                    current.insert(
                        signal,
                        RtlExpr::Mux {
                            select: Box::new(condition.clone()),
                            when_true: Box::new(then_value),
                            when_false: Box::new(else_value),
                        },
                    );
                }
            }
        }
    }
    Ok(())
}

/// True when every path through the statement list assigns `target`.
fn fully_assigned(statements: &[RtlStmt], target: RtlSignalRef) -> bool {
    statements.iter().any(|statement| match statement {
        RtlStmt::Assign { lhs, .. } => *lhs == target,
        RtlStmt::If {
            then_branch,
            else_branch,
            ..
        } => {
            !else_branch.is_empty()
                && fully_assigned(then_branch, target)
                && fully_assigned(else_branch, target)
        }
    })
}

fn ensure_nonblocking(kind: RtlAssignKind) -> eyre::Result<()> {
    if kind != RtlAssignKind::NonBlocking {
        eyre::bail!("only nonblocking sequential assignments are supported");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::verilog::parser::parse_module;
    use crate::verilog::rtl::lower_rtl_module;

    #[test]
    fn synthesizes_latch_and_dff_cells() -> eyre::Result<()> {
        let (rtl, netlist) = synthesize(d_latch_source())?;
        assert_eq!(
            netlist.cells,
            vec![SynthCell::DLatch {
                output: rtl.signal_ref("q")?,
                data: rtl.signal_ref("d")?,
                enable: rtl.signal_ref("en")?,
            }]
        );

        let (rtl, netlist) = synthesize(dff_source())?;
        assert_eq!(
            netlist.cells,
            vec![SynthCell::Dff {
                output: rtl.signal_ref("q")?,
                data: rtl.signal_expr("d")?,
                clock: rtl.signal_ref("clk")?,
                edge: SynthEdge::Posedge,
            }]
        );

        Ok(())
    }

    #[test]
    fn synthesizes_clocked_next_state_cells() -> eyre::Result<()> {
        let (rtl, netlist) = synthesize(enabled_dff_source())?;
        assert_eq!(
            netlist.cells,
            vec![SynthCell::Dff {
                output: rtl.signal_ref("q")?,
                data: RtlExpr::Mux {
                    select: Box::new(rtl.signal_expr("en")?),
                    when_true: Box::new(rtl.signal_expr("d")?),
                    when_false: Box::new(rtl.signal_expr("q")?),
                },
                clock: rtl.signal_ref("clk")?,
                edge: SynthEdge::Posedge,
            }]
        );

        let (rtl, netlist) = synthesize(toggle_source())?;
        assert_eq!(
            netlist.cells,
            vec![SynthCell::Dff {
                output: rtl.signal_ref("q")?,
                data: RtlExpr::Not(Box::new(rtl.signal_expr("q")?)),
                clock: rtl.signal_ref("clk")?,
                edge: SynthEdge::Posedge,
            }]
        );

        let (rtl, netlist) = synthesize(counter_source())?;
        assert_eq!(
            netlist.cells,
            vec![SynthCell::Register {
                output: rtl.signal_ref("q")?,
                data: RtlExpr::Add(
                    Box::new(rtl.signal_expr("q")?),
                    Box::new(RtlExpr::Const { value: 1, width: 1 }),
                ),
                clock: rtl.signal_ref("clk")?,
                edge: SynthEdge::Posedge,
            }]
        );

        Ok(())
    }

    #[test]
    fn synthesizes_if_else_and_negedge() -> eyre::Result<()> {
        let (rtl, netlist) = synthesize(if_else_source())?;
        assert_eq!(
            netlist.cells,
            vec![SynthCell::Dff {
                output: rtl.signal_ref("q")?,
                data: RtlExpr::Mux {
                    select: Box::new(rtl.signal_expr("en")?),
                    when_true: Box::new(rtl.signal_expr("d")?),
                    when_false: Box::new(rtl.signal_expr("e")?),
                },
                clock: rtl.signal_ref("clk")?,
                edge: SynthEdge::Posedge,
            }]
        );

        let (rtl, netlist) = synthesize(negedge_source())?;
        assert_eq!(
            netlist.cells,
            vec![SynthCell::Dff {
                output: rtl.signal_ref("q")?,
                data: rtl.signal_expr("d")?,
                clock: rtl.signal_ref("clk")?,
                edge: SynthEdge::Negedge,
            }]
        );

        Ok(())
    }

    #[test]
    fn synthesizes_multiple_assignments_in_one_process() -> eyre::Result<()> {
        let (rtl, netlist) = synthesize(two_registers_source())?;

        assert_eq!(
            netlist.cells,
            vec![
                SynthCell::Dff {
                    output: rtl.signal_ref("a")?,
                    data: rtl.signal_expr("x")?,
                    clock: rtl.signal_ref("clk")?,
                    edge: SynthEdge::Posedge,
                },
                SynthCell::Dff {
                    output: rtl.signal_ref("b")?,
                    data: rtl.signal_expr("y")?,
                    clock: rtl.signal_ref("clk")?,
                    edge: SynthEdge::Posedge,
                },
            ]
        );

        Ok(())
    }

    #[test]
    fn expands_case_into_priority_mux_chain() -> eyre::Result<()> {
        let (rtl, netlist) = synthesize(case_source())?;
        let [SynthCell::Register { output, data, .. }] = netlist.cells.as_slice() else {
            panic!("expected one register cell, got {:?}", netlist.cells);
        };
        assert_eq!(*output, rtl.signal_ref("state")?);
        assert!(matches!(data, RtlExpr::Mux { .. }));

        Ok(())
    }

    #[test]
    fn fully_assigned_combinational_process_becomes_logic() -> eyre::Result<()> {
        let (rtl, netlist) = synthesize(combinational_if_else_source())?;
        assert_eq!(
            netlist.cells,
            vec![SynthCell::Combinational {
                output: rtl.signal_ref("y")?,
                data: RtlExpr::Mux {
                    select: Box::new(rtl.signal_expr("s")?),
                    when_true: Box::new(rtl.signal_expr("a")?),
                    when_false: Box::new(rtl.signal_expr("b")?),
                },
            }]
        );

        Ok(())
    }

    fn synthesize(source: &str) -> eyre::Result<(RtlModule, SynthNetlist)> {
        let rtl = lower_rtl_module(&parse_module(source)?)?;
        let netlist = synthesize_module(&rtl)?;
        Ok((rtl, netlist))
    }

    fn d_latch_source() -> &'static str {
        r#"
        module d_latch(d, en, q);
          input d, en;
          output reg q;
          always @(*) begin
            if (en) begin
              q <= d;
            end
          end
        endmodule
        "#
    }

    fn dff_source() -> &'static str {
        r#"
        module dff(clk, d, q);
          input clk, d;
          output reg q;
          always @(posedge clk) begin
            q <= d;
          end
        endmodule
        "#
    }

    fn enabled_dff_source() -> &'static str {
        r#"
        module dff_en(clk, en, d, q);
          input clk, en, d;
          output reg q;
          always @(posedge clk) begin
            if (en) begin
              q <= d;
            end
          end
        endmodule
        "#
    }

    fn toggle_source() -> &'static str {
        r#"
        module toggle(clk, q);
          input clk;
          output reg q;
          always @(posedge clk) begin
            q <= ~q;
          end
        endmodule
        "#
    }

    fn counter_source() -> &'static str {
        r#"
        module counter(clk, q);
          input clk;
          output reg [3:0] q;
          always @(posedge clk) begin
            q <= q + 1;
          end
        endmodule
        "#
    }

    fn if_else_source() -> &'static str {
        r#"
        module mux_ff(clk, en, d, e, q);
          input clk, en, d, e;
          output reg q;
          always @(posedge clk) begin
            if (en) begin
              q <= d;
            end else begin
              q <= e;
            end
          end
        endmodule
        "#
    }

    fn negedge_source() -> &'static str {
        r#"
        module neg_ff(clk, d, q);
          input clk, d;
          output reg q;
          always @(negedge clk) begin
            q <= d;
          end
        endmodule
        "#
    }

    fn two_registers_source() -> &'static str {
        r#"
        module pair(clk, x, y, a, b);
          input clk, x, y;
          output reg a, b;
          always @(posedge clk) begin
            a <= x;
            b <= y;
          end
        endmodule
        "#
    }

    fn case_source() -> &'static str {
        r#"
        module fsm(clk, state);
          input clk;
          output reg [1:0] state;
          always @(posedge clk) begin
            case (state)
              0: state <= 1;
              1, 2: state <= 3;
              default: state <= 0;
            endcase
          end
        endmodule
        "#
    }

    fn combinational_if_else_source() -> &'static str {
        r#"
        module choose(s, a, b, y);
          input s, a, b;
          output reg y;
          always @(*) begin
            if (s) begin
              y <= a;
            end else begin
              y <= b;
            end
          end
        endmodule
        "#
    }
}
