# Redstone Compiler 架构分析报告

> Phase 0 交付物：仓库代码考察与架构分析。本文只描述现状与证据，不包含任何代码修改。

| 项目 | 值 |
| --- | --- |
| 仓库 | https://github.com/Redstone-Compiler/redstone-compiler |
| 本地路径 | `H:\Tools\Hardware\MCHDL\redstone-compiler` |
| 分析提交 | `cc997732b82d957a8b5cc80d14c07b375562dd9d`（master，2026-07-19） |
| 分析日期 | 2026-09-10 |
| 代码规模 | `src/` 约 30,000 行 Rust（12 个顶层模块，80+ 文件）；另有 `crates/nbt-sim-wasm` 与 `tools/nbt-viewer` |
| 工具链 | `rust-toolchain.toml` 要求 `nightly-2025-02-08`；已在本机安装 |
| 验证状态 | `cargo check --all-targets` 通过；`cargo test --release` 跳过 8 个资源密集型测试后 **253 passed / 0 failed / 4 ignored** |

---

## 0. 执行摘要（关键结论）

1. **该项目不是玩具原型，而是一个已分层的 EDA 编译器骨架**。它已经独立实现了：
   自研 Verilog 子集 frontend → Logical IR（RCIR）→ Routable IR（RCIR）→ 本地单元候选生成（LocalPlacer）→ 全局布局（8 种启发式）→ 全局布线（4 种策略，含 rip-up/refine）→ World3D → 结构 NBT 输出，以及可重放快照（`.snapshot/` + `.rsnap`）。
2. **项目已经自研了 Verilog frontend，而不是 Yosys 路线**。`src/verilog/` 包含 lexer、parser、AST、RTL 过程模型、综合（synth）与 lowering。原提示词中"不要自己实现 Verilog parser，使用 Yosys"的前提与本仓库现状不符，Phase 1 路线需要重新评估（见 §5）。
3. **真正的瓶颈不在 frontend，而在 Logical → Routable 的通用映射与 technology mapping**。目前该层是一个围绕 counter/DFF/D-latch 的**特例引擎**（硬编码模式匹配），没有 `TargetSpec` / `MappingPolicy` / 标准单元库。Yosys 只能解决 frontend 一小段，不会自动解决这个核心问题。
4. **最值得保留的资产**（不要浪费）：
   - RCIR 两阶段 IR 契约与确定性文本格式（`docs/intermediate_representation_design.md`）；
   - 可重放快照边界（local candidate preparation 与 global PnR 分离，`.rsnap` 可跳过昂贵的本地搜索）；
   - World3D / NBT backend 与离散事件红石模拟器（`src/world/simulator.rs`，WASM 版供浏览器使用）；
   - 全局布局/布线与物理意图（floorplan intent）框架；
   - nbt-viewer（3D 渲染 + 波形 + Verilog/RCIR 联动高亮）。
5. **端到端已验证的样例**：组合半加器（`test/half-adder.v`）、2-bit 计数器（`test/counter.rsnap`，8 实例/14 路由）、结构化 D 触发器（`test/d-flip-flop.rsnap`，3 实例/7 路由）。`test/alu.v` 是**孤儿文件**（名字拼错、无 `assign`、无引用），并非真实 ALU 编译样例。

---

## 1. 总体 Pipeline

### 1.1 README 声明的栈

```text
HDL -> Synthesis -> Cluster -> Logic Graph -> Place and Route -> Synthesis -> World -> NBT
```

### 1.2 代码中实际存在的路径

主路径（可编译、可端到端验证）：

```text
Verilog source (.v)
  │  src/verilog/lexer.rs → parser.rs → ast.rs
  ▼
RTL process model                    src/verilog/rtl.rs        (always/if/posedge、nonblocking)
  │  src/verilog/synth.rs
  ▼
SynthCell (DLatch / Dff / Register)  src/verilog/synth.rs
  │  src/ir/logical_adapter.rs
  ▼
LogicalDesign  [bus-aware, target-independent]        src/ir/logical.rs      → ir/logical.rcir
  │  src/ir/logical_lowering.rs（硬编码特例映射 + 图回退）
  ▼
RoutableDesign [scalar, structural, target="redstone-v1"]  src/ir/routable.rs → ir/routable.rcir
  │  src/transform/place_and_route/global_pnr/topology.rs
  ▼
ResolvedPnrTopology
  │  global_pnr/candidate.rs（LocalPlacer 生成 leaf 候选，可缓存）
  ▼
PreparedPnrDesign                     [preparation 边界，写入 candidates/*、pnr/*]
  │  global_pnr/placer.rs（全局布局） + global_pnr/router.rs（全局布线）
  ▼
assemble_world → PlacedWorld          src/transform/place_and_route/global_pnr/assembly.rs
  │  World3D（可运行世界） + verifier hook（模拟器行为验证）
  ▼
NBT（结构方块格式）+ interface.json + snapshot 快照
```

另一条**遗留路径**（仅组合逻辑，勿用于新功能）：

```text
Verilog → parser → lower.rs → LogicGraph（prepare_place 后交 LocalPlacer）
```

该路径会**静默忽略 always 块**，且表达式构建器遇到 `+`/数字会 `panic!/unimplemented!`（`src/graph/logic.rs:391-423,517`）。只有 `LogicalDesign` 路径支持时序与算术。

### 1.3 CLI 入口

`src/main.rs` 按扩展名分派（`main.rs:40-45`）：

| 输入 | 行为 |
| --- | --- |
| `*.v` | Verilog → LogicalDesign → `lower_to_routable()` → P&R（`main.rs:94-142`） |
| `*.rcir` | 解析 Logical 或 Routable RCIR → 对应流程（`main.rs:144-197`） |
| `*.rsnap` / `*.snapshot` | 加载 prepared snapshot，**跳过本地候选生成**，只重跑全局 P&R（`main.rs:48-92`） |

输出参数可选：给定输出路径时写 `<output>.snapshot/` 目录与 `<output>.rsnap` 归档；不给出时只打印 IR 摘要。

### 1.4 关键入口函数（file:line）

| 阶段 | 函数 | 位置 |
| --- | --- | --- |
| Verilog → Logical | `LogicalDesign::from_verilog_source_named` | `src/ir/logical_adapter.rs:23` |
| Logical → Routable | `LogicalDesign::lower_to_routable` | `src/ir/logical_adapter.rs:47` |
| 完整逻辑 PnR | `place_and_route_logical_design_with_visualization` | `global_pnr/mod.rs:629` |
| 完整可布线 PnR | `place_and_route_routable_design_with_visualization` | `global_pnr/mod.rs:592` |
| Preparation 边界 | `prepare_routable_design_for_global_pnr` | `global_pnr/mod.rs:606` |
| 运行 prepared PnR | `run_prepared_pnr_with_visualization` | `global_pnr/mod.rs:729` |
| 布线入口 | `route_resolved_topology_with_order_from_prefix` | `global_pnr/router.rs:288` |
| 快照写/读 | `emit_prepared_pnr_snapshot` / `load_prepared_pnr_snapshot` | `global_pnr/mod.rs:805` / `prepared_snapshot.rs:165` |
| 快照会话 | `compile_with_snapshot` | `src/snapshot.rs:131` |

---

## 2. 核心数据结构

### 2.1 "Cell / Net / Gate / Node / Edge / World / Block" 对照

| 概念 | 是否存在 | 实际类型与位置 | 说明 |
| --- | --- | --- | --- |
| Cell（逻辑） | ✅ | `LogicalCell { name, kind, inputs, outputs }` `src/ir/logical.rs:49` | 算子级单元；`LogicalCellKind` 见 §2.2 |
| Cell（可布线） | ❌ | 不存在 `RoutableCell` | Routable 叶子用 `RoutableNode`；层次用 `RoutableInstance` |
| Cell（标准单元库） | ❌ | 不存在 | 没有 `*.rcell`、没有 `TargetSpec`/capability 注册表 |
| Net | ✅ | `LogicalNet` `logical.rs:27`；`RoutableNet` `routable.rs:60`（driver + sinks + class） | Routable 层显式 driver/sink 端点 |
| Gate | ❌（死代码） | `Gate`/`GateKind` `src/world/gate.rs:6-36` | 全仓库无引用，属遗留死代码 |
| Node | ✅ | `GraphNode` `src/graph/mod.rs:114`；`RoutableNode` `routable.rs:87` | 图模型节点 / 可布线叶子节点 |
| Edge | ❌（无独立类型） | 边隐含在 `GraphNode.inputs/outputs`、`RoutableNode.inputs`、`RoutableNet.driver/sinks` | 没有 `EdgeId`/`GraphEdge` |
| World | ✅ | `World`（稀疏）`src/world/mod.rs:15`；`World3D`（稠密 `Vec<Vec<Vec<Block>>>`）`:30` | `Position(x,y,z)`，z 向上 |
| Block | ✅ | `Block { kind, direction }` `src/world/block.rs:220`；`BlockKind` `:77` | 见 §2.5 |

### 2.2 逻辑层（Logical IR，目标无关、总线感知）

```rust
LogicalDesign { version, top, modules, debug }          // src/ir/logical.rs:9
LogicalModule { name, nets, ports, cells, instances }   // :18
LogicalCellKind:                                        // :79
    Buffer | Not | And | Or | Xor | Add | Inc | Mux
  | DLatch { width }
  | Dff { edge }            // edge = Posedge | Negedge
  | Register { width, edge }
LogicalValue = Net { net } | Constant { value: u128, width }   // :72
LogicalInstance { name, module, bindings }              // :101
```

要点：

- 引脚模式由 `LogicalCellKind::pin_names` 定义（`logical.rs:464`）：`and/or/xor/add` 为 `lhs,rhs→result`；`mux` 为 `select,when_true,when_false→result`；`register` 为 `d,clock→q`；`d_latch` 为 `d,enable→q`。
- 校验规则（`LogicalDesign::validate`，`logical.rs:120`）：每 net 单一 driver；引脚集合必须精确匹配；宽度规则；层次无递归；**组合环检测（时序单元除外）**。
- 层次保留为 `instances`，但当前 lowering 只支持浅层次。

### 2.3 可布线层（Routable IR，标量、结构化、已选 target）

```rust
RoutableDesign { version, target, top, modules, debug } // src/ir/routable.rs:10
RoutableModuleBody = Leaf { nodes } | Composite { instances, nets }  // :28
RoutableNodeKind:                                        // :97
    Input{name} | Output{name} | Not | And | Or | Xor
  | Sequential { primitive: RsLatch|DLatch, input_ports, output_ports }
RoutableNet { name, class: Data|Clock|Reset|Io, driver, sinks }  // :60
Endpoint = SelfPort{port} | InstancePort{instance,port}          // :81
```

要点：

- **没有算术、没有总线、没有 reset/enable、没有 negedge**；DFF 只以 master/slave D-latch 分解形式存在。
- `target` 只是字符串，唯一语义是必须等于 `"redstone-v1"`（`routable.rs:7,128`）。文档中提出的 `std.not/std.and/.../redstone.dff` capability 集与 `TargetSpec`/`MappingPolicy` API **尚未实现**。
- 校验（`routable.rs:127`）：叶子端口与图 Input/Output 节点一一对应；复合层 driver/sink 方向与唯一驱动；无未连接端口。

### 2.4 图模型（本地放置的输入）

```rust
GraphNodeId = usize;                                     // src/graph/mod.rs:21（无 EdgeId）
GraphNodeKind = None | Input(String) | Block(Block)
              | Logic(Logic) | Sequential(SequentialPrimitive)
              | Output(String) | Clustered(Clustered)     // :23-33
Graph { nodes, producers, consumers }                    // :330
LogicGraph = Deref<Graph> + 逻辑构建/真值表/prepare_place // src/graph/logic.rs:10
WorldGraph { graph, positions, routings }                // src/graph/world.rs:14
```

- `Graph::topological_order` 内部 `toposort(...).unwrap()`（`graph/mod.rs:395`）→ **有环图会 panic**；时序环通过 `Sequential` 不透明节点隔离。
- `SequentialPrimitive { sequential_type, input_ports, output_ports, inner_graph }`（`src/sequential/mod.rs:22`）持有门级 `inner_graph`（RS/D latch 的反馈方程），供识别与放置使用。

### 2.5 物理层

| 类型 | 位置 | 说明 |
| --- | --- | --- |
| `World { size, blocks: Vec<(Position, Block)> }` | `world/mod.rs:15` | 稀疏表示 |
| `World3D { size, map[z][y][x] }` | `world/mod.rs:30` | 稠密表示；NBT 与 P&R 的实际工作对象 |
| `Position(x, y, z)` / `DimSize` | `world/position.rs:5,179` | z 向上；East=x+，North=y+ |
| `BlockKind` | `world/block.rs:77` | `Air, Cobble{on_count,on_base_count}, Switch{is_on}, Redstone{on_count,state,strength}, Torch{is_on}, Repeater{is_on,is_locked,delay,...}, RedstoneBlock, Piston{...}` |
| `PlaceBound(PropagateType, Position, Direction)` | `place_and_route/place_bound.rs:17` | `PropagateType = Soft|Hard|Torch|Repeater`；红石传播语义核心 |
| `PlacedNode` | `placed_node.rs:9` | 冲突/短路检测（cobble/redstone/repeater） |
| `LayoutCandidate { world, bbox, ports, occupied_cells, blocked_cells, cost }` | `global_pnr/ir.rs:66` | 本地单元候选；cost 目前仅 `block_count + bbox_volume` |
| `PhysicalPort { position, route_position, access_points, connection }` | `global_pnr/ir.rs:26` | `connection = Direct|InputDiode|OutputDiode` |
| `RoutedNet { net_id, source/sink_endpoint, source/sink, blocks, path, ... }` | `global_pnr/router.rs:188` | 带类型化 `NetId`/端点的物理路由 |
| `PlacedWorld { world, inputs, outputs }` | `src/output.rs:14` | 最终产物容器 |

---

## 3. 当前支持能力

### 3.1 已支持

**Verilog frontend（自研子集）**

- `module ... endmodule`（简单标识符端口表，多模块，top = 最后一个模块）
- `input/output [msb:lsb]`、`output reg [..]`、`wire [..]`
- 连续赋值 `assign lhs = expr;`（LHS 仅普通标识符）
- 表达式：`~ & ^ | +`、括号、十进制常量、位选 `a[0]`（展平为 `a_0`）
- `always @(*)`（单语句 `if(sig) q <= d` → D latch）
- `always @(posedge clk)`（单条 nonblocking `q <= expr` → DFF/Register；`if(en)` → 反馈 Mux 数据）
- 具名端口实例化 `Mod inst(.p(sig));`（仅具名，且当前仅一层）

**IR 与格式**

- Logical / Routable 两阶段 RCIR 文本格式，确定性 writer + round-trip 测试（`ir/logical_text.rs`、`ir/text.rs`）
- 独立校验器（驱动唯一性、引脚模式、宽度、层次、组合环）
- `ir/source-map.json` 源映射（Verilog ↔ logical.rcir ↔ routable.rcir），供 viewer 联动高亮

**物理设计**

- 本地放置：`LocalPlacer` 队列搜索 + 采样（`StepPolicy/Cost/Ranked`），支持 `Not/Or` 及 RS latch / D latch（≤ 40 节点）
- 全局布局启发式（`global_pnr/policy.rs:28`）：`Shelf, Grid, Layered3D, Free3D` + 5 种寄存器专用启发式
- 全局布线策略（`global_pnr/router.rs:109`）：`BreadthFirst, AStar, DirectGreedy, GreedyBeam`；校验模式 `Incremental/Deferred`；多轮 refinement（prefix + variant seed 重试）
- 候选缓存：内存 child cache + 可选磁盘 candidate cache（内容哈希校验）
- 物理意图（`global_pnr/physical_intent.rs`）：`region/require/lock/priority/avoid/prefer`，可嵌入 `routable.rcir`
- 可重放快照：preparation 与 global PnR 分离，`.rsnap` zip 归档

**验证与输出**

- 红石模拟器（`src/world/simulator.rs`）：离散事件模型，支持 lever、redstone wire（强度衰减）、torch（含 burnout 近似）、repeater（延迟/锁定）、cobble 电源计数
- PnR verifier hook（`GlobalPnrConfig.verifier`）在布局布线后用模拟器验证行为（counter/DFF 已用）
- `world_to_logic_with_outputs` 反向提取，用于等价性检查
- NBT 输出（结构方块格式，gzip）；`interface.json` 端口元数据；`candidates/`、`instances/`、`routes/`、`pnr/`、`intent/` 等快照产物
- 工具链：`tools/nbt-viewer`（WebGL 3D、波形、Graphviz、IR 对照、WASM 模拟）、`crates/nbt-sim-wasm`、`schem2nbt.py`（WorldEdit `.schem` → 结构 NBT）

**端到端样例**

| 样例 | 类型 | 结果 |
| --- | --- | --- |
| `test/half-adder.v` | 组合 | 生成 NBT 并按真值表验证 |
| `test/counter.snapshot/counter.v` | 时序（2-bit 递增寄存器） | `counter.rsnap` 成功，8 实例/14 路由 |
| `test/d-flip-flop.snapshot/d-flip-flop.v` | 结构层次（2×D latch + 反相器） | `d-flip-flop.rsnap` 成功，3 实例/7 路由 |

### 3.2 尚未支持

**Verilog 语言层**

- `negedge`、`else`、`case/endcase`、`parameter/localparam`、ANSI 端口声明
- 每个 `begin/end` 只能一条语句；`always` 只能一条语句
- 阻塞赋值 `=`（过程内）、`initial`、`generate/for/while`、`function/task`
- 运算符：`- * / % == != < > ?: {} << >> && ||`（lexer 直接拒绝）
- sized/radix 常量（`4'b1010`、`'hFF`）、`/* */` 块注释
- part-select `a[3:0]`、拼接、LHS 切片、数组
- 异步 reset、多时钟、enable 的通用建模

**IR / 映射层**

- 通用 `Add/Inc/Mux` 的组合映射（非 state next 场景一律报错）
- Logical 常量进入标量叶子
- 一个模块多个时序单元、cells 与 instances 混用、嵌套层次（>1 层）
- `LoweringMap`（文档承诺的 many-to-many 映射对象）未实现
- `TargetSpec` / `MappingPolicy` / 标准单元 capability 集未实现
- Routable 层无 reset/enable/negedge/总线

**P&R / 物理层**

- 全局 PnR 只支持 top 的 leaf children（`topology.rs:202`）
- LocalPlacer 硬上限 40 节点，只支持 `Not/Or` 逻辑与 RS/D latch；**只允许一个暴露的时序输出**（边不携带端口信息）
- RS latch 识别依赖字面名 `q`/`nq` 与 NOR 交叉耦合结构（`sequential/core.rs:32`）
- 仅 1 个硬编码 RS latch macro；`Piston` 全链路 `todo!()`
- full adder 本地放置是已知脆弱点（依赖昂贵搜索与真值表过滤）

**输出 / 工具**

- NBT 无 `DataVersion`；未知 palette 直接 panic；导入丢弃 repeater `powered/locked` 与 wire 强度；NBT→World→NBT 往返尺寸 +1
- 没有 `redstone build cpu.v` 式的完整 CLI UX；没有 `.schem` 输出（只有反向的 `schem2nbt.py`）
- 没有 RTL/IR 级模拟器（模拟器只针对最终物理 World）

---

## 4. 当前不足（为什么它还不是完整 Verilog Compiler）

按提示词要求的四个重点逐条回答：

### 4.1 HDL frontend 是否完整？——否，是一个窄子集

`src/verilog/` 只实现 4 种行为形状：`assign` 组合树、`always @(*) if(en) q<=d`（D latch）、`always @(posedge clk) q<=expr`（DFF/Register）、以及 `if(en)` 包一层 next-state。没有 `else/case/negedge/parameter/多语句/算术`。parser 中 `TODO` 明确承认单语句限制（`parser.rs:141`）。此外 lexer 只支持十进制数、只支持 `//` 注释。

### 4.2 是否依赖硬编码逻辑？——是，而且是当前最核心的结构性问题

`src/ir/logical_lowering.rs` 不是通用 mapper，而是特例引擎：

- `lower_state_design` 要求**整个模块恰好一个时序单元**，且 `Dff/Register` 的数据驱动只允许 `Inc`/`Not`/`Buffer` 等少数模式；多比特寄存器只支持 `q <= q + 1`（ripple carry 专门代码 `register_increment_design`，`logical_lowering.rs:478`）。
- enabled DFF 因数据驱动是 `Mux` 而直接失败（`logical_lowering.rs:381`）。
- 标量图回退路径要求所有 net 为标量、无实例、无常量，且 `Add/Inc/Mux/Dff/Register` 一律报 "requires target mapping"（`:700-709`）。
- 寄存器分解为 master/slave D latch 的过程是手写模块构造（`scalar_dff_design`、`d_latch_routable_module`），而不是从 cell library 选择。
- 物理侧同样硬编码：`RegisterCarryChain` 等启发式靠模块命名 `<bit>_{clk_inv,next,master,slave}` 识别（`global_pnr/placer.rs:1125`），RS latch 靠 `q/nq` 命名识别，宏只有一个。

### 4.3 是否缺少 technology mapping？——是，完全没有实现

- `target` 是字符串常量检查，没有 capability 注册表；文档中的 `std.*` 操作集与 `MappingPolicy` 只是设计提案。
- 没有 `TargetRules`、没有 "一个 logical 操作 → 多个目标实现变体" 的选择机制。
- 物理实现变体（`xor.nor_network` / `xor.buffered` / …）只存在于 `docs/physical_design_intent.md` 的示例中。

### 4.4 是否缺少标准 cell library？——是

- 没有 `*.rcell` 文件、没有 `CellLayoutLibrary` 类型；单元布局由 `LocalPlacer` 现场搜索生成，代价高且不可声明复用（磁盘 candidate cache 是缓存而非库）。
- 没有 cell 的物理契约（端口访问、blockage、halo、合法变换）与 Pareto 候选保留（`LayoutCandidateCost` 只有块数 + bbox 体积）。

### 4.5 是否缺少 CLI？——有最小 CLI，但缺少完整 build UX

`src/main.rs` 可编译 `.v/.rcir/.rsnap`，但：没有子命令（`build/verify/simulate`）、没有 schematic 导出、没有多文件/参数覆盖、没有清晰的成功/失败报告格式。文档承诺的 `redstone build cpu.v → cpu.schem` 尚不存在。

### 4.6 是否缺少 simulator？——物理级有，RTL/IR 级没有

`src/world/simulator.rs` 是**物理级**离散事件模拟器（约 3000 行），用于验证生成的 NBT 世界；`crates/nbt-sim-wasm` 将其带到浏览器。但不存在对 Logical/Routable IR 的行为模拟器，因此"在生成 Minecraft 之前验证逻辑正确"目前只能依赖：(a) 真值表/结构等价检查（组合），(b) 生成后的物理模拟（昂贵，counter 编译约 84 秒）。

### 4.7 其他重要缺口

- **层次**：只支持一层（top 实例化 leaf children）；更深层次直接报错。
- **快照引导缺陷**：`src/ir/debug.rs:548,576,614` 用 `include_str!` 引用 `test/counter.snapshot/counter.v`、`test/d-flip-flop.snapshot/d-flip-flop.v`，而 `*.snapshot/` 被 gitignore，文件由 `#[ignore]` 测试生成 → **全新 clone 无法编译 lib test 目标**（本次已按测试内嵌源码重建这两个生成物，未改代码）。
- **测试资源**：8 个 `test_generate_component_*` 搜索密集测试在本机完整并行运行时内存耗尽（`0xc0000409`）；跳过它们后其余 253 个测试 4.85 秒全绿。
- **潜在 bug/债务**：`graph/mod.rs:1287` 对 `GraphNodeId` 调用不存在的 `.id()`（仅测试调用）；`WorldGraph::verify` 为 `todo!()`；`Graph::remove_input` 查找 Output 节点（疑似笔误）；`world/gate.rs` 死代码；`cluster` 模块生产代码中基本未用。

---

## 5. 对 Phase 1 改造路线的影响（Yosys 决策）

原提示词预设 "不要自己实现 Verilog parser，使用 Yosys 作为 frontend"。基于代码考察，实际情况是：

```text
用户预设:   Verilog/SV → Yosys → RTLIL/JSON → Redstone IR → ...
实际现状:   Verilog/SV → [自研 frontend] → LogicalDesign → RoutableDesign → P&R → World → NBT
```

因此 Phase 1 有三条可选路线：

### 路线 A：增强现有自研 frontend（保持单栈）

- 优点：不引入外部二进制依赖；与现有 IR 校验、source-map、快照、测试完全一致；作者契约（`docs/verilog_rtl_interface_design.md` "Extension rules"）就是按这个方向写的。
- 缺点：要补齐 `else/case/negedge/parameter/算术/位宽语义` 等工作量巨大，且长期难达到工业级 Verilog/SV 覆盖率。

### 路线 B：Yosys 作为**额外** frontend，桥接到 LogicalDesign

```text
Verilog/SV → Yosys (read_verilog/synth) → JSON netlist → [新 bridge] → LogicalDesign
                                                                     → 现有 lowering/P&R/NBT 不变
```

- 优点：快速获得完整 parsing/elaboration/synthesis/optimization；`$and/$or/$not/$xor/$mux/$dff/$dffe/$adff` 等 cell 可映射到现有 `LogicalCellKind`（`And/Or/Not/Xor/Mux/Dff/Register`）；符合作者"Verilog AST → LogicalDesign 之后不得重建 AST"的契约——只是把 AST 来源换成 Yosys JSON。
- 成本与风险：
  - Yosys 是外部工具（Windows 安装、CI 配置、版本固定）；本仓库当前完全离线可测。
  - 需要处理 Yosys JSON 的位宽/参数/常量语义、flatten 后的层次、`$mem`、tristate、异步复位等，其中很多概念**现有 Routable IR 表达不了**（无 reset/enable/总线）。
  - 桥接质量决定成败：若直接接 gate-level 网表，LogicalDesign 的总线意图（inc/register）会丢失，反而触发 §4.2 的特例引擎限制。
- 结论：Yosys 能解决 frontend 的"广度"，但**不解决** Logical→Routable 通用映射、cell library、P&R 扩展这些项目核心瓶颈。

### 路线 C（推荐）：先补核心瓶颈，再并行接 Yosys

顺序建议（与提示词 Phase 4 的 Step 1-5 对齐但修正优先级）：

1. **通用 Logical→Routable mapper + TargetSpec/capability + MappingPolicy**（真正的核心）；
2. **可复用 cell library / 物理契约 + Pareto 候选**（让 LocalPlacer 从"每次现场搜索"变成"库选择"）；
3. **扩展 Routable IR 的表达力**：reset/enable/negedge、多时序单元、浅层次 flatten；
4. **frontend 补强或接 Yosys**：视目标用户决定（自研子集 vs 完整 Verilog/SV）；
5. **FSM/Counter/RAM/CPU demo**：在 1-3 完成前做 CPU 只会反复撞上特例引擎。

> 关键判断：Redstone Compiler 的最大价值在 **IR + P&R + World/NBT + 模拟验证闭环**，这些正是作者已经解决的难点；frontend 反而是相对可替换的一段。不要在未评估通用 mapper 之前，把预算全部投入 Yosys 集成。

---

## 6. 环境与工程状态（本次分析实测）

| 项目 | 状态 |
| --- | --- |
| Rust | rustup 已装：stable 1.98.1 + `nightly-2025-02-08`（项目要求） |
| `cargo check --all-targets` | ✅ 通过（补回被 gitignore 的快照源文件后） |
| `cargo test --release`（完整） | ⚠️ 8 个搜索密集 component 测试并行运行时本机 OOM（exit `0xc0000409`）；其中 `test_generate_component_rs_latch` 单独可过（约 127s） |
| `cargo test --release -- --skip test_generate_component` | ✅ 253 passed / 0 failed / 4 ignored / 8 filtered（4.85s） |
| 测试快照引导 | 需先重建 `test/counter.snapshot/counter.v` 与 `test/d-flip-flop.snapshot/d-flip-flop.v`（由测试内嵌源码逐字重建，非源码修改） |
| AGENTS.md 约定 | 测试用 `cargo test --release`；文档放 `docs/` 并在 AGENTS.md 登记；commit message 正文写意图 |

---

## 7. 附录：模块地图

| 模块 | 职责 | 关键文件 |
| --- | --- | --- |
| `src/verilog/` | Verilog 子集 frontend（lexer/parser/AST/RTL/synth/lower） | `parser.rs`、`rtl.rs`、`synth.rs`、`lower.rs` |
| `src/ir/` | 两阶段 IR、RCIR 文本、lowering、source-map | `logical.rs`、`routable.rs`、`logical_lowering.rs`、`text.rs`、`logical_text.rs`、`debug.rs` |
| `src/graph/` | 图模型、逻辑图、世界图、分析、graphviz | `mod.rs`、`logic.rs`、`world.rs` |
| `src/sequential/` | RS/D latch 语义、反馈核识别、宏布局 | `core.rs`、`layout.rs` |
| `src/transform/logic/` | 逻辑图变换（分解/组合/QMC/CSE/buffer） | `decompose.rs`、`compose.rs`、`optimize.rs`、`buffers.rs` |
| `src/transform/place_and_route/local_placer/` | 本地单元候选生成与顺序单元放置 | `mod.rs`、`routing.rs`、`sequential/*` |
| `src/transform/place_and_route/global_pnr/` | preparation、全局布局/布线、快照、物理意图 | `mod.rs`、`placer.rs`、`router.rs`、`candidate.rs`、`physical_intent.rs` |
| `src/world/` | World/World3D、Block、传播语义、模拟器 | `mod.rs`、`block.rs`、`simulator.rs` |
| `src/nbt/` | 结构方块 NBT 序列化/反序列化 | `mod.rs` |
| `src/snapshot.rs` / `src/output.rs` | 快照会话、产物、归档、接口元数据 | `snapshot.rs`、`output.rs` |
| `src/bin/` | 调试工具（NBT 环检测、toggle 复现） | `check_nbt_world_cycle.rs`、`repro_toggle.rs` |
| `crates/nbt-sim-wasm/` | 模拟器 WASM 封装 | `src/lib.rs` |
| `tools/nbt-viewer/` | 浏览器 3D/波形/IR 查看器 | `src/main.ts`、`src/sim/`、`src/render/` |

---

## 8. Phase 0 结论与下一步

- Phase 0（代码考察 + 架构分析）完成：现有项目的真实边界、资产与瓶颈已记录在案。
- 未修改任何源码；仅新增本文档，并重建了两个被 gitignore 的测试生成物（`test/*.snapshot/*.v`）。
- 建议 Phase 1 首先决策：**是否保留自研 frontend、是否引入 Yosys、以及通用 mapper/cell library 的优先级**（见 §5 路线对比）。
- 在 Phase 1 方案确定前，不应开始功能开发。
