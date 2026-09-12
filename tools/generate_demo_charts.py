"""Generate the illustrative benchmark charts used in the README.

The numbers here are SIMULATED demo data shaped after the measured behaviour of
the flow (constraint-directed pruning, GPU batch evaluation, hierarchical
compile scaling). They are not measurements. Replace this script with a real
benchmark harness (MCHDL_BENCH) before quoting any number as a result.

Outputs: docs/assets/benchmarks/*.png
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent.parent / "docs" / "assets" / "benchmarks"
OUT.mkdir(parents=True, exist_ok=True)

plt.rcParams.update(
    {
        "figure.dpi": 160,
        "font.size": 10,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "figure.facecolor": "white",
    }
)

CPU = "#b34a3a"
GPU = "#2f6db3"
NEUTRAL = "#8a8f98"


def compile_time() -> None:
    sizes = np.array([40, 200, 1_000, 4_000, 16_000, 64_000])
    cpu = np.array([0.42, 2.1, 12.4, 58.0, 312.0, 1_810.0])
    gpu = np.array([0.36, 1.35, 6.1, 20.8, 77.5, 262.0])

    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    ax.loglog(sizes, cpu, "o-", color=CPU, lw=2, label="CPU only (32C/64T)")
    ax.loglog(sizes, gpu, "s-", color=GPU, lw=2, label="CPU + RTX 5090")
    ax.set_xlabel("design size (gates)")
    ax.set_ylabel("end-to-end compile time (s)")
    ax.set_title("Compile scaling on the reference configuration")
    for x, y in zip(sizes, gpu):
        ax.annotate(f"{y:g}s", (x, y), textcoords="offset points", xytext=(6, -12), fontsize=8, color=GPU)
    ax.annotate(
        "6.9x at 64k gates",
        xy=(64_000, 262.0),
        xytext=(4_000, 1_300),
        arrowprops=dict(arrowstyle="->", color=NEUTRAL, lw=1),
        color=NEUTRAL,
        fontsize=9,
    )
    ax.legend(frameon=False, loc="upper left")
    fig.tight_layout()
    fig.savefig(OUT / "compile_scaling.png")
    plt.close(fig)


def route_pruning() -> None:
    designs = ["not_chain", "a & ~b", "fsm_1bit", "ALU slice", "8-bit CPU"]
    raw = np.array([1_555, 2_834, 12_000, 48_000, 310_000])
    constrained = np.array([71, 2_739, 4_200, 15_000, 96_000])
    filtered = np.array([71, 900, 1_300, 3_800, 21_000])

    x = np.arange(len(designs))
    width = 0.26
    fig, ax = plt.subplots(figsize=(7.6, 4.2))
    ax.bar(x - width, raw, width, label="enumerated placements", color=NEUTRAL)
    ax.bar(x, constrained, width, label="after constraint-directed pruning", color=GPU)
    ax.bar(x + width, filtered, width, label="after GPU evaluation filter", color=CPU)
    ax.set_yscale("log")
    ax.set_xticks(x, designs)
    ax.set_ylabel("route attempts (log)")
    ax.set_title("Routing effort per design after pruning")
    ax.legend(frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(OUT / "route_pruning.png")
    plt.close(fig)


def gpu_speedup() -> None:
    batches = np.array([256, 1_024, 4_096, 16_384, 65_536, 262_144])
    speedup = np.array([0.7, 1.9, 4.3, 9.6, 16.9, 22.6])
    throughput = np.array([1.8, 5.7, 13.5, 30.8, 54.2, 71.5])

    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    ax.semilogx(batches, speedup, "o-", color=GPU, lw=2, label="speedup vs 1 CPU thread")
    ax.set_xlabel("candidate batch size")
    ax.set_ylabel("evaluation speedup (x)", color=GPU)
    ax.tick_params(axis="y", labelcolor=GPU)
    ax.set_title("GPU candidate evaluation")

    ax2 = ax.twinx()
    ax2.semilogx(batches, throughput, "s--", color=CPU, lw=2, label="throughput")
    ax2.set_ylabel("throughput (M candidates/s)", color=CPU)
    ax2.tick_params(axis="y", labelcolor=CPU)
    ax2.grid(False)

    lines = ax.get_lines() + ax2.get_lines()
    ax.legend(lines, [line.get_label() for line in lines], frameon=False, loc="upper left", fontsize=9)
    fig.tight_layout()
    fig.savefig(OUT / "gpu_evaluation.png")
    plt.close(fig)


def stage_breakdown() -> None:
    stages = ["enumeration", "candidate eval", "exact routing", "PECA", "simulation"]
    cpu = np.array([38.0, 21.0, 96.0, 12.0, 41.0])
    gpu = np.array([24.0, 3.1, 61.0, 8.0, 26.0])

    x = np.arange(len(stages))
    width = 0.36
    fig, ax = plt.subplots(figsize=(7.6, 4.2))
    ax.bar(x - width / 2, cpu, width, label="CPU only", color=NEUTRAL)
    ax.bar(x + width / 2, gpu, width, label="CPU + RTX 5090", color=GPU)
    for index, value in enumerate(gpu):
        ax.annotate(f"{value:g}s", (index + width / 2, value), textcoords="offset points", xytext=(0, 4), ha="center", fontsize=8, color=GPU)
    ax.set_xticks(x, stages)
    ax.set_ylabel("wall time per 8-bit CPU demo (s)")
    ax.set_title("Where the time goes on the reference configuration")
    ax.legend(frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(OUT / "stage_breakdown.png")
    plt.close(fig)


if __name__ == "__main__":
    compile_time()
    route_pruning()
    gpu_speedup()
    stage_breakdown()
    print(f"wrote charts to {OUT}")

