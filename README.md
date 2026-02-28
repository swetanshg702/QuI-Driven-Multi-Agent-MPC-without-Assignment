# QI-Driven Multi-Agent MPC (Without Assignment)

A Julia implementation of a **multi-agent Model Predictive Control (MPC)** system enhanced with **Quasi-Interpolation (QI)**-based approximation for real-time control synthesis in discrete-time linear systems.

Two simulation modes are supported:

- **ManyToOne** — 10 agents with random initial states all converging to one fixed target
- **OneToMany** — 1 agent from a fixed initial state steering toward 10 different random targets

---

## Overview

Each agent is governed by a 2D discrete-time linear system. Instead of solving a full QP optimization at every time step, the controller uses **quasi-interpolation over a precomputed grid** of optimal cost and control values — falling back to a direct QP solver only when the QI approximation is unavailable or unreliable.

### Key Ideas

- **Block MPC**: The horizon is divided into blocks of `ν` micro-steps (where `ν` is the system's reachability index), reducing the number of QP variables.
- **Quasi-Interpolation (QI)**: A Gaussian kernel-based interpolation scheme approximates optimal controls from a precomputed grid, enabling fast online evaluation.
- **Fallback QP**: When QI fails (e.g., query point is far from the grid), the system falls back to solving the block QP exactly using the Clarabel solver.
- **Fresh randomness**: Initial states (ManyToOne) and target states (OneToMany) are sampled uniformly from `[-3, 3]²` on every run — no fixed seed.

---

## Project Structure

```
QuI Driven Multi-Agent MPC without Assignment/
│
├── ManyToOne/                          # 10 random agents → 1 fixed target
│   ├── run_qi_mpc.jl                   # Main entry point for ManyToOne
│   └── src/
│       ├── Constants.jl                # X0 removed; XT fixed; system params
│       ├── SystemDynamics.jl           # Dynamics utilities (unchanged)
│       ├── BlockQP.jl                  # Block QP solver (unchanged)
│       ├── QI.jl                       # Quasi-interpolation (unchanged)
│       ├── QIGridPrecompute.jl         # Grid precomputation (XT as argument)
│       └── MPCPlotting.jl              # Plotting: save_mpc_plot_multiagent
│
└── OneToMany/                          # 1 fixed agent → 10 random targets
    ├── run_qi_mpc.jl                   # Main entry point for OneToMany
    └── src/
        ├── Constants.jl                # X0 fixed; XT removed; system params
        ├── SystemDynamics.jl           # Dynamics utilities (unchanged)
        ├── BlockQP.jl                  # Block QP solver (unchanged)
        ├── QI.jl                       # Quasi-interpolation (unchanged)
        ├── QIGridPrecompute.jl         # Grid precomputation (XT as argument)
        └── MPCPlotting.jl              # Plotting: save_mpc_plot_multitarget
```

---

## System Parameters

Defined in `src/Constants.jl` (shared across both modes):

| Parameter         | Value                  | Description                        |
|-------------------|------------------------|------------------------------------|
| `N`               | `20`                   | MPC horizon (macro blocks)         |
| `max_micro_steps` | `200`                  | Maximum simulation steps per agent |
| `A`               | `[0.1 -0.5; 0.2 0.8]` | System matrix                      |
| `B`               | `[10.0; 0.0]`          | Input matrix                       |
| `Q`               | `Diagonal([5.0, 5.0])` | State cost weight                  |
| `R_scalar`        | `10.0`                 | Control cost weight                |

### ManyToOne specific

| Parameter | Value         | Description                                      |
|-----------|---------------|--------------------------------------------------|
| `XT`      | `[0.0, 0.0]`  | Fixed target — same every run                    |
| `X0`      | *(not used)*  | 10 initial states sampled randomly each run      |

### OneToMany specific

| Parameter | Value          | Description                                     |
|-----------|----------------|-------------------------------------------------|
| `X0`      | `[0.5, -2.0]`  | Fixed starting state — same every run           |
| `XT`      | *(not used)*   | 10 target states sampled randomly each run      |

---

## Module Descriptions

### `Constants.jl`
Single source of truth for all system parameters. `XT` is defined in ManyToOne; `X0` is defined in OneToMany. The dynamic quantity (random states or targets) is generated in `run_qi_mpc.jl` at runtime.

### `SystemDynamics.jl`
Core dynamics utilities — identical in both modes:
- `build_block_B`: Constructs the lifted input matrix for a block of `ν` steps.
- `compute_reachability_index`: Finds the minimum `ν` such that the system is reachable to `XT` in `ν` steps.
- `compute_Uref`: Computes the reference control sequence that steers the system to `XT` in one block.
- `apply_micro_step`: Applies one discrete-time step `x_{t+1} = Ax_t + Bu_t`.

### `BlockQP.jl`
Formulates and solves the block MPC optimization problem using **JuMP** and the **Clarabel** solver. Includes stage and terminal costs with a 10× terminal weight on `Q`. Accepts `xT` as an explicit argument.

### `QI.jl`
Implements 2D Gaussian quasi-interpolation:
- Grid defined over `[-3, 3] × [-3, 3]` with spacing `h = 0.2`.
- `quasi_interpolate_cost`: Interpolates the optimal cost at a query point.
- `quasi_interpolate_control_vector`: Interpolates the optimal control vector at a query point.
- Falls back to the nearest finite neighbor if kernel weights are too small.

### `QIGridPrecompute.jl`
Precomputes the cost and control grids by solving the block QP at every grid node offline. Accepts `XT` as an explicit argument so it can be called for any target — critical for OneToMany where each of the 10 targets needs its own grid.

### `MPCPlotting.jl`
Saves simulation results as interactive HTML plots using PlotlyJS. Contains three functions:
- `save_mpc_plot`: Original single-agent plot (kept for compatibility).
- `save_mpc_plot_multiagent`: Used by ManyToOne — plots 10 agent trajectories converging to one target.
- `save_mpc_plot_multitarget`: Used by OneToMany — plots 1 agent's trajectory toward each of 10 targets.

---

## Simulation Modes

### ManyToOne

10 agents start from random positions in `[-3, 3]²` and all converge to the same fixed target `XT`.

**Algorithm flow:**
```
1. Load fixed XT from Constants.jl
2. Compute reachability index ν
3. Precompute one shared QI grid for XT
4. Sample 10 random initial states (fresh every run)
5. For each agent:
   a. Query QI grid at current state z
   b. If QI succeeds → use interpolated control block
   c. If QI fails   → solve Block QP exactly (fallback)
   d. Apply ν micro-steps using chosen control
   e. Record state, control, and cost
6. Save position_plot.html and velocity_plot.html to ManyToOne/
```

**Output plots:**
- `ManyToOne/position_plot.html` — 10 position traces converging to `XT[1]`
- `ManyToOne/velocity_plot.html` — 10 velocity traces converging to `XT[2]`

---

### OneToMany

1 agent starts from a fixed `X0` and is steered toward 10 different random targets.

**Algorithm flow:**
```
1. Load fixed X0 from Constants.jl
2. Sample 10 random target states (fresh every run)
3. For each target:
   a. Compute reachability index ν for that target
   b. Precompute a dedicated QI grid for that target
   c. Run the agent from X0 toward the target:
      - Query QI grid at current state z
      - If QI succeeds → use interpolated control block
      - If QI fails   → solve Block QP exactly (fallback)
      - Apply ν micro-steps using chosen control
      - Record state, control, and cost
4. Save position_plot.html and velocity_plot.html to OneToMany/
```

**Output plots:**
- `OneToMany/position_plot.html` — 10 position traces from `X0[1]` to 10 different targets
- `OneToMany/velocity_plot.html` — 10 velocity traces from `X0[2]` to 10 different targets

> **Note:** OneToMany is significantly slower than ManyToOne because it precomputes a separate 31×31 QI grid for each of the 10 targets (10× the precomputation work).

---

## Getting Started

### Prerequisites

- Julia **1.8+**
- The following Julia packages:
  ```
  JuMP
  Clarabel
  LinearAlgebra
  Printf
  PlotlyJS
  ```

Install dependencies via the Julia REPL:
```julia
using Pkg
Pkg.add(["JuMP", "Clarabel", "PlotlyJS"])
```

### Running ManyToOne

```bash
cd ManyToOne
julia run_qi_mpc.jl
```

### Running OneToMany

```bash
cd OneToMany
julia run_qi_mpc.jl
```

### Output

Both modes print a summary to the terminal and save two HTML plots:

```
Target XT = [0.0, 0.0]
Reachability index ν = 2,  Nblk = 10

Agent initial states:
  Agent  1 : [ 1.2341, -2.7812]
  Agent  2 : [-0.4521,  1.1034]
  ...

Starting Agent 1 ...
  Final state : [ 0.00001,  0.00000]   error : 1.23e-05   cost : 4821.33
...

Position plot saved → ManyToOne/position_plot.html
Velocity plot saved → ManyToOne/velocity_plot.html
```

Open either HTML file in any browser to view interactive trajectory plots.

---

## Tunable Parameters

| Where | Parameter | Effect |
|-------|-----------|--------|
| `Constants.jl` | `XT` (ManyToOne) | Change the fixed convergence target |
| `Constants.jl` | `X0` (OneToMany) | Change the fixed starting state |
| `Constants.jl` | `max_micro_steps` | Longer/shorter simulation |
| `Constants.jl` | `N` | Longer/shorter MPC horizon |
| `QI.jl` | `h_fine` | Grid resolution (smaller = finer but slower) |
| `QI.jl` | `x_min_f / x_max_f` | Grid coverage range |
| `run_qi_mpc.jl` | `N_AGENTS / N_TARGETS` | Number of agents or targets |
| `run_qi_mpc.jl` | output filenames | Where plots are saved |

---

## Notes

- The `B` matrix uses `[10.0; 0.0]` — calibrated to avoid overshoot. Larger values caused QI-approximated controls to produce a persistent period-2 orbit.
- The QI grid range `[-3, 3]²` must cover the expected state trajectories. States outside this range automatically fall back to the QP solver.
- In OneToMany, since each target gets its own QI grid, precomputation time scales linearly with the number of targets.

---

## License

This project was developed as part of a B.Tech final year project (BTP). All rights reserved.
