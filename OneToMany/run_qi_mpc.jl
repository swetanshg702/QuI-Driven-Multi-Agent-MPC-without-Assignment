# run_qi_mpc.jl  ── Reverse Multi-Target version (1 agent, 10 targets)
#   - Starting state : fixed X0 from Constants.jl
#   - Target states  : 10 sampled uniformly at random from [-3, 3]² — fresh every run
#   - QI grid        : precomputed separately for each target
include("src/Constants.jl")
include("src/SystemDynamics.jl")
include("src/BlockQP.jl")
include("src/QI.jl")
include("src/MPCPlotting.jl")
include("src/QIGridPrecompute.jl")

using .Constants, .SystemDynamics, .BlockQP, .QI, .MPCPlotting, .QIGridPrecompute
using LinearAlgebra, Printf, Random

const N_TARGETS = 10

# ── random point uniformly in [-3, 3]² ──────────────────────────────────────
rand_state() = [rand() * 6.0 - 3.0, rand() * 6.0 - 3.0]

# ── single run: one agent from X0 to a given XT ─────────────────────────────
function run_single_target(XT, cost_grid, control_grid, ν, Nblk)
    z               = copy(X0)
    history_z       = [copy(z)]
    history_z_times = [0]
    history_u       = Float64[]
    history_errors  = Float64[]
    micro_t         = 0
    J_accumulated   = 0.0

    while micro_t < max_micro_steps
        # --- QI lookup ---
        u_block = quasi_interpolate_control_vector(
            z[1], z[2], x_fine, y_fine, control_grid, D, h_fine, rho
        )

        Uref = compute_Uref(A, B, XT, ν)

        # --- Fallback to exact QP if QI cannot help ---
        if u_block === nothing
            try
                sol     = solve_block_QP(z, XT, Nblk; A=A, B=B, Q=Q, R=R_scalar, ν=ν)
                u_block = sol.U_sol[:, 1]
                Uref    = sol.Uref
            catch e
                @warn "Both QI and Fallback failed at state $z. Stopping."
                break
            end
        end

        # --- Apply ν micro-steps ---
        for j = 1:ν
            micro_t >= max_micro_steps && break
            u      = u_block[j]
            z_next = apply_micro_step(A, B, z, u)

            control_cost   = R_scalar * (u - Uref[j])^2
            state_cost     = dot(z_next - XT, Q * (z_next - XT))
            J_accumulated += state_cost + control_cost

            z = z_next
            push!(history_u, u)
            micro_t += 1
        end

        push!(history_z,       copy(z))
        push!(history_z_times, micro_t)
        push!(history_errors,  norm(z - XT))
    end

    return (history_z       = history_z,
            history_z_times = history_z_times,
            history_u       = history_u,
            history_errors  = history_errors,
            J               = J_accumulated,
            z_final         = z)
end

# ── main ─────────────────────────────────────────────────────────────────────
function main()
    println("Starting state X0 = [$(X0[1]), $(X0[2])]")

    # 1. Sample 10 random targets — fresh every run
    targets = [rand_state() for _ in 1:N_TARGETS]

    println("\n10 randomly sampled targets:")
    for (k, t) in enumerate(targets)
        @printf("  Target %2d : [%7.4f, %7.4f]\n", k, t[1], t[2])
    end

    # 2. For each target: compute ν, precompute QI grid, run agent
    results = Vector{NamedTuple}(undef, N_TARGETS)

    for k in 1:N_TARGETS
        XT = targets[k]
        println("\n── Target $k : XT = [$(round(XT[1],digits=4)), $(round(XT[2],digits=4))] ──")

        ν    = compute_reachability_index(A, B, XT; max_ν = 10)
        Nblk = max(1, Int(N ÷ ν))
        println("  ν = $ν,  Nblk = $Nblk")

        cost_grid, control_grid = precompute_cost_control_grids(ν, Nblk, XT)

        results[k] = run_single_target(XT, cost_grid, control_grid, ν, Nblk)

        @printf("  Final state : [%8.5f, %8.5f]   error : %.4e   cost : %.4f\n",
                results[k].z_final[1], results[k].z_final[2],
                norm(results[k].z_final - XT), results[k].J)
    end

    # 3. Save plots — position and velocity
mkpath("OneToMany")
save_mpc_plot_multitarget(results, targets, X0, max_micro_steps, "OneToMany/position_plot.html", "OneToMany/velocity_plot.html")

end

main()
