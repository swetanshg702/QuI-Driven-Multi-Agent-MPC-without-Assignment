include("src/Constants.jl")
include("src/SystemDynamics.jl")
include("src/BlockQP.jl")
include("src/QI.jl")
include("src/MPCPlotting.jl")
include("src/QIGridPrecompute.jl")

using .Constants, .SystemDynamics, .BlockQP, .QI, .MPCPlotting, .QIGridPrecompute
using LinearAlgebra, Printf, Random

const N_AGENTS = 10

# Random point uniformly in [-3, 3]^2
rand_state() = [rand() * 6.0 - 3.0, rand() * 6.0 - 3.0]

# Single-agent MPC loop 
function run_single_agent(z0, XT, ν, Nblk, cost_grid, control_grid)
    z               = copy(z0)
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
                @warn "Both QI and Fallback failed at state $z. Stopping agent."
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

# main ===============================================================
function main()
    # 1. Fixed target from Constants.jl
    println("Target XT = [$(XT[1]), $(XT[2])]")

    # 2. Shared system setup
    ν    = compute_reachability_index(A, B, XT; max_ν = 10)
    Nblk = max(1, Int(N ÷ ν))
    println("Reachability index ν = $ν,  Nblk = $Nblk")

    # 3. Precompute QI grids once for XT (shared across all agents)
    cost_grid, control_grid = precompute_cost_control_grids(ν, Nblk, XT)

    # 4. Sample 10 random initial states — fresh every run
    init_states = [rand_state() for _ in 1:N_AGENTS]

    println("\nAgent initial states:")
    for (k, s) in enumerate(init_states)
        @printf("  Agent %2d : [%7.4f, %7.4f]\n", k, s[1], s[2])
    end

    # 5. Run each agent
    results = Vector{NamedTuple}(undef, N_AGENTS)
    for k in 1:N_AGENTS
        println("\nStarting Agent $k ...")
        results[k] = run_single_agent(init_states[k], XT, ν, Nblk, cost_grid, control_grid)
        @printf("  Final state : [%8.5f, %8.5f]   error : %.4e   cost : %.4f\n",
                results[k].z_final[1], results[k].z_final[2],
                norm(results[k].z_final - XT), results[k].J)
    end

    # 6. Save plots into ManyToOne folder
    mkpath("ManyToOne")
    save_mpc_plot_multiagent(results, XT, max_micro_steps, ν, "ManyToOne/position_plot.html")

    println("\nDone. Open ManyToOne/position_plot.html and ManyToOne/velocity_plot.html in any browser.")
end

main()
