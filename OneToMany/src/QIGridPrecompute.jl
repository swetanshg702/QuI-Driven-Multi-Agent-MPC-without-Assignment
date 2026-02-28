module QIGridPrecompute

using ..Constants, ..SystemDynamics, ..BlockQP, ..QI
using LinearAlgebra

export precompute_cost_control_grids

# XT is passed in explicitly so the grid is always built for the correct target
function precompute_cost_control_grids(ν::Int, Nblk::Int, XT::Vector{Float64})
    nx = length(x_fine)
    ny = length(y_fine)

    cost_grid    = fill(Inf, nx, ny)
    control_grid = Matrix{Union{Nothing, Vector{Float64}}}(nothing, nx, ny)

    println("Starting fresh grid precomputation ($nx × $ny points) for XT = $(round.(XT, digits=3)) ...")

    for i in 1:nx
        for j in 1:ny
            z0 = [x_fine[i], y_fine[j]]
            try
                sol = solve_block_QP(z0, XT, Nblk; A=A, B=B, Q=Q, R=R_scalar, ν=ν)
                cost_grid[i, j]    = sol.J
                control_grid[i, j] = sol.U_sol[:, 1]
            catch
                # Infeasible grid points remain Inf / nothing
            end
        end
    end

    println("Precomputation complete.")
    return cost_grid, control_grid
end

end # module QIGridPrecompute
