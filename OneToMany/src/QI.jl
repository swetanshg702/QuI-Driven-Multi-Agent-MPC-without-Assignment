module QI
using LinearAlgebra

export gaussian_kernel_2d, quasi_interpolate_cost, quasi_interpolate_control_vector,
       h_fine, D, delta, x_fine, y_fine, rho

const h_fine = 0.2
const D = 2.0
const delta = 1e-4
const x_min_f = -3.0; const x_max_f = 3.0
const y_min_f = -3.0; const y_max_f = 3.0
const x_fine = collect(x_min_f:h_fine:x_max_f)
const y_fine = collect(y_min_f:h_fine:y_max_f)
const rho = h_fine * sqrt(-D * log(delta))

function gaussian_kernel_2d(x, y, xi, yj, D, h)
    (1.0/(π*D)) * exp(-((x - xi)^2 + (y - yj)^2) / (D * h^2))
end

function quasi_interpolate_cost(xq, yq, x_grid, y_grid, values, D, h, rho)
    total_w = 0.0; weighted = 0.0
    nx = length(x_grid); ny = length(y_grid)
    for i in 1:nx, j in 1:ny
        dx = xq - x_grid[i]; dy = yq - y_grid[j]
        if sqrt(dx^2 + dy^2) > rho continue end
        w = gaussian_kernel_2d(xq, yq, x_grid[i], y_grid[j], D, h)
        if w < 1e-15 continue end
        val = values[i,j]
        if !isfinite(val) continue end
        total_w += w; weighted += val * w
    end
    if total_w > 1e-12 return weighted / total_w end
    # nearest finite neighbor fallback
    min_dist = Inf; ni = 0; nj = 0
    for i in 1:nx, j in 1:ny
        val = values[i,j]
        if !isfinite(val) continue end
        d = (xq - x_grid[i])^2 + (yq - y_grid[j])^2
        if d < min_dist min_dist = d; ni = i; nj = j end
    end
    return ni != 0 ? values[ni, nj] : Inf
end

function quasi_interpolate_control_vector(xq, yq, x_grid, y_grid, u0_matrix, D, h, rho)
    nx = length(x_grid); ny = length(y_grid)
    ν = nothing
    for i in 1:nx, j in 1:ny
        if u0_matrix[i,j] !== nothing
            ν = length(u0_matrix[i,j]); break
        end
    end
    ν === nothing && return nothing
    weighted_u = zeros(ν); total_weight = 0.0
    for i in 1:nx, j in 1:ny
        uvec = u0_matrix[i,j]
        uvec === nothing && continue
        dx = xq - x_grid[i]; dy = yq - y_grid[j]
        dist = sqrt(dx^2 + dy^2); dist > rho && continue
        w = gaussian_kernel_2d(xq, yq, x_grid[i], y_grid[j], D, h)
        w < 1e-15 && continue
        weighted_u .+= w .* uvec; total_weight += w
    end
    if total_weight > 1e-12 return weighted_u ./ total_weight end
    # Nearest neighbor fallback
    min_dist = Inf; best_u = nothing
    for i in 1:nx, j in 1:ny
        uvec = u0_matrix[i,j]
        uvec === nothing && continue
        d = (xq - x_grid[i])^2 + (yq - y_grid[j])^2
        if d < min_dist min_dist = d; best_u = uvec end
    end
    return best_u
end
end
