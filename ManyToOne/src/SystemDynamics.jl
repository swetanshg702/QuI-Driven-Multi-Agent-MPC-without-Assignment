module SystemDynamics

using LinearAlgebra

export build_block_B,
       compute_reachability_index,
       compute_Uref,
       apply_micro_step

function build_block_B(A::AbstractMatrix, B::AbstractMatrix, ν::Int)
    n = size(A, 1)
    Bl = zeros(n, ν)
    for i = 1:ν
        Bl[:, i] = A^(ν - i) * vec(B)
    end
    return Bl
end

function compute_reachability_index(
    A::AbstractMatrix,
    B::AbstractMatrix,
    xT::Vector{Float64};
    max_ν = 10,
    tol = 1e-10
)
    n = size(A, 1)
    for ν = 1:max_ν
        Bl = build_block_B(A, B, ν)
        if rank(Bl) >= n
            Aν = A^ν
            d = (I(n) - Aν) * xT
            if norm(Bl * (pinv(Bl) * d) - d) < tol
                return ν
            end
        end
    end
    error("No reachability index found up to ν = $max_ν")
end

function compute_Uref(
    A::AbstractMatrix,
    B::AbstractMatrix,
    xT::Vector{Float64},
    ν::Int
)
    Aν = A^ν
    Bl = build_block_B(A, B, ν)
    d = (I(size(A,1)) - Aν) * xT
    return vec(pinv(Bl) * d)
end

function apply_micro_step(
    A::AbstractMatrix,
    B::AbstractMatrix,
    x::Vector{Float64},
    u::Real
)
    return A * x .+ vec(B) * u
end

end # module SystemDynamics
