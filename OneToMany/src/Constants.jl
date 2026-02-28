module Constants

using LinearAlgebra

export N, max_micro_steps, html_filename,
       A, B, Q, R_scalar, X0

const X0 = [0.5; -2.0]         # fixed starting state — same every run
# XT is NOT defined here — 10 random targets are sampled in run_qi_mpc.jl

const N               = 20
const max_micro_steps = 200
const html_filename   = "subsampled_qi_mpc.html"

const A        = [0.1 -0.5;
                  0.2  0.8]
const B        = reshape([10.0, 0.0], (2,1))
const Q        = Diagonal([5.0, 5.0]) |> Matrix
const R_scalar = 10.0

end # module Constants
