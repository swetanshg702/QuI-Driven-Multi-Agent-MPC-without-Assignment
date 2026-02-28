module MPCPlotting

using PlotlyJS
using PlotlyJS: attr

export save_mpc_plot, save_mpc_plot_multiagent

# ── original single-agent plot (kept for compatibility) ──────────────────────
function save_mpc_plot(
    history_z, history_z_times, history_u, history_errors,
    XT, max_micro_steps, ν, html_filename
)
    n_z = length(history_z)
    n_u = length(history_u)
    n_e = length(history_errors)

    t_z  = history_z_times[1:n_z]
    t_u  = collect(1:n_u)
    p_sub = [s[1] for s in history_z]
    v_sub = [s[2] for s in history_z]
    log_err = log10.(history_errors[1:n_e] .+ 1e-16)

    tr_p  = scatter(x=t_z, y=p_sub,  mode="markers+lines", name="position",
                    marker=attr(size=8, symbol="diamond"), yaxis="y1")
    tr_pt = scatter(x=[0,max_micro_steps], y=[XT[1],XT[1]], mode="lines",
                    name="p_target", line=attr(dash="dash",width=2), yaxis="y1")
    tr_v  = scatter(x=t_z, y=v_sub,  mode="markers+lines", name="velocity",
                    marker=attr(size=8, symbol="diamond"), yaxis="y2")
    tr_vt = scatter(x=[0,max_micro_steps], y=[XT[2],XT[2]], mode="lines",
                    name="v_target", line=attr(dash="dash",width=2), yaxis="y2")
    tr_u  = scatter(x=t_u, y=history_u, mode="lines", name="u (micro)",
                    line=attr(width=1.5), yaxis="y3")
    tr_le = scatter(x=t_z[1:n_e], y=log_err, mode="lines+markers",
                    name="log10(error)", line=attr(width=2),
                    marker=attr(size=6), yaxis="y4")

    layout = Layout(
        title = "QI-MPC: Subsampled States (nu=$ν)",
        xaxis  = attr(title="micro-steps", range=[0,max_micro_steps]),
        yaxis  = attr(domain=[0.76,1.00], title="position"),
        yaxis2 = attr(domain=[0.52,0.74], title="velocity"),
        yaxis3 = attr(domain=[0.26,0.50], title="control u"),
        yaxis4 = attr(domain=[0.00,0.24], title="log10 error"),
        margin = attr(l=70, r=140, t=80, b=60)
    )
    savefig(Plot([tr_p,tr_pt,tr_v,tr_vt,tr_u,tr_le], layout), html_filename)
end


# ── multi-agent plot ──────────────────────────────────────────────────────────
"""
    save_mpc_plot_multiagent(results, XT, max_micro_steps, ν, html_filename)

Saves **two** interactive HTML files:
  - `html_filename`          → position traces for all agents
  - `velocity_<html_filename>` → velocity traces for all agents

Each file shows every agent converging to the shared target.
"""
function save_mpc_plot_multiagent(
    results,          # Vector of NamedTuples from run_single_agent
    XT,
    max_micro_steps,
    ν,
    html_filename
)
    N = length(results)

    # ── colour palette (cycle if N > 10) ─────────────────────────────────────
    palette = [
        "#1f77b4","#ff7f0e","#2ca02c","#d62728","#9467bd",
        "#8c564b","#e377c2","#7f7f7f","#bcbd22","#17becf"
    ]
    colour(k) = palette[mod1(k, length(palette))]

    # ═══════════════════════════════════════════════════════════════════════════
    # PLOT 1 — POSITION
    # ═══════════════════════════════════════════════════════════════════════════
    pos_traces = GenericTrace[]

    # Target dashed line
    push!(pos_traces, scatter(
        x    = [0, max_micro_steps],
        y    = [XT[1], XT[1]],
        mode = "lines",
        name = "p_target  ($(round(XT[1],digits=2)))",
        line = attr(dash="dash", width=2, color="black"),
    ))

    for k in 1:N
        r   = results[k]
        t_z = r.history_z_times
        p   = [s[1] for s in r.history_z]
        x0  = round(r.history_z[1][1]; digits=2)
        push!(pos_traces, scatter(
            x      = t_z,
            y      = p,
            mode   = "lines+markers",
            name   = "Agent $k  (x₀=$(x0))",
            line   = attr(width=1.8, color=colour(k)),
            marker = attr(size=5, symbol="circle", color=colour(k)),
        ))
    end

    pos_layout = Layout(
        title  = "QI-MPC Multi-Agent: Position Convergence (ν=$ν)",
        xaxis  = attr(title="micro-steps", range=[0, max_micro_steps]),
        yaxis  = attr(title="position  x₁"),
        legend = attr(x=1.02, y=1.0, xanchor="left"),
        margin = attr(l=70, r=200, t=80, b=60),
    )

    pos_fig  = Plot(pos_traces, pos_layout)
    savefig(pos_fig, html_filename)
    println("Position plot saved → $html_filename")

    # ═══════════════════════════════════════════════════════════════════════════
    # PLOT 2 — VELOCITY
    # ═══════════════════════════════════════════════════════════════════════════
    vel_filename = joinpath(dirname(html_filename), "velocity_plot.html")
    vel_traces   = GenericTrace[]

    push!(vel_traces, scatter(
        x    = [0, max_micro_steps],
        y    = [XT[2], XT[2]],
        mode = "lines",
        name = "v_target  ($(round(XT[2],digits=2)))",
        line = attr(dash="dash", width=2, color="black"),
    ))

    for k in 1:N
        r   = results[k]
        t_z = r.history_z_times
        v   = [s[2] for s in r.history_z]
        y0  = round(r.history_z[1][2]; digits=2)
        push!(vel_traces, scatter(
            x      = t_z,
            y      = v,
            mode   = "lines+markers",
            name   = "Agent $k  (x₀=$(y0))",
            line   = attr(width=1.8, color=colour(k)),
            marker = attr(size=5, symbol="circle", color=colour(k)),
        ))
    end

    vel_layout = Layout(
        title  = "QI-MPC Multi-Agent: Velocity Convergence (ν=$ν)",
        xaxis  = attr(title="micro-steps", range=[0, max_micro_steps]),
        yaxis  = attr(title="velocity  x₂"),
        legend = attr(x=1.02, y=1.0, xanchor="left"),
        margin = attr(l=70, r=200, t=80, b=60),
    )

    vel_fig = Plot(vel_traces, vel_layout)
    savefig(vel_fig, vel_filename)
    println("Velocity plot saved  → $vel_filename")
end

end # module MPCPlotting
