module MPCPlotting

using PlotlyJS
using PlotlyJS: attr

export save_mpc_plot, save_mpc_plot_multiagent, save_mpc_plot_multitarget

# ── colour palette ───────────────────────────────────────────────────────────
const PALETTE = [
    "#1f77b4","#ff7f0e","#2ca02c","#d62728","#9467bd",
    "#8c564b","#e377c2","#7f7f7f","#bcbd22","#17becf"
]
_colour(k) = PALETTE[mod1(k, length(PALETTE))]

# ── original single-agent plot (kept for compatibility) ──────────────────────
function save_mpc_plot(
    history_z, history_z_times, history_u, history_errors,
    XT, max_micro_steps, ν, html_filename
)
    n_z = length(history_z)
    n_u = length(history_u)
    n_e = length(history_errors)

    t_z     = history_z_times[1:n_z]
    t_u     = collect(1:n_u)
    p_sub   = [s[1] for s in history_z]
    v_sub   = [s[2] for s in history_z]
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
        title  = "QI-MPC: Subsampled States (nu=$ν)",
        xaxis  = attr(title="micro-steps", range=[0,max_micro_steps]),
        yaxis  = attr(domain=[0.76,1.00], title="position"),
        yaxis2 = attr(domain=[0.52,0.74], title="velocity"),
        yaxis3 = attr(domain=[0.26,0.50], title="control u"),
        yaxis4 = attr(domain=[0.00,0.24], title="log10 error"),
        margin = attr(l=70, r=140, t=80, b=60)
    )
    savefig(Plot([tr_p,tr_pt,tr_v,tr_vt,tr_u,tr_le], layout), html_filename)
end


# ── multi-agent plot (10 agents → 1 target) ──────────────────────────────────
function save_mpc_plot_multiagent(
    results, XT, max_micro_steps, ν, html_filename
)
    N = length(results)

    # --- Position plot ---
    pos_traces = GenericTrace[]
    push!(pos_traces, scatter(
        x=[0,max_micro_steps], y=[XT[1],XT[1]],
        mode="lines", name="p_target ($(round(XT[1],digits=2)))",
        line=attr(dash="dash", width=2, color="black")
    ))
    for k in 1:N
        r  = results[k]
        p  = [s[1] for s in r.history_z]
        x0 = round(r.history_z[1][1]; digits=2)
        push!(pos_traces, scatter(
            x=r.history_z_times, y=p, mode="lines+markers",
            name="Agent $k  (x₀=$x0)",
            line=attr(width=1.8, color=_colour(k)),
            marker=attr(size=5, color=_colour(k))
        ))
    end
    savefig(Plot(pos_traces, Layout(
        title  = "QI-MPC Multi-Agent: Position Convergence (ν=$ν)",
        xaxis  = attr(title="micro-steps", range=[0,max_micro_steps]),
        yaxis  = attr(title="position  x₁"),
        legend = attr(x=1.02, y=1.0, xanchor="left"),
        margin = attr(l=70, r=200, t=80, b=60)
    )), html_filename)
    println("Position plot saved → $html_filename")

    # --- Velocity plot ---
    vel_filename = "velocity_plot.html"
    vel_traces   = GenericTrace[]
    push!(vel_traces, scatter(
        x=[0,max_micro_steps], y=[XT[2],XT[2]],
        mode="lines", name="v_target ($(round(XT[2],digits=2)))",
        line=attr(dash="dash", width=2, color="black")
    ))
    for k in 1:N
        r  = results[k]
        v  = [s[2] for s in r.history_z]
        y0 = round(r.history_z[1][2]; digits=2)
        push!(vel_traces, scatter(
            x=r.history_z_times, y=v, mode="lines+markers",
            name="Agent $k  (x₀=$y0)",
            line=attr(width=1.8, color=_colour(k)),
            marker=attr(size=5, color=_colour(k))
        ))
    end
    savefig(Plot(vel_traces, Layout(
        title  = "QI-MPC Multi-Agent: Velocity Convergence (ν=$ν)",
        xaxis  = attr(title="micro-steps", range=[0,max_micro_steps]),
        yaxis  = attr(title="velocity  x₂"),
        legend = attr(x=1.02, y=1.0, xanchor="left"),
        margin = attr(l=70, r=200, t=80, b=60)
    )), vel_filename)
    println("Velocity plot saved  → $vel_filename")
end


# ── multi-target plot (1 agent → 10 targets) ─────────────────────────────────
function save_mpc_plot_multitarget(
    results,          # Vector of NamedTuples from run_single_target
    targets,          # Vector of 10 XT vectors
    X0,               # Single starting state
    max_micro_steps,
    pos_filename,     # e.g. "position_plot.html"
    vel_filename      # e.g. "velocity_plot.html"
)
    N = length(results)

    # ── Position plot ────────────────────────────────────────────────────────
    pos_traces = GenericTrace[]

    # Mark X0 as a single point reference
    push!(pos_traces, scatter(
        x=[0], y=[X0[1]],
        mode="markers", name="X0 ($(round(X0[1],digits=2)))",
        marker=attr(size=12, symbol="star", color="black")
    ))

    for k in 1:N
        r   = results[k]
        p   = [s[1] for s in r.history_z]
        xt1 = round(targets[k][1]; digits=2)
        xt2 = round(targets[k][2]; digits=2)

        # Trajectory
        push!(pos_traces, scatter(
            x=r.history_z_times, y=p,
            mode="lines+markers",
            name="Target $k  (XT=[$xt1,$xt2])",
            line=attr(width=1.8, color=_colour(k)),
            marker=attr(size=5, color=_colour(k))
        ))

        # Dashed horizontal line at each target's position component
        push!(pos_traces, scatter(
            x=[0, max_micro_steps],
            y=[targets[k][1], targets[k][1]],
            mode="lines",
            name="p_target $k",
            line=attr(dash="dot", width=1.2, color=_colour(k)),
            showlegend=false
        ))
    end

    savefig(Plot(pos_traces, Layout(
        title  = "QI-MPC Multi-Target: Position Trajectories (1 Agent → 10 Targets)",
        xaxis  = attr(title="micro-steps", range=[0, max_micro_steps]),
        yaxis  = attr(title="position  x₁"),
        legend = attr(x=1.02, y=1.0, xanchor="left"),
        margin = attr(l=70, r=250, t=80, b=60)
    )), pos_filename)
    println("Position plot saved → $pos_filename")

    # ── Velocity plot ────────────────────────────────────────────────────────
    vel_traces = GenericTrace[]

    push!(vel_traces, scatter(
        x=[0], y=[X0[2]],
        mode="markers", name="X0 ($(round(X0[2],digits=2)))",
        marker=attr(size=12, symbol="star", color="black")
    ))

    for k in 1:N
        r   = results[k]
        v   = [s[2] for s in r.history_z]
        xt1 = round(targets[k][1]; digits=2)
        xt2 = round(targets[k][2]; digits=2)

        push!(vel_traces, scatter(
            x=r.history_z_times, y=v,
            mode="lines+markers",
            name="Target $k  (XT=[$xt1,$xt2])",
            line=attr(width=1.8, color=_colour(k)),
            marker=attr(size=5, color=_colour(k))
        ))

        push!(vel_traces, scatter(
            x=[0, max_micro_steps],
            y=[targets[k][2], targets[k][2]],
            mode="lines",
            name="v_target $k",
            line=attr(dash="dot", width=1.2, color=_colour(k)),
            showlegend=false
        ))
    end

    savefig(Plot(vel_traces, Layout(
        title  = "QI-MPC Multi-Target: Velocity Trajectories (1 Agent → 10 Targets)",
        xaxis  = attr(title="micro-steps", range=[0, max_micro_steps]),
        yaxis  = attr(title="velocity  x₂"),
        legend = attr(x=1.02, y=1.0, xanchor="left"),
        margin = attr(l=70, r=250, t=80, b=60)
    )), vel_filename)
    println("Velocity plot saved  → $vel_filename")
end

end # module MPCPlotting
