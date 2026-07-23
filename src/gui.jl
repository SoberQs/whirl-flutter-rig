# Interactive live monitor and CSV recorder for the `whirl-rig` experiment.
# julia --project=. -e 'using Pkg; Pkg.instantiate()'
# julia --project=. src/gui.jl --demo

using Dates
using GLMakie
using HelicDAQ

const DEFAULT_HOST = "192.168.1.238"
const WHIRL_SOURCES = (:pitch, :yaw, :rpm)
const MAX_PLOT_POINTS = 2_000
const PLOT_REFRESH_SECONDS = 0.025
const STREAM_TIMEOUT_SECONDS = 5.0
const MAX_STREAM_RESTARTS = 3
const CONTROL_KEEPALIVE_SECONDS = 10.0
const ANGLE_MARGIN_DEGREES = 10.0f0

mutable struct WhirlApp
    host::String
    demo::Bool
    decimation::Int
    window_seconds::Float64
    sample_rate::Float64
    running::Bool
    busy::Bool
    session::UInt64
    device::Union{Nothing, Device}
    receiver::Union{Nothing, StreamReceiver}
    times::Vector{Float64}
    sample_indices::Vector{UInt64}
    pitch_degrees::Vector{Float32}
    yaw_degrees::Vector{Float32}
    rpm::Vector{Float32}
    pitch_zero_degrees::Union{Nothing, Float32}
    yaw_zero_degrees::Union{Nothing, Float32}
    first_index::Union{Nothing, UInt64}
    previous_raw_index::Union{Nothing, UInt32}
    index_wraps::UInt64
    demo_index::UInt64
    dropped::UInt32
    lost_packets::Int
    plot_time::Observable{Vector{Float64}}
    plot_pitch_points::Observable{Vector{Point2f}}
    plot_yaw_points::Observable{Vector{Point2f}}
    plot_rpm_points::Observable{Vector{Point2f}}
    status_text::Observable{String}
    stats_text::Observable{String}
    info_text::Observable{String}
    angle_axis::Any
    rpm_axis::Any
    save_path::Any
    figure::Any
end

function WhirlApp(host, demo, decimation, window_seconds)
    return WhirlApp(
        host,
        demo,
        decimation,
        window_seconds,
        2_000.0,
        false,
        false,
        UInt64(0),
        nothing,
        nothing,
        Float64[],
        UInt64[],
        Float32[],
        Float32[],
        Float32[],
        nothing,
        nothing,
        nothing,
        nothing,
        UInt64(0),
        UInt64(0),
        UInt32(0),
        0,
        Observable(Float64[]),
        Observable(Point2f[]),
        Observable(Point2f[]),
        Observable(Point2f[]),
        Observable("Idle"),
        Observable("0 samples"),
        Observable(demo ? "Demo mode — click Start" : "Target: $host"),
        nothing,
        nothing,
        nothing,
        nothing,
    )
end

function build_figure!(app::WhirlApp)
    set_theme!(
        Theme(
            fontsize = 16,
            backgroundcolor = RGBf(0.965, 0.973, 0.985),
            Axis = (
                backgroundcolor = :white,
                xgridcolor = RGBf(0.87, 0.89, 0.93),
                ygridcolor = RGBf(0.87, 0.89, 0.93),
                topspinevisible = false,
                rightspinevisible = false,
            ),
        ),
    )
    figure = Figure(; size = (1480, 900), figure_padding = 24)
    Label(
        figure[1, 1:2],
        "HELIC-DAQ · Whirl Rig Live Monitor";
        fontsize = 28,
        font = :bold,
        color = RGBf(0.08, 0.15, 0.28),
        tellwidth = false,
    )
    mode_label = app.demo ? "SIMULATED INPUT" : "ETHERNET · $(app.host)"
    Label(
        figure[2, 1:2],
        mode_label;
        fontsize = 13,
        color = RGBf(0.32, 0.42, 0.58),
        tellwidth = false,
    )

    angle_axis = Axis(
        figure[3, 1];
        title = "Encoder angle",
        xlabel = "Time [s]",
        ylabel = "Angle [deg]",
    )
    rpm_axis = Axis(
        figure[4, 1];
        title = "Rotor speed",
        xlabel = "Time [s]",
        ylabel = "Speed [RPM]",
    )
    lines!(angle_axis, app.plot_pitch_points; color = RGBf(0.1, 0.42, 0.9), linewidth = 2, label = "Pitch")
    lines!(angle_axis, app.plot_yaw_points; color = RGBf(0.92, 0.28, 0.32), linewidth = 2, label = "Yaw")
    lines!(rpm_axis, app.plot_rpm_points; color = RGBf(0.12, 0.66, 0.46), linewidth = 2.2)
    axislegend(angle_axis; position = :lb, framevisible = false, orientation = :horizontal)
    ylims!(angle_axis, -ANGLE_MARGIN_DEGREES, ANGLE_MARGIN_DEGREES)
    ylims!(rpm_axis, 0, 6_500)
    xlims!(angle_axis, 0, app.window_seconds)
    xlims!(rpm_axis, 0, app.window_seconds)

    controls = GridLayout(;
        tellwidth = true,
        width = 260,
        valign = :top,
        rowgap = 14,
    )
    figure[3:4, 2] = controls
    Label(controls[1, 1], "Controls"; fontsize = 21, font = :bold, halign = :left, tellwidth = false)
    start_button = Button(
        controls[2, 1];
        label = "Start",
        height = 48,
        width = 190,
        tellwidth = false,
        buttoncolor = RGBf(0.15, 0.67, 0.46),
        buttoncolor_hover = RGBf(0.11, 0.58, 0.39),
        labelcolor = :white,
        labelcolor_hover = :white,
    )
    pause_button = Button(
        controls[3, 1];
        label = "Pause",
        height = 48,
        width = 190,
        tellwidth = false,
        buttoncolor = RGBf(0.96, 0.65, 0.16),
        buttoncolor_hover = RGBf(0.88, 0.55, 0.1),
        labelcolor = :white,
        labelcolor_hover = :white,
    )
    save_button = Button(
        controls[4, 1];
        label = "Save CSV",
        height = 48,
        width = 190,
        tellwidth = false,
        buttoncolor = RGBf(0.16, 0.43, 0.78),
        buttoncolor_hover = RGBf(0.12, 0.35, 0.69),
        labelcolor = :white,
        labelcolor_hover = :white,
    )
    clear_button = Button(
        controls[5, 1];
        label = "Clear",
        height = 48,
        width = 190,
        tellwidth = false,
        buttoncolor = RGBf(0.76, 0.29, 0.34),
        buttoncolor_hover = RGBf(0.67, 0.22, 0.28),
        labelcolor = :white,
        labelcolor_hover = :white,
    )
    Label(controls[6, 1], "Optional save path (press Enter)"; halign = :left, tellwidth = false, color = RGBf(0.32, 0.38, 0.48))
    save_path = Textbox(
        controls[7, 1];
        placeholder = "auto: captures/whirl_*.csv",
        height = 42,
        tellwidth = false,
        halign = :left,
    )
    Label(controls[8, 1], app.status_text; fontsize = 18, font = :bold, halign = :left, tellwidth = false)
    Label(controls[9, 1], app.stats_text; halign = :left, tellwidth = false, color = RGBf(0.25, 0.32, 0.43))
    Label(
        controls[10, 1],
        app.info_text;
        halign = :left,
        tellwidth = false,
        width = 245,
        color = RGBf(0.25, 0.32, 0.43),
        justification = :left,
        word_wrap = true,
    )

    colsize!(figure.layout, 1, Relative(0.81))
    colsize!(figure.layout, 2, Fixed(270))
    rowsize!(figure.layout, 3, Relative(0.5))
    rowsize!(figure.layout, 4, Relative(0.5))

    app.angle_axis = angle_axis
    app.rpm_axis = rpm_axis
    app.save_path = save_path
    app.figure = figure

    on(start_button.clicks) do _
        @async start_receiving!(app)
        return nothing
    end
    on(pause_button.clicks) do _
        @async pause_receiving!(app)
        return nothing
    end
    on(save_button.clicks) do _
        @async save_csv!(app)
        return nothing
    end
    on(clear_button.clicks) do _
        clear_data!(app)
        return nothing
    end
    on(events(figure).window_open) do is_open
        !is_open && @async shutdown!(app)
        return nothing
    end
    return figure
end

function _record_index!(app::WhirlApp, raw::UInt32)
    previous = app.previous_raw_index
    if !isnothing(previous) && raw < previous && previous - raw > (UInt32(1) << 31)
        app.index_wraps += 1
    end
    app.previous_raw_index = raw
    extended = (app.index_wraps << 32) + UInt64(raw)
    isnothing(app.first_index) && (app.first_index = extended)
    return extended
end

function _zero_relative_angle(angle::Float32, zero::Float32)
    return mod(angle - zero + 180.0f0, 360.0f0) - 180.0f0
end

function _angle_plot_limits(pitch, yaw)
    absolute_maximum = max(maximum(abs, pitch), maximum(abs, yaw))
    extent = absolute_maximum + ANGLE_MARGIN_DEGREES
    return -Float64(extent), Float64(extent)
end

function append_packet!(app::WhirlApp, header, values)
    size(values, 2) == length(WHIRL_SOURCES) ||
        throw(ArgumentError("expected three whirl sources, received $(size(values, 2))"))
    for row in axes(values, 1)
        offset = UInt32(mod(UInt64(row - 1) * UInt64(header.decimation), UInt64(1) << 32))
        extended = _record_index!(app, header.first_index + offset)
        pitch = 360.0f0 * values[row, 1]
        yaw = 360.0f0 * values[row, 2]
        isnothing(app.pitch_zero_degrees) && (app.pitch_zero_degrees = pitch)
        isnothing(app.yaw_zero_degrees) && (app.yaw_zero_degrees = yaw)
        push!(app.sample_indices, extended)
        push!(app.times, (extended - app.first_index) / app.sample_rate)
        push!(app.pitch_degrees, _zero_relative_angle(pitch, app.pitch_zero_degrees))
        push!(app.yaw_degrees, _zero_relative_angle(yaw, app.yaw_zero_degrees))
        push!(app.rpm, values[row, 3])
    end
    app.dropped = header.dropped
    isnothing(app.receiver) || (app.lost_packets = app.receiver.lost_packets)
    return nothing
end

function append_demo_chunk!(app::WhirlApp)
    count = max(1, round(Int, 0.02 * app.sample_rate / app.decimation))
    for _ in 1:count
        index = app.demo_index
        time_s = index / app.sample_rate
        pitch = 360 * mod(0.17 * time_s + 0.025 * sinpi(0.7 * time_s), 1)
        yaw = 360 * mod(0.11 * time_s + 0.018 * sinpi(1.1 * time_s + 0.3), 1)
        speed = 4_100 + 650 * sinpi(0.18 * time_s) + 120 * sinpi(1.7 * time_s)
        isnothing(app.pitch_zero_degrees) && (app.pitch_zero_degrees = Float32(pitch))
        isnothing(app.yaw_zero_degrees) && (app.yaw_zero_degrees = Float32(yaw))
        push!(app.sample_indices, index)
        push!(app.times, time_s)
        push!(app.pitch_degrees, _zero_relative_angle(Float32(pitch), app.pitch_zero_degrees))
        push!(app.yaw_degrees, _zero_relative_angle(Float32(yaw), app.yaw_zero_degrees))
        push!(app.rpm, Float32(speed))
        app.demo_index += UInt64(app.decimation)
    end
    return nothing
end

function _check_whirl_device(device::Device)
    experiment = try
        String(device[:experiment])
    catch error
        error isa DeviceError ? "unknown" : rethrow()
    end
    experiment == "whirl-rig" ||
        throw(DeviceError("connected experiment is '$experiment', expected 'whirl-rig'"))
    available = Set(source.name for source in device.sources)
    missing = [String(source) for source in WHIRL_SOURCES if String(source) ∉ available]
    isempty(missing) || throw(DeviceError("firmware is missing sources: $(join(missing, ", "))"))
    return nothing
end

function start_receiving!(app::WhirlApp)
    (app.running || app.busy) && return nothing
    app.busy = true
    app.session += 1
    session = app.session
    app.status_text[] = app.demo ? "Starting demo…" : "Connecting…"
    app.info_text[] = app.demo ? "Generating synthetic whirl data" : "Opening TCP control and UDP stream"
    try
        if app.demo
            app.running = true
            app.status_text[] = "Receiving · DEMO"
            app.info_text[] = "Synthetic 2 kHz source; no MCU required"
            app.busy = false
            @async demo_loop!(app, session)
            return nothing
        end

        if isnothing(app.device) || !isopen(app.device)
            app.device = Device(app.host; timeout = 3.0)
        end
        _check_whirl_device(app.device)
        device_status = status(app.device)
        app.sample_rate = Float64(device_status.sample_rate)
        configure_stream!(app.device, WHIRL_SOURCES; decimation = app.decimation, count = 0)
        receiver = StreamReceiver(; port = 0, timeout = STREAM_TIMEOUT_SECONDS)
        app.receiver = receiver
        prime!(receiver, app.host)
        if session != app.session
            _close_receiver!(app)
            _close_device!(app)
            app.busy = false
            return nothing
        end
        start_stream!(app.device, receiver.port)
        if session != app.session
            try
                stop_stream!(app.device)
            finally
                _close_receiver!(app)
                _close_device!(app)
                app.busy = false
            end
            return nothing
        end
        app.running = true
        app.status_text[] = "Receiving · LIVE"
        app.info_text[] = "$(app.sample_rate) Hz firmware · decimation $(app.decimation)"
        app.busy = false
        @async receive_loop!(app, session)
        @async keepalive_loop!(app, session)
    catch error
        app.running = false
        app.busy = false
        _close_receiver!(app)
        _close_device!(app)
        app.status_text[] = "Connection error"
        app.info_text[] = sprint(showerror, error)
    end
    return nothing
end

function keepalive_loop!(app::WhirlApp, session::UInt64)
    while app.running && session == app.session
        sleep(CONTROL_KEEPALIVE_SECONDS)
        app.running && session == app.session || break
        try
            status(app.device)
        catch error
            app.running && session == app.session || break
            app.running = false
            app.status_text[] = "Control error"
            app.info_text[] = "Control heartbeat failed: $(sprint(showerror, error))"
            _close_receiver!(app)
            _close_device!(app)
            break
        end
    end
    return nothing
end

function demo_loop!(app::WhirlApp, session::UInt64)
    while app.running && session == app.session
        append_demo_chunk!(app)
        sleep(0.02)
    end
    return nothing
end

function _restart_stream!(app::WhirlApp, session::UInt64, attempt::Int)
    app.status_text[] = "Reconnecting…"
    app.info_text[] = "No packet for $(STREAM_TIMEOUT_SECONDS) s; restart $attempt/$MAX_STREAM_RESTARTS"
    if isnothing(app.device) || !isopen(app.device)
        throw(DeviceError("control connection closed during stream restart"))
    end
    stop_stream!(app.device)
    _close_receiver!(app)
    session == app.session || return false
    receiver = StreamReceiver(; port = 0, timeout = STREAM_TIMEOUT_SECONDS)
    app.receiver = receiver
    prime!(receiver, app.host)
    start_stream!(app.device, receiver.port)
    app.status_text[] = "Receiving · LIVE"
    app.info_text[] = "Stream recovered automatically · decimation $(app.decimation)"
    return true
end

function receive_loop!(app::WhirlApp, session::UInt64)
    restart_attempts = 0
    while app.running && session == app.session
        try
            header, values = receive(app.receiver)
            session == app.session || break
            append_packet!(app, header, values)
            restart_attempts = 0
        catch error
            if error isa StreamTimeout && restart_attempts < MAX_STREAM_RESTARTS
                restart_attempts += 1
                try
                    _restart_stream!(app, session, restart_attempts) || break
                    continue
                catch restart_error
                    error = restart_error
                end
            end
            app.running && session == app.session || break
            app.running = false
            app.status_text[] = "Stream error"
            app.info_text[] = sprint(showerror, error)
            _close_receiver!(app)
            _close_device!(app)
            break
        end
    end
    return nothing
end

function pause_receiving!(app::WhirlApp)
    (!app.running && !app.busy) && return nothing
    app.session += 1
    was_running = app.running
    app.running = false
    app.busy = false
    if !app.demo && was_running && !isnothing(app.device) && isopen(app.device)
        try
            stop_stream!(app.device)
        catch error
            app.info_text[] = "Paused; control stop reported: $(sprint(showerror, error))"
            _close_device!(app)
        end
    end
    _close_receiver!(app)
    app.status_text[] = "Paused"
    app.info_text[] = "Stored data are retained; Start resumes acquisition"
    return nothing
end

function clear_data!(app::WhirlApp)
    empty!(app.times)
    empty!(app.sample_indices)
    empty!(app.pitch_degrees)
    empty!(app.yaw_degrees)
    empty!(app.rpm)
    app.pitch_zero_degrees = nothing
    app.yaw_zero_degrees = nothing
    app.first_index = nothing
    app.previous_raw_index = nothing
    app.index_wraps = 0
    app.demo_index = 0
    app.dropped = 0
    app.lost_packets = 0
    app.plot_time[] = Float64[]
    app.plot_pitch_points[] = Point2f[]
    app.plot_yaw_points[] = Point2f[]
    app.plot_rpm_points[] = Point2f[]
    ylims!(app.angle_axis, -ANGLE_MARGIN_DEGREES, ANGLE_MARGIN_DEGREES)
    app.status_text[] = app.running ? app.status_text[] : "Idle"
    app.info_text[] = app.running ? "Buffer cleared; acquisition continues" : "Buffer cleared"
    return nothing
end

function _default_save_path(
        directory = joinpath(@__DIR__, "..", "captures");
        timestamp = now(),
    )
    stem = "whirl_capture_$(Dates.format(timestamp, "yyyymmdd_HHMMSS_sss"))"
    path = joinpath(directory, "$stem.csv")
    suffix = 2
    while ispath(path)
        path = joinpath(directory, "$(stem)_$suffix.csv")
        suffix += 1
    end
    return path
end

function save_csv!(app::WhirlApp; default_directory = joinpath(@__DIR__, "..", "captures"))
    isempty(app.times) && begin
        app.info_text[] = "Nothing to save yet"
        return nothing
    end
    requested = app.save_path.stored_string[]
    automatic_path = isnothing(requested) || isempty(strip(requested))
    path = automatic_path ? _default_save_path(default_directory) : abspath(expanduser(strip(requested)))
    times = copy(app.times)
    indices = copy(app.sample_indices)
    pitch = copy(app.pitch_degrees)
    yaw = copy(app.yaw_degrees)
    speeds = copy(app.rpm)
    try
        mkpath(dirname(path))
        open(path, "w") do io
            println(io, "time_s,sample_index,pitch_deg,yaw_deg,rpm")
            for row in eachindex(times)
                println(io, times[row], ',', indices[row], ',', pitch[row], ',', yaw[row], ',', speeds[row])
            end
        end
        app.info_text[] = "Saved $(length(times)) samples to $path"
    catch error
        app.status_text[] = "Save error"
        app.info_text[] = sprint(showerror, error)
    end
    return nothing
end

function update_plots!(app::WhirlApp)
    count = length(app.times)
    app.stats_text[] = "$(count) samples\nDevice drops: $(app.dropped)\nUDP gaps: $(app.lost_packets)"
    count == 0 && return nothing
    first_visible = searchsortedfirst(app.times, max(0.0, app.times[end] - app.window_seconds))
    stride = max(1, cld(count - first_visible + 1, MAX_PLOT_POINTS))
    selection = first_visible:stride:count
    plot_time = app.times[selection]
    plot_pitch = app.pitch_degrees[selection]
    plot_yaw = app.yaw_degrees[selection]
    # A single point-vector observable keeps x/y lengths atomic for GLMakie's
    # asynchronous renderer and avoids the mismatch storm caused by separate
    # x and y observables.
    app.plot_time[] = plot_time
    app.plot_pitch_points[] = Point2f.(plot_time, plot_pitch)
    app.plot_yaw_points[] = Point2f.(plot_time, plot_yaw)
    app.plot_rpm_points[] = Point2f.(plot_time, app.rpm[selection])
    angle_lower, angle_upper = _angle_plot_limits(
        @view(app.pitch_degrees[first_visible:count]),
        @view(app.yaw_degrees[first_visible:count]),
    )
    ylims!(app.angle_axis, angle_lower, angle_upper)
    right = max(app.window_seconds, app.times[end])
    left = max(0.0, right - app.window_seconds)
    xlims!(app.angle_axis, left, right)
    xlims!(app.rpm_axis, left, right)
    return nothing
end

function _close_receiver!(app::WhirlApp)
    if !isnothing(app.receiver)
        isopen(app.receiver) && close(app.receiver)
        app.receiver = nothing
    end
    return nothing
end

function _close_device!(app::WhirlApp)
    if !isnothing(app.device)
        isopen(app.device) && close(app.device)
        app.device = nothing
    end
    return nothing
end

function shutdown!(app::WhirlApp)
    app.session += 1
    app.running = false
    app.busy = false
    _close_receiver!(app)
    _close_device!(app)
    return nothing
end

function parse_options(arguments)
    host = get(ENV, "HELIC_DAQ_HOST", DEFAULT_HOST)
    demo = false
    decimation = 1
    window_seconds = 10.0
    index = 1
    while index <= length(arguments)
        argument = arguments[index]
        if argument == "--demo"
            demo = true
        elseif argument in ("--host", "--decimation", "--window")
            index == length(arguments) && error("$argument requires a value")
            index += 1
            value = arguments[index]
            argument == "--host" && (host = value)
            argument == "--decimation" && (decimation = parse(Int, value))
            argument == "--window" && (window_seconds = parse(Float64, value))
        elseif argument in ("-h", "--help")
            println("Usage: julia --project=. src/gui.jl [--demo] [--host IP] [--decimation N] [--window SECONDS]")
            return nothing
        else
            error("unknown argument: $argument")
        end
        index += 1
    end
    decimation > 0 || error("--decimation must be positive")
    decimation <= typemax(UInt16) || error("--decimation must fit a UInt16")
    window_seconds > 0 || error("--window must be positive")
    return (; host, demo, decimation, window_seconds)
end

function main(arguments = ARGS)
    options = parse_options(arguments)
    isnothing(options) && return nothing
    GLMakie.activate!()
    app = WhirlApp(options.host, options.demo, options.decimation, options.window_seconds)
    figure = build_figure!(app)
    screen = display(figure)
    # `window_open` is false until GLMakie creates the screen. Starting this
    # task before `display` can make it exit immediately, leaving live data in
    # memory without ever refreshing the plotted observables.
    @async begin
        while events(figure).window_open[]
            update_plots!(app)
            sleep(PLOT_REFRESH_SECONDS)
        end
    end
    wait(screen)
    shutdown!(app)
    return nothing
end

abspath(PROGRAM_FILE) == abspath(@__FILE__) && main()
