# Offline CSV plotting for captures saved by the whirl-rig live GUI.
# julia --project=. src/plot.jl captures/example.csv --plot rpm

using GLMakie
import CairoMakie

const LEGACY_WHIRL_CSV_COLUMNS = ("time_s", "sample_index", "pitch_deg", "yaw_deg", "rpm")
const WHIRL_CSV_COLUMNS = (
    LEGACY_WHIRL_CSV_COLUMNS...,
    "rpm_target",
    "esc_throttle",
    "ctrl_saturated",
)
const MAX_OFFLINE_PLOT_POINTS = 100_000
const OFFLINE_ANGLE_MARGIN_DEGREES = 10.0
const OFFLINE_ORBIT_MINIMUM_EXTENT_DEGREES = 1.0
const OFFLINE_ORBIT_MARGIN_FRACTION = 0.1
const OFFLINE_INTERACTIVE_FIGURE_SIZE = (1280, 800)
const OFFLINE_EXPORT_FIGURE_SIZE = (1540, 1080)
const SUPPORTED_OUTPUT_EXTENSIONS = (".png", ".pdf", ".svg")
const OFFLINE_PLOTS = (:angles, :rpm, :pitch_yaw, :esc)
const OFFLINE_PLOT_USAGE = """Usage: julia --project=. src/plot.jl CAPTURE.csv [OUTPUT] [OPTIONS]

Options:
  --plot NAME[,NAME...]  Plot angles, rpm, pitch-yaw, or esc; may be repeated
  --start SECONDS        Include samples at or after this capture time
  --end SECONDS          Include samples at or before this capture time
  --output PATH          Save to PNG, PDF, or SVG instead of opening a window
  -h, --help             Show this help
"""

function _parse_csv_value(::Type{T}, text, row, column) where {T}
    value = tryparse(T, strip(text))
    isnothing(value) && throw(ArgumentError("invalid $column value on CSV row $row: '$text'"))
    return value
end

"""Read a CSV file written by `gui.jl` and return its typed columns."""
function read_whirl_csv(path::AbstractString)
    csv_path = abspath(expanduser(path))
    isfile(csv_path) || throw(ArgumentError("CSV file does not exist: $csv_path"))

    times = Float64[]
    indices = UInt64[]
    pitch = Float64[]
    yaw = Float64[]
    rpm = Float64[]
    rpm_target = Float64[]
    esc_throttle = Float64[]
    ctrl_saturated = Float64[]
    open(csv_path, "r") do io
        eof(io) && throw(ArgumentError("CSV file is empty: $csv_path"))
        columns = Tuple(strip.(split(readline(io), ',')))
        columns in (LEGACY_WHIRL_CSV_COLUMNS, WHIRL_CSV_COLUMNS) || throw(
            ArgumentError(
                "expected legacy or motor-control whirl CSV columns; received $(join(columns, ','))",
            ),
        )
        has_motor_control = columns == WHIRL_CSV_COLUMNS
        for (offset, line) in enumerate(eachline(io))
            line_number = offset + 1
            isempty(strip(line)) && continue
            fields = split(line, ',')
            length(fields) == length(columns) || throw(
                ArgumentError(
                    "expected $(length(columns)) fields on CSV row $line_number; received $(length(fields))",
                ),
            )
            time = _parse_csv_value(Float64, fields[1], line_number, "time_s")
            sample_index = _parse_csv_value(UInt64, fields[2], line_number, "sample_index")
            pitch_value = _parse_csv_value(Float64, fields[3], line_number, "pitch_deg")
            yaw_value = _parse_csv_value(Float64, fields[4], line_number, "yaw_deg")
            rpm_value = _parse_csv_value(Float64, fields[5], line_number, "rpm")
            target_value = has_motor_control ?
                _parse_csv_value(Float64, fields[6], line_number, "rpm_target") : 0.0
            throttle_value = has_motor_control ?
                _parse_csv_value(Float64, fields[7], line_number, "esc_throttle") : 0.0
            saturated_value = has_motor_control ?
                _parse_csv_value(Float64, fields[8], line_number, "ctrl_saturated") : 0.0
            all(
                isfinite,
                (time, pitch_value, yaw_value, rpm_value, target_value, throttle_value, saturated_value),
            ) ||
                throw(ArgumentError("non-finite value on CSV row $line_number"))
            push!(times, time)
            push!(indices, sample_index)
            push!(pitch, pitch_value)
            push!(yaw, yaw_value)
            push!(rpm, rpm_value)
            push!(rpm_target, target_value)
            push!(esc_throttle, throttle_value)
            push!(ctrl_saturated, saturated_value)
        end
    end
    isempty(times) && throw(ArgumentError("CSV file contains no samples: $csv_path"))
    issorted(times) || throw(ArgumentError("time_s must be in ascending order"))
    return (
        path = csv_path,
        time_s = times,
        sample_index = indices,
        pitch_deg = pitch,
        yaw_deg = yaw,
        rpm = rpm,
        rpm_target = rpm_target,
        esc_throttle = esc_throttle,
        ctrl_saturated = ctrl_saturated,
    )
end

function _offline_selection(count)
    stride = max(1, cld(count, MAX_OFFLINE_PLOT_POINTS))
    selection = collect(1:stride:count)
    selection[end] == count || push!(selection, count)
    return selection
end

function _time_interval(times; start_time = nothing, end_time = nothing)
    lower = isnothing(start_time) ? first(times) : Float64(start_time)
    upper = isnothing(end_time) ? last(times) : Float64(end_time)
    isfinite(lower) || throw(ArgumentError("start time must be finite"))
    isfinite(upper) || throw(ArgumentError("end time must be finite"))
    lower <= upper || throw(ArgumentError("start time must not exceed end time"))

    first_index = searchsortedfirst(times, lower)
    last_index = searchsortedlast(times, upper)
    first_index <= last_index || throw(
        ArgumentError(
            "the requested time range [$lower, $upper] s contains no capture samples",
        ),
    )
    return first_index:last_index
end

function _plot_name(name)
    text = lowercase(replace(strip(String(name)), '_' => '-', '–' => '-'))
    text in ("angle", "angles", "encoder") && return :angles
    text in ("rpm", "speed", "rotor-speed") && return :rpm
    text in ("pitch-yaw", "pitchyaw", "orbit") && return :pitch_yaw
    text in ("esc", "throttle", "esc-command") && return :esc
    text == "all" && return :all
    throw(
        ArgumentError(
            "unknown plot '$name'; expected angles, rpm, pitch-yaw, esc, or all",
        ),
    )
end

function _normalise_plots(plots)
    requested = plots isa Union{AbstractString, Symbol} ? (plots,) : plots
    names = unique(_plot_name(name) for name in requested)
    isempty(names) && throw(ArgumentError("at least one plot must be selected"))
    :all in names && return OFFLINE_PLOTS
    return Tuple(name for name in OFFLINE_PLOTS if name in names)
end

function _panel_cell(layout, index, count)
    if count == 1
        return layout[1, 1:2]
    elseif count == 2
        return layout[1, index]
    elseif count == 3
        return index <= 2 ? layout[1, index] : layout[2, 1:2]
    elseif index == 1
        return layout[1, 1:2]
    elseif index == 2
        return layout[2, 1]
    elseif index == 3
        return layout[2, 2]
    end
    return layout[3, 1:2]
end

function _expanded_limits(values...; fraction = 0.05, minimum_margin = 1.0)
    lower = minimum(minimum(value) for value in values)
    upper = maximum(maximum(value) for value in values)
    margin = max(minimum_margin, fraction * (upper - lower))
    return lower - margin, upper + margin
end

function _offline_angle_plot_limits(pitch, yaw)
    absolute_maximum = max(maximum(abs, pitch), maximum(abs, yaw))
    extent = absolute_maximum + OFFLINE_ANGLE_MARGIN_DEGREES
    return -extent, extent
end

function _offline_orbit_plot_limits(pitch, yaw)
    absolute_maximum = max(maximum(abs, pitch), maximum(abs, yaw))
    extent = max(
        OFFLINE_ORBIT_MINIMUM_EXTENT_DEGREES,
        (1 + OFFLINE_ORBIT_MARGIN_FRACTION) * absolute_maximum,
    )
    return -extent, extent
end

function _output_backend(path::AbstractString)
    extension = lowercase(splitext(path)[2])
    extension == ".png" && return GLMakie
    extension in (".pdf", ".svg") && return CairoMakie
    throw(
        ArgumentError(
            "unsupported output extension '$extension'; expected one of " *
                join(SUPPORTED_OUTPUT_EXTENSIONS, ", "),
        ),
    )
end

"""Plot selected signals from a whirl GUI CSV capture.

Use `plots` to select any of `:angles`, `:rpm`, `:pitch_yaw`, and `:esc`.
`start_time` and `end_time` bound the plotted capture time in seconds,
inclusively. Set `output` to a `.png`, `.pdf`, or `.svg` path to save.
"""
function plot_whirl_csv(
    path::AbstractString;
    output::Union{Nothing, AbstractString} = nothing,
    plots = OFFLINE_PLOTS,
    start_time = nothing,
    end_time = nothing,
)
    data = read_whirl_csv(path)
    plot_names = _normalise_plots(plots)
    interval = _time_interval(data.time_s; start_time, end_time)
    offsets = _offline_selection(length(interval))
    selection = first(interval) .+ offsets .- 1
    time_values = @view data.time_s[interval]
    pitch_values = @view data.pitch_deg[interval]
    yaw_values = @view data.yaw_deg[interval]
    rpm_values = @view data.rpm[interval]
    target_values = @view data.rpm_target[interval]

    figure_size = isnothing(output) ? OFFLINE_INTERACTIVE_FIGURE_SIZE : OFFLINE_EXPORT_FIGURE_SIZE
    figure = Figure(; size = figure_size, figure_padding = 24)
    Label(
        figure[1, 1],
        "Whirl Rig Capture · $(basename(data.path))";
        fontsize = 25,
        font = :bold,
        tellwidth = false,
    )
    first_time = first(time_values)
    last_time = last(time_values)
    Label(
        figure[2, 1],
        "$(length(interval)) samples · $(round(first_time; digits = 3))–$(round(last_time; digits = 3)) s";
        color = RGBf(0.35, 0.42, 0.52),
        tellwidth = false,
    )

    panel_layout = GridLayout(; rowgap = 12, colgap = 18)
    figure[3, 1] = panel_layout
    time_axes = Any[]
    x_lower, x_upper = _expanded_limits(time_values; fraction = 0.0, minimum_margin = 0.001)

    for (index, name) in enumerate(plot_names)
        cell = _panel_cell(panel_layout, index, length(plot_names))
        if name == :angles
            axis = Axis(
                cell;
                title = "Encoder angle",
                xlabel = "Time [s]",
                ylabel = "Relative angle [deg]",
                xgridcolor = RGBf(0.87, 0.89, 0.93),
                ygridcolor = RGBf(0.87, 0.89, 0.93),
            )
            lines!(
                axis,
                data.time_s[selection],
                data.pitch_deg[selection];
                color = RGBf(0.1, 0.42, 0.9),
                linewidth = 2,
                label = "Pitch",
            )
            lines!(
                axis,
                data.time_s[selection],
                data.yaw_deg[selection];
                color = RGBf(0.92, 0.28, 0.32),
                linewidth = 2,
                label = "Yaw",
            )
            axislegend(axis; position = :lb, framevisible = false, orientation = :horizontal)
            ylims!(axis, _offline_angle_plot_limits(pitch_values, yaw_values)...)
            push!(time_axes, axis)
        elseif name == :rpm
            axis = Axis(
                cell;
                title = "Rotor speed",
                xlabel = "Time [s]",
                ylabel = "Speed [RPM]",
                xgridcolor = RGBf(0.87, 0.89, 0.93),
                ygridcolor = RGBf(0.87, 0.89, 0.93),
            )
            lines!(
                axis,
                data.time_s[selection],
                data.rpm[selection];
                color = RGBf(0.12, 0.66, 0.46),
                linewidth = 2.2,
                label = "Measured",
            )
            lines!(
                axis,
                data.time_s[selection],
                data.rpm_target[selection];
                color = RGBf(0.93, 0.45, 0.12),
                linewidth = 2,
                linestyle = :dash,
                label = "Target",
            )
            axislegend(axis; position = :lb, framevisible = false, orientation = :horizontal)
            ylims!(axis, _expanded_limits(rpm_values, target_values; minimum_margin = 10.0)...)
            push!(time_axes, axis)
        elseif name == :pitch_yaw
            axis = Axis(
                cell;
                title = "Pitch–yaw orbit (orange = latest)",
                xlabel = "Pitch [deg]",
                ylabel = "Yaw [deg]",
                aspect = DataAspect(),
                xgridcolor = RGBf(0.87, 0.89, 0.93),
                ygridcolor = RGBf(0.87, 0.89, 0.93),
            )
            hlines!(axis, [0.0]; color = RGBf(0.68, 0.71, 0.76), linewidth = 1, linestyle = :dot)
            vlines!(axis, [0.0]; color = RGBf(0.68, 0.71, 0.76), linewidth = 1, linestyle = :dot)
            lines!(
                axis,
                data.pitch_deg[selection],
                data.yaw_deg[selection];
                color = RGBf(0.43, 0.25, 0.78),
                linewidth = 2,
            )
            scatter!(
                axis,
                [data.pitch_deg[last(interval)]],
                [data.yaw_deg[last(interval)]];
                color = RGBf(0.93, 0.45, 0.12),
                markersize = 11,
                strokecolor = :white,
                strokewidth = 1.5,
            )
            orbit_limits = _offline_orbit_plot_limits(pitch_values, yaw_values)
            xlims!(axis, orbit_limits...)
            ylims!(axis, orbit_limits...)
        else
            axis = Axis(
                cell;
                title = "ESC command",
                xlabel = "Time [s]",
                ylabel = "Throttle fraction",
                xgridcolor = RGBf(0.87, 0.89, 0.93),
                ygridcolor = RGBf(0.87, 0.89, 0.93),
            )
            lines!(
                axis,
                data.time_s[selection],
                data.esc_throttle[selection];
                color = RGBf(0.43, 0.25, 0.78),
                linewidth = 2,
            )
            ylims!(axis, -0.05, 1.05)
            push!(time_axes, axis)
        end
    end

    length(time_axes) > 1 && linkxaxes!(time_axes...)
    for axis in time_axes
        xlims!(axis, x_lower, x_upper)
    end
    rowsize!(figure.layout, 3, Auto(1))
    row_count = length(plot_names) <= 2 ? 1 : length(plot_names) == 3 ? 2 : 3
    for row in 1:row_count
        ratio = length(plot_names) == 4 && row == 3 ? 0.6 : 1.0
        rowsize!(panel_layout, row, Auto(ratio))
    end

    if !isnothing(output)
        output_path = abspath(expanduser(output))
        backend = _output_backend(output_path)
        mkpath(dirname(output_path))
        save(output_path, figure; backend)
    end
    return figure
end

function _parse_option_time(text, option)
    value = tryparse(Float64, text)
    isnothing(value) && throw(ArgumentError("$option requires a numeric value"))
    isfinite(value) || throw(ArgumentError("$option requires a finite value"))
    return value
end

function _parse_plot_arguments(arguments)
    input = nothing
    output = nothing
    plot_values = String[]
    plot_option_seen = false
    start_time = nothing
    end_time = nothing
    index = 1
    while index <= length(arguments)
        argument = arguments[index]
        if argument in ("-h", "--help")
            return (; help = true)
        elseif argument in ("--plot", "--plots", "--start", "--end", "--output")
            index < length(arguments) || throw(ArgumentError("$argument requires a value"))
            index += 1
            value = arguments[index]
            if argument in ("--plot", "--plots")
                append!(plot_values, split(value, ','))
                plot_option_seen = true
            elseif argument == "--start"
                start_time = _parse_option_time(value, argument)
            elseif argument == "--end"
                end_time = _parse_option_time(value, argument)
            else
                isnothing(output) || throw(ArgumentError("output path was specified more than once"))
                output = value
            end
        elseif startswith(argument, '-')
            throw(ArgumentError("unknown option: $argument"))
        elseif isnothing(input)
            input = argument
        elseif isnothing(output)
            output = argument
        else
            throw(ArgumentError("unexpected positional argument: $argument"))
        end
        index += 1
    end
    isnothing(input) && throw(ArgumentError("a capture CSV path is required"))
    plots = plot_option_seen ? _normalise_plots(plot_values) : OFFLINE_PLOTS
    return (; help = false, input, output, plots, start_time, end_time)
end

function _plot_main(arguments)
    options = try
        _parse_plot_arguments(arguments)
    catch error
        error isa ArgumentError || rethrow()
        println(stderr, "Error: $(sprint(showerror, error))\n")
        println(stderr, OFFLINE_PLOT_USAGE)
        return 2
    end
    if options.help
        print(OFFLINE_PLOT_USAGE)
        return 0
    end
    # Importing CairoMakie selects its non-interactive backend. Restore GLMakie
    # before constructing the figure that will own an interactive window.
    isnothing(options.output) && GLMakie.activate!()
    figure = plot_whirl_csv(
        options.input;
        output = options.output,
        plots = options.plots,
        start_time = options.start_time,
        end_time = options.end_time,
    )
    if isnothing(options.output)
        wait(display(figure))
    else
        println("Saved plot to $(abspath(expanduser(options.output)))")
    end
    return 0
end

if abspath(PROGRAM_FILE) == abspath(@__FILE__)
    exit(_plot_main(ARGS))
end
