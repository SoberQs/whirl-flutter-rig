# Offline CSV plotting for captures saved by the whirl-rig live GUI.

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
const SUPPORTED_OUTPUT_EXTENSIONS = (".png", ".pdf", ".svg")

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

"""Plot angles, rotor speed, and the ESC command from a whirl GUI CSV capture.

Set `output` to a `.png`, `.pdf`, or `.svg` path to save the figure. The
returned `Figure` can also be displayed or further customised by the caller.
"""
function plot_whirl_csv(path::AbstractString; output::Union{Nothing, AbstractString} = nothing)
    data = read_whirl_csv(path)
    selection = _offline_selection(length(data.time_s))

    figure = Figure(; size = (1400, 980), figure_padding = 24)
    Label(
        figure[1, 1],
        "Whirl Rig Capture · $(basename(data.path))";
        fontsize = 25,
        font = :bold,
        tellwidth = false,
    )
    Label(
        figure[2, 1],
        "$(length(data.time_s)) samples · $(round(data.time_s[end] - data.time_s[1]; digits = 3)) s";
        color = RGBf(0.35, 0.42, 0.52),
        tellwidth = false,
    )
    angle_axis = Axis(
        figure[3, 1];
        title = "Encoder angle",
        xlabel = "Time [s]",
        ylabel = "Relative angle [deg]",
        xgridcolor = RGBf(0.87, 0.89, 0.93),
        ygridcolor = RGBf(0.87, 0.89, 0.93),
    )
    rpm_axis = Axis(
        figure[4, 1];
        title = "Rotor speed",
        xlabel = "Time [s]",
        ylabel = "Speed [RPM]",
        xgridcolor = RGBf(0.87, 0.89, 0.93),
        ygridcolor = RGBf(0.87, 0.89, 0.93),
    )
    throttle_axis = Axis(
        figure[5, 1];
        title = "ESC command",
        xlabel = "Time [s]",
        ylabel = "Throttle fraction",
        xgridcolor = RGBf(0.87, 0.89, 0.93),
        ygridcolor = RGBf(0.87, 0.89, 0.93),
    )
    linkxaxes!(angle_axis, rpm_axis, throttle_axis)
    lines!(
        angle_axis,
        data.time_s[selection],
        data.pitch_deg[selection];
        color = RGBf(0.1, 0.42, 0.9),
        linewidth = 2,
        label = "Pitch",
    )
    lines!(
        angle_axis,
        data.time_s[selection],
        data.yaw_deg[selection];
        color = RGBf(0.92, 0.28, 0.32),
        linewidth = 2,
        label = "Yaw",
    )
    lines!(
        rpm_axis,
        data.time_s[selection],
        data.rpm[selection];
        color = RGBf(0.12, 0.66, 0.46),
        linewidth = 2.2,
        label = "Measured",
    )
    lines!(
        rpm_axis,
        data.time_s[selection],
        data.rpm_target[selection];
        color = RGBf(0.93, 0.45, 0.12),
        linewidth = 2,
        linestyle = :dash,
        label = "Target",
    )
    lines!(
        throttle_axis,
        data.time_s[selection],
        data.esc_throttle[selection];
        color = RGBf(0.43, 0.25, 0.78),
        linewidth = 2,
    )
    axislegend(angle_axis; position = :lb, framevisible = false, orientation = :horizontal)
    axislegend(rpm_axis; position = :lb, framevisible = false, orientation = :horizontal)
    angle_lower, angle_upper = _offline_angle_plot_limits(data.pitch_deg, data.yaw_deg)
    rpm_lower, rpm_upper = _expanded_limits(data.rpm, data.rpm_target; minimum_margin = 10.0)
    ylims!(angle_axis, angle_lower, angle_upper)
    ylims!(rpm_axis, rpm_lower, rpm_upper)
    ylims!(throttle_axis, -0.05, 1.05)
    x_lower, x_upper = _expanded_limits(data.time_s; fraction = 0.0, minimum_margin = 0.001)
    xlims!(angle_axis, x_lower, x_upper)
    rowgap!(figure.layout, 12)
    rowsize!(figure.layout, 3, Relative(0.5))
    rowsize!(figure.layout, 4, Relative(0.5))
    rowsize!(figure.layout, 5, Relative(0.3))

    if !isnothing(output)
        output_path = abspath(expanduser(output))
        backend = _output_backend(output_path)
        mkpath(dirname(output_path))
        save(output_path, figure; backend)
    end
    return figure
end

function _plot_main(arguments)
    1 <= length(arguments) <= 2 || begin
        println(stderr, "Usage: julia --project=. src/plot.jl CAPTURE.csv [OUTPUT.png]")
        return 2
    end
    output = length(arguments) == 2 ? arguments[2] : nothing
    figure = plot_whirl_csv(arguments[1]; output)
    if isnothing(output)
        wait(display(figure))
    else
        println("Saved plot to $(abspath(expanduser(output)))")
    end
    return 0
end

if abspath(PROGRAM_FILE) == abspath(@__FILE__)
    exit(_plot_main(ARGS))
end
