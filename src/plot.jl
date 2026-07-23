# Offline CSV plotting for captures saved by the whirl-rig live GUI.

using GLMakie

const WHIRL_CSV_COLUMNS = ("time_s", "sample_index", "pitch_deg", "yaw_deg", "rpm")
const MAX_OFFLINE_PLOT_POINTS = 100_000
const OFFLINE_ANGLE_MARGIN_DEGREES = 10.0

function _parse_csv_value(::Type{T}, text, row, column) where {T}
    value = tryparse(T, strip(text))
    isnothing(value) && throw(ArgumentError("invalid $column value on CSV row $row: '$text'"))
    return value
end

"""Read a CSV file written by `gui.jl` and return its five typed columns."""
function read_whirl_csv(path::AbstractString)
    csv_path = abspath(expanduser(path))
    isfile(csv_path) || throw(ArgumentError("CSV file does not exist: $csv_path"))

    times = Float64[]
    indices = UInt64[]
    pitch = Float64[]
    yaw = Float64[]
    rpm = Float64[]
    open(csv_path, "r") do io
        eof(io) && throw(ArgumentError("CSV file is empty: $csv_path"))
        columns = Tuple(strip.(split(readline(io), ',')))
        columns == WHIRL_CSV_COLUMNS || throw(
            ArgumentError(
                "expected CSV columns $(join(WHIRL_CSV_COLUMNS, ',')); received $(join(columns, ','))",
            ),
        )
        for (offset, line) in enumerate(eachline(io))
            line_number = offset + 1
            isempty(strip(line)) && continue
            fields = split(line, ',')
            length(fields) == length(WHIRL_CSV_COLUMNS) || throw(
                ArgumentError(
                    "expected $(length(WHIRL_CSV_COLUMNS)) fields on CSV row $line_number; received $(length(fields))",
                ),
            )
            time = _parse_csv_value(Float64, fields[1], line_number, "time_s")
            sample_index = _parse_csv_value(UInt64, fields[2], line_number, "sample_index")
            pitch_value = _parse_csv_value(Float64, fields[3], line_number, "pitch_deg")
            yaw_value = _parse_csv_value(Float64, fields[4], line_number, "yaw_deg")
            rpm_value = _parse_csv_value(Float64, fields[5], line_number, "rpm")
            all(isfinite, (time, pitch_value, yaw_value, rpm_value)) ||
                throw(ArgumentError("non-finite value on CSV row $line_number"))
            push!(times, time)
            push!(indices, sample_index)
            push!(pitch, pitch_value)
            push!(yaw, yaw_value)
            push!(rpm, rpm_value)
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

"""Plot pitch, yaw, and rotor speed from a whirl GUI CSV capture.

Set `output` to a `.png`, `.pdf`, or `.svg` path to save the figure. The
returned `Figure` can also be displayed or further customised by the caller.
"""
function plot_whirl_csv(path::AbstractString; output::Union{Nothing, AbstractString} = nothing)
    data = read_whirl_csv(path)
    selection = _offline_selection(length(data.time_s))

    figure = Figure(; size = (1400, 850), figure_padding = 24)
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
    linkxaxes!(angle_axis, rpm_axis)
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
    )
    axislegend(angle_axis; position = :lb, framevisible = false, orientation = :horizontal)
    angle_lower, angle_upper = _offline_angle_plot_limits(data.pitch_deg, data.yaw_deg)
    rpm_lower, rpm_upper = _expanded_limits(data.rpm; minimum_margin = 10.0)
    ylims!(angle_axis, angle_lower, angle_upper)
    ylims!(rpm_axis, rpm_lower, rpm_upper)
    x_lower, x_upper = _expanded_limits(data.time_s; fraction = 0.0, minimum_margin = 0.001)
    xlims!(angle_axis, x_lower, x_upper)
    rowgap!(figure.layout, 12)
    rowsize!(figure.layout, 3, Relative(0.5))
    rowsize!(figure.layout, 4, Relative(0.5))

    if !isnothing(output)
        output_path = abspath(expanduser(output))
        mkpath(dirname(output_path))
        save(output_path, figure)
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
