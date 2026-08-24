# Interactive live monitor and CSV recorder for the `whirl-rig` experiment.
# julia --project=. -e 'using Pkg; Pkg.instantiate()'
# julia --project=. src/gui.jl --demo

using Dates
using GLMakie
import HelicDAQ
using HelicDAQ: configure_stream!, start_stream!, stop_stream!
using Printf

include(joinpath(@__DIR__, "motor_control.jl"))
using .WhirlMotorControl

const DEFAULT_HOST = "192.168.1.238"
const WHIRL_SOURCES = (
    :pitch,
    :yaw,
    :rpm,
    :rpm_target_used,
    :ctrl_saturated,
    :esc_throttle,
)
const MAX_PLOT_POINTS = 2_000
const PLOT_REFRESH_SECONDS = 0.025
const STREAM_TIMEOUT_SECONDS = 5.0
const MAX_STREAM_RESTARTS = 3
const CONTROL_STATE_POLL_SECONDS = 0.5
const ANGLE_MARGIN_DEGREES = 10.0f0
const ORBIT_MINIMUM_EXTENT_DEGREES = 1.0
const ORBIT_MARGIN_FRACTION = 0.1
const RPM_MARGIN = 1_000.0
const MOTOR_COMMAND_INTERVAL_SECONDS = 0.05

mutable struct WhirlApp
    host::String
    demo::Bool
    decimation::Int
    window_seconds::Float64
    sample_rate::Float64
    running::Bool
    busy::Bool
    session::UInt64
    device::Union{Nothing, HelicDAQ.Device}
    receiver::Union{Nothing, HelicDAQ.StreamReceiver}
    times::Vector{Float64}
    sample_indices::Vector{UInt64}
    pitch_degrees::Vector{Float32}
    yaw_degrees::Vector{Float32}
    rpm::Vector{Float32}
    rpm_target::Vector{Float32}
    ctrl_saturated::Vector{Float32}
    esc_throttle::Vector{Float32}
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
    plot_target_points::Observable{Vector{Point2f}}
    plot_orbit_points::Observable{Vector{Point2f}}
    plot_orbit_current::Observable{Vector{Point2f}}
    status_text::Observable{String}
    stats_text::Observable{String}
    info_text::Observable{String}
    rpm_text::Observable{String}
    motor_text::Observable{String}
    target_range_text::Observable{String}
    manual_throttle::Float32
    target_rpm::Float32
    motor_mode::Symbol
    profile_set::MotorProfileSet
    motor_profile::MotorProfile
    armed::Bool
    tripped::Bool
    safety_flags::UInt32
    motor_busy::Bool
    last_motor_command_s::Float64
    angle_axis::Any
    rpm_axis::Any
    orbit_axis::Any
    save_path::Any
    target_rpm_box::Any
    profile_menu::Any
    figure::Any
end

function WhirlApp(host, demo, decimation, window_seconds)
    profile_set = load_motor_profiles()
    motor_profile = profile_by_id(profile_set, profile_set.default_id)
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
        Observable(Point2f[]),
        Observable(Point2f[]),
        Observable(Point2f[]),
        Observable("Idle"),
        Observable("0 samples"),
        Observable(demo ? "Demo mode — click Start" : "Target: $host"),
        Observable("— RPM"),
        Observable("DISARMED · MANUAL · 0%"),
        Observable("Target RPM: 0 or $(Int(motor_profile.target_min_rpm))–$(Int(motor_profile.target_max_rpm)) (press Enter)"),
        0.0f0,
        0.0f0,
        :manual,
        profile_set,
        motor_profile,
        false,
        false,
        UInt32(0),
        false,
        -Inf,
        nothing,
        nothing,
        nothing,
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
    figure = Figure(; size = (1540, 1080), figure_padding = 24)
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
    lower_plots = GridLayout(; colgap = 18)
    figure[4, 1] = lower_plots
    rpm_axis = Axis(
        lower_plots[1, 1];
        title = "Rotor speed",
        xlabel = "Time [s]",
        ylabel = "Speed [RPM]",
    )
    orbit_axis = Axis(
        lower_plots[1, 2];
        title = "Pitch–yaw orbit (orange = latest)",
        xlabel = "Pitch [deg]",
        ylabel = "Yaw [deg]",
        aspect = DataAspect(),
    )
    lines!(angle_axis, app.plot_pitch_points; color = RGBf(0.1, 0.42, 0.9), linewidth = 2, label = "Pitch")
    lines!(angle_axis, app.plot_yaw_points; color = RGBf(0.92, 0.28, 0.32), linewidth = 2, label = "Yaw")
    lines!(rpm_axis, app.plot_rpm_points; color = RGBf(0.12, 0.66, 0.46), linewidth = 2.2, label = "Measured")
    lines!(
        rpm_axis,
        app.plot_target_points;
        color = RGBf(0.93, 0.45, 0.12),
        linewidth = 2,
        linestyle = :dash,
        label = "Target",
    )
    hlines!(orbit_axis, [0.0]; color = RGBf(0.68, 0.71, 0.76), linewidth = 1, linestyle = :dot)
    vlines!(orbit_axis, [0.0]; color = RGBf(0.68, 0.71, 0.76), linewidth = 1, linestyle = :dot)
    lines!(
        orbit_axis,
        app.plot_orbit_points;
        color = RGBf(0.43, 0.25, 0.78),
        linewidth = 2,
    )
    scatter!(
        orbit_axis,
        app.plot_orbit_current;
        color = RGBf(0.93, 0.45, 0.12),
        markersize = 11,
        strokecolor = :white,
        strokewidth = 1.5,
    )
    axislegend(angle_axis; position = :lb, framevisible = false, orientation = :horizontal)
    axislegend(rpm_axis; position = :lb, framevisible = false, orientation = :horizontal)
    ylims!(angle_axis, -ANGLE_MARGIN_DEGREES, ANGLE_MARGIN_DEGREES)
    ylims!(rpm_axis, 0, 6_500)
    xlims!(angle_axis, 0, app.window_seconds)
    xlims!(rpm_axis, 0, app.window_seconds)
    xlims!(orbit_axis, -ORBIT_MINIMUM_EXTENT_DEGREES, ORBIT_MINIMUM_EXTENT_DEGREES)
    ylims!(orbit_axis, -ORBIT_MINIMUM_EXTENT_DEGREES, ORBIT_MINIMUM_EXTENT_DEGREES)
    colsize!(lower_plots, 1, Relative(0.5))
    colsize!(lower_plots, 2, Relative(0.5))

    controls = GridLayout(;
        tellwidth = true,
        width = 330,
        valign = :top,
        rowgap = 8,
    )
    figure[3:4, 2] = controls
    Label(
        controls[1, 1:2],
        "LIVE ROTOR SPEED";
        fontsize = 14,
        font = :bold,
        halign = :right,
        tellwidth = false,
        color = RGBf(0.32, 0.42, 0.58),
    )
    Label(
        controls[2, 1:2],
        app.rpm_text;
        fontsize = 32,
        font = :bold,
        halign = :right,
        tellwidth = false,
        color = RGBf(0.08, 0.52, 0.36),
    )
    Label(controls[3, 1:2], "Motor control"; fontsize = 21, font = :bold, halign = :left, tellwidth = false)
    Label(controls[4, 1:2], "Motor profile"; halign = :left, tellwidth = false, fontsize = 13)
    profile_options = [(profile.label, profile.id) for profile in app.profile_set.profiles]
    profile_menu = Menu(
        controls[5, 1:2];
        options = profile_options,
        default = app.motor_profile.label,
        width = 300,
        tellwidth = false,
    )
    manual_button = Button(controls[6, 1]; label = "Manual", width = 145, tellwidth = false)
    speed_button = Button(controls[6, 2]; label = "Closed loop", width = 145, tellwidth = false)
    Label(
        controls[7, 1:2],
        app.target_range_text;
        halign = :left,
        tellwidth = false,
        fontsize = 13,
    )
    target_rpm_box = Textbox(
        controls[8, 1:2];
        placeholder = "e.g. 2500",
        height = 40,
        tellwidth = false,
        halign = :left,
    )
    arm_button = Button(
        controls[9, 1];
        label = "ARM",
        width = 145,
        tellwidth = false,
        buttoncolor = RGBf(0.15, 0.67, 0.46),
        buttoncolor_hover = RGBf(0.11, 0.58, 0.39),
        labelcolor = :white,
        labelcolor_hover = :white,
    )
    stop_button = Button(
        controls[9, 2];
        label = "STOP / DISARM",
        width = 145,
        tellwidth = false,
        buttoncolor = RGBf(0.76, 0.16, 0.2),
        buttoncolor_hover = RGBf(0.62, 0.1, 0.14),
        labelcolor = :white,
        labelcolor_hover = :white,
    )
    Label(
        controls[10, 1:2],
        "Manual: ↑/↓ = ±1% · Space = STOP";
        halign = :left,
        tellwidth = false,
        fontsize = 13,
        color = RGBf(0.32, 0.38, 0.48),
    )
    Label(
        controls[11, 1:2],
        app.motor_text;
        halign = :left,
        tellwidth = false,
        font = :bold,
        color = RGBf(0.55, 0.12, 0.16),
    )
    Label(controls[12, 1:2], "Acquisition"; fontsize = 18, font = :bold, halign = :left, tellwidth = false)
    start_button = Button(
        controls[13, 1];
        label = "Start",
        height = 42,
        width = 145,
        tellwidth = false,
        buttoncolor = RGBf(0.15, 0.67, 0.46),
        buttoncolor_hover = RGBf(0.11, 0.58, 0.39),
        labelcolor = :white,
        labelcolor_hover = :white,
    )
    pause_button = Button(
        controls[13, 2];
        label = "Pause + Disarm",
        height = 42,
        width = 145,
        tellwidth = false,
        buttoncolor = RGBf(0.96, 0.65, 0.16),
        buttoncolor_hover = RGBf(0.88, 0.55, 0.1),
        labelcolor = :white,
        labelcolor_hover = :white,
    )
    save_button = Button(
        controls[14, 1];
        label = "Save CSV",
        height = 42,
        width = 145,
        tellwidth = false,
        buttoncolor = RGBf(0.16, 0.43, 0.78),
        buttoncolor_hover = RGBf(0.12, 0.35, 0.69),
        labelcolor = :white,
        labelcolor_hover = :white,
    )
    clear_button = Button(
        controls[14, 2];
        label = "Clear",
        height = 42,
        width = 145,
        tellwidth = false,
        buttoncolor = RGBf(0.76, 0.29, 0.34),
        buttoncolor_hover = RGBf(0.67, 0.22, 0.28),
        labelcolor = :white,
        labelcolor_hover = :white,
    )
    Label(controls[15, 1:2], "Optional save path (press Enter)"; halign = :left, tellwidth = false, color = RGBf(0.32, 0.38, 0.48))
    save_path = Textbox(
        controls[16, 1:2];
        placeholder = "auto: captures/whirl_*.csv",
        height = 42,
        tellwidth = false,
        halign = :left,
    )
    Label(controls[17, 1:2], app.status_text; fontsize = 18, font = :bold, halign = :left, tellwidth = false)
    Label(controls[18, 1:2], app.stats_text; halign = :left, tellwidth = false, color = RGBf(0.25, 0.32, 0.43))
    Label(
        controls[19, 1:2],
        app.info_text;
        halign = :left,
        tellwidth = false,
        width = 315,
        color = RGBf(0.25, 0.32, 0.43),
        justification = :left,
        word_wrap = true,
    )
    colsize!(figure.layout, 1, Relative(0.78))
    colsize!(figure.layout, 2, Fixed(340))
    rowsize!(figure.layout, 3, Relative(0.5))
    rowsize!(figure.layout, 4, Relative(0.5))

    app.angle_axis = angle_axis
    app.rpm_axis = rpm_axis
    app.orbit_axis = orbit_axis
    app.save_path = save_path
    app.target_rpm_box = target_rpm_box
    app.profile_menu = profile_menu
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
    on(profile_menu.selection) do profile_id
        isnothing(profile_id) || @async select_motor_profile!(app, String(profile_id))
        return nothing
    end
    on(manual_button.clicks) do _
        @async select_motor_mode!(app, :manual)
        return nothing
    end
    on(speed_button.clicks) do _
        @async select_motor_mode!(app, :speed)
        return nothing
    end
    on(arm_button.clicks) do _
        @async arm_motor!(app)
        return nothing
    end
    on(stop_button.clicks) do _
        @async emergency_stop!(app)
        return nothing
    end
    on(target_rpm_box.stored_string) do text
        isnothing(text) || @async set_target_rpm!(app, text)
        return nothing
    end
    on(events(figure).keyboardbutton, priority = 10) do event
        typing = target_rpm_box.focused[] || save_path.focused[]
        active = event.action == Keyboard.press || event.action == Keyboard.repeat
        if !typing && active && event.key in (Keyboard.up, Keyboard.down)
            direction = event.key == Keyboard.up ? 1 : -1
            @async adjust_manual_throttle!(app, direction)
            return Consume(true)
        elseif !typing && event.action == Keyboard.press && event.key == Keyboard.space
            @async emergency_stop!(app)
            return Consume(true)
        end
        return Consume(false)
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

function _orbit_plot_limits(pitch, yaw)
    absolute_maximum = max(maximum(abs, pitch), maximum(abs, yaw))
    extent = max(
        ORBIT_MINIMUM_EXTENT_DEGREES,
        (1 + ORBIT_MARGIN_FRACTION) * absolute_maximum,
    )
    return -Float64(extent), Float64(extent)
end

function _rpm_plot_limits(rpm)
    value = Float64(rpm)
    return max(0.0, value - RPM_MARGIN), value + RPM_MARGIN
end

function _motor_device_available(app::WhirlApp)
    return !isnothing(app.device) && isopen(app.device)
end

function _normal_status(app::WhirlApp)
    if app.running
        return app.demo ? "Receiving · DEMO" : "Receiving · LIVE"
    end
    return app.status_text[] == "Paused" ? "Paused" : "Idle"
end

function _restore_normal_status!(app::WhirlApp)
    app.tripped || (app.status_text[] = _normal_status(app))
    return nothing
end

function _update_motor_text!(app::WhirlApp)
    safety = app.tripped ? "TRIPPED" : (app.armed ? "ARMED" : "DISARMED")
    mode = app.motor_mode == :manual ? "MANUAL" : "CLOSED LOOP"
    command = if app.motor_mode == :manual
        @sprintf("%.0f%%", 100 * app.manual_throttle)
    else
        @sprintf("%.0f RPM", app.target_rpm)
    end
    saturated = !isempty(app.ctrl_saturated) && app.ctrl_saturated[end] != 0.0f0
    suffix = saturated && !app.tripped ? " · SATURATED / TARGET MAY BE UNREACHABLE" : ""
    app.motor_text[] = "$safety · $mode · $command$suffix"
    return nothing
end

function _update_profile_text!(app::WhirlApp)
    profile = app.motor_profile
    app.target_range_text[] =
        "Target RPM: 0 or $(Int(profile.target_min_rpm))–$(Int(profile.target_max_rpm)) (press Enter)"
    return nothing
end

function _target_coefficients(device::HelicDAQ.Device, target_rpm::Real)
    count = HelicDAQ.parameter(device, :target_coeffs).count
    return target_coefficients(target_rpm; count)
end

function _write_motor_profile!(device::HelicDAQ.Device, profile::MotorProfile)
    device[:ctrl_kp] = profile.kp
    device[:ctrl_ki] = profile.ki
    device[:ctrl_ramp] = profile.ramp_per_s
    return nothing
end

function _apply_safety_snapshot!(app::WhirlApp, arm_value, safety_flags; report_trip::Bool = true)
    previous_tripped = app.tripped
    state = decode_safety_flags(safety_flags)
    app.safety_flags = UInt32(safety_flags)
    app.armed = arm_value != 0 && state.armed
    app.tripped = state.tripped
    if app.tripped
        app.status_text[] = "Motor safety trip"
        if report_trip && !previous_tripped
            app.info_text[] =
                "MCU latched an output trip; drive is at minimum. Check RPM feedback and overspeed diagnostics before re-arming."
        end
    elseif previous_tripped
        _restore_normal_status!(app)
        report_trip && (app.info_text[] = "MCU safety trip cleared by a new arm command")
    end
    _update_motor_text!(app)
    return state
end

function _sync_safety_state!(app::WhirlApp; report_trip::Bool = true)
    if app.demo
        flags = UInt32(app.armed) | (UInt32(app.tripped) << 1)
        return _apply_safety_snapshot!(app, UInt32(app.armed), flags; report_trip)
    end
    _motor_device_available(app) || throw(HelicDAQ.DeviceError("motor control link is not connected"))
    state = HelicDAQ.getparams(app.device, (:arm, :safety))
    return _apply_safety_snapshot!(app, state.arm, state.safety; report_trip)
end

function initialise_motor_control!(app::WhirlApp)
    app.manual_throttle = 0.0f0
    app.target_rpm = 0.0f0
    app.motor_mode = :manual
    app.armed = false
    app.tripped = false
    app.safety_flags = UInt32(0)
    if !app.demo
        # Disarm first: every later write may fail without leaving drive enabled.
        app.device[:arm] = UInt32(0)
        app.device[:ctrl_manual] = 0.0f0
        app.device[:ctrl_mode] = MANUAL_MODE
        app.device[:target_coeffs] = _target_coefficients(app.device, 0.0f0)
        _write_motor_profile!(app.device, app.motor_profile)
        _sync_safety_state!(app; report_trip = false)
    end
    _update_profile_text!(app)
    _update_motor_text!(app)
    return nothing
end

function select_motor_profile!(app::WhirlApp, profile_id::AbstractString)
    profile = profile_by_id(app.profile_set, profile_id)
    profile.id == app.motor_profile.id && return nothing
    previous_profile = app.motor_profile
    app.motor_busy && return nothing
    app.motor_busy = true
    try
        if !app.demo && _motor_device_available(app)
            # Profile changes are transactional from a safety perspective: the
            # first write removes drive before any gains or ranges are changed.
            app.device[:arm] = UInt32(0)
            app.device[:ctrl_manual] = 0.0f0
            app.device[:ctrl_mode] = MANUAL_MODE
            app.device[:target_coeffs] = _target_coefficients(app.device, 0.0f0)
            _write_motor_profile!(app.device, profile)
        elseif !app.demo && app.running
            throw(HelicDAQ.DeviceError("motor control link is not connected"))
        end
        app.motor_profile = profile
        app.manual_throttle = 0.0f0
        app.target_rpm = 0.0f0
        app.motor_mode = :manual
        app.armed = false
        app.tripped = false
        app.safety_flags = UInt32(0)
        !app.demo && _motor_device_available(app) &&
            _sync_safety_state!(app; report_trip = false)
        _update_profile_text!(app)
        _restore_normal_status!(app)
        app.info_text[] =
            "Profile $(profile.label) applied; configured manual limit $(Int(round(100 * profile.manual_max_throttle)))%, RPM range $(Int(profile.target_min_rpm))–$(Int(profile.target_max_rpm))"
    catch error
        if !isnothing(app.profile_menu)
            previous_index = findfirst(
                candidate -> candidate.id == previous_profile.id,
                app.profile_set.profiles,
            )
            isnothing(previous_index) || (app.profile_menu.i_selected[] = previous_index)
        end
        app.status_text[] = "Motor profile error"
        app.info_text[] = sprint(showerror, error)
    finally
        app.motor_busy = false
        _update_motor_text!(app)
    end
    return nothing
end

function select_motor_mode!(app::WhirlApp, mode::Symbol)
    mode in (:manual, :speed) || throw(ArgumentError("unknown motor mode '$mode'"))
    app.motor_busy && return nothing
    app.motor_busy = true
    try
        if !app.demo
            _motor_device_available(app) || throw(HelicDAQ.DeviceError("connect before selecting a motor mode"))
            app.device[:arm] = UInt32(0)
            app.device[:ctrl_mode] = mode == :manual ? MANUAL_MODE : SPEED_MODE
            _sync_safety_state!(app; report_trip = false)
        end
        app.demo && (app.armed = false)
        app.motor_mode = mode
        _restore_normal_status!(app)
        app.info_text[] = mode == :manual ?
            "Manual mode selected; ARM at zero, then use ↑/↓" :
            "Closed-loop mode selected; enter a target RPM, then ARM"
    catch error
        app.status_text[] = "Motor command error"
        app.info_text[] = sprint(showerror, error)
    finally
        app.motor_busy = false
        _update_motor_text!(app)
    end
    return nothing
end

function set_target_rpm!(app::WhirlApp, text::AbstractString)
    target = try
        parse_target_rpm(text, app.motor_profile)
    catch error
        app.status_text[] = "Invalid target"
        app.info_text[] = sprint(showerror, error)
        return nothing
    end
    app.motor_busy && return nothing
    app.motor_busy = true
    try
        if !app.demo
            _motor_device_available(app) || throw(HelicDAQ.DeviceError("connect before setting a target"))
            app.device[:target_coeffs] = _target_coefficients(app.device, target)
        end
        app.target_rpm = target
        _restore_normal_status!(app)
        app.info_text[] = @sprintf("Target set to %.0f RPM", target)
    catch error
        app.status_text[] = "Motor command error"
        app.info_text[] = sprint(showerror, error)
    finally
        app.motor_busy = false
        _update_motor_text!(app)
    end
    return nothing
end

function arm_motor!(app::WhirlApp)
    app.running || begin
        app.info_text[] = "Start live acquisition before arming the motor"
        return nothing
    end
    app.motor_busy && return nothing
    if app.motor_mode == :manual && app.manual_throttle != 0.0f0
        app.info_text[] = "Manual throttle must be 0% before arming"
        return nothing
    elseif app.motor_mode == :speed && app.target_rpm == 0.0f0
        app.info_text[] = "Enter a non-zero closed-loop target before arming"
        return nothing
    end
    app.motor_busy = true
    try
        if app.demo
            app.armed = true
            app.tripped = false
        else
            app.device[:arm] = UInt32(1)
            _sync_safety_state!(app; report_trip = false)
            app.armed && !app.tripped || throw(HelicDAQ.DeviceError("MCU did not enter the armed state"))
        end
        _restore_normal_status!(app)
        app.info_text[] = "Motor armed; Space or STOP / DISARM stops immediately"
    catch error
        app.armed = false
        app.status_text[] = "Arm failed"
        app.info_text[] = sprint(showerror, error)
    finally
        app.motor_busy = false
        _update_motor_text!(app)
    end
    return nothing
end

function adjust_manual_throttle!(app::WhirlApp, direction::Integer)
    app.running && app.armed && !app.tripped && app.motor_mode == :manual || return nothing
    now_s = time()
    now_s - app.last_motor_command_s >= MOTOR_COMMAND_INTERVAL_SECONDS || return nothing
    app.motor_busy && return nothing
    app.motor_busy = true
    app.last_motor_command_s = now_s
    next_throttle = manual_step(
        app.manual_throttle,
        direction;
        upper = app.motor_profile.manual_max_throttle,
    )
    try
        !app.demo && (app.device[:ctrl_manual] = next_throttle)
        app.manual_throttle = next_throttle
        app.info_text[] = @sprintf("Manual throttle %.0f%%", 100 * next_throttle)
    catch error
        app.status_text[] = "Motor command error"
        app.info_text[] = sprint(showerror, error)
    finally
        app.motor_busy = false
        _update_motor_text!(app)
    end
    return nothing
end

function emergency_stop!(app::WhirlApp; report::Bool = true)
    # Let an in-flight arm or throttle request finish before issuing disarm;
    # otherwise an older request could arrive after the stop request.
    while app.motor_busy
        yield()
    end
    app.motor_busy = true
    app.armed = false
    app.manual_throttle = 0.0f0
    app.target_rpm = 0.0f0
    app.motor_mode = :manual
    _update_motor_text!(app)
    message = "Motor disarmed and commands cleared"
    try
        if !app.demo && _motor_device_available(app)
            # The first accepted write removes drive. The remaining writes
            # establish a zero-command state for any later re-arm.
            app.device[:arm] = UInt32(0)
            app.device[:ctrl_manual] = 0.0f0
            app.device[:ctrl_mode] = MANUAL_MODE
            app.device[:target_coeffs] = _target_coefficients(app.device, 0.0f0)
            _sync_safety_state!(app; report_trip = false)
        else
            app.tripped = false
            app.safety_flags = UInt32(0)
        end
    catch error
        message = "Disarm link error: $(sprint(showerror, error)); closing the control link also disarms"
    finally
        app.motor_busy = false
    end
    report && (app.info_text[] = message)
    return nothing
end

function append_packet!(app::WhirlApp, header, values)
    size(values, 2) == length(WHIRL_SOURCES) ||
        throw(ArgumentError("expected $(length(WHIRL_SOURCES)) whirl sources, received $(size(values, 2))"))
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
        push!(app.rpm_target, values[row, 4])
        push!(app.ctrl_saturated, values[row, 5])
        push!(app.esc_throttle, values[row, 6])
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
        # Ramp away from the sampled zero before settling into a slightly
        # distorted ellipse, so demo mode exercises the orbit display.
        envelope = min(1.0, time_s / 0.5)
        pitch = 45 + 3.2 * envelope * sinpi(4 * time_s)
        yaw = 120 + envelope * (2.2 * sinpi(4 * time_s + 0.55) + 0.35 * sinpi(8 * time_s))
        target = app.motor_mode == :speed && app.armed ? app.target_rpm : 0.0f0
        speed = target > 0 ? target + 120 * sinpi(1.7 * time_s) : 0.0
        throttle = app.armed ? app.manual_throttle : 0.0f0
        if app.motor_mode == :speed && app.armed
            throttle = clamp(target / 6_000, 0, 1)
        end
        isnothing(app.pitch_zero_degrees) && (app.pitch_zero_degrees = Float32(pitch))
        isnothing(app.yaw_zero_degrees) && (app.yaw_zero_degrees = Float32(yaw))
        push!(app.sample_indices, index)
        push!(app.times, time_s)
        push!(app.pitch_degrees, _zero_relative_angle(Float32(pitch), app.pitch_zero_degrees))
        push!(app.yaw_degrees, _zero_relative_angle(Float32(yaw), app.yaw_zero_degrees))
        push!(app.rpm, Float32(speed))
        push!(app.rpm_target, Float32(target))
        push!(app.ctrl_saturated, 0.0f0)
        push!(app.esc_throttle, Float32(throttle))
        app.demo_index += UInt64(app.decimation)
    end
    return nothing
end

function _check_whirl_device(device::HelicDAQ.Device)
    experiment = try
        String(device[:experiment])
    catch error
        error isa HelicDAQ.DeviceError ? "unknown" : rethrow()
    end
    experiment == "whirl-rig" ||
        throw(HelicDAQ.DeviceError("connected experiment is '$experiment', expected 'whirl-rig'"))
    available = Set(source.name for source in device.sources)
    missing = [String(source) for source in WHIRL_SOURCES if String(source) ∉ available]
    isempty(missing) ||
        throw(HelicDAQ.DeviceError("firmware is missing sources: $(join(missing, ", "))"))
    required_parameters = (
        :arm,
        :safety,
        :ctrl_mode,
        :ctrl_manual,
        :ctrl_kp,
        :ctrl_ki,
        :ctrl_ramp,
        :target_coeffs,
    )
    missing_parameters = [
        String(name) for name in required_parameters if !haskey(device.parameter_by_name, String(name))
    ]
    isempty(missing_parameters) || throw(
        HelicDAQ.DeviceError(
            "firmware is missing motor parameters: $(join(missing_parameters, ", "))",
        ),
    )
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
            initialise_motor_control!(app)
            app.running = true
            app.status_text[] = "Receiving · DEMO"
            app.info_text[] = "Synthetic 2 kHz source; no MCU required"
            app.busy = false
            @async demo_loop!(app, session)
            return nothing
        end

        if isnothing(app.device) || !isopen(app.device)
            app.device = HelicDAQ.Device(app.host; timeout = 3.0)
        end
        _check_whirl_device(app.device)
        initialise_motor_control!(app)
        device_status = HelicDAQ.status(app.device)
        app.sample_rate = Float64(device_status.sample_rate)
        configure_stream!(app.device, WHIRL_SOURCES; decimation = app.decimation, count = 0)
        receiver = HelicDAQ.StreamReceiver(; port = 0, timeout = STREAM_TIMEOUT_SECONDS)
        app.receiver = receiver
        HelicDAQ.prime!(receiver, app.host)
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
        if app.tripped
            app.status_text[] = "Motor safety trip"
            app.info_text[] =
                "MCU connected with a latched output trip; inspect feedback and overspeed diagnostics before re-arming."
        else
            app.status_text[] = "Receiving · LIVE"
            app.info_text[] =
                "$(app.sample_rate) Hz firmware · $(app.motor_profile.label) · decimation $(app.decimation)"
        end
        app.busy = false
        @async receive_loop!(app, session)
        @async keepalive_loop!(app, session)
    catch error
        app.running = false
        app.busy = false
        app.armed = false
        _update_motor_text!(app)
        _close_receiver!(app)
        _close_device!(app)
        app.status_text[] = "Connection error"
        app.info_text[] = sprint(showerror, error)
    end
    return nothing
end

function keepalive_loop!(app::WhirlApp, session::UInt64)
    while app.running && session == app.session
        sleep(CONTROL_STATE_POLL_SECONDS)
        app.running && session == app.session || break
        try
            _sync_safety_state!(app)
        catch error
            app.running && session == app.session || break
            app.running = false
            app.armed = false
            _update_motor_text!(app)
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
        throw(HelicDAQ.DeviceError("control connection closed during stream restart"))
    end
    stop_stream!(app.device)
    _close_receiver!(app)
    session == app.session || return false
    receiver = HelicDAQ.StreamReceiver(; port = 0, timeout = STREAM_TIMEOUT_SECONDS)
    app.receiver = receiver
    HelicDAQ.prime!(receiver, app.host)
    start_stream!(app.device, receiver.port)
    app.status_text[] = "Receiving · LIVE"
    app.info_text[] = "Stream recovered automatically · decimation $(app.decimation)"
    return true
end

function receive_loop!(app::WhirlApp, session::UInt64)
    restart_attempts = 0
    while app.running && session == app.session
        try
            header, values = HelicDAQ.receive(app.receiver)
            session == app.session || break
            append_packet!(app, header, values)
            restart_attempts = 0
        catch error
            if error isa HelicDAQ.StreamTimeout && restart_attempts < MAX_STREAM_RESTARTS
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
            app.armed = false
            _update_motor_text!(app)
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
    emergency_stop!(app; report = false)
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
    app.info_text[] = "Motor disarmed; stored data are retained; Start resumes acquisition"
    return nothing
end

function clear_data!(app::WhirlApp)
    empty!(app.times)
    empty!(app.sample_indices)
    empty!(app.pitch_degrees)
    empty!(app.yaw_degrees)
    empty!(app.rpm)
    empty!(app.rpm_target)
    empty!(app.ctrl_saturated)
    empty!(app.esc_throttle)
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
    app.plot_target_points[] = Point2f[]
    app.plot_orbit_points[] = Point2f[]
    app.plot_orbit_current[] = Point2f[]
    app.rpm_text[] = "— RPM"
    ylims!(app.angle_axis, -ANGLE_MARGIN_DEGREES, ANGLE_MARGIN_DEGREES)
    ylims!(app.rpm_axis, 0, 6_500)
    xlims!(app.orbit_axis, -ORBIT_MINIMUM_EXTENT_DEGREES, ORBIT_MINIMUM_EXTENT_DEGREES)
    ylims!(app.orbit_axis, -ORBIT_MINIMUM_EXTENT_DEGREES, ORBIT_MINIMUM_EXTENT_DEGREES)
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
    targets = copy(app.rpm_target)
    throttles = copy(app.esc_throttle)
    saturated = copy(app.ctrl_saturated)
    try
        mkpath(dirname(path))
        open(path, "w") do io
            println(io, "time_s,sample_index,pitch_deg,yaw_deg,rpm,rpm_target,esc_throttle,ctrl_saturated")
            for row in eachindex(times)
                println(
                    io,
                    times[row],
                    ',',
                    indices[row],
                    ',',
                    pitch[row],
                    ',',
                    yaw[row],
                    ',',
                    speeds[row],
                    ',',
                    targets[row],
                    ',',
                    throttles[row],
                    ',',
                    saturated[row],
                )
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
    current_rpm = app.rpm[end]
    app.rpm_text[] = @sprintf("%.0f RPM", current_rpm)
    rpm_extent = max(current_rpm, app.rpm_target[end])
    ylims!(app.rpm_axis, _rpm_plot_limits(rpm_extent)...)
    first_visible = searchsortedfirst(app.times, max(0.0, app.times[end] - app.window_seconds))
    stride = max(1, cld(count - first_visible + 1, MAX_PLOT_POINTS))
    selection = collect(first_visible:stride:count)
    selection[end] == count || push!(selection, count)
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
    app.plot_target_points[] = Point2f.(plot_time, app.rpm_target[selection])
    app.plot_orbit_points[] = Point2f.(plot_pitch, plot_yaw)
    app.plot_orbit_current[] = Point2f[(app.pitch_degrees[end], app.yaw_degrees[end])]
    _update_motor_text!(app)
    angle_lower, angle_upper = _angle_plot_limits(
        @view(app.pitch_degrees[first_visible:count]),
        @view(app.yaw_degrees[first_visible:count]),
    )
    ylims!(app.angle_axis, angle_lower, angle_upper)
    orbit_lower, orbit_upper = _orbit_plot_limits(
        @view(app.pitch_degrees[first_visible:count]),
        @view(app.yaw_degrees[first_visible:count]),
    )
    xlims!(app.orbit_axis, orbit_lower, orbit_upper)
    ylims!(app.orbit_axis, orbit_lower, orbit_upper)
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
    emergency_stop!(app; report = false)
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
