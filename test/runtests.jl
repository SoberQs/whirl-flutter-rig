# Tests for whirl motor-control validation and command construction.

using Test

include(joinpath(@__DIR__, "..", "src", "gui.jl"))

function _test_plot_sample(time_s)
    value = Float32(time_s)
    return PlotSample(time_s, value, -value, value, value + 1, 0.0f0, 0.5f0)
end

@testset "plot ring buffer" begin
    @test _plot_buffer_capacity(2_000, 1, 10.0) == 20_001
    @test _plot_buffer_capacity(2_000, 10, 10.0) == 2_001
    @test_throws ArgumentError PlotRingBuffer(0)

    buffer = PlotRingBuffer(3)
    for time_s in 1.0:5.0
        push!(buffer, _test_plot_sample(time_s))
    end
    @test length(buffer) == 3
    @test [buffer[index].time_s for index in 1:length(buffer)] == [3.0, 4.0, 5.0]
    @test buffer[end].time_s == 5.0
    @test _plot_searchsortedfirst(buffer, 4.0) == 2
    @test _plot_searchsortedfirst(buffer, 4.5) == 3

    _resize_plot_buffer!(buffer, 2)
    @test [buffer[index].time_s for index in 1:length(buffer)] == [4.0, 5.0]
    _resize_plot_buffer!(buffer, 4)
    @test [buffer[index].time_s for index in 1:length(buffer)] == [4.0, 5.0]
    empty!(buffer)
    @test isempty(buffer)
end

@testset "live angle FFT" begin
    short_plot_app = WhirlApp("offline", false, 2, 3.0)
    @test length(short_plot_app.plot_buffer.storage) == 10_001

    sample_rate = 200.0
    times = collect(0:(1 / sample_rate):(10 - 1 / sample_rate))
    signal = @. 3 * sinpi(25 * times)
    spectrum = _amplitude_spectrum(signal, sample_rate)
    peak = argmax(spectrum.amplitude)
    @test spectrum.frequency_hz[peak] ≈ 12.5
    @test spectrum.amplitude[peak] ≈ 3.0 atol = 0.01

    buffer = PlotRingBuffer(1_300)
    for time_s in 0.0:0.01:12.0
        time_s == 7.0 && continue
        push!(buffer, _test_plot_sample(time_s))
    end
    window = _interpolated_angle_window(buffer, 100.0, 1)
    @test length(window.pitch) == 1_000
    @test window.pitch[1] ≈ 2.01 atol = 1.0e-6
    @test window.pitch[end] ≈ 12.0 atol = 1.0e-6
    @test window.pitch[500] ≈ 7.0 atol = 1.0e-6
    @test window.yaw == -window.pitch
end

@testset "packet plotting window is bounded" begin
    app = WhirlApp("offline", false, 1, 10.0)
    _resize_plot_buffer!(app.plot_buffer, 2)
    header = (first_index = UInt32(10), decimation = UInt16(1), dropped = UInt32(0))
    values = Float32[
        0.1 0.2 100.0 110.0 0.0 0.1
        0.2 0.3 200.0 210.0 0.0 0.2
        0.3 0.4 300.0 310.0 1.0 0.3
    ]
    append_packet!(app, header, values)
    @test app.total_samples == 3
    @test length(app.plot_buffer) == 2
    @test [app.plot_buffer[index].rpm for index in 1:2] == Float32[200.0, 300.0]
    @test [app.plot_buffer[index].time_s for index in 1:2] == [0.0005, 0.001]
end

@testset "streaming capture" begin
    mktempdir() do directory
        app = WhirlApp("offline", false, 1, 10.0; capture_directory = directory)
        writer = _start_capture!(app)
        @test endswith(writer.partial_path, ".csv.partial")
        @test isfile(writer.partial_path)

        header = (first_index = UInt32(20), decimation = UInt16(1), dropped = UInt32(0))
        values = Float32[
            0.1 0.2 100.0 110.0 0.0 0.1
            0.2 0.3 200.0 210.0 1.0 0.2
        ]
        append_packet!(app, header, values)
        flush(writer.io)
        @test writer.row_count == 2
        @test length(readlines(writer.partial_path)) == 3

        result = _finalize_capture!(app)
        @test result.rows == 2
        @test isfile(result.path)
        @test !ispath(writer.partial_path)
        @test first(readlines(result.path)) == CAPTURE_HEADER
        @test isnothing(app.capture)
    end

    mktempdir() do directory
        app = WhirlApp("offline", true, 1, 10.0; capture_directory = directory)
        first_writer = _start_capture!(app)
        app.running = true
        append_demo_chunk!(app)
        first_rows = first_writer.row_count
        saved_path = save_csv!(app)
        @test isfile(saved_path)
        @test length(readlines(saved_path)) == first_rows + 1
        @test !isnothing(app.capture)
        @test app.capture.partial_path != first_writer.partial_path
        @test isfile(app.capture.partial_path)
        app.running = false
        second_result = _finalize_capture!(app)
        @test isfile(second_result.path)
    end
end

@testset "partial capture recovery" begin
    mktempdir() do directory
        partial_path = joinpath(directory, "interrupted.csv.partial")
        contents = CAPTURE_HEADER * "\n" *
            "0.0,10,1.0,2.0,100.0,110.0,0.1,0.0\n" *
            "0.0005,11,1.1,2.1,101.0,111.0,0.2,0.0\n" *
            "0.001,12,broken"
        write(partial_path, contents)

        result = recover_partial_capture!(partial_path)
        @test result.rows == 2
        @test result.discarded_tail
        @test !ispath(partial_path)
        @test read(result.original, String) == contents
        recovered_lines = readlines(result.recovered)
        @test recovered_lines[1] == CAPTURE_HEADER
        @test length(recovered_lines) == 3
        @test endswith(recovered_lines[end], ",0.0")
    end

    mktempdir() do directory
        invalid_path = joinpath(directory, "invalid.csv.partial")
        write(invalid_path, "wrong,header\n0.0,1,2,3,4,5,6,7\n")
        @test_throws ArgumentError recover_partial_capture!(invalid_path)
        @test isfile(invalid_path)
    end
end

@testset "pitch-yaw orbit limits" begin
    @test _orbit_plot_limits(Float32[0.0], Float32[0.0]) == (-1.0, 1.0)
    lower, upper = _orbit_plot_limits(Float32[-2.0, 3.0], Float32[-4.0, 1.0])
    @test lower ≈ -4.4
    @test upper ≈ 4.4
end

const TEST_PROFILE_SET = load_motor_profiles()
const TEST_PROFILE = profile_by_id(TEST_PROFILE_SET, "mn2806_kv400_air40a_16v5")
const CINE66_PROFILE = profile_by_id(TEST_PROFILE_SET, "cine66_kv925_air40a_25v")
const CINE66_PROP_PROFILE =
    profile_by_id(TEST_PROFILE_SET, "cine66_kv925_air40a_25v_prop8x6")

@testset "manual throttle" begin
    @test manual_step(0.5, 1) == 0.51f0
    @test manual_step(0.5, -1) == 0.49f0
    @test manual_step(1.0, 1) == 1.0f0
    @test manual_step(0.0, -1) == 0.0f0
    @test_throws ArgumentError manual_step(0.5, 0)
    @test manual_step(0.2, 1; upper = 0.2) == 0.2f0
    @test_throws ArgumentError manual_step(0.1, 1; upper = 1.1)
end

@testset "profile-specific manual throttle" begin
    app = WhirlApp("demo", true, 1, 10.0)
    app.running = true
    app.armed = true
    adjust_manual_throttle!(app, 1)
    @test app.manual_throttle ≈ CINE66_PROFILE.manual_step_throttle
    @test app.motor_text[] == "ARMED · MANUAL · 0.12%"
    @test occursin("≈1.0 µs", app.manual_step_text[])
    app.last_motor_command_s = -Inf
    adjust_manual_throttle!(app, -1)
    @test app.manual_throttle == 0.0f0
end

@testset "RPM target" begin
    @test parse_target_rpm("0", TEST_PROFILE) == 0.0f0
    @test parse_target_rpm("2000", TEST_PROFILE) == 2_000.0f0
    @test parse_target_rpm("6000", TEST_PROFILE) == 6_000.0f0
    @test_throws ArgumentError parse_target_rpm("1999", TEST_PROFILE)
    @test_throws ArgumentError parse_target_rpm("6001", TEST_PROFILE)
    @test_throws ArgumentError parse_target_rpm("fast", TEST_PROFILE)
end

@testset "motor profile" begin
    @test TEST_PROFILE.id == "mn2806_kv400_air40a_16v5"
    @test TEST_PROFILE.motor_model == "T-Motor MN2806 KV400"
    @test TEST_PROFILE.tested_supply_v == 16.5f0
    @test TEST_PROFILE.manual_max_throttle == 0.9f0
    @test TEST_PROFILE.manual_step_throttle == 0.01f0
    @test TEST_PROFILE.target_min_rpm == 2_000.0f0
    @test TEST_PROFILE.target_max_rpm == 6_000.0f0
    @test TEST_PROFILE.esc_pwm_hz == 50
    @test TEST_PROFILE_SET.default_id == CINE66_PROP_PROFILE.id
    @test CINE66_PROFILE.target_min_rpm == 4_000.0f0
    @test CINE66_PROFILE.target_max_rpm == 6_000.0f0
    @test CINE66_PROFILE.manual_step_throttle ≈ 1.0f0 / 840.0f0
    @test CINE66_PROFILE.manual_max_throttle == 0.9f0
    @test CINE66_PROP_PROFILE.target_min_rpm == 2_000.0f0
    @test CINE66_PROP_PROFILE.target_max_rpm == 6_000.0f0
    @test CINE66_PROP_PROFILE.manual_step_throttle ≈ 1.0f0 / 840.0f0
    @test CINE66_PROP_PROFILE.manual_max_throttle ≈ 194.0f0 / 840.0f0
    @test CINE66_PROP_PROFILE.kp == 0.0001f0
    @test CINE66_PROP_PROFILE.ki == 0.00003f0
    @test CINE66_PROP_PROFILE.ramp_per_s == 0.1f0
    @test CINE66_PROP_PROFILE.feedback_required_above == 0.15f0
    @test_throws ArgumentError profile_by_id(TEST_PROFILE_SET, "missing")

    profile_values = Any[getfield(TEST_PROFILE, field) for field in 1:fieldcount(MotorProfile)]
    profile_values[1] = "future_motor"
    profile_values[2] = "Future motor"
    future_profile = MotorProfile(profile_values...)
    app = WhirlApp("offline", false, 1, 10.0)
    app.profile_set = MotorProfileSet(TEST_PROFILE.id, [TEST_PROFILE, future_profile])
    select_motor_profile!(app, future_profile.id)
    @test app.motor_profile.id == future_profile.id
    @test app.status_text[] == "Idle"
end

@testset "MCU safety state" begin
    @test decode_safety_flags(8) ==
        (armed = false, tripped = false, clamped = false, quieted = true)
    @test decode_safety_flags(9).armed
    @test decode_safety_flags(11).tripped

    app = WhirlApp("demo", true, 1, 10.0)
    app.running = true
    _apply_safety_snapshot!(app, UInt32(1), UInt32(11))
    @test app.armed
    @test app.tripped
    @test app.status_text[] == "Motor safety trip"
    @test startswith(app.motor_text[], "TRIPPED")
    _apply_safety_snapshot!(app, UInt32(1), UInt32(9))
    @test app.armed
    @test !app.tripped
    @test app.status_text[] == "Receiving · DEMO"
end

@testset "valid target clears an input error" begin
    app = WhirlApp("demo", true, 1, 10.0)
    app.running = true
    set_target_rpm!(app, "7000")
    @test app.status_text[] == "Invalid target"
    set_target_rpm!(app, "4500")
    @test app.target_rpm == 4_500.0f0
    @test app.status_text[] == "Receiving · DEMO"
end

@testset "constant target coefficients" begin
    coefficients = target_coefficients(3_500)
    @test length(coefficients) == 33
    @test coefficients[1] == 3_500.0f0
    @test all(iszero, coefficients[2:end])
end

include(joinpath(@__DIR__, "..", "src", "plot.jl"))

@testset "offline pitch-yaw orbit limits" begin
    @test _offline_orbit_plot_limits([0.0], [0.0]) == (-1.0, 1.0)
    lower, upper = _offline_orbit_plot_limits([-2.0, 3.0], [-4.0, 1.0])
    @test lower ≈ -4.4
    @test upper ≈ 4.4
end

@testset "offline plot and time selection" begin
    @test _normalise_plots((:rpm, :angles)) == (:angles, :rpm)
    @test _normalise_plots(("orbit", "throttle")) == (:pitch_yaw, :esc)
    @test _normalise_plots("all") == OFFLINE_PLOTS
    @test_throws ArgumentError _normalise_plots(())
    @test_throws ArgumentError _normalise_plots(("temperature",))

    times = [0.0, 0.5, 1.0, 1.5]
    @test _time_interval(times; start_time = 0.2, end_time = 1.1) == 2:3
    @test _time_interval(times; start_time = 1.0) == 3:4
    @test _time_interval(times; end_time = 0.5) == 1:2
    @test_throws ArgumentError _time_interval(times; start_time = 2.0)
    @test_throws ArgumentError _time_interval(times; start_time = 1.0, end_time = 0.5)

    options = _parse_plot_arguments(
        [
            "capture.csv",
            "plot.png",
            "--plot",
            "rpm,pitch-yaw",
            "--start",
            "10",
            "--end",
            "20.5",
        ],
    )
    @test options.input == "capture.csv"
    @test options.output == "plot.png"
    @test options.plots == (:rpm, :pitch_yaw)
    @test options.start_time == 10.0
    @test options.end_time == 20.5
    @test_throws ArgumentError _parse_plot_arguments(String[])
    @test_throws ArgumentError _parse_plot_arguments(["capture.csv", "--plot", "unknown"])
end

@testset "capture CSV compatibility" begin
    mktempdir() do directory
        legacy_path = joinpath(directory, "legacy.csv")
        write(
            legacy_path,
            "time_s,sample_index,pitch_deg,yaw_deg,rpm\n0.0,1,2.0,3.0,4000.0\n",
        )
        legacy = read_whirl_csv(legacy_path)
        @test legacy.rpm == [4_000.0]
        @test legacy.rpm_target == [0.0]
        @test legacy.esc_throttle == [0.0]

        motor_path = joinpath(directory, "motor.csv")
        write(
            motor_path,
            "time_s,sample_index,pitch_deg,yaw_deg,rpm,rpm_target,esc_throttle,ctrl_saturated\n" *
                "0.0,1,2.0,3.0,3900.0,4000.0,0.55,1.0\n",
        )
        motor = read_whirl_csv(motor_path)
        @test motor.rpm_target == [4_000.0]
        @test motor.esc_throttle == [0.55]
        @test motor.ctrl_saturated == [1.0]
    end
end
