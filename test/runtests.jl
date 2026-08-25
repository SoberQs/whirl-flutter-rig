# Tests for whirl motor-control validation and command construction.

using Test

include(joinpath(@__DIR__, "..", "src", "gui.jl"))

@testset "pitch-yaw orbit limits" begin
    @test _orbit_plot_limits(Float32[0.0], Float32[0.0]) == (-1.0, 1.0)
    lower, upper = _orbit_plot_limits(Float32[-2.0, 3.0], Float32[-4.0, 1.0])
    @test lower ≈ -4.4
    @test upper ≈ 4.4
end

const TEST_PROFILE_SET = load_motor_profiles()
const TEST_PROFILE = profile_by_id(TEST_PROFILE_SET, TEST_PROFILE_SET.default_id)

@testset "manual throttle" begin
    @test manual_step(0.5, 1) == 0.51f0
    @test manual_step(0.5, -1) == 0.49f0
    @test manual_step(1.0, 1) == 1.0f0
    @test manual_step(0.0, -1) == 0.0f0
    @test_throws ArgumentError manual_step(0.5, 0)
    @test manual_step(0.2, 1; upper = 0.2) == 0.2f0
    @test_throws ArgumentError manual_step(0.1, 1; upper = 1.1)
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
    @test TEST_PROFILE.target_min_rpm == 2_000.0f0
    @test TEST_PROFILE.target_max_rpm == 6_000.0f0
    @test TEST_PROFILE.esc_pwm_hz == 50
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
    set_target_rpm!(app, "2500")
    @test app.target_rpm == 2_500.0f0
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
