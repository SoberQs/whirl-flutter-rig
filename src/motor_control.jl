# Pure GUI-side motor profiles, validation, and command construction.

module WhirlMotorControl

using TOML

export DEFAULT_MOTOR_PROFILES_PATH,
    MANUAL_STEP,
    MANUAL_MODE,
    SPEED_MODE,
    MotorProfile,
    MotorProfileSet,
    decode_safety_flags,
    load_motor_profiles,
    manual_step,
    parse_target_rpm,
    profile_by_id,
    target_coefficients

const MANUAL_MODE = 0.0f0
const SPEED_MODE = 1.0f0
const MANUAL_STEP = 0.01f0
const FIRMWARE_TARGET_MAX_RPM = 6_000.0f0
const TARGET_COEFFICIENT_COUNT = 33
const DEFAULT_MOTOR_PROFILES_PATH = normpath(joinpath(@__DIR__, "..", "motor_profiles.toml"))

"""One host-side operating profile inside the MCU's independent hard limits."""
struct MotorProfile
    id::String
    label::String
    motor_model::String
    motor_kv::Float32
    supply_cells_min::Int
    supply_cells_max::Int
    tested_supply_v::Float32
    target_min_rpm::Float32
    target_max_rpm::Float32
    manual_max_throttle::Float32
    kp::Float32
    ki::Float32
    ramp_per_s::Float32
    feedback_timeout_s::Float32
    feedback_required_above::Float32
    esc_pwm_hz::Int
    esc_min_pulse_us::Int
    esc_max_pulse_us::Int
    esc_calibration_status::String
    hardware_evidence::String
end

"""Validated profile collection and the profile selected at application start."""
struct MotorProfileSet
    default_id::String
    profiles::Vector{MotorProfile}
end

function _required(table::AbstractDict, key::AbstractString, context::AbstractString)
    haskey(table, key) || throw(ArgumentError("$context is missing '$key'"))
    return table[key]
end

function _string(table, key, context)
    value = _required(table, key, context)
    value isa AbstractString || throw(ArgumentError("$context.$key must be a string"))
    isempty(strip(value)) && throw(ArgumentError("$context.$key must not be empty"))
    return String(value)
end

function _float(table, key, context)
    value = _required(table, key, context)
    value isa Real || throw(ArgumentError("$context.$key must be numeric"))
    result = Float32(value)
    isfinite(result) || throw(ArgumentError("$context.$key must be finite"))
    return result
end

function _integer(table, key, context)
    value = _required(table, key, context)
    value isa Integer || throw(ArgumentError("$context.$key must be an integer"))
    return Int(value)
end

function _motor_profile(table::AbstractDict, index::Integer)
    context = "profiles[$index]"
    profile = MotorProfile(
        _string(table, "id", context),
        _string(table, "label", context),
        _string(table, "motor_model", context),
        _float(table, "motor_kv", context),
        _integer(table, "supply_cells_min", context),
        _integer(table, "supply_cells_max", context),
        _float(table, "tested_supply_v", context),
        _float(table, "target_min_rpm", context),
        _float(table, "target_max_rpm", context),
        _float(table, "manual_max_throttle", context),
        _float(table, "kp", context),
        _float(table, "ki", context),
        _float(table, "ramp_per_s", context),
        _float(table, "feedback_timeout_s", context),
        _float(table, "feedback_required_above", context),
        _integer(table, "esc_pwm_hz", context),
        _integer(table, "esc_min_pulse_us", context),
        _integer(table, "esc_max_pulse_us", context),
        _string(table, "esc_calibration_status", context),
        _string(table, "hardware_evidence", context),
    )
    occursin(r"^[a-z0-9][a-z0-9_-]*$", profile.id) ||
        throw(ArgumentError("$context.id must use lowercase letters, digits, '_' or '-'"))
    profile.motor_kv > 0 || throw(ArgumentError("$context.motor_kv must be positive"))
    1 <= profile.supply_cells_min <= profile.supply_cells_max <= 12 ||
        throw(ArgumentError("$context supply cell range is invalid"))
    profile.tested_supply_v > 0 || throw(ArgumentError("$context.tested_supply_v must be positive"))
    0 < profile.target_min_rpm <= profile.target_max_rpm <= FIRMWARE_TARGET_MAX_RPM ||
        throw(ArgumentError("$context target range must fit inside 0–$(Int(FIRMWARE_TARGET_MAX_RPM)) RPM"))
    0 < profile.manual_max_throttle <= 1 ||
        throw(ArgumentError("$context.manual_max_throttle must be in (0, 1]"))
    profile.kp >= 0 || throw(ArgumentError("$context.kp must be non-negative"))
    profile.ki >= 0 || throw(ArgumentError("$context.ki must be non-negative"))
    0 < profile.ramp_per_s <= 10 || throw(ArgumentError("$context.ramp_per_s must be in (0, 10]"))
    profile.feedback_timeout_s > 0 ||
        throw(ArgumentError("$context.feedback_timeout_s must be positive"))
    0 <= profile.feedback_required_above <= 1 ||
        throw(ArgumentError("$context.feedback_required_above must be in [0, 1]"))
    profile.esc_pwm_hz > 0 || throw(ArgumentError("$context.esc_pwm_hz must be positive"))
    0 < profile.esc_min_pulse_us < profile.esc_max_pulse_us ||
        throw(ArgumentError("$context ESC pulse range is invalid"))
    return profile
end

"""Load and validate the versioned motor-profile configuration."""
function load_motor_profiles(path::AbstractString = DEFAULT_MOTOR_PROFILES_PATH)
    configuration = TOML.parsefile(path)
    get(configuration, "schema_version", nothing) == 1 ||
        throw(ArgumentError("motor profile schema_version must be 1"))
    default_id = _string(configuration, "default_profile", "motor profiles")
    raw_profiles = _required(configuration, "profiles", "motor profiles")
    raw_profiles isa AbstractVector || throw(ArgumentError("motor profiles must be an array"))
    isempty(raw_profiles) && throw(ArgumentError("at least one motor profile is required"))
    profiles = [_motor_profile(table, index) for (index, table) in enumerate(raw_profiles)]
    ids = getproperty.(profiles, :id)
    allunique(ids) || throw(ArgumentError("motor profile ids must be unique"))
    default_id in ids || throw(ArgumentError("default motor profile '$default_id' does not exist"))
    return MotorProfileSet(default_id, profiles)
end

"""Resolve a profile id without relying on its position in the TOML file."""
function profile_by_id(profile_set::MotorProfileSet, id::AbstractString)
    index = findfirst(profile -> profile.id == id, profile_set.profiles)
    isnothing(index) && throw(ArgumentError("unknown motor profile '$id'"))
    return profile_set.profiles[index]
end

"""Decode the MCU-owned output-safety parameter bitfield."""
function decode_safety_flags(flags::Integer)
    value = UInt32(flags)
    return (
        armed = value & UInt32(0x01) != 0,
        tripped = value & UInt32(0x02) != 0,
        clamped = value & UInt32(0x04) != 0,
        quieted = value & UInt32(0x08) != 0,
    )
end

"""Move a bounded manual throttle command by one signed keyboard step."""
function manual_step(
        current::Real,
        direction::Integer;
        step::Real = MANUAL_STEP,
        upper::Real = 1.0,
    )
    direction in (-1, 1) || throw(ArgumentError("direction must be -1 or 1"))
    step > 0 || throw(ArgumentError("manual throttle step must be positive"))
    0 < upper <= 1 || throw(ArgumentError("manual throttle upper bound must be in (0, 1]"))
    return Float32(clamp(current + direction * step, 0.0, upper))
end

"""Parse a stopped target or an RPM target inside one configured profile."""
function parse_target_rpm(text::AbstractString, profile::MotorProfile)
    value = tryparse(Float32, strip(text))
    isnothing(value) && throw(ArgumentError("target RPM must be a number"))
    isfinite(value) || throw(ArgumentError("target RPM must be finite"))
    value == 0.0f0 && return value
    profile.target_min_rpm <= value <= profile.target_max_rpm || throw(
        ArgumentError(
            "target RPM must be 0 or between $(Int(profile.target_min_rpm)) and $(Int(profile.target_max_rpm)) for $(profile.label)",
        ),
    )
    return value
end


"""Construct a constant standard-programme RPM reference."""
function target_coefficients(target_rpm::Real; count::Integer = TARGET_COEFFICIENT_COUNT)
    count > 0 || throw(ArgumentError("coefficient count must be positive"))
    coefficients = zeros(Float32, count)
    coefficients[1] = Float32(target_rpm)
    return coefficients
end

end
