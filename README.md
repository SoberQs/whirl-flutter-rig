# Whirl Flutter Rig

Live data acquisition and offline plotting tools for the HELIC whirl-flutter
rig. The live interface displays pitch, yaw, rotor-speed, speed-reference, and
ESC-command measurements, provides manual and closed-loop motor control, and
can save complete acquisitions as CSV files. Saved captures can then be
inspected interactively or exported as PNG, PDF, or SVG figures.

## HELIC-DAQ dependency

This project uses the Julia interface provided by
[`dawbarton/helic-daq`](https://github.com/dawbarton/helic-daq). The Julia
package is located in the upstream repository's `host-julia/` directory.

The upstream repository is included as the `helic-daq` Git submodule and is
pinned to release `v0.3.0`, commit
[`3dbe99619c3066e8a24e75b46b99e7e68acef566`](https://github.com/dawbarton/helic-daq/commit/3dbe99619c3066e8a24e75b46b99e7e68acef566).

Clone this project and initialise the submodule in one step:

```sh
git clone --recurse-submodules \
    https://github.com/SoberQs/whirl-flutter-rig.git
```

If the project has already been cloned without its submodule, run the
following command from the project root:

```sh
git submodule update --init --recursive
```

## Requirements

- Julia 1.10 or later
- Git
- A graphical environment that supports GLMakie
- Access to the HELIC-DAQ hardware network for live acquisition

## Environment setup

Run the following command from the project root:

```sh
julia --project=. -e '
using Pkg
Pkg.develop(path="helic-daq/host-julia")
Pkg.instantiate()
Pkg.precompile()
'
```

This links the `HelicDAQ` source tree from the pinned submodule, installs the
GLMakie and CairoMakie plotting backends, and precompiles the environment.

To verify that Julia is loading the expected package:

```sh
julia --project=. -e '
using HelicDAQ, GLMakie
println("HelicDAQ source: ", pathof(HelicDAQ))
println("HelicDAQ version: ", pkgversion(HelicDAQ))
println("GLMakie version: ", pkgversion(GLMakie))
'
```

## Live GUI

Start the GUI with simulated data, without connecting to acquisition hardware:

```sh
julia --project=. src/gui.jl --demo
```

Connect to the default device address, `192.168.1.238`:

```sh
julia --project=. src/gui.jl
```

The device address, stream decimation, and visible time window can be
specified explicitly:

```sh
julia --project=. src/gui.jl \
    --host 192.168.1.238 --decimation 2 --window 20
```

On macOS, `run_gui.command` can also be opened directly from Finder. Pass
`--demo` when launching it from Terminal to use simulated input:

```sh
./run_gui.command --demo
```

The GUI's **Save CSV** button writes captures to the project-level `captures/`
directory by default.

### Motor operation

The GUI requires motor-control firmware whose discovered sources include
`rpm_target_used`, `ctrl_saturated`, and `esc_throttle`, and whose parameters
include `arm`, `safety`, `ctrl_mode`, `ctrl_manual`, `ctrl_kp`, `ctrl_ki`,
`ctrl_ramp`, and `target_coeffs`. It deliberately starts every new connection
disarmed with zero manual throttle and a zero RPM target.

1. Start acquisition and confirm that the live RPM and encoder signals are
   plausible.
2. Select the fitted motor profile, then select **Manual** or **Closed loop**
   while disarmed. In closed-loop mode, enter `0` or a value inside the
   profile's displayed configured RPM range and press Enter.
3. Press **ARM**. Manual mode can only be armed at 0%; closed-loop mode can
   only be armed with a non-zero target.
4. In manual mode, use the Up and Down arrow keys for 1% throttle steps. Key
   repeat is rate-limited to 20 commands per second, and the selected profile
   limits the largest GUI command. In closed-loop mode, the MCU adjusts
   throttle and indicates when the requested speed is unreachable at the safe
   output limit.
5. Press Space or **STOP / DISARM** at any time. Pause, window closure, stream
   failure, and control-link closure also disarm the firmware.

The GUI reads the MCU's `arm` and `safety` parameters twice per second. Its
`ARMED`, `DISARMED`, and `TRIPPED` text therefore reflects the firmware safety
gate rather than only the last button press. A trip commands minimum output;
the GUI reports the transition and requires the operator to inspect RPM
feedback and overspeed diagnostics before re-arming.

Keyboard motor commands are ignored while either text box has focus. The GUI
is a host interface, not the safety authority: firmware clamps the actual
output and applies its feedback-loss and overspeed trips. Do not connect or
power a motor until the ESC wiring, PWM endpoints, throttle-range calibration,
rotation direction, and ordered bring-up in the firmware repository's
`notes.md` have been completed.

### Motor profiles

`motor_profiles.toml` holds versioned host operating profiles. A profile names
the motor, ESC, and tested supply, configures the RPM and manual-throttle
envelope exposed by the GUI, and applies `ctrl_kp`, `ctrl_ki`, and `ctrl_ramp`
while disarmed. Its feedback and ESC fields document the matching compiled
firmware profile; they are not writable safety overrides.

The checked-in MN2806 profile exposes 2000--6000 RPM closed loop and up to 90%
manual throttle. This configured range is wider than the 16.5 V hardware
evidence recorded on 2026-08-24, which reached 3000 RPM closed loop and 20%
manual throttle; the untested portion must therefore remain an explicit
bring-up activity. Add a separately evidenced `[[profiles]]` entry before
using a different motor. A motor needing a lower overspeed limit or different
feedback timing also requires a firmware configuration change and a new
hardware test; editing the GUI profile alone cannot weaken the MCU's 6500 RPM
trip.

The CSV format now adds `rpm_target`, `esc_throttle`, and `ctrl_saturated`.
`src/plot.jl` remains compatible with older five-column captures.

Run the GUI-side validation and CSV compatibility tests with:

```sh
julia --project=. test/runtests.jl
```

## Offline plotting

Plot a saved CSV interactively by omitting the output path:

```sh
julia --project=. src/plot.jl \
    captures/whirl_capture_20260723_132504_521.csv
```

Provide an output path to save the plot without opening a window:

```sh
julia --project=. src/plot.jl \
    captures/whirl_capture_20260723_132504_521.csv plot.png
```

PNG, PDF, and SVG output formats are supported. PNG output uses GLMakie, while
PDF and SVG output use CairoMakie for native vector rendering.

```sh
julia --project=. src/plot.jl captures/example.csv plot.pdf
julia --project=. src/plot.jl captures/example.csv plot.svg
```
