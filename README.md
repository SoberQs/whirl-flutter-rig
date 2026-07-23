# Whirl Flutter Rig

Live data acquisition and offline plotting tools for the HELIC whirl-flutter
rig. The live interface displays pitch, yaw, and rotor-speed measurements and
can save complete acquisitions as CSV files. Saved captures can then be
inspected interactively or exported as PNG, PDF, or SVG figures.

## HELIC-DAQ dependency

This project uses the Julia interface provided by
[`dawbarton/helic-daq`](https://github.com/dawbarton/helic-daq). The Julia
package is located in the upstream repository's `host-julia/` directory.

The upstream repository is included as the `helic-daq` Git submodule and is
pinned to commit
[`f9a73544cbd3a9601974cb3fe0cc8ef8eaa0d31b`](https://github.com/dawbarton/helic-daq/commit/f9a73544cbd3a9601974cb3fe0cc8ef8eaa0d31b).
Clone this project together with its submodule:

```sh
git clone --recurse-submodules \
    https://github.com/SoberQs/whirl-flutter-rig.git
```

If the project has already been cloned without its submodule, initialise it
from the project root:

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

This updates `Manifest.toml`, links the `HelicDAQ` source tree from the
submodule, installs GLMakie, and precompiles the environment.

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

PNG, PDF, and SVG output formats are supported.
