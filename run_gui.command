#!/bin/zsh
# Launch the whirl-rig GUI from Finder or Terminal on macOS.

set -e

script_directory=${0:A:h}
julia_executable=${JULIA_BIN:-$(command -v julia || true)}

if [[ -z ${julia_executable} && -x /opt/homebrew/bin/julia ]]; then
    julia_executable=/opt/homebrew/bin/julia
fi
if [[ -z ${julia_executable} ]]; then
    print -u2 "Julia was not found. Install Julia or set JULIA_BIN before running this file."
    read "?Press Enter to close..."
    exit 1
fi

cd ${script_directory}
exec ${julia_executable} --project=. src/gui.jl "$@"
