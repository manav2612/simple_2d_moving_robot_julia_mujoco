#!/usr/bin/env bash
set -euo pipefail

# Simple wrapper to run the Julia CLI from the project root.
# Usage: ./scripts/run_mujoco.sh [<julia run_cli args>]

proj_dir="$(cd "$(dirname "$0")/.." && pwd)"
cd "$proj_dir"

if ! command -v julia >/dev/null 2>&1; then
  echo "Error: julia not found in PATH. Install Julia or add it to PATH." >&2
  exit 1
fi

if [ "$#" -eq 0 ]; then
  echo "No arguments passed — running example with large-goal params."
  julia --project=. scripts/run_cli.jl --goal 4.8 4.8 --init 2.5 2.5 --ctrl_scale 5.0 --steps 2000 --save_plot simulation_cli_largegoal.png --verbose true
else
  julia --project=. scripts/run_cli.jl "$@"
fi
