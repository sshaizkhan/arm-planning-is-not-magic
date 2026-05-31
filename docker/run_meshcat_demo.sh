#!/usr/bin/env bash
# Run the meshcat visualization demo inside Docker.
#
# Meshcat serves a web UI on port 7000. This script exposes that port so you
# can open http://localhost:7000/static/ in your host browser.
#
# Usage:
#   ./docker/run_meshcat_demo.sh            # runs demo 08 (full, needs OMPL)
#   ./docker/run_meshcat_demo.sh sweep      # runs a simple joint sweep (no OMPL)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
IMAGE="arm-planning-is-not-magic:ci"

# Build image if it doesn't exist
if ! docker image inspect "$IMAGE" &>/dev/null; then
    echo "Image $IMAGE not found — building..."
    docker build -t "$IMAGE" -f "$SCRIPT_DIR/Dockerfile" "$PROJECT_ROOT"
fi

MODE="${1:-demo}"

if [ "$MODE" = "sweep" ]; then
    echo "Running joint sweep demo — open http://localhost:7000/static/ in your browser"
    docker run --rm \
        --network host \
        -v "$PROJECT_ROOT":/workspace/arm-planning-is-not-magic \
        -w /workspace/arm-planning-is-not-magic \
        -e PYTHONPATH=/workspace/arm-planning-is-not-magic \
        "$IMAGE" \
        python3 -c "
import numpy as np
from visualization.meshcat_visualizer import MeshcatVisualizer
import time

viz = MeshcatVisualizer(open_browser=False)
print()
print('Open this URL in your browser: http://localhost:7000/static/')
print('Waiting 5s for you to open the browser...')
time.sleep(5)

print('Running joint sweep...')
for angle in np.linspace(0, np.pi, 120):
    q = np.array([angle, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0])
    viz.visualize_configuration(q, duration=0.05)

print('Sweep done. Press Ctrl+C to exit.')
time.sleep(60)
"
else
    echo "Running demo 08 — open http://localhost:7000/static/ in your browser"
    docker run --rm -it \
        --network host \
        -v "$PROJECT_ROOT":/workspace/arm-planning-is-not-magic \
        -w /workspace/arm-planning-is-not-magic \
        -e PYTHONPATH=/workspace/arm-planning-is-not-magic \
        "$IMAGE" \
        python3 demos/08_meshcat_visualization.py
fi
