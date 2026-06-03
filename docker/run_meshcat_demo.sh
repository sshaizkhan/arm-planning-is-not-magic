#!/usr/bin/env bash
# Run the meshcat visualization demo inside Docker.
#
# Meshcat serves a web UI on port 7000. This script exposes it so you can open
# the URL in your browser. Works in DinD environments (no --network host needed).
#
# Usage:
#   ./docker/run_meshcat_demo.sh            # joint sweep, no OMPL required
#   ./docker/run_meshcat_demo.sh demo       # full demo 08 (requires OMPL in image)
#
# After starting, open: http://<your-host-ip>:7000/static/

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="arm-planning-is-not-magic:ci"

# Build image if it doesn't exist
if ! docker image inspect "$IMAGE" &>/dev/null; then
    echo "Image $IMAGE not found — building (this takes a few minutes)..."
    DOCKER_BUILDKIT=0 docker build -t "$IMAGE" -f "$SCRIPT_DIR/Dockerfile" "$(cd "$SCRIPT_DIR/.." && pwd)"
fi

MODE="${1:-sweep}"

if [ "$MODE" = "demo" ]; then
    echo "Running demo 08 (full, requires OMPL)..."
    echo "Open http://localhost:7000/static/ in your browser"
    docker run --rm -it \
        -p 7000:7000 \
        "$IMAGE" \
        python3 demos/08_meshcat_visualization.py
else
    echo "Running joint sweep demo..."
    echo "Open http://localhost:7000/static/ in your browser"
    docker run --rm \
        -p 7000:7000 \
        "$IMAGE" \
        python3 -c "
import numpy as np, time
from visualization.meshcat_visualizer import MeshcatVisualizer

viz = MeshcatVisualizer(open_browser=False)
print()
print('==> Open in browser: http://localhost:7000/static/')
print('Waiting 10s before animating...')
time.sleep(10)

print('Animating joint sweep (3 cycles)...')
for _ in range(3):
    for a in np.linspace(0, 3.14, 120):
        viz.visualize_configuration(np.array([a, -1.57, 1.57, -1.57, -1.57, 0.0]), duration=0.05)
    for a in np.linspace(3.14, 0, 120):
        viz.visualize_configuration(np.array([a, -1.57, 1.57, -1.57, -1.57, 0.0]), duration=0.05)

print('Done. Keeping server alive — Ctrl+C to stop.')
time.sleep(300)
"
fi
