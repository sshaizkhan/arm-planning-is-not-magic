# Docker Setup for Arm Planning

This directory contains Docker configuration files for containerizing the arm-planning-is-not-magic development environment.

## Quick Start

### Build the Docker image

```bash
cd docker
docker-compose build
```

Or using Docker directly:

```bash
docker build -t arm-planning-is-not-magic:latest -f docker/Dockerfile ..
```

### Run the container

Using docker-compose (recommended):

```bash
docker-compose up -d
docker-compose exec arm-planning bash
```

Or using Docker directly:

```bash
docker run -it --rm \
  -v $(pwd)/..:/workspace/arm-planning-is-not-magic \
  -w /workspace/arm-planning-is-not-magic \
  arm-planning-is-not-magic:latest \
  bash
```

### Run demos

Once inside the container:

```bash
# Run a demo
python3 demos/01_plan_and_time.py

# Run planner comparison
python3 demos/03_compare_planners.py --parameterizer toppra

# Run collision demo
python3 demos/04_collision_demo.py
```

## Development Workflow

The docker-compose setup mounts the project directory, so changes to Python files are immediately available in the container. However, if you install new Python packages, you'll need to rebuild the image or install them inside the container.

### Installing additional packages

Inside the container:

```bash
pip install <package-name>
```

Or add to `requirements.txt` and rebuild:

```bash
docker-compose build
```

## Image Details

- **Base Image**: Ubuntu 22.04
- **Python**: 3.10+
- **System Packages**: OMPL (libompl-dev), Eigen3, build tools
- **Python Packages**: numpy, toppra, ruckig, matplotlib, ompl

## Troubleshooting

### OMPL not found

If OMPL Python bindings are not available, the Dockerfile will attempt to install via pip. If this fails, you may need to:

1. Install OMPL from source
2. Use a ROS 2 base image that includes OMPL
3. Build OMPL Python bindings manually

### GUI/Display issues

For demos that use matplotlib with GUI backends, you may need to:

1. Use `matplotlib.use('Agg')` for non-interactive plotting
2. Set up X11 forwarding (see commented section in docker-compose.yml)
3. Use `--headless` mode if available

### Permission issues

If you encounter permission issues with mounted volumes:

```bash
docker-compose down
sudo chown -R $USER:$USER ..
docker-compose up -d
```
