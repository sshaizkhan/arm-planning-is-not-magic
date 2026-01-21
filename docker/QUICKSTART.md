# Quick Start Guide

## Build and Run

```bash
# Navigate to docker directory
cd docker

# Build the image
make build
# or: docker-compose build

# Start the container
make up
# or: docker-compose up -d

# Open a shell
make shell
# or: docker-compose exec arm-planning bash
```

## Run Demos

Inside the container:

```bash
# Basic planning demo
python3 demos/01_plan_and_time.py

# Compare planners
python3 demos/03_compare_planners.py --parameterizer toppra

# Collision demo
python3 demos/04_collision_demo.py
```

## Stop Container

```bash
make down
# or: docker-compose down
```

## Clean Up

```bash
make clean
# Removes containers, volumes, and images
```
