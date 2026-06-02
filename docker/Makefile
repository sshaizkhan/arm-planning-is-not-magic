.PHONY: build up down shell clean help

# Default target
help:
	@echo "Available targets:"
	@echo "  test-build    - Build the Docker image"
	@echo "  test-run       - Start the container"
	@echo "  test-down     - Stop the container"
	@echo "  test-ssh    - Open a shell in the running container"
	@echo "  test-clean    - Remove containers and volumes"
	@echo "  test-rebuild  - Rebuild the image from scratch"
	@echo "  test     - Run all demos to test the setup"

test-build:
	docker compose build

test-run:
	docker compose up -d

test-down:
	docker compose down

test-ssh:
	docker compose exec arm-planning bash

test-clean:
	docker compose down -v
	docker rmi arm-planning-is-not-magic:latest 2>/dev/null || true

test-rebuild: test-clean build

test:
	docker compose exec arm-planning python3 demos/01_plan_and_time.py
	docker compose exec arm-planning python3 demos/03_compare_planners.py --parameterizer toppra
