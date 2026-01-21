.PHONY: build up down shell clean help

# Default target
help:
	@echo "Available targets:"
	@echo "  build    - Build the Docker image"
	@echo "  up       - Start the container"
	@echo "  down     - Stop the container"
	@echo "  shell    - Open a shell in the running container"
	@echo "  clean    - Remove containers and volumes"
	@echo "  rebuild  - Rebuild the image from scratch"
	@echo "  test     - Run all demos to test the setup"

build:
	docker-compose build

up:
	docker-compose up -d

down:
	docker-compose down

shell:
	docker-compose exec arm-planning bash

clean:
	docker-compose down -v
	docker rmi arm-planning-is-not-magic:latest 2>/dev/null || true

rebuild: clean build

test:
	docker-compose exec arm-planning python3 demos/01_plan_and_time.py
	docker-compose exec arm-planning python3 demos/03_compare_planners.py --parameterizer toppra
