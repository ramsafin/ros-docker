DOCKER_IMAGE_LINUX = ros-noetic-linux:latest
DOCKER_IMAGE_WSL = ros-noetic-wsl:latest

build_linux:
	@echo "Building for Linux..."
	bash scripts/build_linux.sh

build_wsl:
	@echo "Building for WSL..."
	bash scripts/build_wsl.sh

run_linux:
	@echo "Running Linux container..."
	bash scripts/run_linux.sh

run_wsl:
	@echo "Running WSL container..."
	bash scripts/run_wsl.sh

clean:
	docker rmi $(DOCKER_IMAGE_LINUX) $(DOCKER_IMAGE_WSL)

stop:
	@echo "Stopping containers..."
	docker stop ros-noetic-dev-linux || true
	docker stop ros-noetic-dev-wsl || true
