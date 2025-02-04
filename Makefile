DOCKER_IMAGE = ros-noetic-wsl:gpu
DOCKER_CONTAINER = ros-noetic-dev

run:
	@echo "Running a Docker container: '${DOCKER_CONTAINER}'..."
	bash run_docker.sh --image $(DOCKER_IMAGE) --name $(DOCKER_CONTAINER)

build:
	@echo "Building a Docker image: '${DOCKER_IMAGE}'"
	docker build -t $(DOCKER_IMAGE) -f docker/Dockerfile .

clean:
	docker rmi $(DOCKER_IMAGE)

stop:
	@echo "Stopping the Docker container: '${DOCKER_CONTAINER}'"
	docker stop $(DOCKER_CONTAINER) || true
