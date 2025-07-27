all: ensure-build run

# Build the image
build:
	docker build -t quadson-py:latest .

# Save the image to a tar file
save:
	mkdir -p tmp/image
	docker save quadson-py:latest -o tmp/image/quadson-py.tar
	@echo "Image saved to tmp/image/quadson-py.tar"

# Load the image from a tar file
load:
	docker load -i tmp/image/quadson-py.tar

# Build the image if network is present
#     If the cached image is different from the current image, replace the cached image
#     If the cached image is the same as the current image, do nothing
# If not, find the cached image
# If cached image is not present, check if the image is already present
# If all fail, exit with error code
ensure-build:
	@set -e; \
	if ping -c 1 -W 1 8.8.8.8 > /dev/null 2>&1; then \
		echo -e "\e[1;36mNetwork is up, building the image\e[0m"; \
		$(MAKE) build; \
		if [ -f "tmp/dockerfile-hash.txt" ] && sha256sum -c tmp/dockerfile-hash.txt > /dev/null 2>&1; then \
			echo -e "\e[1;36mCached image is up to date\e[0m"; \
		else \
			echo -e "\e[1;36mCached image is different, replacing the cached image\e[0m"; \
			$(MAKE) save; \
			sha256sum Dockerfile > tmp/dockerfile-hash.txt; \
		fi; \
	else \
		echo -e "\e[1;36mNetwork is down, finding the cached image\e[0m"; \
		if [ -f "tmp/image/kiborpc-2025.tar" ]; then \
			echo -e "\e[1;36mCached image found, loading the image\e[0m"; \
			$(MAKE) load; \
		else \
			echo -e "\e[1;36mCached image not found, checking if the image is already present\e[0m"; \
			if [ -z "$$(docker images -q kiborpc-2025:latest)" ]; then \
				echo -e "\e[31mImage not found, please build the image when connected to a network\e[0m"; \
				exit 1; \
			else \
				echo -e "\e[1;36mImage found in docker.\e[0m"; \
			fi; \
		fi; \
	fi

# Run the container
# Note: The following command requires X11 forwarding to be set up on your host machine.
run:
	@if [ -z "$$(docker images -q kiborpc-2025:latest)" ]; then \
		echo "Image kiborpc-2025:latest not found. Please build the image first."; \
		exit 1; \
	fi
	xhost +local:root
	docker run --rm -it \
		--privileged \
		--network=host \
		--cap-add=NET_ADMIN \
		--cap-add=SYS_MODULE \
		-v /lib/modules:/lib/modules:ro \
		-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
		-e XDG_RUNTIME_DIR=/tmp \
		-e QT_X11_NO_MITSHM=1 \
		--env="DISPLAY" \
		--mount type=bind,source=$(CURDIR)/assets,target=/root/quadson_py/assets \
		--mount type=bind,source=$(CURDIR)/src,target=/root/quadson_py/src \
		--mount type=bind,source=$(CURDIR)/tests/,target=/root/quadson_py/tests \
		quadson-py:latest zsh
	xhost -local:root

ssh: