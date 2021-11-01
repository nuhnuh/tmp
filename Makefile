default: docker_run


######################################
#

build:
	catkin_make

#
clean:
	rm -rf build devel


######################################
# Docker

docker_create_image:
	docker build -t jda/veleta/tecna_turtle:melodic -f Dockerfile .

docker_run:
	docker run \
		--gpus all \
		--rm -ti \
		-e DISPLAY=$${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
		-v $(MKFILE_DIR):/WD \
		--workdir /WD \
		--name tecna_turtle \
		jda/veleta/tecna_turtle:melodic \
		bash

docker_connect:
	docker exec -it tecna_turtle bash


######################################
# Util

MKFILE_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
