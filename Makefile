# TIP:
#    make docker_create_image
#    make docker_run
#
default: docker_run



######################################
#



######################################
# Docker

docker_create_image:
	docker build -t jda/veleta/parse_i3code_xml:ubuntu18 -f Dockerfile .

docker_run:
	docker run \
		--rm -ti \
		-e DISPLAY=$${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
		-v $(MKFILE_DIR):/WD:ro \
		--workdir /WD \
		--name parse_i3code_xml \
		jda/veleta/parse_i3code_xml:ubuntu18 \
		python

docker_connect:
	docker exec -it parse_i3code_xml bash


######################################
# Util

MKFILE_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
