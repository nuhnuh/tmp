#


default: docker-run-ubuntu20


################################################################################
# docker

docker-create_image-ubuntu20:
	docker build -t manu/ubuntu20:sudo -f Dockerfile.ubuntu20 .

docker-run-ubuntu20:
	docker run -ti --rm \
	   -e DISPLAY=$(DISPLAY) -v /tmp/.X11-unix:/tmp/.X11-unix \
		 -v $(HOME):/SHARED:ro \
		 -v $(MKFILE_DIR):/home/ubuntu20/snippets-vim \
	   --name ubuntu20_wsudo \
		 manu/ubuntu20:sudo bash

docker-connect-ubuntu20:
	docker exec -ti ubuntu20_wsudo bash

docker-delete-ubuntu20:
	docker rmi manu/ubuntu20:sudo


################################################################################
# utils

# asigned at runtime
MKFILE_DIR = $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
## asigned when declared
#MKFILE_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
