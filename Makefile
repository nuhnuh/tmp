#

default:

BUILD_DIR := /BUILD_DIR


######################################
#

run_jupyter:
	~/.local/bin/jupyter notebook --no-browser --ip 0.0.0.0


######################################
# Dependencies (GTSAM)

build_gtsam:
	#
	test -d $(BUILD_DIR)
	cmake --version
	python --version
	cd $(BUILD_DIR) \
		&& cmake $(MKFILE_DIR)/3rdparty/gtsam \
			-DGTSAM_BUILD_PYTHON=ON \
			-DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TIMING_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF \
			-DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
			-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_C_COMPILER_LAUNCHER="ccache" -DCMAKE_CXX_COMPILER_LAUNCHER="ccache"
	cd $(BUILD_DIR) \
		&& make -j 4

install_gtsam:
	cd $(BUILD_DIR) \
		&& make install

install_gtsam_python:
	## WARNING: creates files owned by root on BUILD_DIR
	## TIP: find ~/tmp/ -type f -mmin -20 | xargs ls -lt | grep root
	cd $(BUILD_DIR) \
		&& make python-install
	#
	## Solves the permisions problem partially
	#python $(BUILD_DIR)/python/setup.py install --user
	#
	## Solves the permisions problem partially
	#python $(BUILD_DIR)/python/setup.py install

test_gtsam_python:
	python -c "import gtsam; from gtsam import GenericProjectionFactorCal3Unified; help(gtsam.gtsam)"


######################################
# Docker

docker_create_image:
	docker build -t jda/veleta/gtsam_sensor_fusion_snippet:ubuntu20 -f Dockerfile .

docker_run:
	docker run \
		--rm -ti \
		-e DISPLAY=$${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
		-v $(MKFILE_DIR):/WD \
		-v $(BUILD_DIR):/BUILD_DIR \
		-p 8888:8888 \
		--workdir /WD \
		--name gtsam_sensor_fusion_snippet \
		jda/veleta/gtsam_sensor_fusion_snippet:ubuntu20 \
		bash

docker_connect:
	docker exec -it gtsam_sensor_fusion_snippet bash


######################################
# Util

MKFILE_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
