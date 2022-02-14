#

default:

BUILD_DIR := /tmp/sensor_fusion_snippet

PYTHONPATH := $(BUILD_DIR)/gtsam_build/python:$(PYTHONPATH)
PYTHONPATH := $(BUILD_DIR)/apriltag3_build:$(PYTHONPATH)
PYTHONPATH := $(BUILD_DIR)/opengv_build/lib:$(PYTHONPATH)
LD_LIBRARY_PATH := $(BUILD_DIR)/gtsam_build/gtsam:$(LD_LIBRARY_PATH)
LD_LIBRARY_PATH := $(BUILD_DIR)/apriltag3_build:$(LD_LIBRARY_PATH)
LD_LIBRARY_PATH := $(BUILD_DIR)/opengv/lib:$(LD_LIBRARY_PATH)


######################################
# Debug & Development

python_setup:
	mkdir -p $(BUILD_DIR)/venv
	python3 -m venv $(BUILD_DIR)/venv
	. $(BUILD_DIR)/venv/bin/activate \
		&& pip install --upgrade pip \
		&& pip install -r requirements.txt

run_jupyter:
	. $(BUILD_DIR)/venv/bin/activate \
		&& PYTHONPATH=$(PYTHONPATH) LD_LIBRARY_PATH=$(LD_LIBRARY_PATH) \
			jupyter notebook --no-browser --ip 0.0.0.0


######################################
# Dependencies (opengv)
#
# required to create python bindings for jupyter notebooks
opengv_build:
	#
	mkdir -p $(BUILD_DIR)/opengv_build
	cd $(BUILD_DIR)/opengv_build \
		&& . $(BUILD_DIR)/venv/bin/activate \
		&& which python \
		&& cmake $(MKFILE_DIR)/3rdparty/opengv \
			-DCMAKE_BUILD_TYPE=Release \
			-DBUILD_PYTHON=ON
	cd $(BUILD_DIR)/opengv_build \
		&& cmake --build . -- -j 7

opengv_python_test:
	. $(BUILD_DIR)/venv/bin/activate \
		&& PYTHONPATH=$(PYTHONPATH) LD_LIBRARY_PATH=$(LD_LIBRARY_PATH) \
			python -c "import pyopengv; help(pyopengv)"


######################################
# Dependencies (apriltag3)

# TIP: https://github.com/AprilRobotics/apriltag/issues/45
apriltag3_build:
	# TIP: python setup in apriltag3/CMakeLists.txt is a mesh (hardcoded for the system python3 interpreter)
	rm -rf $(BUILD_DIR)/apriltag3_build
	mkdir -p $(BUILD_DIR)/apriltag3_build
	cd $(BUILD_DIR)/apriltag3_build \
		&& . $(BUILD_DIR)/venv/bin/activate \
		&& which python \
		&& python --version \
		&& cmake $(MKFILE_DIR)/3rdparty/apriltag3 \
			-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
			-DCMAKE_BUILD_TYPE=Release
	cd $(BUILD_DIR)/apriltag3_build \
		&& . $(BUILD_DIR)/venv/bin/activate \
		&& which python \
		&& python --version \
		&& make -j 7

apriltag3_python_test:
	. $(BUILD_DIR)/venv/bin/activate \
		&& PYTHONPATH=$(PYTHONPATH) LD_LIBRARY_PATH=$(LD_LIBRARY_PATH) \
			python -c "import apriltag; help(apriltag.apriltag)"


######################################
# Dependencies (GTSAM)

gtsam_build:
	#
	mkdir -p $(BUILD_DIR)/gtsam_build
	cmake --version
	cd $(BUILD_DIR)/gtsam_build \
		&& . $(BUILD_DIR)/venv/bin/activate \
		&& cmake $(MKFILE_DIR)/3rdparty/gtsam \
			-DGTSAM_BUILD_PYTHON=ON \
			-DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TIMING_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF \
			-DGTSAM_BUILD_UNSTABLE=OFF \
			-DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
			-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
			-DCMAKE_BUILD_TYPE=Release
	cd $(BUILD_DIR)/gtsam_build \
		&& make -j 7

gtsam_python_test:
	. $(BUILD_DIR)/venv/bin/activate \
		&& PYTHONPATH=$(PYTHONPATH) LD_LIBRARY_PATH=$(LD_LIBRARY_PATH) \
			python -c "import gtsam; from gtsam import GenericProjectionFactorCal3Unified; help(gtsam.gtsam)"


######################################
# Docker

docker_create_image:
	docker build -t jda/sensor_fusion_snippet:ubuntu20 -f Dockerfile .

docker_run:
	docker run \
		--rm -ti \
		--gpus all \
		-e DISPLAY=$${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
		-v $(MKFILE_DIR):/WD \
		-p 8888:8888 \
		--workdir /WD \
		--name sensor_fusion_snippet \
		jda/sensor_fusion_snippet:ubuntu20 \
		bash

docker_connect:
	docker exec -it sensor_fusion_snippet bash


######################################
# Util

MKFILE_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
