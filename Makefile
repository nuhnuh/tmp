
default: demo_vio
#default: build


MKFILE_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
.PHONY: build
BUILD_DIR := ~/tmp/VELETA/basalt/build
#DATASETS_DIR := /media/manu/JdA_Backup/basalt_datasets
#DATASETS_DIR := /basalt.data
DATASETS_DIR := $(MKFILE_DIR)/../data


######################################
# Run demo

# record dataset (requires an intel realsense t265 connected)
demo_record:
	$(BUILD_DIR)/basalt_rs_t265_record \
		--dataset-path $(DATASETS_DIR)
	echo "INFO: you can check the recorded data with: basalt/scripts/basalt_verify_dataset.py -d /tmp/VELETA.basalt/dataset_202XXXXXX"

# useful for a eassier analysis of the optical flow stage
demo_optflow:
	$(BUILD_DIR)/basalt_opt_flow \
		--dataset-path $(DATASETS_DIR)/t265/sequence0 \
		--cam-calib    $(DATASETS_DIR)/t265/sequence0/calibration.json \
		--dataset-type euroc \
		--config-path $(MKFILE_DIR)/basalt/data/euroc_config.json \
		--show-gui 1

# vio demo (on realsense ds)
demo_vio:
	$(BUILD_DIR)/basalt_vio \
		--dataset-path $(DATASETS_DIR)/t265/sequence0 \
		--cam-calib    $(DATASETS_DIR)/t265/sequence0/calibration.json \
		--dataset-type euroc \
		--config-path $(MKFILE_DIR)/basalt/data/euroc_config.json \
		--marg-data /tmp/basalt.marg_data.t265.sequence0 \
		--show-gui 1

# vio demo (on my dataset)
demo_vio_my_dataset:
	$(BUILD_DIR)/basalt_vio \
		--dataset-path /tmp/dataset_2021_10_04_09_08_11 \
		--cam-calib    /tmp/dataset_2021_10_04_09_08_11/calibration.json \
		--dataset-type euroc \
		--config-path $(MKFILE_DIR)/basalt/data/euroc_config.json \
		--marg-data /tmp/basalt.marg_data.dataset_2021_10_04_09_08_11 \
		--show-gui 1

# vio demo (on UPNA dataset)
demo_vio_upna_jda:
	$(BUILD_DIR)/basalt_vio \
		--dataset-path $(DATASETS_DIR)/t265/upna_jda \
		--cam-calib    $(DATASETS_DIR)/t265/upna_jda/calibration.json \
		--dataset-type euroc \
		--config-path $(MKFILE_DIR)/basalt/data/euroc_config.json \
		--show-gui 1

# vio demo (on UPNA dataset)
demo_vio_upna_exterior:
	$(BUILD_DIR)/basalt_vio \
		--dataset-path $(DATASETS_DIR)/t265/upna_exterior \
		--cam-calib    $(DATASETS_DIR)/t265/upna_exterior/calibration.json \
		--dataset-type euroc \
		--config-path $(MKFILE_DIR)/basalt/data/euroc_config.json \
		--show-gui 1

# vio demo (on TUM ds)
demo_vio_tum:
	$(BUILD_DIR)/basalt_vio \
		--dataset-path "$(DATASETS_DIR)/tum/dataset-calib-cam4_512_16" \
		--cam-calib    $(MKFILE_DIR)/basalt/data/tumvi_512_ds_calib.json \
		--dataset-type euroc \
		--config-path $(MKFILE_DIR)/basalt/data/euroc_config.json \
		--marg-data /tmp/basalt.marg_data.calib-cam4_512_16 \
		--show-gui 1

# vio demo (on TUM ds)
demo_vio_tum_magistrale1:
	$(BUILD_DIR)/basalt_vio \
		--dataset-path "$(DATASETS_DIR)/tum/dataset-magistrale1_512_16" \
		--cam-calib    $(MKFILE_DIR)/basalt/data/tumvi_512_ds_calib.json \
		--dataset-type euroc \
		--config-path $(MKFILE_DIR)/basalt/data/euroc_config.json \
		--marg-data /tmp/basalt.marg_data.magistrale1_512_16 \
		--show-gui 1

# vio demo (on TUM ds)
demo_vio_tum_outdoors1:
	$(BUILD_DIR)/basalt_vio \
		--dataset-path "$(DATASETS_DIR)/tum/dataset-outdoors1_512_16" \
		--cam-calib    $(MKFILE_DIR)/basalt/data/tumvi_512_ds_calib.json \
		--dataset-type euroc \
		--config-path $(MKFILE_DIR)/basalt/data/euroc_config.json \
		--marg-data /tmp/basalt.marg_data.outdoors1_512_16 \
		--show-gui 1

# vio demo (on TUM ds)
demo_vio_tum_slides2:
	$(BUILD_DIR)/basalt_vio \
		--dataset-path "$(DATASETS_DIR)/tum/dataset-slides2_512_16" \
		--cam-calib    $(MKFILE_DIR)/basalt/data/tumvi_512_ds_calib.json \
		--dataset-type euroc \
		--config-path $(MKFILE_DIR)/basalt/data/euroc_config.json \
		--marg-data /tmp/basalt.marg_data.slides2_512_16 \
		--show-gui 1

# vio demo (on EUROC ds)
demo_vio_euroc:
	$(BUILD_DIR)/basalt_vio \
		--dataset-path $(DATASETS_DIR)/euroc/MH_05_difficult \
		--cam-calib    $(MKFILE_DIR)/basalt/data/euroc_ds_calib.json \
		--dataset-type euroc \
		--config-path $(MKFILE_DIR)/basalt/data/euroc_config.json \
		--marg-data /tmp/basalt.marg_data.MH_05_difficult \
		--show-gui 1

# vio demo (on EUROC ds)
demo_vio_euroc_2:
	$(BUILD_DIR)/basalt_vio \
		--dataset-path $(DATASETS_DIR)/euroc/V2_03_difficult \
		--cam-calib    $(MKFILE_DIR)/basalt/data/euroc_ds_calib.json \
		--dataset-type euroc \
		--config-path $(MKFILE_DIR)/basalt/data/euroc_config.json \
		--marg-data /tmp/basalt.marg_data.V2_03_difficult \
		--show-gui 1

# realtime vio demo (requires an intel realsense t265 connected)
demo_vio_t265:
	$(BUILD_DIR)/basalt_rs_t265_vio \
		--config-path $(MKFILE_DIR)/basalt/data/euroc_config.json

# creates a 3d map of tracked points
# IMPORTANT: requires to run first demo_vio
demo_mapper:
	$(BUILD_DIR)/basalt_mapper \
		--cam-calib $(DATASETS_DIR)/t265/sequence0/calibration.json \
		--marg-data /tmp/basalt.marg_data.t265.sequence0

demo_mapper_tum_slides2:
	$(BUILD_DIR)/basalt_mapper \
		--cam-calib $(MKFILE_DIR)/basalt/data/tumvi_512_ds_calib.json \
		--marg-data /tmp/basalt.marg_data.slides2_512_16


# work in progress:

_demo:
	echo "TODO"
	exit 1 # or system will crash? 
	#basalt_vio --dataset-path /data/MH_01_easy.bag --cam-calib /usr/etc/basalt/tumvi_512_eucm_calib.json --dataset-type bag --config-path /usr/etc/basalt/tumvi_512_config.json --marg-data tumi_marg_data --show-gui 1
	basalt_vio --dataset-path /data/TUM_IV_dataset/dataset-magistrale1_512_16/ --cam-calib /usr/etc/basalt/tumvi_512_ds_calib.json --dataset-type euroc --config-path /usr/etc/basalt/tumvi_512_config.json --marg-data tumvi_marg_data --show-gui 1 
	
#	echo $(shell pwd)
#	cd $(MKFILE_DIR); \
#		echo $(shell pwd);
#	@cd $(MKFILE_DIR); \
#		echo $(shell pwd);
#	./demo.bash

_demo_calibrate:
	basalt_calibrate --dataset-path /basalt.data/t265_calib_data/cam_calib \
		--dataset-type euroc --result-path ~/t265_calib_results/ --aprilgrid /usr/etc/basalt/aprilgrid_6x6.json \
		--cam-types eucm eucm
		#--cam-types kb4 kb4

#demo_calibrate_native:
#	LD_LIBRARY_PATH=$(MKFILE_DIR)/tmp/local_install/lib:${LD_LIBRARY_PATH} \
#		$(MKFILE_DIR)/tmp/local_install/bin/basalt_calibrate --dataset-path ../data/t265_calib_data/cam_calib \
#			--dataset-type euroc --result-path /tmp/t265_calib_results/ --aprilgrid basalt/data/aprilgrid_6x6.json \
#			--cam-types eucm eucm
#		#--cam-types kb4 kb4
_demo_calibrate_native:
	$(MKFILE_DIR)/tmp/build/basalt_calibrate --dataset-path ../data/t265_calib_data/cam_calib \
		--dataset-type euroc --result-path /tmp/t265_calib_results/ --aprilgrid basalt/data/aprilgrid_6x6.json \
		--cam-types kb4 kb4
		#--cam-types eucm eucm

_demo_calibrate_imu_native:
	$(MKFILE_DIR)/tmp/build/basalt_calibrate_imu --dataset-path ../basalt.data/t265_calib_data/imu_calib \
		--dataset-type euroc --result-path /tmp/t265_calib_results/ --aprilgrid basalt/data/aprilgrid_6x6.json \
		--accel-noise-std 0.00818 --gyro-noise-std 0.00226 --accel-bias-std 0.01 --gyro-bias-std 0.0007


######################################
# Build

clean:
	rm -rf $(BUILD_DIR)/*
	rm -rf $(MKFILE_DIR)/tmp/local_install

build:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake $(MKFILE_DIR)/basalt \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
		-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
		-DCMAKE_BUILD_TYPE=RelWithDebInfo \
		-DCMAKE_C_COMPILER_LAUNCHER="ccache" -DCMAKE_CXX_COMPILER_LAUNCHER="ccache"
	cd $(BUILD_DIR) && cmake --build . -- -j 3
	#-DCMAKE_INSTALL_PREFIX=$(MKFILE_DIR)/tmp/local_install
	#cd $(BUILD_DIR) && make install


######################################
# Docker

docker-create_image:
	#docker build -t veleta/basalt:ubuntu20 -f Dockerfile.ubuntu20 .
	docker build -t veleta/basalt:ubuntu18 -f Dockerfile.ubuntu18 .

docker-run:
	docker run -ti --rm \
		-e DISPLAY=$${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix \
		-v $(MKFILE_DIR):/basalt.src:ro \
		-v $(DATASETS_DIR):/basalt.data:ro \
		-v $(BUILD_DIR):/home/ubuntu18/tmp/VELETA/basalt/build \
		--name basalt \
		veleta/basalt:ubuntu18 \
		bash

docker-run_realsense:
	docker run -ti --rm \
		--privileged --volume=/dev:/dev \
		-e DISPLAY=$${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix \
		-v $(MKFILE_DIR):/basalt.src:ro \
		-v $(DATASETS_DIR):/basalt.data \
		-v $(BUILD_DIR):/home/ubuntu18/tmp/VELETA/basalt/build \
		--name basalt \
		veleta/basalt:ubuntu18 \
		bash

docker-connect:
	docker exec -it basalt bash
