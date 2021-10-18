

# TODO

https://gitlab.com/VladyslavUsenko/basalt/-/blob/master/doc/Realsense.md
https://gitlab.com/VladyslavUsenko/basalt/-/blob/master/doc/VioMapping.md

calibraiton_helper.cpp/estimateTransformation() shows how to use opengv to recover camera pose from a set of 3d points and the corresponding image points

opengv::triangulation::triangulate()



# Datasets

TIP: More downloaded datasets are in the external HD

TIP: I been using the ASL format

## TUM VI

https://vision.in.tum.de/data/datasets/visual-inertial-dataset

  dataset-magistrale1_512_16.tar

## EUROC

https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

  MH_05_difficult.zip

TIP: maplab works also with this dataset but in .bag format

## realsense (t265_calib_data)

https://gitlab.com/VladyslavUsenko/basalt/-/blob/master/doc/Realsense.md

  sequence0.zip



# Setup

Tested on the dell laptop with ubuntu-18.04 (dependencies can be found in Dockerfile)
and in ubuntu-18 and ubuntu-20 (check again) through docker

First download basalt into the directory of this file
```bash
    git clone --recurse-submodules -j 3 https://gitlab.com/VladyslavUsenko/basalt
```
then build
```bash
    make clean # if needed
    make build
```

## TIP

Build basalt in a docker container
```bash
    make docker-create_image
    make docker-run DATASETS_DIR=/media/manu/JdA_Backup/basalt_datasets
    sudo apt update && sudo apt install ccache # if needed
    make -f /basalt.src/Makefile clean # if needed
    make -f /basalt.src/Makefile build # if needed
    make -f /basalt.src/Makefile demo_vio_euroc DATASETS_DIR=/basalt.data
```

Record dataset from a realsense t265 using a docker container
```bash
    make docker-run_realsense DATASETS_DIR=/tmp
    make -f /basalt.src/Makefile demo_record DATASETS_DIR=/basalt.data
```


# Run

Visual inertial odometry demo (uses saved data)
(tested in ubuntu18 and ubuntu20)

```bash
    make demo_vio
```

TIP: the DATASETS_DIR argument allows to change default path
```bash
    make demo_vio_euroc DATASETS_DIR=/media/manu/JdA_Backup/basalt_datasets
```

TIP: record dataset (to /tmp) from a realsense t265
```bash
    make demo_record DATASETS_DIR=/tmp
```

## TIP

.vscode/launch.json contains the config to debug the app using vscode

## t265 demo

VIO works online using the realsense t265 :)
```bash
    make demo_vio_t265
```

## misc

The next script works on the NUC but not on the laptop (not enought memory)

```bash
    make docker-run
    #sudo apt install python3-opencv python3-matplotlib
    basalt_response_calib.py -d /basalt.data/t265_calib_data/imu_calib
```

works on the laptop:

Calibration demo
```bash
    make demo_calibrate
    make demo_calibrate
```


------------------------------------




