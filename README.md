The objective of the project is to use gtsam to combine data from to data sources to improve accuracy and periodicity, avoid drift, etc.
For example, by using a visual odometry source such as basalt (fast but with dift) and a pose estimation based on place recognition or tag detection (slow, with supply interruptions, but drift free). Other option might be VO + GPS.


# Setup and run

    cd 3rdparty
    git clone https://github.com/borglab/gtsam.git
    cd gtsam
    # TIP: tested on 0f8353f7e
    git checkout 0f8353f7e
    cd ../../

    make docker_create_image
    # create BUILD_DIR (TIP: if BUILD_DIR does not exist docker creates it with root as owner)
    BUILD_DIR=~/tmp/VELETA/gtsam_sensor_fusion_snippet/build
    mkdir -p ${BUILD_DIR}
    #
    make docker_run BUILD_DIR=${BUILD_DIR}

    # (inside docker)
    make build_gtsam
    sudo make install_gtsam  # WARNING: creates files owned by root on BUILD_DIR
    sudo make install_gtsam_python # WARNING: creates files owned by root on BUILD_DIR
    make test_gtsam_python

    # (inside docker)
    pip install jupyter
    pip install matplotlib
    make run_jupyter


TIP: save the docker image with gtsam and jupyter installed

    docker ps
    docker commit c6XXX jda/veleta/gtsam_sensor_fusion_snippet:ubuntu20

    make docker_run BUILD_DIR=${BUILD_DIR}
    make run_jupyter
