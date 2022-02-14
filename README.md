The objective of the project is to use gtsam to combine data from to data sources to improve accuracy and periodicity, avoid drift, etc.
For example, by using a visual odometry source such as basalt (fast but with dift) and a pose estimation based on place recognition or tag detection (slow, with supply interruptions, but drift free). Other option might be VO + GPS.


# Setup and run

    # download dependencies

    # gtsam
    cd 3rdparty
    git clone https://github.com/borglab/gtsam.git
    cd gtsam
    # TIP: tested on 0f8353f7e
    git checkout 0f8353f7e
    cd ../../

    # apriltag
    cd 3rdparty
    git clone https://github.com/AprilRobotics/apriltag apriltag3
    cd apriltag3

    # opengv
    cd 3rdparty
    git clone https://github.com/laurentkneip/opengv
    cd opengv
    git submodule init
    git submodule update

    # docker

    make docker_create_image
    make docker_run
    # TIP: tested on 04c4ec3
    git checkout 04c4ec3
    cd ../../

    # build (inside docker)

    make python_setup  # installs jupyter, etc.

    # opengv
    make opengv_build

    # gtsam
    make gtsam_build
    make gtsam_python_test

    # apriltag3
    make apriltag3_build
    make apriltag3_python_test

    # run (inside docker)
    make run_jupyter





TIP: save the docker image with gtsam and jupyter installed

    docker ps
    docker commit c6XXX jda/veleta/gtsam_sensor_fusion_snippet:ubuntu20

    make docker_run
    make run_jupyter
