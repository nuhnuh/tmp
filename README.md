
Example on how to use pybind11


# Setup and run

```bash
make docker-create_image-ubuntu20

make docker-run-ubuntu20
sudo apt install build-esential cmake python3-pybind11
sudo apt install libeigen3-dev  # project dependenies (mylib uses eigen)
# sudo apt install python3.8-distutils  # TIP: fix errors
sudo apt install python3-distutils  # TIP: fix errors
sudo apt install python3-dev  # TIP: fix errors
```

```bash
make demo
```


# TIPs

pybin11 [first steps](https://pybind11.readthedocs.io/en/stable/basics.html)

TIP: pybind11 in use
- /home/manu/COVID/VELETA/USLAM/src/3rdparty/opengv/python/pyopengv.cpp
- /home/manu/COVID/VELETA/USLAM/dbg/pangolin-python/pangolin/python/pangolin.cpp

